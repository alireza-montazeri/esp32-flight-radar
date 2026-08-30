#include "radar_network.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "mdns.h"
#include "radar_display.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAILED_BIT    BIT1
#define WIFI_MAX_RETRIES   8
#define RESPONSE_LIMIT     (256 * 1024)

typedef struct {
    char *data;
    size_t length;
    size_t capacity;
    bool overflow;
} response_buffer_t;

static const char *TAG = "radar_network";
static EventGroupHandle_t wifi_events;
static SemaphoreHandle_t http_mutex;
static int wifi_retries;
static bool connected;
static bool setup_ap;
static bool mdns_started;
static radar_config_t web_config;
static httpd_handle_t web_server;
static char bearer_token[2048];
static int64_t bearer_expiry_us;

static esp_err_t http_event(esp_http_client_event_t *event)
{
    response_buffer_t *buffer = event->user_data;
    if (event->event_id != HTTP_EVENT_ON_DATA || event->data_len <= 0 || buffer->overflow) {
        return ESP_OK;
    }
    const size_t needed = buffer->length + event->data_len + 1;
    if (needed > RESPONSE_LIMIT) {
        buffer->overflow = true;
        return ESP_ERR_NO_MEM;
    }
    if (needed > buffer->capacity) {
        size_t capacity = buffer->capacity ? buffer->capacity : 4096;
        while (capacity < needed) capacity *= 2;
        if (capacity > RESPONSE_LIMIT) capacity = RESPONSE_LIMIT;
        char *grown = heap_caps_realloc(buffer->data, capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!grown) grown = realloc(buffer->data, capacity);
        if (!grown) {
            buffer->overflow = true;
            return ESP_ERR_NO_MEM;
        }
        buffer->data = grown;
        buffer->capacity = capacity;
    }
    memcpy(buffer->data + buffer->length, event->data, event->data_len);
    buffer->length += event->data_len;
    buffer->data[buffer->length] = '\0';
    return ESP_OK;
}

static esp_err_t perform_http(const char *url, esp_http_client_method_t method,
                              const char *body, const char *bearer,
                              response_buffer_t *response, int *status,
                              int total_timeout_ms)
{
    memset(response, 0, sizeof(*response));
    *status = 0;
    if (!http_mutex ||
        xSemaphoreTake(http_mutex, pdMS_TO_TICKS(total_timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_http_client_config_t config = {
        .url = url,
        .method = method,
        .timeout_ms = 2000,
        .buffer_size = 4096,
        .buffer_size_tx = 2048,
        .event_handler = http_event,
        .user_data = response,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
        .is_async = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        xSemaphoreGive(http_mutex);
        return ESP_ERR_NO_MEM;
    }
    if (bearer && bearer[0]) {
        char header[2080];
        snprintf(header, sizeof(header), "Bearer %s", bearer);
        esp_http_client_set_header(client, "Authorization", header);
    }
    if (method == HTTP_METHOD_POST) {
        esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
        esp_http_client_set_post_field(client, body, strlen(body));
    }
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)total_timeout_ms * 1000;
    esp_err_t err;
    do {
        err = esp_http_client_perform(client);
        if (err != ESP_ERR_HTTP_EAGAIN) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (esp_timer_get_time() < deadline_us);

    if (err == ESP_ERR_HTTP_EAGAIN) {
        ESP_LOGW(TAG, "HTTPS request exceeded %d ms", total_timeout_ms);
        esp_http_client_close(client);
        err = ESP_ERR_TIMEOUT;
    }
    *status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    if (response->overflow) err = ESP_ERR_NO_MEM;
    xSemaphoreGive(http_mutex);
    return err;
}

static void form_encode(const char *source, char *dest, size_t dest_size)
{
    static const char hex[] = "0123456789ABCDEF";
    size_t out = 0;
    for (size_t i = 0; source[i] && out + 1 < dest_size; ++i) {
        const unsigned char c = (unsigned char)source[i];
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            dest[out++] = c;
        } else if (out + 3 < dest_size) {
            dest[out++] = '%';
            dest[out++] = hex[c >> 4];
            dest[out++] = hex[c & 0x0f];
        }
    }
    dest[out] = '\0';
}

static esp_err_t refresh_bearer(const radar_config_t *config)
{
    if (!config->opensky_client_id[0] || !config->opensky_client_secret[0]) {
        bearer_token[0] = '\0';
        return ESP_OK;
    }
    if (bearer_token[0] && esp_timer_get_time() < bearer_expiry_us) return ESP_OK;

    char encoded_id[sizeof(config->opensky_client_id) * 3];
    char encoded_secret[sizeof(config->opensky_client_secret) * 3];
    form_encode(config->opensky_client_id, encoded_id, sizeof(encoded_id));
    form_encode(config->opensky_client_secret, encoded_secret, sizeof(encoded_secret));
    char body[sizeof(encoded_id) + sizeof(encoded_secret) + 64];
    snprintf(body, sizeof(body), "grant_type=client_credentials&client_id=%s&client_secret=%s",
             encoded_id, encoded_secret);

    response_buffer_t response;
    int status = 0;
    esp_err_t err = perform_http(
        "https://auth.opensky-network.org/auth/realms/opensky-network/protocol/openid-connect/token",
        HTTP_METHOD_POST, body, NULL, &response, &status, 20000);
    if (err != ESP_OK || status != 200 || !response.data) {
        ESP_LOGW(TAG, "OpenSky OAuth failed: %s, HTTP %d", esp_err_to_name(err), status);
        free(response.data);
        bearer_token[0] = '\0';
        return err == ESP_OK ? ESP_FAIL : err;
    }

    cJSON *root = cJSON_Parse(response.data);
    cJSON *token = root ? cJSON_GetObjectItemCaseSensitive(root, "access_token") : NULL;
    cJSON *expires = root ? cJSON_GetObjectItemCaseSensitive(root, "expires_in") : NULL;
    if (!cJSON_IsString(token)) {
        err = ESP_ERR_INVALID_RESPONSE;
    } else {
        strlcpy(bearer_token, token->valuestring, sizeof(bearer_token));
        int lifetime = cJSON_IsNumber(expires) ? expires->valueint : 1800;
        if (lifetime > 60) lifetime -= 60;
        bearer_expiry_us = esp_timer_get_time() + (int64_t)lifetime * 1000000;
        err = ESP_OK;
    }
    cJSON_Delete(root);
    free(response.data);
    return err;
}

static double json_number(cJSON *array, int index, double fallback)
{
    cJSON *item = cJSON_GetArrayItem(array, index);
    return cJSON_IsNumber(item) ? item->valuedouble : fallback;
}

static void copy_json_string(cJSON *array, int index, char *dest, size_t dest_size)
{
    cJSON *item = cJSON_GetArrayItem(array, index);
    dest[0] = '\0';
    if (cJSON_IsString(item)) strlcpy(dest, item->valuestring, dest_size);
    size_t length = strlen(dest);
    while (length && isspace((unsigned char)dest[length - 1])) dest[--length] = '\0';
}

static void copy_object_string(cJSON *object, const char *name, char *dest, size_t dest_size)
{
    cJSON *item = cJSON_GetObjectItemCaseSensitive(object, name);
    dest[0] = '\0';
    if (cJSON_IsString(item)) strlcpy(dest, item->valuestring, dest_size);
}

esp_err_t radar_network_fetch_aircraft(const radar_config_t *config,
                                       radar_aircraft_list_t *aircraft,
                                       int *http_status)
{
    memset(aircraft, 0, sizeof(*aircraft));
    *http_status = 0;
    if (!connected) return ESP_ERR_INVALID_STATE;

    if (refresh_bearer(config) != ESP_OK) {
        ESP_LOGW(TAG, "Continuing with anonymous OpenSky access");
    }
    char url[384];
    snprintf(url, sizeof(url),
             "https://opensky-network.org/api/states/all?lamin=%.6f&lamax=%.6f&lomin=%.6f&lomax=%.6f&extended=1",
             config->latitude - config->radius_deg,
             config->latitude + config->radius_deg,
             config->longitude - config->radius_deg,
             config->longitude + config->radius_deg);

    response_buffer_t response;
    ESP_LOGI(TAG, "OpenSky states request started");
    esp_err_t err = perform_http(url, HTTP_METHOD_GET, NULL, bearer_token, &response,
                                 http_status, 20000);
    ESP_LOGI(TAG, "OpenSky states request finished: %s, HTTP %d, %u bytes",
             esp_err_to_name(err), *http_status, (unsigned)response.length);
    if (err != ESP_OK || *http_status != 200 || !response.data) {
        if (*http_status == 401) {
            bearer_token[0] = '\0';
            bearer_expiry_us = 0;
        }
        free(response.data);
        return err == ESP_OK ? ESP_ERR_INVALID_RESPONSE : err;
    }

    cJSON *root = cJSON_Parse(response.data);
    free(response.data);
    if (!root) return ESP_ERR_INVALID_RESPONSE;
    cJSON *states = cJSON_GetObjectItemCaseSensitive(root, "states");
    if (!cJSON_IsArray(states)) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_RESPONSE;
    }

    cJSON *entry;
    cJSON_ArrayForEach(entry, states) {
        if (aircraft->count >= RADAR_MAX_AIRCRAFT) break;
        if (!cJSON_IsArray(entry) || !cJSON_IsNumber(cJSON_GetArrayItem(entry, 5)) ||
            !cJSON_IsNumber(cJSON_GetArrayItem(entry, 6))) continue;
        cJSON *ground = cJSON_GetArrayItem(entry, 8);
        if (cJSON_IsTrue(ground)) continue;

        radar_aircraft_t *item = &aircraft->items[aircraft->count++];
        copy_json_string(entry, 0, item->icao24, sizeof(item->icao24));
        copy_json_string(entry, 1, item->callsign, sizeof(item->callsign));
        copy_json_string(entry, 2, item->origin_country, sizeof(item->origin_country));
        item->longitude = json_number(entry, 5, 0);
        item->latitude = json_number(entry, 6, 0);
        item->altitude_m = json_number(entry, 7, 0);
        item->on_ground = false;
        item->velocity_mps = json_number(entry, 9, 0);
        item->track_deg = json_number(entry, 10, 0);
        item->vertical_rate_mps = json_number(entry, 11, 0);
        copy_json_string(entry, 14, item->squawk, sizeof(item->squawk));
        item->category = (uint8_t)json_number(entry, 17, 0);
        item->received_us = esp_timer_get_time();
    }
    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t radar_network_fetch_aircraft_details(const radar_aircraft_t *aircraft,
                                               radar_aircraft_details_t *details,
                                               int *http_status)
{
    if (!aircraft || !details || !http_status || !aircraft->icao24[0]) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(details, 0, sizeof(*details));
    strlcpy(details->icao24, aircraft->icao24, sizeof(details->icao24));
    *http_status = 0;
    if (!connected) return ESP_ERR_INVALID_STATE;

    char url[256];
    if (aircraft->callsign[0]) {
        char encoded_callsign[sizeof(aircraft->callsign) * 3];
        form_encode(aircraft->callsign, encoded_callsign, sizeof(encoded_callsign));
        snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/aircraft/%s?callsign=%s",
                 aircraft->icao24, encoded_callsign);
    } else {
        snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/aircraft/%s",
                 aircraft->icao24);
    }

    response_buffer_t response;
    esp_err_t err = perform_http(url, HTTP_METHOD_GET, NULL, NULL, &response, http_status, 8000);
    if (err != ESP_OK || *http_status != 200 || !response.data) {
        free(response.data);
        return err == ESP_OK && *http_status == 404 ? ESP_ERR_NOT_FOUND :
               err == ESP_OK ? ESP_ERR_INVALID_RESPONSE : err;
    }

    cJSON *root = cJSON_Parse(response.data);
    free(response.data);
    cJSON *payload = root ? cJSON_GetObjectItemCaseSensitive(root, "response") : NULL;
    cJSON *aircraft_json = cJSON_IsObject(payload)
                               ? cJSON_GetObjectItemCaseSensitive(payload, "aircraft")
                               : NULL;
    cJSON *route = cJSON_IsObject(payload)
                       ? cJSON_GetObjectItemCaseSensitive(payload, "flightroute")
                       : NULL;

    if (cJSON_IsObject(aircraft_json)) {
        details->aircraft_found = true;
        copy_object_string(aircraft_json, "registration", details->registration,
                           sizeof(details->registration));
        copy_object_string(aircraft_json, "type", details->type, sizeof(details->type));
        copy_object_string(aircraft_json, "icao_type", details->icao_type,
                           sizeof(details->icao_type));
        copy_object_string(aircraft_json, "manufacturer", details->manufacturer,
                           sizeof(details->manufacturer));
        copy_object_string(aircraft_json, "registered_owner", details->owner,
                           sizeof(details->owner));
    }

    if (cJSON_IsObject(route)) {
        cJSON *airline = cJSON_GetObjectItemCaseSensitive(route, "airline");
        cJSON *origin = cJSON_GetObjectItemCaseSensitive(route, "origin");
        cJSON *destination = cJSON_GetObjectItemCaseSensitive(route, "destination");
        if (cJSON_IsObject(airline)) {
            copy_object_string(airline, "name", details->airline, sizeof(details->airline));
        }
        if (cJSON_IsObject(origin) && cJSON_IsObject(destination)) {
            details->route_found = true;
            copy_object_string(origin, "icao_code", details->origin_icao,
                               sizeof(details->origin_icao));
            copy_object_string(origin, "iata_code", details->origin_iata,
                               sizeof(details->origin_iata));
            copy_object_string(origin, "name", details->origin_name,
                               sizeof(details->origin_name));
            copy_object_string(origin, "municipality", details->origin_city,
                               sizeof(details->origin_city));
            copy_object_string(destination, "icao_code", details->destination_icao,
                               sizeof(details->destination_icao));
            copy_object_string(destination, "iata_code", details->destination_iata,
                               sizeof(details->destination_iata));
            copy_object_string(destination, "name", details->destination_name,
                               sizeof(details->destination_name));
            copy_object_string(destination, "municipality", details->destination_city,
                               sizeof(details->destination_city));
        }
    }

    const bool found = details->aircraft_found || details->route_found;
    cJSON_Delete(root);
    return found ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void url_decode(const char *source, size_t length, char *dest, size_t dest_size)
{
    size_t out = 0;
    for (size_t i = 0; i < length && out + 1 < dest_size; ++i) {
        if (source[i] == '+' ) {
            dest[out++] = ' ';
        } else if (source[i] == '%' && i + 2 < length) {
            int high = hex_value(source[i + 1]);
            int low = hex_value(source[i + 2]);
            if (high >= 0 && low >= 0) {
                dest[out++] = (char)((high << 4) | low);
                i += 2;
            }
        } else {
            dest[out++] = source[i];
        }
    }
    dest[out] = '\0';
}

static bool form_value(const char *body, const char *key, char *dest, size_t dest_size)
{
    const size_t key_len = strlen(key);
    const char *part = body;
    while (*part) {
        const char *end = strchr(part, '&');
        if (!end) end = part + strlen(part);
        const char *equals = memchr(part, '=', end - part);
        if (equals && (size_t)(equals - part) == key_len && !memcmp(part, key, key_len)) {
            url_decode(equals + 1, end - equals - 1, dest, dest_size);
            return true;
        }
        part = *end ? end + 1 : end;
    }
    dest[0] = '\0';
    return false;
}

static esp_err_t config_get(httpd_req_t *request)
{
    radar_config_t current;
    if (radar_config_load(&current) != ESP_OK) current = web_config;
    char *page = malloc(6144);
    if (!page) return ESP_ERR_NO_MEM;
    snprintf(page, 6144,
        "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>Flight Radar Setup</title><style>body{background:#071109;color:#45ff70;font:16px monospace;max-width:700px;margin:auto;padding:24px}"
        "fieldset,input,button{border:1px solid #24c950;background:#071109;color:#45ff70;padding:10px;margin:5px 0}label{display:block;margin-top:12px}"
        "input{box-sizing:border-box;width:100%%}button{background:#24c950;color:#021a08;font-weight:bold;width:100%%}</style></head><body>"
        "<h1>ESP32 Flight Radar</h1><p>Configure Wi-Fi and the radar centre. Blank password/secret fields keep their stored values.</p>"
        "<form method=post action=/save><fieldset><legend>Network</legend>"
        "<label>Wi-Fi SSID<input name=ssid maxlength=32 value='%s' required></label>"
        "<label>Wi-Fi password<input name=password type=password maxlength=64></label></fieldset>"
        "<fieldset><legend>Radar</legend><label>Latitude<input name=latitude type=number min=-90 max=90 step=.000001 value='%.6f' required></label>"
        "<label>Longitude<input name=longitude type=number min=-180 max=180 step=.000001 value='%.6f' required></label>"
        "<label>Radius in degrees (0.05-2.5)<input name=radius type=number min=.05 max=2.5 step=.05 value='%.2f' required></label>"
        "<label><input style='width:auto' name=sweep type=checkbox %s> animated sweep</label>"
        "<label><input style='width:auto' name=labels type=checkbox %s> aircraft labels</label></fieldset>"
        "<fieldset><legend>OpenSky (optional)</legend><label>OAuth client ID<input name=client_id maxlength=95 value='%s'></label>"
        "<label>OAuth client secret<input name=client_secret type=password maxlength=127></label></fieldset>"
        "<button type=submit>Save and restart</button></form><p>After connection: <b>http://flight-radar.local/</b></p></body></html>",
        current.wifi_ssid, current.latitude, current.longitude, current.radius_deg,
        current.show_sweep ? "checked" : "", current.show_labels ? "checked" : "",
        current.opensky_client_id);
    httpd_resp_set_type(request, "text/html");
    esp_err_t err = httpd_resp_send(request, page, HTTPD_RESP_USE_STRLEN);
    free(page);
    return err;
}

static esp_err_t config_save(httpd_req_t *request)
{
    if (request->content_len <= 0 || request->content_len > 2048) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid form");
    }
    char *body = calloc(1, request->content_len + 1);
    if (!body) return ESP_ERR_NO_MEM;
    int received = 0;
    while (received < request->content_len) {
        int got = httpd_req_recv(request, body + received, request->content_len - received);
        if (got <= 0) {
            free(body);
            return ESP_FAIL;
        }
        received += got;
    }

    radar_config_t updated;
    if (radar_config_load(&updated) != ESP_OK) updated = web_config;
    char value[160];
    form_value(body, "ssid", updated.wifi_ssid, sizeof(updated.wifi_ssid));
    if (form_value(body, "password", value, sizeof(value)) && value[0])
        strlcpy(updated.wifi_password, value, sizeof(updated.wifi_password));
    if (form_value(body, "latitude", value, sizeof(value))) updated.latitude = strtod(value, NULL);
    if (form_value(body, "longitude", value, sizeof(value))) updated.longitude = strtod(value, NULL);
    if (form_value(body, "radius", value, sizeof(value))) updated.radius_deg = strtod(value, NULL);
    form_value(body, "client_id", updated.opensky_client_id, sizeof(updated.opensky_client_id));
    if (form_value(body, "client_secret", value, sizeof(value)) && value[0])
        strlcpy(updated.opensky_client_secret, value, sizeof(updated.opensky_client_secret));
    updated.show_sweep = strstr(body, "sweep=") != NULL;
    updated.show_labels = strstr(body, "labels=") != NULL;
    free(body);

    if (!updated.wifi_ssid[0] || updated.latitude < -90 || updated.latitude > 90 ||
        updated.longitude < -180 || updated.longitude > 180 ||
        updated.radius_deg < 0.05 || updated.radius_deg > 2.5) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Configuration values out of range");
    }
    esp_err_t err = radar_config_save(&updated);
    if (err != ESP_OK) return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
    httpd_resp_sendstr(request, "Saved. The radar is restarting...");
    vTaskDelay(pdMS_TO_TICKS(600));
    esp_restart();
    return ESP_OK;
}

static esp_err_t start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.stack_size = 6144;
    esp_err_t err = httpd_start(&web_server, &config);
    if (err != ESP_OK) return err;
    const httpd_uri_t save = {.uri = "/save", .method = HTTP_POST, .handler = config_save};
    const httpd_uri_t root = {.uri = "/*", .method = HTTP_GET, .handler = config_get};
    ESP_ERROR_CHECK(httpd_register_uri_handler(web_server, &save));
    ESP_ERROR_CHECK(httpd_register_uri_handler(web_server, &root));
    return ESP_OK;
}

static void start_mdns(void)
{
    if (mdns_started) return;
    if (mdns_init() == ESP_OK) {
        mdns_hostname_set("flight-radar");
        mdns_instance_name_set("ESP32 Flight Radar");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        mdns_started = true;
    }
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START && web_config.wifi_ssid[0]) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        connected = false;
        radar_display_set_wifi_connected(false);
        if (wifi_retries++ < WIFI_MAX_RETRIES) {
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(wifi_events, WIFI_FAILED_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        wifi_retries = 0;
        connected = true;
        radar_display_set_wifi_connected(true);
        xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
        start_mdns();
        radar_display_set_status("Open flight-radar.local to configure");
    }
}

static esp_err_t start_setup_ap(void)
{
    wifi_config_t ap = {
        .ap = {
            .ssid = "FlightRadar-Setup",
            .ssid_len = 17,
            .channel = 1,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
            .pmf_cfg = {.required = false},
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    setup_ap = true;
    radar_display_set_status("Join FlightRadar-Setup - 192.168.4.1");
    ESP_LOGI(TAG, "Setup AP ready: FlightRadar-Setup, http://192.168.4.1/");
    return ESP_OK;
}

esp_err_t radar_network_start(const radar_config_t *config)
{
    web_config = *config;
    wifi_events = xEventGroupCreate();
    http_mutex = xSemaphoreCreateMutex();
    if (!wifi_events || !http_mutex) return ESP_ERR_NO_MEM;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));

    wifi_config_t sta = {0};
    strlcpy((char *)sta.sta.ssid, config->wifi_ssid, sizeof(sta.sta.ssid));
    strlcpy((char *)sta.sta.password, config->wifi_password, sizeof(sta.sta.password));
    sta.sta.threshold.authmode = WIFI_AUTH_OPEN;
    sta.sta.pmf_cfg.capable = true;
    sta.sta.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(start_web_server());

    if (!radar_config_has_wifi(config)) return start_setup_ap();
    radar_display_set_status("Connecting to Wi-Fi...");
    EventBits_t bits = xEventGroupWaitBits(wifi_events, WIFI_CONNECTED_BIT | WIFI_FAILED_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));
    if (!(bits & WIFI_CONNECTED_BIT)) return start_setup_ap();
    return ESP_OK;
}

bool radar_network_is_connected(void)
{
    return connected;
}

bool radar_network_setup_ap_active(void)
{
    return setup_ap;
}

bool radar_network_is_authenticated(void)
{
    return bearer_token[0] != '\0' && esp_timer_get_time() < bearer_expiry_us;
}
