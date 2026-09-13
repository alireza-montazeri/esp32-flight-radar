#include "radar_controller.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "radar_battery.h"
#include "radar_display.h"
#include "radar_haptics.h"
#include "radar_network.h"
#include "user_config.h"
#include "user_encoder_bsp.h"
#include "waveshare_display_port.h"

#define KNOB_IDLE_SAVE_MS 1500
#define RADAR_RADIUS_STEP_DEG 0.05
#define RADAR_RADIUS_MIN_DEG 0.05
#define RADAR_RADIUS_MAX_DEG 2.5
#define AUTHENTICATED_POLL_SECONDS 60
#define ANONYMOUS_POLL_SECONDS 220
#define KNOB_TASK_STACK_BYTES 3072
#define RADAR_TASK_STACK_BYTES (24 * 1024)
#define DETAILS_TASK_STACK_BYTES (14 * 1024)
#define DETAILS_DEBOUNCE_MS 700
#define DETAILS_CACHE_SIZE 8
#define ERROR_RETRY_SECONDS 10
#define ERROR_RETRY_MAX_SECONDS 120
#define WEATHER_REFRESH_SECONDS (15 * 60)
#define WEATHER_RETRY_SECONDS 60
#define WEATHER_TASK_STACK_BYTES (14 * 1024)
#define BATTERY_REFRESH_SECONDS 60
#define BATTERY_TASK_STACK_BYTES 3072

typedef struct
{
    bool used;
    char key[20];
    radar_aircraft_details_t details;
    bool available;
    uint32_t sequence;
} details_cache_entry_t;

static const char *TAG = "radar_controller";
static radar_config_t app_config;
static SemaphoreHandle_t config_mutex;
static TaskHandle_t knob_task_handle;
static TaskHandle_t radar_task_handle;
static TaskHandle_t details_task_handle;
static TaskHandle_t weather_task_handle;
static details_cache_entry_t details_cache[DETAILS_CACHE_SIZE];
static uint32_t details_cache_sequence;

static void read_config(radar_config_t *snapshot)
{
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    *snapshot = app_config;
    xSemaphoreGive(config_mutex);
}

static void update_radius(EventBits_t encoder_bits)
{
    radar_config_t snapshot;
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    if (encoder_bits & BIT0)
        app_config.radius_deg += RADAR_RADIUS_STEP_DEG;
    if (encoder_bits & BIT1)
        app_config.radius_deg -= RADAR_RADIUS_STEP_DEG;
    if (app_config.radius_deg < RADAR_RADIUS_MIN_DEG)
    {
        app_config.radius_deg = RADAR_RADIUS_MIN_DEG;
    }
    if (app_config.radius_deg > RADAR_RADIUS_MAX_DEG)
    {
        app_config.radius_deg = RADAR_RADIUS_MAX_DEG;
    }
    snapshot = app_config;
    xSemaphoreGive(config_mutex);

    radar_display_set_center(snapshot.latitude, snapshot.longitude, snapshot.radius_deg);
}

static void save_config(void)
{
    radar_config_t snapshot;
    read_config(&snapshot);
    ESP_ERROR_CHECK_WITHOUT_ABORT(radar_config_save(&snapshot));
}

static void knob_task(void *arg)
{
    (void)arg;
    bool config_changed = false;

    while (true)
    {
        EventBits_t bits = xEventGroupWaitBits(knob_even_, BIT_EVEN_ALL, pdTRUE, pdFALSE,
                                               pdMS_TO_TICKS(KNOB_IDLE_SAVE_MS));
        if (bits & (BIT0 | BIT1))
        {
            if (waveshare_display_note_activity())
                continue;

            radar_haptics_play_detent();
            const int direction = (bits & BIT1) ? 1 : -1;
            if (!radar_display_rotate_selection(direction))
            {
                update_radius(bits);
                config_changed = true;
            }
        }
        else if (config_changed)
        {
            save_config();
            config_changed = false;
        }
    }
}

static radar_aircraft_list_t *allocate_aircraft_list(void)
{
    radar_aircraft_list_t *aircraft = heap_caps_calloc(
        1, sizeof(*aircraft), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!aircraft)
        aircraft = calloc(1, sizeof(*aircraft));
    return aircraft;
}

static void make_details_key(const radar_aircraft_t *aircraft, char *key, size_t key_size)
{
    snprintf(key, key_size, "%s/%s", aircraft->icao24, aircraft->callsign);
}

static bool details_cache_find(const char *key, radar_aircraft_details_t *details,
                               bool *available)
{
    for (size_t i = 0; i < DETAILS_CACHE_SIZE; ++i)
    {
        if (details_cache[i].used && strcmp(details_cache[i].key, key) == 0)
        {
            *details = details_cache[i].details;
            *available = details_cache[i].available;
            details_cache[i].sequence = ++details_cache_sequence;
            return true;
        }
    }
    return false;
}

static void details_cache_store(const char *key, const radar_aircraft_details_t *details,
                                bool available)
{
    size_t slot = 0;
    for (size_t i = 0; i < DETAILS_CACHE_SIZE; ++i)
    {
        if (!details_cache[i].used)
        {
            slot = i;
            break;
        }
        if (details_cache[i].sequence < details_cache[slot].sequence)
            slot = i;
    }
    details_cache[slot].used = true;
    strlcpy(details_cache[slot].key, key, sizeof(details_cache[slot].key));
    details_cache[slot].details = *details;
    details_cache[slot].available = available;
    details_cache[slot].sequence = ++details_cache_sequence;
}

static void aircraft_details_task(void *arg)
{
    (void)arg;
    char completed_key[20] = {0};

    while (true)
    {
        radar_display_wait_for_selection_change(UINT32_MAX);

        radar_aircraft_t selected;
        if (!radar_network_is_connected() ||
            !radar_display_get_selected_aircraft(&selected))
        {
            completed_key[0] = '\0';
            continue;
        }

        char selected_key[20];
        make_details_key(&selected, selected_key, sizeof(selected_key));
        if (strcmp(selected_key, completed_key) == 0)
            continue;

        radar_display_set_details_loading(selected.icao24);
        if (radar_display_wait_for_selection_change(DETAILS_DEBOUNCE_MS))
            continue;

        radar_aircraft_details_t details = {0};
        bool available = false;
        int http_status = 0;
        if (!details_cache_find(selected_key, &details, &available))
        {
            esp_err_t err = radar_network_fetch_aircraft_details(
                &selected, &details, &http_status);
            available = err == ESP_OK;
            if (available || http_status == 404)
                details_cache_store(selected_key, &details, available);
            if (!available)
            {
                ESP_LOGW(TAG, "Aircraft detail lookup failed: HTTP %d (%s)",
                         http_status, esp_err_to_name(err));
            }
        }

        radar_display_set_aircraft_details(selected.icao24, &details, available);
        strlcpy(completed_key, selected_key, sizeof(completed_key));
    }
}

static void show_connection_status(void)
{
    if (radar_network_setup_ap_active())
    {
        radar_display_set_status("Join FlightRadar-Setup - 192.168.4.1");
    }
    else
    {
        radar_display_set_status("Waiting for Wi-Fi...");
    }
}

static int current_poll_interval_seconds(void)
{
    return radar_network_is_authenticated()
               ? AUTHENTICATED_POLL_SECONDS
               : ANONYMOUS_POLL_SECONDS;
}

static void show_fetch_result(esp_err_t err, int http_status,
                              const radar_aircraft_list_t *aircraft)
{
    if (err == ESP_OK)
    {
        radar_display_update_aircraft(aircraft);
        radar_display_set_status("Live");
        ESP_LOGI(TAG, "OpenSky update: %u aircraft", (unsigned)aircraft->count);
        return;
    }

    char status[64];
    snprintf(status, sizeof(status), "OpenSky error HTTP %d (%s)",
             http_status, esp_err_to_name(err));
    radar_display_set_status(status);
    ESP_LOGW(TAG, "%s", status);
}

static bool fetch_aircraft(radar_aircraft_list_t *aircraft)
{
    radar_config_t snapshot;
    read_config(&snapshot);
    radar_display_set_status("Fetching");

    int http_status = 0;
    esp_err_t err = radar_network_fetch_aircraft(&snapshot, aircraft, &http_status);
    show_fetch_result(err, http_status, aircraft);
    return err == ESP_OK;
}

static void wait_for_next_fetch(void)
{
    const int wait_seconds = current_poll_interval_seconds();
    vTaskDelay(pdMS_TO_TICKS(wait_seconds * 1000));
}

static void radar_task(void *arg)
{
    (void)arg;
    radar_aircraft_list_t *aircraft = allocate_aircraft_list();
    assert(aircraft);
    unsigned retry_seconds = ERROR_RETRY_SECONDS;

    while (true)
    {
        if (!radar_network_is_connected())
        {
            show_connection_status();
            radar_network_wait_until_connected(UINT32_MAX);
            continue;
        }

        const bool fetched = fetch_aircraft(aircraft);
        ESP_LOGI(TAG, "Radar task minimum free stack: %u bytes",
                 (unsigned)uxTaskGetStackHighWaterMark(NULL));
        if (fetched)
        {
            retry_seconds = ERROR_RETRY_SECONDS;
            wait_for_next_fetch();
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(retry_seconds * 1000));
            retry_seconds = retry_seconds < ERROR_RETRY_MAX_SECONDS / 2
                                ? retry_seconds * 2
                                : ERROR_RETRY_MAX_SECONDS;
        }
    }
}

static void weather_task(void *arg)
{
    (void)arg;
    static const char *const city_names[RADAR_CITY_COUNT] = {
        [RADAR_CITY_MELBOURNE] = "Melbourne",
        [RADAR_CITY_TEHRAN] = "Tehran",
    };
    int64_t next_refresh_us[RADAR_CITY_COUNT] = {0};

    while (true)
    {
        if (!radar_network_is_connected())
        {
            radar_network_wait_until_connected(UINT32_MAX);
            continue;
        }

        int64_t now_us = esp_timer_get_time();
        for (radar_city_t city = RADAR_CITY_MELBOURNE;
             city < RADAR_CITY_COUNT; ++city)
        {
            if (now_us < next_refresh_us[city])
                continue;

            radar_weather_t weather = {0};
            int http_status = 0;
            const esp_err_t err = radar_network_fetch_city_weather(
                city, &weather, &http_status);
            if (err == ESP_OK)
            {
                radar_display_set_weather(city, &weather);
                ESP_LOGI(TAG, "%s weather updated: %.1f C, WMO %d",
                         city_names[city], weather.temperature_c,
                         weather.weather_code);
            }
            else
            {
                ESP_LOGW(TAG, "%s weather update failed: HTTP %d (%s)",
                         city_names[city], http_status, esp_err_to_name(err));
            }
            next_refresh_us[city] = esp_timer_get_time() +
                                    (int64_t)(err == ESP_OK
                                                  ? WEATHER_REFRESH_SECONDS
                                                  : WEATHER_RETRY_SECONDS) *
                                        1000000;
            now_us = esp_timer_get_time();
        }

        int64_t earliest_us = next_refresh_us[0];
        for (radar_city_t city = RADAR_CITY_MELBOURNE + 1;
             city < RADAR_CITY_COUNT; ++city)
        {
            if (next_refresh_us[city] < earliest_us)
                earliest_us = next_refresh_us[city];
        }
        now_us = esp_timer_get_time();
        const int64_t wait_ms = earliest_us > now_us
                                    ? (earliest_us - now_us + 999) / 1000
                                    : 1;
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }
}

static void battery_task(void *arg)
{
    (void)arg;
    while (true)
    {
        int percentage = 0;
        const esp_err_t err = radar_battery_read_percentage(&percentage);
        if (err == ESP_OK)
            radar_display_set_battery(percentage);
        else
            ESP_LOGW(TAG, "Battery ADC read failed: %s", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(BATTERY_REFRESH_SECONDS * 1000));
    }
}

esp_err_t radar_controller_start(const radar_config_t *initial_config)
{
    if (!initial_config)
        return ESP_ERR_INVALID_ARG;

    config_mutex = xSemaphoreCreateMutex();
    if (!config_mutex)
        return ESP_ERR_NO_MEM;
    app_config = *initial_config;

    esp_err_t battery_err = radar_battery_init();
    if (battery_err != ESP_OK)
        return battery_err;

    user_encoder_init();
    BaseType_t created = xTaskCreate(knob_task, "radar_knob", KNOB_TASK_STACK_BYTES,
                                     NULL, 3, &knob_task_handle);
    if (created != pdPASS)
        return ESP_ERR_NO_MEM;

    created = xTaskCreate(radar_task, "opensky_radar", RADAR_TASK_STACK_BYTES,
                          NULL, 4, &radar_task_handle);
    if (created != pdPASS)
    {
        vTaskDelete(knob_task_handle);
        knob_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    created = xTaskCreate(aircraft_details_task, "aircraft_details",
                          DETAILS_TASK_STACK_BYTES, NULL, 3, &details_task_handle);
    if (created != pdPASS)
    {
        vTaskDelete(radar_task_handle);
        radar_task_handle = NULL;
        vTaskDelete(knob_task_handle);
        knob_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    created = xTaskCreate(weather_task, "city_weather",
                          WEATHER_TASK_STACK_BYTES, NULL, 3, &weather_task_handle);
    if (created != pdPASS)
    {
        vTaskDelete(details_task_handle);
        details_task_handle = NULL;
        vTaskDelete(radar_task_handle);
        radar_task_handle = NULL;
        vTaskDelete(knob_task_handle);
        knob_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    created = xTaskCreate(battery_task, "battery_monitor",
                          BATTERY_TASK_STACK_BYTES, NULL, 2, NULL);
    if (created != pdPASS)
    {
        vTaskDelete(weather_task_handle);
        weather_task_handle = NULL;
        vTaskDelete(details_task_handle);
        details_task_handle = NULL;
        vTaskDelete(radar_task_handle);
        radar_task_handle = NULL;
        vTaskDelete(knob_task_handle);
        knob_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
