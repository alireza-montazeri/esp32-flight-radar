#include "radar_config.h"

#include <string.h>
#include "nvs.h"

#define CONFIG_NAMESPACE "radar"

static void get_string(nvs_handle_t nvs, const char *key, char *dest, size_t dest_size)
{
    size_t required = dest_size;
    if (nvs_get_str(nvs, key, dest, &required) != ESP_OK) {
        dest[0] = '\0';
    }
}

void radar_config_defaults(radar_config_t *config)
{
    memset(config, 0, sizeof(*config));
    config->latitude = 0.0;  /* Configure the actual location at runtime. */
    config->longitude = 0.0;
    config->radius_deg = 0.75;
    config->show_sweep = true;
    config->show_labels = true;
}

esp_err_t radar_config_load(radar_config_t *config)
{
    radar_config_defaults(config);
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(CONFIG_NAMESPACE, NVS_READONLY, &nvs);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    get_string(nvs, "ssid", config->wifi_ssid, sizeof(config->wifi_ssid));
    get_string(nvs, "password", config->wifi_password, sizeof(config->wifi_password));
    get_string(nvs, "client_id", config->opensky_client_id, sizeof(config->opensky_client_id));
    get_string(nvs, "client_secret", config->opensky_client_secret, sizeof(config->opensky_client_secret));
    (void)nvs_get_blob(nvs, "latitude", &config->latitude, &(size_t){sizeof(config->latitude)});
    (void)nvs_get_blob(nvs, "longitude", &config->longitude, &(size_t){sizeof(config->longitude)});
    (void)nvs_get_blob(nvs, "radius", &config->radius_deg,
                       &(size_t){sizeof(config->radius_deg)});
    uint8_t flag;
    if (nvs_get_u8(nvs, "sweep", &flag) == ESP_OK) config->show_sweep = flag != 0;
    if (nvs_get_u8(nvs, "labels", &flag) == ESP_OK) config->show_labels = flag != 0;
    nvs_close(nvs);

    if (config->radius_deg < 0.05 || config->radius_deg > 2.5) {
        config->radius_deg = 0.75;
    }
    return ESP_OK;
}

esp_err_t radar_config_save(const radar_config_t *config)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(CONFIG_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

#define SAVE(call) do { err = (call); if (err != ESP_OK) goto done; } while (0)
    SAVE(nvs_set_str(nvs, "ssid", config->wifi_ssid));
    SAVE(nvs_set_str(nvs, "password", config->wifi_password));
    SAVE(nvs_set_blob(nvs, "latitude", &config->latitude, sizeof(config->latitude)));
    SAVE(nvs_set_blob(nvs, "longitude", &config->longitude, sizeof(config->longitude)));
    SAVE(nvs_set_blob(nvs, "radius", &config->radius_deg, sizeof(config->radius_deg)));
    SAVE(nvs_set_str(nvs, "client_id", config->opensky_client_id));
    SAVE(nvs_set_str(nvs, "client_secret", config->opensky_client_secret));
    SAVE(nvs_set_u8(nvs, "sweep", config->show_sweep));
    SAVE(nvs_set_u8(nvs, "labels", config->show_labels));
    err = nvs_commit(nvs);
done:
    nvs_close(nvs);
    return err;
#undef SAVE
}

bool radar_config_has_wifi(const radar_config_t *config)
{
    return config->wifi_ssid[0] != '\0';
}
