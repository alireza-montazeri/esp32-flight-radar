#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    char wifi_ssid[33];
    char wifi_password[65];
    double latitude;
    double longitude;
    double radius_deg;
    char opensky_client_id[96];
    char opensky_client_secret[128];
    bool show_sweep;
    bool show_labels;
} radar_config_t;

void radar_config_defaults(radar_config_t *config);
esp_err_t radar_config_load(radar_config_t *config);
esp_err_t radar_config_save(const radar_config_t *config);
bool radar_config_has_wifi(const radar_config_t *config);
