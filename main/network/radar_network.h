#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "radar_config.h"
#include "radar_types.h"

esp_err_t radar_network_start(const radar_config_t *config);
bool radar_network_is_connected(void);
bool radar_network_setup_ap_active(void);
bool radar_network_is_authenticated(void);
esp_err_t radar_network_fetch_city_weather(radar_city_t city,
                                           radar_weather_t *weather,
                                           int *http_status);
esp_err_t radar_network_fetch_aircraft(const radar_config_t *config,
                                       radar_aircraft_list_t *aircraft,
                                       int *http_status);
esp_err_t radar_network_fetch_aircraft_details(const radar_aircraft_t *aircraft,
                                               radar_aircraft_details_t *details,
                                               int *http_status);
