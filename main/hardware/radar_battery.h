#pragma once

#include "esp_err.h"

esp_err_t radar_battery_init(void);
esp_err_t radar_battery_read_percentage(int *percentage);
