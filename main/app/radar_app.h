#pragma once

#include "esp_err.h"

/** Initializes the board and starts all flight-radar services. */
esp_err_t radar_app_start(void);
