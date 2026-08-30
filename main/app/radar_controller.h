#pragma once

#include "esp_err.h"
#include "radar_config.h"

/** Starts knob handling and periodic OpenSky updates. */
esp_err_t radar_controller_start(const radar_config_t *initial_config);
