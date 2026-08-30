#pragma once

#include <stdbool.h>
#include "radar_types.h"

void radar_display_init(void);
void radar_display_update_aircraft(const radar_aircraft_list_t *aircraft);
void radar_display_set_status(const char *status);
void radar_display_set_center(double latitude, double longitude, double radius_deg);
void radar_display_set_options(bool show_sweep, bool show_labels);
bool radar_display_labels_enabled(void);
