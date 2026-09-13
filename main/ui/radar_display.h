#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "radar_types.h"

void radar_display_init(void);
void radar_display_update_aircraft(const radar_aircraft_list_t *aircraft);
void radar_display_set_status(const char *status);
void radar_display_set_wifi_connected(bool connected);
void radar_display_set_weather(radar_city_t city,
                               const radar_weather_t *weather);
void radar_display_set_battery(int percentage);
void radar_display_set_center(double latitude, double longitude, double radius_deg);
void radar_display_set_options(bool show_sweep, bool show_labels,
                               bool show_airports, bool show_coastlines);

/** Cycles the detail card selection. Returns true when detail mode consumed the input. */
bool radar_display_rotate_selection(int direction);

/** Copies the selected live aircraft for asynchronous metadata lookup. */
bool radar_display_get_selected_aircraft(radar_aircraft_t *aircraft);

/** Blocks until the selected aircraft or network state may have changed. */
bool radar_display_wait_for_selection_change(uint32_t timeout_ms);
void radar_display_set_details_loading(const char *icao24);
void radar_display_set_aircraft_details(const char *icao24,
                                        const radar_aircraft_details_t *details,
                                        bool available);
