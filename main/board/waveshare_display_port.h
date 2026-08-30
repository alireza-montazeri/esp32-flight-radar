#pragma once

#include <stdbool.h>

/** Initializes the Waveshare SH8601 display, touch controller, and LVGL port. */
void waveshare_display_port_init(void);

/** Serializes access to LVGL, which is not thread-safe. */
bool waveshare_display_lock(int timeout_ms);
void waveshare_display_unlock(void);
