#pragma once

#include <stdbool.h>

/** Initializes the Waveshare SH8601 display, touch controller, and LVGL port. */
void waveshare_display_port_init(void);

/**
 * Records physical user activity and restores normal brightness.
 * Returns true when the display was dimmed, allowing the wake-up input to be
 * consumed instead of also activating the UI.
 */
bool waveshare_display_note_activity(void);

/** Serializes access to LVGL, which is not thread-safe. */
bool waveshare_display_lock(int timeout_ms);
void waveshare_display_unlock(void);
