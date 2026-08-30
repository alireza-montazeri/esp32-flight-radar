#include "radar_display.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "user_config.h"
#include "waveshare_display_port.h"

#define RADAR_SIZE       360
#define RADAR_CENTER     180
#define RADAR_RADIUS_PX  166
#define DEG_TO_RAD       0.01745329251994329577

typedef struct {
    radar_aircraft_list_t aircraft;
    char status[64];
    double latitude;
    double longitude;
    double radius_deg;
    bool show_sweep;
    bool show_labels;
} display_state_t;

static const char *DISPLAY_TAG = "radar_display";
static SemaphoreHandle_t state_mutex;
static display_state_t state;
static lv_obj_t *canvas;
static lv_color_t *canvas_buffer;

static void canvas_clicked(lv_event_t *event)
{
    (void)event;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.show_labels = !state.show_labels;
    xSemaphoreGive(state_mutex);
}

static void draw_line(int x1, int y1, int x2, int y2, lv_color_t color, uint8_t width, lv_opa_t opa)
{
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = color;
    dsc.width = width;
    dsc.opa = opa;
    lv_point_t points[] = {{x1, y1}, {x2, y2}};
    lv_canvas_draw_line(canvas, points, 2, &dsc);
}

static void draw_text(int x, int y, int width, const char *text, lv_color_t color, lv_text_align_t align)
{
    lv_draw_label_dsc_t dsc;
    lv_draw_label_dsc_init(&dsc);
    dsc.color = color;
    dsc.font = &lv_font_montserrat_12;
    dsc.align = align;
    lv_canvas_draw_text(canvas, x, y, width, &dsc, text);
}

static void draw_aircraft(const radar_aircraft_t *aircraft, const display_state_t *snapshot)
{
    double elapsed_s = (esp_timer_get_time() - aircraft->received_us) / 1000000.0;
    if (elapsed_s < 0) elapsed_s = 0;
    if (elapsed_s > 300) elapsed_s = 300;

    const double heading = aircraft->track_deg * DEG_TO_RAD;
    const double metres_per_degree = 111320.0;
    double predicted_lat = aircraft->latitude;
    double predicted_lon = aircraft->longitude;
    if (!aircraft->on_ground) {
        predicted_lat += (aircraft->velocity_mps * elapsed_s * cos(heading)) / metres_per_degree;
        const double longitude_scale = fmax(0.1, cos(aircraft->latitude * DEG_TO_RAD));
        predicted_lon += (aircraft->velocity_mps * elapsed_s * sin(heading)) /
                         (metres_per_degree * longitude_scale);
    }

    const double x_norm = (predicted_lon - snapshot->longitude) *
                          cos(snapshot->latitude * DEG_TO_RAD) / snapshot->radius_deg;
    const double y_norm = (predicted_lat - snapshot->latitude) / snapshot->radius_deg;
    const int x = RADAR_CENTER + (int)lround(x_norm * RADAR_RADIUS_PX);
    const int y = RADAR_CENTER - (int)lround(y_norm * RADAR_RADIUS_PX);
    if (x < 7 || x >= RADAR_SIZE - 7 || y < 7 || y >= RADAR_SIZE - 7) return;

    const int nose_x = x + (int)lround(sin(heading) * 7.0);
    const int nose_y = y - (int)lround(cos(heading) * 7.0);
    const int left_x = x + (int)lround(sin(heading + 2.45) * 5.0);
    const int left_y = y - (int)lround(cos(heading + 2.45) * 5.0);
    const int right_x = x + (int)lround(sin(heading - 2.45) * 5.0);
    const int right_y = y - (int)lround(cos(heading - 2.45) * 5.0);
    const lv_color_t bright = aircraft->on_ground ? lv_palette_main(LV_PALETTE_AMBER) : lv_color_hex(0x00ff55);
    draw_line(nose_x, nose_y, left_x, left_y, bright, 2, LV_OPA_COVER);
    draw_line(left_x, left_y, right_x, right_y, bright, 2, LV_OPA_COVER);
    draw_line(right_x, right_y, nose_x, nose_y, bright, 2, LV_OPA_COVER);

    if (snapshot->show_labels) {
        char label[28];
        const char *name = aircraft->callsign[0] ? aircraft->callsign : aircraft->icao24;
        snprintf(label, sizeof(label), "%s %.0fft", name, aircraft->altitude_m * 3.28084f);
        draw_text(x + 8, y - 7, 112, label, lv_color_hex(0x7dff9d), LV_TEXT_ALIGN_LEFT);
    }
}

static void render_frame(const display_state_t *snapshot)
{
    const lv_color_t green = lv_color_hex(0x00d040);
    const lv_color_t dim_green = lv_color_hex(0x006822);
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);

    lv_draw_arc_dsc_t arc;
    lv_draw_arc_dsc_init(&arc);
    arc.color = dim_green;
    arc.width = 1;
    arc.opa = LV_OPA_COVER;
    lv_canvas_draw_arc(canvas, RADAR_CENTER, RADAR_CENTER, 55, 0, 360, &arc);
    lv_canvas_draw_arc(canvas, RADAR_CENTER, RADAR_CENTER, 110, 0, 360, &arc);
    lv_canvas_draw_arc(canvas, RADAR_CENTER, RADAR_CENTER, RADAR_RADIUS_PX, 0, 360, &arc);
    draw_line(14, RADAR_CENTER, 346, RADAR_CENTER, dim_green, 1, LV_OPA_60);
    draw_line(RADAR_CENTER, 14, RADAR_CENTER, 346, dim_green, 1, LV_OPA_60);

    if (snapshot->show_sweep) {
        const double angle = fmod(esp_timer_get_time() / 3000000.0, 2.0 * M_PI);
        const int sx = RADAR_CENTER + (int)lround(sin(angle) * RADAR_RADIUS_PX);
        const int sy = RADAR_CENTER - (int)lround(cos(angle) * RADAR_RADIUS_PX);
        draw_line(RADAR_CENTER, RADAR_CENTER, sx, sy, green, 2, LV_OPA_70);
    }

    for (size_t i = 0; i < snapshot->aircraft.count; ++i) {
        draw_aircraft(&snapshot->aircraft.items[i], snapshot);
    }

    draw_line(RADAR_CENTER - 2, RADAR_CENTER, RADAR_CENTER + 2, RADAR_CENTER, green, 2, LV_OPA_COVER);
    draw_line(RADAR_CENTER, RADAR_CENTER - 2, RADAR_CENTER, RADAR_CENTER + 2, green, 2, LV_OPA_COVER);
    char footer[48];
    snprintf(footer, sizeof(footer), "%u aircraft   range %.2f deg",
             (unsigned)snapshot->aircraft.count, snapshot->radius_deg);
    draw_text(50, 334, 260, footer, green, LV_TEXT_ALIGN_CENTER);
    draw_text(45, 12, 270, snapshot->status, green, LV_TEXT_ALIGN_CENTER);
}

static void render_task(void *arg)
{
    (void)arg;
    display_state_t *snapshot = heap_caps_malloc(sizeof(*snapshot),
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!snapshot) snapshot = malloc(sizeof(*snapshot));
    assert(snapshot);

    while (true) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        *snapshot = state;
        xSemaphoreGive(state_mutex);
        if (waveshare_display_lock(1000)) {
            render_frame(snapshot);
            waveshare_display_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void radar_display_init(void)
{
    state_mutex = xSemaphoreCreateMutex();
    assert(state_mutex);
    state.latitude = 0.0;
    state.longitude = 0.0;
    state.radius_deg = 0.75;
    state.show_sweep = true;
    state.show_labels = true;
    strlcpy(state.status, "Flight Radar starting", sizeof(state.status));

    waveshare_display_port_init();
    canvas_buffer = heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(RADAR_SIZE, RADAR_SIZE),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!canvas_buffer) {
        canvas_buffer = heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(RADAR_SIZE, RADAR_SIZE),
                                         MALLOC_CAP_8BIT);
    }
    assert(canvas_buffer);

    if (waveshare_display_lock(-1)) {
        lv_obj_clean(lv_scr_act());
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
        canvas = lv_canvas_create(lv_scr_act());
        lv_canvas_set_buffer(canvas, canvas_buffer, RADAR_SIZE, RADAR_SIZE, LV_IMG_CF_TRUE_COLOR);
        lv_obj_center(canvas);
        lv_obj_add_flag(canvas, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(canvas, canvas_clicked, LV_EVENT_CLICKED, NULL);
        waveshare_display_unlock();
    }
    ESP_LOGI(DISPLAY_TAG, "360x360 radar display ready");
    xTaskCreate(render_task, "radar_render", 6144, NULL, 3, NULL);
}

void radar_display_update_aircraft(const radar_aircraft_list_t *aircraft)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.aircraft = *aircraft;
    xSemaphoreGive(state_mutex);
}

void radar_display_set_status(const char *status)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    strlcpy(state.status, status, sizeof(state.status));
    xSemaphoreGive(state_mutex);
}

void radar_display_set_center(double latitude, double longitude, double radius_deg)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.latitude = latitude;
    state.longitude = longitude;
    state.radius_deg = fmin(2.5, fmax(0.05, radius_deg));
    xSemaphoreGive(state_mutex);
}

void radar_display_set_options(bool show_sweep, bool show_labels)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.show_sweep = show_sweep;
    state.show_labels = show_labels;
    xSemaphoreGive(state_mutex);
}

bool radar_display_labels_enabled(void)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool enabled = state.show_labels;
    xSemaphoreGive(state_mutex);
    return enabled;
}
