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
#include "airport_data.h"
#include "user_config.h"
#include "waveshare_display_port.h"

#define RADAR_SIZE 360
#define RADAR_CENTER 180
#define RADAR_RADIUS_PX 166
#define DEG_TO_RAD 0.01745329251994329577
#define AIRCRAFT_HIT_RADIUS_PX 28
#define AIRCRAFT_ICON_SIZE 18
#define AIRCRAFT_ICON_DIRECTIONS 16
#define SWEEP_BEAM_RAYS 10
#define SWEEP_BEAM_WIDTH_DEG 10.0
#define SWEEP_ROTATION_PERIOD_US 15000000.0
#define RADAR_FRAME_PERIOD_MS 50
#define KILOMETRES_PER_DEGREE 111.32
#define RADAR_AIRPORT_LABEL_WIDTH 30

typedef enum
{
    DETAILS_NOT_REQUESTED,
    DETAILS_LOADING,
    DETAILS_READY,
    DETAILS_UNAVAILABLE,
} details_state_t;

typedef struct
{
    radar_aircraft_list_t aircraft;
    char status[64];
    double latitude;
    double longitude;
    double radius_deg;
    bool show_sweep;
    bool show_labels;
    bool wifi_connected;
    bool detail_active;
    char selected_icao24[7];
    details_state_t details_state;
    radar_aircraft_details_t details;
} display_state_t;

static const char *DISPLAY_TAG = "radar_display";
static SemaphoreHandle_t state_mutex;
static display_state_t state;
static radar_aircraft_list_t pending_aircraft;
static double previous_sweep_angle;
static bool sweep_angle_initialized;
static lv_obj_t *canvas;
static lv_color_t *canvas_buffer;
static lv_img_dsc_t aircraft_icon_images[AIRCRAFT_ICON_DIRECTIONS];
static void *aircraft_icon_buffers[AIRCRAFT_ICON_DIRECTIONS];
static bool aircraft_icons_ready;

static void clear_details_locked(void)
{
    state.details_state = DETAILS_NOT_REQUESTED;
    memset(&state.details, 0, sizeof(state.details));
}

static void predicted_position(const radar_aircraft_t *aircraft, double *latitude,
                               double *longitude)
{
    double elapsed_s = (esp_timer_get_time() - aircraft->received_us) / 1000000.0;
    if (elapsed_s < 0)
        elapsed_s = 0;
    if (elapsed_s > 300)
        elapsed_s = 300;

    *latitude = aircraft->latitude;
    *longitude = aircraft->longitude;
    if (aircraft->on_ground)
        return;

    const double heading = aircraft->track_deg * DEG_TO_RAD;
    const double metres_per_degree = 111320.0;
    *latitude += (aircraft->velocity_mps * elapsed_s * cos(heading)) / metres_per_degree;
    const double longitude_scale = fmax(0.1, cos(aircraft->latitude * DEG_TO_RAD));
    *longitude += (aircraft->velocity_mps * elapsed_s * sin(heading)) /
                  (metres_per_degree * longitude_scale);
}

static bool aircraft_screen_position(const radar_aircraft_t *aircraft,
                                     const display_state_t *snapshot, int *x, int *y)
{
    const double x_norm = (aircraft->longitude - snapshot->longitude) *
                          cos(snapshot->latitude * DEG_TO_RAD) / snapshot->radius_deg;
    const double y_norm = (aircraft->latitude - snapshot->latitude) /
                          snapshot->radius_deg;
    *x = RADAR_CENTER + (int)lround(x_norm * RADAR_RADIUS_PX);
    *y = RADAR_CENTER - (int)lround(y_norm * RADAR_RADIUS_PX);

    const int dx = *x - RADAR_CENTER;
    const int dy = *y - RADAR_CENTER;
    const int visible_radius = RADAR_RADIUS_PX - 8;
    return dx * dx + dy * dy <= visible_radius * visible_radius;
}

static bool airport_screen_position(const radar_airport_t *airport,
                                    const display_state_t *snapshot,
                                    double longitude_scale, int *x, int *y,
                                    double *distance_squared)
{
    const double latitude_delta = airport->latitude - snapshot->latitude;
    double longitude_delta = airport->longitude - snapshot->longitude;
    if (longitude_delta > 180.0)
        longitude_delta -= 360.0;
    else if (longitude_delta < -180.0)
        longitude_delta += 360.0;

    if (fabs(latitude_delta) > snapshot->radius_deg ||
        fabs(longitude_delta) * longitude_scale > snapshot->radius_deg)
    {
        return false;
    }

    const double x_norm = longitude_delta * longitude_scale /
                          snapshot->radius_deg;
    const double y_norm = latitude_delta / snapshot->radius_deg;
    *distance_squared = x_norm * x_norm + y_norm * y_norm;
    if (*distance_squared > 1.0)
        return false;

    *x = RADAR_CENTER + (int)lround(x_norm * RADAR_RADIUS_PX);
    *y = RADAR_CENTER - (int)lround(y_norm * RADAR_RADIUS_PX);
    return true;
}

static double aircraft_bearing(double latitude, double longitude,
                               const display_state_t *snapshot)
{
    const double north = latitude - snapshot->latitude;
    const double east = (longitude - snapshot->longitude) *
                        cos(snapshot->latitude * DEG_TO_RAD);
    double bearing = atan2(east, north);
    if (bearing < 0)
        bearing += 2.0 * M_PI;
    return bearing;
}

static bool sweep_crossed_angle(double previous, double current, double target)
{
    if (current >= previous)
        return target > previous && target <= current;
    return target > previous || target <= current;
}

static bool pending_contains_aircraft(const char *icao24)
{
    for (size_t i = 0; i < pending_aircraft.count; ++i)
    {
        if (strcmp(pending_aircraft.items[i].icao24, icao24) == 0)
            return true;
    }
    return false;
}

static void ensure_selected_aircraft_locked(void)
{
    if (!state.detail_active)
        return;

    bool selected_is_visible = false;
    int first_visible = -1;
    for (size_t i = 0; i < state.aircraft.count; ++i)
    {
        int x;
        int y;
        if (!aircraft_screen_position(&state.aircraft.items[i], &state, &x, &y))
            continue;
        if (first_visible < 0)
            first_visible = (int)i;
        if (strcmp(state.selected_icao24, state.aircraft.items[i].icao24) == 0)
        {
            selected_is_visible = true;
        }
    }

    if (!selected_is_visible && first_visible >= 0)
    {
        strlcpy(state.selected_icao24, state.aircraft.items[first_visible].icao24,
                sizeof(state.selected_icao24));
        clear_details_locked();
    }
    else if (!selected_is_visible)
    {
        state.detail_active = false;
        state.selected_icao24[0] = '\0';
        clear_details_locked();
    }
}

static void update_aircraft_at_sweep_locked(double sweep_angle)
{
    if (!sweep_angle_initialized)
    {
        previous_sweep_angle = sweep_angle;
        sweep_angle_initialized = true;
        return;
    }

    for (size_t i = 0; i < pending_aircraft.count; ++i)
    {
        double latitude;
        double longitude;
        predicted_position(&pending_aircraft.items[i], &latitude, &longitude);
        const double bearing = aircraft_bearing(latitude, longitude, &state);
        if (!sweep_crossed_angle(previous_sweep_angle, sweep_angle, bearing))
            continue;

        radar_aircraft_t scanned = pending_aircraft.items[i];
        scanned.latitude = latitude;
        scanned.longitude = longitude;
        scanned.received_us = esp_timer_get_time();

        size_t displayed = 0;
        while (displayed < state.aircraft.count &&
               strcmp(state.aircraft.items[displayed].icao24, scanned.icao24) != 0)
        {
            ++displayed;
        }
        if (displayed < state.aircraft.count)
        {
            state.aircraft.items[displayed] = scanned;
        }
        else if (state.aircraft.count < RADAR_MAX_AIRCRAFT)
        {
            state.aircraft.items[state.aircraft.count++] = scanned;
        }
    }

    for (size_t i = 0; i < state.aircraft.count;)
    {
        radar_aircraft_t *displayed = &state.aircraft.items[i];
        const double bearing = aircraft_bearing(displayed->latitude,
                                                displayed->longitude, &state);
        if (!pending_contains_aircraft(displayed->icao24) &&
            sweep_crossed_angle(previous_sweep_angle, sweep_angle, bearing))
        {
            memmove(displayed, displayed + 1,
                    (state.aircraft.count - i - 1) * sizeof(*displayed));
            --state.aircraft.count;
            continue;
        }
        ++i;
    }

    previous_sweep_angle = sweep_angle;
    ensure_selected_aircraft_locked();
}

static int find_nearest_aircraft(const display_state_t *snapshot, int tap_x, int tap_y)
{
    int nearest = -1;
    int nearest_distance_sq = AIRCRAFT_HIT_RADIUS_PX * AIRCRAFT_HIT_RADIUS_PX + 1;
    for (size_t i = 0; i < snapshot->aircraft.count; ++i)
    {
        int x;
        int y;
        if (!aircraft_screen_position(&snapshot->aircraft.items[i], snapshot, &x, &y))
            continue;
        const int dx = tap_x - x;
        const int dy = tap_y - y;
        const int distance_sq = dx * dx + dy * dy;
        if (distance_sq < nearest_distance_sq)
        {
            nearest = (int)i;
            nearest_distance_sq = distance_sq;
        }
    }
    return nearest;
}

static void canvas_clicked(lv_event_t *event)
{
    lv_indev_t *indev = lv_event_get_indev(event);
    if (!indev)
        return;
    lv_point_t point;
    lv_area_t canvas_area;
    lv_indev_get_point(indev, &point);
    lv_obj_get_coords(canvas, &canvas_area);

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.detail_active)
    {
        state.detail_active = false;
        state.selected_icao24[0] = '\0';
        clear_details_locked();
    }
    else
    {
        const int selected = find_nearest_aircraft(
            &state, point.x - canvas_area.x1, point.y - canvas_area.y1);
        if (selected >= 0)
        {
            state.detail_active = true;
            strlcpy(state.selected_icao24, state.aircraft.items[selected].icao24,
                    sizeof(state.selected_icao24));
            clear_details_locked();
        }
    }
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

static void draw_text_font(int x, int y, int width, const char *text, lv_color_t color,
                           lv_text_align_t align, const lv_font_t *font)
{
    lv_draw_label_dsc_t dsc;
    lv_draw_label_dsc_init(&dsc);
    dsc.color = color;
    dsc.font = font;
    dsc.align = align;
    lv_canvas_draw_text(canvas, x, y, width, &dsc, text);
}

static void draw_text(int x, int y, int width, const char *text, lv_color_t color,
                      lv_text_align_t align)
{
    draw_text_font(x, y, width, text, color, align, &lv_font_montserrat_12);
}

static void draw_text_single_line(int x, int y, int width, const char *text,
                                  lv_color_t color, lv_text_align_t align)
{
    lv_draw_label_dsc_t dsc;
    lv_draw_label_dsc_init(&dsc);
    dsc.color = color;
    dsc.font = &lv_font_montserrat_12;
    dsc.align = align;
    dsc.flag |= LV_TEXT_FLAG_EXPAND;
    lv_canvas_draw_text(canvas, x, y, width, &dsc, text);
}

static void draw_text_single_line_font(int x, int y, int width,
                                       const char *text, lv_color_t color,
                                       lv_text_align_t align,
                                       const lv_font_t *font)
{
    lv_draw_label_dsc_t dsc;
    lv_draw_label_dsc_init(&dsc);
    dsc.color = color;
    dsc.font = font;
    dsc.align = align;
    dsc.flag |= LV_TEXT_FLAG_EXPAND;
    lv_canvas_draw_text(canvas, x, y, width, &dsc, text);
}

static bool text_fits_detail_row(const char *text, int width)
{
    return lv_txt_get_width(text, strlen(text), &lv_font_montserrat_12, 0,
                            LV_TEXT_FLAG_EXPAND) <= width;
}

static void init_rotated_aircraft_icons(void)
{
    const size_t buffer_size = LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(
        AIRCRAFT_ICON_SIZE, AIRCRAFT_ICON_SIZE);
    void *source_buffer = heap_caps_calloc(1, buffer_size,
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!source_buffer)
        source_buffer = heap_caps_calloc(1, buffer_size, MALLOC_CAP_8BIT);
    if (!source_buffer)
        return;

    lv_obj_t *source_canvas = lv_canvas_create(lv_scr_act());
    lv_obj_t *rotation_canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(source_canvas, source_buffer, AIRCRAFT_ICON_SIZE,
                         AIRCRAFT_ICON_SIZE, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_canvas_fill_bg(source_canvas, lv_color_white(), LV_OPA_TRANSP);

    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_white();
    label_dsc.font = &lv_font_montserrat_12;
    label_dsc.align = LV_TEXT_ALIGN_CENTER;
    label_dsc.flag |= LV_TEXT_FLAG_EXPAND;
    lv_canvas_draw_text(source_canvas, 2, 2, AIRCRAFT_ICON_SIZE - 4,
                        &label_dsc, LV_SYMBOL_GPS);

    const lv_img_dsc_t source_image = *lv_canvas_get_img(source_canvas);
    for (size_t i = 0; i < AIRCRAFT_ICON_DIRECTIONS; ++i)
    {
        aircraft_icon_buffers[i] = heap_caps_calloc(
            1, buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!aircraft_icon_buffers[i])
        {
            aircraft_icon_buffers[i] = heap_caps_calloc(1, buffer_size, MALLOC_CAP_8BIT);
        }
        if (!aircraft_icon_buffers[i])
            break;

        lv_canvas_set_buffer(rotation_canvas, aircraft_icon_buffers[i],
                             AIRCRAFT_ICON_SIZE, AIRCRAFT_ICON_SIZE,
                             LV_IMG_CF_TRUE_COLOR_ALPHA);
        lv_canvas_fill_bg(rotation_canvas, lv_color_white(), LV_OPA_TRANSP);

        /* Font Awesome's location arrow points north-east. Rotate direction 0 north. */
        const int16_t angle = (3150 + (int16_t)i *
                                          (3600 / AIRCRAFT_ICON_DIRECTIONS)) %
                              3600;
        lv_canvas_transform(rotation_canvas, (lv_img_dsc_t *)&source_image, angle,
                            LV_IMG_ZOOM_NONE, 0, 0,
                            AIRCRAFT_ICON_SIZE / 2, AIRCRAFT_ICON_SIZE / 2, true);
        aircraft_icon_images[i] = *lv_canvas_get_img(rotation_canvas);
    }

    aircraft_icons_ready = true;
    for (size_t i = 0; i < AIRCRAFT_ICON_DIRECTIONS; ++i)
    {
        if (!aircraft_icon_buffers[i])
        {
            aircraft_icons_ready = false;
            break;
        }
    }
    lv_obj_del(rotation_canvas);
    lv_obj_del(source_canvas);
    heap_caps_free(source_buffer);

    if (!aircraft_icons_ready)
    {
        for (size_t i = 0; i < AIRCRAFT_ICON_DIRECTIONS; ++i)
        {
            heap_caps_free(aircraft_icon_buffers[i]);
            aircraft_icon_buffers[i] = NULL;
        }
        ESP_LOGW(DISPLAY_TAG, "Rotated GPS icons unavailable; using upright symbols");
    }
}

static void draw_aircraft_symbol(int center_x, int center_y, double heading_deg,
                                 lv_color_t color)
{
    if (!aircraft_icons_ready)
    {
        draw_text_single_line(center_x - 7, center_y - 7, 14, LV_SYMBOL_GPS,
                              color, LV_TEXT_ALIGN_CENTER);
        return;
    }

    const double normalized = fmod(heading_deg + 360.0, 360.0);
    const size_t direction = (size_t)lround(
                                 normalized * AIRCRAFT_ICON_DIRECTIONS / 360.0) %
                             AIRCRAFT_ICON_DIRECTIONS;
    lv_draw_img_dsc_t image_dsc;
    lv_draw_img_dsc_init(&image_dsc);
    image_dsc.recolor = color;
    image_dsc.recolor_opa = LV_OPA_COVER;
    lv_canvas_draw_img(canvas, center_x - AIRCRAFT_ICON_SIZE / 2,
                       center_y - AIRCRAFT_ICON_SIZE / 2,
                       &aircraft_icon_images[direction], &image_dsc);
}

static void draw_aircraft(const radar_aircraft_t *aircraft, const display_state_t *snapshot,
                          bool selected)
{
    int x;
    int y;
    if (!aircraft_screen_position(aircraft, snapshot, &x, &y))
        return;

    const lv_color_t bright = aircraft->on_ground ? lv_palette_main(LV_PALETTE_AMBER) : lv_color_hex(0x00ff55);
    draw_aircraft_symbol(x, y, aircraft->track_deg, bright);

    if (selected)
    {
        lv_draw_arc_dsc_t selection;
        lv_draw_arc_dsc_init(&selection);
        selection.color = lv_color_white();
        selection.width = 2;
        selection.opa = LV_OPA_COVER;
        lv_canvas_draw_arc(canvas, x, y, 12, 0, 360, &selection);
    }

    if (snapshot->show_labels)
    {
        char label[28];
        const char *name = aircraft->callsign[0] ? aircraft->callsign : aircraft->icao24;
        snprintf(label, sizeof(label), "%s", name);
        draw_text(x + 8, y - 7, 112, label, lv_color_hex(0x7dff9d), LV_TEXT_ALIGN_LEFT);
    }
}

typedef struct
{
    uint16_t airport_index;
    int16_t x;
    int16_t y;
} visible_airport_t;

static void draw_airports(const display_state_t *snapshot)
{
    static visible_airport_t *visible;
    static size_t visible_count;
    static double cached_latitude;
    static double cached_longitude;
    static double cached_radius_deg;
    static bool cache_valid;
    static bool allocation_attempted;

    if (!allocation_attempted)
    {
        allocation_attempted = true;
        visible = heap_caps_malloc(radar_airport_count * sizeof(*visible),
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!visible)
            visible = malloc(radar_airport_count * sizeof(*visible));
        if (!visible)
            ESP_LOGW(DISPLAY_TAG, "Airport display cache allocation failed");
    }
    if (!visible)
        return;

    if (!cache_valid || cached_latitude != snapshot->latitude ||
        cached_longitude != snapshot->longitude ||
        cached_radius_deg != snapshot->radius_deg)
    {
        visible_count = 0;
        const double longitude_scale =
            fmax(0.1, fabs(cos(snapshot->latitude * DEG_TO_RAD)));
        for (size_t i = 0; i < radar_airport_count; ++i)
        {
            int x;
            int y;
            double distance_squared;
            if (airport_screen_position(&radar_airports[i], snapshot,
                                        longitude_scale, &x, &y,
                                        &distance_squared))
            {
                visible[visible_count++] = (visible_airport_t){
                    .airport_index = (uint16_t)i,
                    .x = (int16_t)x,
                    .y = (int16_t)y,
                };
            }
        }
        cached_latitude = snapshot->latitude;
        cached_longitude = snapshot->longitude;
        cached_radius_deg = snapshot->radius_deg;
        cache_valid = true;
    }

    const lv_color_t marker_color = lv_color_white();
    for (size_t i = visible_count; i > 0; --i)
    {
        const visible_airport_t *airport = &visible[i - 1];
        draw_text_single_line_font(airport->x - 6, airport->y - 6, 12,
                                   LV_SYMBOL_PLUS, marker_color,
                                   LV_TEXT_ALIGN_CENTER,
                                   &lv_font_montserrat_10);
    }

    for (size_t i = 0; i < visible_count; ++i)
    {
        const visible_airport_t *airport = &visible[i];
        const int label_width = RADAR_AIRPORT_LABEL_WIDTH;
        const int label_x = airport->x < RADAR_CENTER
                                ? airport->x + 6
                                : airport->x - label_width - 6;
        int label_y = airport->y - 6;
        if (label_y < 24)
            label_y = 24;
        else if (label_y > 326)
            label_y = 326;
        draw_text_single_line_font(
            label_x, label_y, label_width,
            radar_airports[airport->airport_index].code, lv_color_white(),
            LV_TEXT_ALIGN_CENTER, &lv_font_montserrat_10);
    }
}

static void draw_sweep_beam(double leading_angle, lv_color_t color)
{
    const double beam_width = SWEEP_BEAM_WIDTH_DEG * DEG_TO_RAD;
    for (int ray = 0; ray < SWEEP_BEAM_RAYS; ++ray)
    {
        const double progress = (ray + 1.0) / SWEEP_BEAM_RAYS;
        const double angle = leading_angle - beam_width * (1.0 - progress);
        const int end_x = RADAR_CENTER +
                          (int)lround(sin(angle) * (RADAR_RADIUS_PX - 2));
        const int end_y = RADAR_CENTER -
                          (int)lround(cos(angle) * (RADAR_RADIUS_PX - 2));
        const lv_opa_t opacity = (lv_opa_t)lround(5.0 + progress * progress * 95.0);
        draw_line(RADAR_CENTER, RADAR_CENTER, end_x, end_y,
                  color, 5, opacity);
    }

    const int leading_x = RADAR_CENTER +
                          (int)lround(sin(leading_angle) * (RADAR_RADIUS_PX - 2));
    const int leading_y = RADAR_CENTER -
                          (int)lround(cos(leading_angle) * (RADAR_RADIUS_PX - 2));
    draw_line(RADAR_CENTER, RADAR_CENTER, leading_x, leading_y,
              color, 2, LV_OPA_70);
}

static int selected_aircraft_index(const display_state_t *snapshot,
                                   size_t *visible_position,
                                   size_t *visible_count)
{
    int selected = -1;
    *visible_position = 0;
    *visible_count = 0;

    for (size_t i = 0; i < snapshot->aircraft.count; ++i)
    {
        int x;
        int y;
        if (!aircraft_screen_position(&snapshot->aircraft.items[i], snapshot, &x, &y))
        {
            continue;
        }

        ++(*visible_count);
        if (strcmp(snapshot->selected_icao24, snapshot->aircraft.items[i].icao24) == 0)
        {
            selected = (int)i;
            *visible_position = *visible_count;
        }
    }
    return selected;
}

static const char *airport_code(const char *iata, const char *icao)
{
    if (iata[0])
        return iata;
    if (icao[0])
        return icao;
    return "---";
}

static const char *heading_direction(double heading_deg)
{
    static const char *directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    return directions[((int)lround(heading_deg / 45.0)) & 7];
}

static void draw_detail_card(const display_state_t *snapshot)
{
    size_t visible_position;
    size_t visible_count;
    const int selected = selected_aircraft_index(snapshot, &visible_position, &visible_count);
    if (selected < 0)
        return;

    const radar_aircraft_t *aircraft = &snapshot->aircraft.items[selected];
    const lv_color_t green = lv_color_hex(0x39ff75);
    const lv_color_t dim_green = lv_color_hex(0x87d99d);
    const bool details_ready = snapshot->details_state == DETAILS_READY;
    const bool loading = snapshot->details_state == DETAILS_LOADING ||
                         snapshot->details_state == DETAILS_NOT_REQUESTED;
    const radar_aircraft_details_t *details = &snapshot->details;
    const char *company = details->airline[0] ? details->airline : details->owner;
    const bool show_company = details_ready && company[0];
    const bool show_aircraft_type = details_ready &&
                                    (details->manufacturer[0] || details->type[0] ||
                                     details->icao_type[0]);
    const bool show_route = details_ready && details->route_found;
    const bool no_additional_details = !loading && !show_company &&
                                       !show_aircraft_type && !show_route;
    const int row_count = 3 + (show_company ? 1 : 0) +
                          (show_aircraft_type ? 1 : 0) + (show_route ? 1 : 0) +
                          (loading ? 1 : 0) + (no_additional_details ? 1 : 0);
    const int card_width = 220;
    const int card_height = 48 + row_count * 20;
    const int card_x = (RADAR_SIZE - card_width) / 2;
    const int card_y = (RADAR_SIZE - card_height) / 2;
    const int text_x = card_x + 10;
    const int text_width = card_width - 20;

    lv_draw_rect_dsc_t card;
    lv_draw_rect_dsc_init(&card);
    card.bg_color = lv_color_hex(0x021408);
    card.bg_opa = (lv_opa_t)217; /* 85% opaque: solid enough to read, still translucent. */
    card.border_color = green;
    card.border_width = 2;
    card.radius = 10;
    lv_canvas_draw_rect(canvas, card_x, card_y, card_width, card_height, &card);

    char line[128];
    const char *name = aircraft->callsign[0] ? aircraft->callsign : "UNKNOWN";
    draw_text_font(text_x, card_y + 9, text_width - 58, name, lv_color_white(),
                   LV_TEXT_ALIGN_LEFT,
                   &lv_font_montserrat_16);
    snprintf(line, sizeof(line), LV_SYMBOL_LIST " %u/%u",
             (unsigned)visible_position, (unsigned)visible_count);
    draw_text(text_x + text_width - 58, card_y + 11, 58, line, green,
              LV_TEXT_ALIGN_RIGHT);
    draw_line(text_x, card_y + 33, text_x + text_width, card_y + 33,
              green, 1, LV_OPA_60);

    int row_y = card_y + 41;
    if (loading)
    {
        draw_text(text_x, row_y, text_width, LV_SYMBOL_REFRESH " Details...",
                  dim_green, LV_TEXT_ALIGN_LEFT);
        row_y += 20;
    }
    if (show_company)
    {
        snprintf(line, sizeof(line), LV_SYMBOL_HOME " %.24s", company);
        draw_text(text_x, row_y, text_width, line, dim_green, LV_TEXT_ALIGN_LEFT);
        row_y += 20;
    }
    if (show_aircraft_type)
    {
        const char *aircraft_type = details->type[0] ? details->type : details->icao_type;
        if (details->manufacturer[0] && aircraft_type[0])
        {
            snprintf(line, sizeof(line), LV_SYMBOL_FILE " %.16s %.18s",
                     details->manufacturer, aircraft_type);
        }
        else
        {
            snprintf(line, sizeof(line), LV_SYMBOL_FILE " %.28s",
                     details->manufacturer[0] ? details->manufacturer : aircraft_type);
        }
        draw_text(text_x, row_y, text_width, line, dim_green, LV_TEXT_ALIGN_LEFT);
        row_y += 20;
    }
    if (show_route)
    {
        const char *origin_location = details->origin_city[0]
                                          ? details->origin_city
                                          : details->origin_name;
        const char *destination_location = details->destination_city[0]
                                               ? details->destination_city
                                               : details->destination_name;
        const char *origin_code = airport_code(details->origin_iata,
                                               details->origin_icao);
        const char *destination_code = airport_code(details->destination_iata,
                                                    details->destination_icao);
        snprintf(line, sizeof(line), "%s %s " LV_SYMBOL_RIGHT " %s %s",
                 origin_code, origin_location, destination_code, destination_location);
        if (!text_fits_detail_row(line, text_width))
        {
            snprintf(line, sizeof(line), "%s %.8s " LV_SYMBOL_RIGHT " %s %.8s",
                     origin_code, origin_location, destination_code, destination_location);
        }
        if (!text_fits_detail_row(line, text_width))
        {
            snprintf(line, sizeof(line), "%s " LV_SYMBOL_RIGHT " %s",
                     origin_code, destination_code);
        }
        draw_text_single_line(text_x, row_y, text_width, line, green,
                              LV_TEXT_ALIGN_LEFT);
        row_y += 20;
    }
    if (no_additional_details)
    {
        draw_text(text_x, row_y, text_width,
                  LV_SYMBOL_WARNING " No additional detail found",
                  dim_green, LV_TEXT_ALIGN_LEFT);
        row_y += 20;
    }

    snprintf(line, sizeof(line), LV_SYMBOL_UPLOAD " %.0f m    %+.1f m/s",
             aircraft->altitude_m, aircraft->vertical_rate_mps);
    draw_text(text_x, row_y, text_width, line, green, LV_TEXT_ALIGN_LEFT);
    row_y += 20;

    snprintf(line, sizeof(line), LV_SYMBOL_CHARGE " %.1f m/s",
             aircraft->velocity_mps);
    draw_text(text_x, row_y, text_width, line, green, LV_TEXT_ALIGN_LEFT);
    row_y += 20;

    double heading = fmod(aircraft->track_deg + 360.0, 360.0);
    snprintf(line, sizeof(line), LV_SYMBOL_GPS " %s %.0f deg",
             heading_direction(heading), heading);
    draw_text(text_x, row_y, text_width, line, green, LV_TEXT_ALIGN_LEFT);
}

static void render_frame(const display_state_t *snapshot, double sweep_angle)
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

    draw_airports(snapshot);

    for (size_t i = 0; i < snapshot->aircraft.count; ++i)
    {
        const bool selected = snapshot->detail_active &&
                              strcmp(snapshot->selected_icao24,
                                     snapshot->aircraft.items[i].icao24) == 0;
        draw_aircraft(&snapshot->aircraft.items[i], snapshot, selected);
    }

    if (snapshot->show_sweep)
    {
        draw_sweep_beam(sweep_angle, green);
    }

    draw_line(RADAR_CENTER - 2, RADAR_CENTER, RADAR_CENTER + 2, RADAR_CENTER, green, 2, LV_OPA_COVER);
    draw_line(RADAR_CENTER, RADAR_CENTER - 2, RADAR_CENTER, RADAR_CENTER + 2, green, 2, LV_OPA_COVER);
    char top_status[96];
    const bool live = strcmp(snapshot->status, "Live") == 0;
    const bool fetching = strcmp(snapshot->status, "Fetching") == 0;
    const bool aircraft_fetch_error =
        strncmp(snapshot->status, "OpenSky error", strlen("OpenSky error")) == 0;
    const bool fully_live = live && snapshot->wifi_connected;
    if (fully_live)
    {
        snprintf(top_status, sizeof(top_status), LV_SYMBOL_WIFI "  " LV_SYMBOL_GPS " %u",
                 (unsigned)snapshot->aircraft.count);
        draw_text_single_line(110, 12, 140, top_status, green, LV_TEXT_ALIGN_CENTER);
    }
    else if (aircraft_fetch_error && snapshot->wifi_connected)
    {
        snprintf(top_status, sizeof(top_status),
                 LV_SYMBOL_WIFI "  " LV_SYMBOL_WARNING "  " LV_SYMBOL_GPS " %u",
                 (unsigned)snapshot->aircraft.count);
        draw_text_single_line(95, 12, 170, top_status,
                              lv_palette_main(LV_PALETTE_AMBER), LV_TEXT_ALIGN_CENTER);
    }
    else if (fetching)
    {
        strlcpy(top_status, snapshot->wifi_connected ? LV_SYMBOL_WIFI "  " LV_SYMBOL_REFRESH " Fetching..." : LV_SYMBOL_REFRESH " Fetching...",
                sizeof(top_status));
        draw_text_single_line(80, 12, 200, top_status, green, LV_TEXT_ALIGN_CENTER);
    }
    else
    {
        if (snapshot->wifi_connected)
        {
            snprintf(top_status, sizeof(top_status), LV_SYMBOL_WIFI "  %s",
                     snapshot->status);
        }
        else
        {
            strlcpy(top_status, snapshot->status, sizeof(top_status));
        }
        draw_text(90, 18, 180, top_status, green, LV_TEXT_ALIGN_CENTER);
    }

    char bottom_scale[32];
    snprintf(bottom_scale, sizeof(bottom_scale), LV_SYMBOL_BARS " %.0f km",
             snapshot->radius_deg * KILOMETRES_PER_DEGREE);
    draw_text(125, 338, 110, bottom_scale, green, LV_TEXT_ALIGN_CENTER);

    if (snapshot->detail_active)
        draw_detail_card(snapshot);
}

static void render_task(void *arg)
{
    (void)arg;
    display_state_t *snapshot = heap_caps_malloc(sizeof(*snapshot),
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!snapshot)
        snapshot = malloc(sizeof(*snapshot));
    assert(snapshot);

    while (true)
    {
        const double sweep_angle = fmod(
            (double)esp_timer_get_time() * 2.0 * M_PI / SWEEP_ROTATION_PERIOD_US,
            2.0 * M_PI);
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (state.show_sweep)
            update_aircraft_at_sweep_locked(sweep_angle);
        *snapshot = state;
        xSemaphoreGive(state_mutex);
        if (waveshare_display_lock(1000))
        {
            render_frame(snapshot, sweep_angle);
            waveshare_display_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(RADAR_FRAME_PERIOD_MS));
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
    state.wifi_connected = false;
    state.detail_active = false;
    state.selected_icao24[0] = '\0';
    memset(&pending_aircraft, 0, sizeof(pending_aircraft));
    previous_sweep_angle = 0.0;
    sweep_angle_initialized = false;
    clear_details_locked();
    strlcpy(state.status, "Flight Radar starting", sizeof(state.status));

    waveshare_display_port_init();
    canvas_buffer = heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(RADAR_SIZE, RADAR_SIZE),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!canvas_buffer)
    {
        canvas_buffer = heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(RADAR_SIZE, RADAR_SIZE),
                                         MALLOC_CAP_8BIT);
    }
    assert(canvas_buffer);

    if (waveshare_display_lock(-1))
    {
        lv_obj_clean(lv_scr_act());
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
        canvas = lv_canvas_create(lv_scr_act());
        lv_canvas_set_buffer(canvas, canvas_buffer, RADAR_SIZE, RADAR_SIZE, LV_IMG_CF_TRUE_COLOR);
        lv_obj_center(canvas);
        init_rotated_aircraft_icons();
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
    pending_aircraft = *aircraft;
    if (!state.show_sweep)
    {
        state.aircraft = pending_aircraft;
        ensure_selected_aircraft_locked();
    }
    xSemaphoreGive(state_mutex);
}

void radar_display_set_status(const char *status)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    strlcpy(state.status, status, sizeof(state.status));
    xSemaphoreGive(state_mutex);
}

void radar_display_set_wifi_connected(bool connected)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.wifi_connected = connected;
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
    if (state.show_sweep != show_sweep)
    {
        sweep_angle_initialized = false;
        if (!show_sweep)
        {
            state.aircraft = pending_aircraft;
            ensure_selected_aircraft_locked();
        }
    }
    state.show_sweep = show_sweep;
    state.show_labels = show_labels;
    xSemaphoreGive(state_mutex);
}

bool radar_display_rotate_selection(int direction)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    const bool detail_active = state.detail_active;
    if (detail_active)
    {
        size_t visible[RADAR_MAX_AIRCRAFT];
        size_t visible_count = 0;
        size_t selected_position = 0;
        bool found_selected = false;

        for (size_t i = 0; i < state.aircraft.count; ++i)
        {
            int x;
            int y;
            if (!aircraft_screen_position(&state.aircraft.items[i], &state, &x, &y))
                continue;
            visible[visible_count] = i;
            if (strcmp(state.selected_icao24, state.aircraft.items[i].icao24) == 0)
            {
                selected_position = visible_count;
                found_selected = true;
            }
            ++visible_count;
        }

        if (visible_count == 0)
        {
            state.detail_active = false;
            state.selected_icao24[0] = '\0';
            clear_details_locked();
        }
        else
        {
            if (!found_selected)
                selected_position = 0;
            if (direction > 0)
            {
                selected_position = (selected_position + 1) % visible_count;
            }
            else if (direction < 0)
            {
                selected_position = (selected_position + visible_count - 1) % visible_count;
            }
            const char *next_icao24 = state.aircraft.items[visible[selected_position]].icao24;
            if (strcmp(state.selected_icao24, next_icao24) != 0)
            {
                strlcpy(state.selected_icao24, next_icao24, sizeof(state.selected_icao24));
                clear_details_locked();
            }
        }
    }
    xSemaphoreGive(state_mutex);
    return detail_active;
}

bool radar_display_get_selected_aircraft(radar_aircraft_t *aircraft)
{
    if (!aircraft)
        return false;
    bool found = false;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.detail_active)
    {
        for (size_t i = 0; i < state.aircraft.count; ++i)
        {
            if (strcmp(state.selected_icao24, state.aircraft.items[i].icao24) == 0)
            {
                *aircraft = state.aircraft.items[i];
                found = true;
                break;
            }
        }
    }
    xSemaphoreGive(state_mutex);
    return found;
}

void radar_display_set_details_loading(const char *icao24)
{
    if (!icao24)
        return;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.detail_active && strcmp(state.selected_icao24, icao24) == 0)
    {
        state.details_state = DETAILS_LOADING;
        memset(&state.details, 0, sizeof(state.details));
    }
    xSemaphoreGive(state_mutex);
}

void radar_display_set_aircraft_details(const char *icao24,
                                        const radar_aircraft_details_t *details,
                                        bool available)
{
    if (!icao24 || !details)
        return;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.detail_active && strcmp(state.selected_icao24, icao24) == 0)
    {
        state.details = *details;
        state.details_state = available ? DETAILS_READY : DETAILS_UNAVAILABLE;
    }
    xSemaphoreGive(state_mutex);
}
