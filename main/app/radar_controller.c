#include "radar_controller.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "radar_display.h"
#include "radar_haptics.h"
#include "radar_network.h"
#include "user_config.h"
#include "user_encoder_bsp.h"

#define KNOB_IDLE_SAVE_MS             1500
#define RADAR_RADIUS_STEP_DEG         0.05
#define RADAR_RADIUS_MIN_DEG          0.05
#define RADAR_RADIUS_MAX_DEG          2.5
#define AUTHENTICATED_POLL_SECONDS    22
#define ANONYMOUS_POLL_SECONDS        220
#define KNOB_TASK_STACK_BYTES         3072
#define RADAR_TASK_STACK_BYTES        (24 * 1024)

static const char *TAG = "radar_controller";
static radar_config_t app_config;
static SemaphoreHandle_t config_mutex;
static TaskHandle_t knob_task_handle;
static TaskHandle_t radar_task_handle;

static void read_config(radar_config_t *snapshot)
{
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    *snapshot = app_config;
    xSemaphoreGive(config_mutex);
}

static void update_radius(EventBits_t encoder_bits)
{
    radar_config_t snapshot;
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    if (encoder_bits & BIT0) app_config.radius_deg -= RADAR_RADIUS_STEP_DEG;
    if (encoder_bits & BIT1) app_config.radius_deg += RADAR_RADIUS_STEP_DEG;
    if (app_config.radius_deg < RADAR_RADIUS_MIN_DEG) {
        app_config.radius_deg = RADAR_RADIUS_MIN_DEG;
    }
    if (app_config.radius_deg > RADAR_RADIUS_MAX_DEG) {
        app_config.radius_deg = RADAR_RADIUS_MAX_DEG;
    }
    snapshot = app_config;
    xSemaphoreGive(config_mutex);

    radar_display_set_center(snapshot.latitude, snapshot.longitude, snapshot.radius_deg);
}

static void save_config(void)
{
    radar_config_t snapshot;
    read_config(&snapshot);
    ESP_ERROR_CHECK_WITHOUT_ABORT(radar_config_save(&snapshot));
}

static void knob_task(void *arg)
{
    (void)arg;
    bool config_changed = false;

    while (true) {
        EventBits_t bits = xEventGroupWaitBits(knob_even_, BIT_EVEN_ALL, pdTRUE, pdFALSE,
                                               pdMS_TO_TICKS(KNOB_IDLE_SAVE_MS));
        if (bits & (BIT0 | BIT1)) {
            radar_haptics_play_detent();
            update_radius(bits);
            config_changed = true;
        } else if (config_changed) {
            save_config();
            config_changed = false;
        }
    }
}

static radar_aircraft_list_t *allocate_aircraft_list(void)
{
    radar_aircraft_list_t *aircraft = heap_caps_calloc(
        1, sizeof(*aircraft), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!aircraft) aircraft = calloc(1, sizeof(*aircraft));
    return aircraft;
}

static void show_connection_status(void)
{
    if (radar_network_setup_ap_active()) {
        radar_display_set_status("Join FlightRadar-Setup - 192.168.4.1");
    } else {
        radar_display_set_status("Waiting for Wi-Fi...");
    }
}

static int current_poll_interval_seconds(void)
{
    return radar_network_is_authenticated()
               ? AUTHENTICATED_POLL_SECONDS
               : ANONYMOUS_POLL_SECONDS;
}

static void show_fetch_result(esp_err_t err, int http_status,
                              const radar_aircraft_list_t *aircraft)
{
    char status[64];
    if (err == ESP_OK) {
        radar_display_update_aircraft(aircraft);
        snprintf(status, sizeof(status), "LIVE - next update in %ds",
                 current_poll_interval_seconds());
        radar_display_set_status(status);
        ESP_LOGI(TAG, "OpenSky update: %u aircraft", (unsigned)aircraft->count);
        return;
    }

    snprintf(status, sizeof(status), "OpenSky error HTTP %d (%s)",
             http_status, esp_err_to_name(err));
    radar_display_set_status(status);
    ESP_LOGW(TAG, "%s", status);
}

static void fetch_aircraft(radar_aircraft_list_t *aircraft)
{
    radar_config_t snapshot;
    read_config(&snapshot);
    radar_display_set_status("Fetching OpenSky aircraft...");

    int http_status = 0;
    esp_err_t err = radar_network_fetch_aircraft(&snapshot, aircraft, &http_status);
    show_fetch_result(err, http_status, aircraft);
}

static void wait_for_next_fetch(void)
{
    const int wait_seconds = current_poll_interval_seconds();
    for (int i = 0; i < wait_seconds && radar_network_is_connected(); ++i) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void radar_task(void *arg)
{
    (void)arg;
    radar_aircraft_list_t *aircraft = allocate_aircraft_list();
    assert(aircraft);

    while (true) {
        if (!radar_network_is_connected()) {
            show_connection_status();
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        fetch_aircraft(aircraft);
        ESP_LOGI(TAG, "Radar task minimum free stack: %u bytes",
                 (unsigned)uxTaskGetStackHighWaterMark(NULL));
        wait_for_next_fetch();
    }
}

esp_err_t radar_controller_start(const radar_config_t *initial_config)
{
    if (!initial_config) return ESP_ERR_INVALID_ARG;

    config_mutex = xSemaphoreCreateMutex();
    if (!config_mutex) return ESP_ERR_NO_MEM;
    app_config = *initial_config;

    user_encoder_init();
    BaseType_t created = xTaskCreate(knob_task, "radar_knob", KNOB_TASK_STACK_BYTES,
                                     NULL, 3, &knob_task_handle);
    if (created != pdPASS) return ESP_ERR_NO_MEM;

    created = xTaskCreate(radar_task, "opensky_radar", RADAR_TASK_STACK_BYTES,
                          NULL, 4, &radar_task_handle);
    if (created != pdPASS) {
        vTaskDelete(knob_task_handle);
        knob_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
