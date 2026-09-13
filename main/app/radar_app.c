#include "radar_app.h"

#include <stdlib.h>
#include <time.h>

#include "nvs_flash.h"
#include "radar_config.h"
#include "radar_controller.h"
#include "radar_display.h"
#include "radar_haptics.h"
#include "radar_network.h"

static esp_err_t initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err != ESP_OK) return err;
        err = nvs_flash_init();
    }
    return err;
}

static void apply_display_config(const radar_config_t *config)
{
    radar_display_set_center(config->latitude, config->longitude, config->radius_deg);
    radar_display_set_options(config->show_sweep, config->update_on_sweep,
                              config->show_labels,
                              config->show_airports, config->show_coastlines);
}

esp_err_t radar_app_start(void)
{
    esp_err_t err = initialize_nvs();
    if (err != ESP_OK) return err;

    radar_config_t config;
    err = radar_config_load(&config);
    if (err != ESP_OK) return err;

    setenv("TZ", "AEST-10AEDT,M10.1.0,M4.1.0/3", 1);
    tzset();

    radar_display_init();
    ESP_ERROR_CHECK_WITHOUT_ABORT(radar_haptics_init());
    apply_display_config(&config);

    err = radar_network_start(&config);
    if (err != ESP_OK) return err;

    return radar_controller_start(&config);
}
