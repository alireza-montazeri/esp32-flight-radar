#include "esp_err.h"
#include "radar_app.h"

void app_main(void)
{
    ESP_ERROR_CHECK(radar_app_start());
}
