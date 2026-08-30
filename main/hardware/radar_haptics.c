#include "radar_haptics.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bsp.h"

#define DRV2605_REG_STATUS       0x00
#define DRV2605_REG_MODE         0x01
#define DRV2605_REG_LIBRARY      0x03
#define DRV2605_REG_WAVESEQ1     0x04
#define DRV2605_REG_WAVESEQ2     0x05
#define DRV2605_REG_GO           0x0C

#define DRV2605_MODE_INTTRIG     0x00
#define DRV2605_LIBRARY_ERM_E    0x05
#define DRV2605_EFFECT_DETENT    0x05 /* Sharp Click, 60%. */

static const char *TAG = "radar_haptics";
static TaskHandle_t haptic_task_handle;
static bool haptics_ready;

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    return i2c_write_buff(drv2605_dev_handle, reg, &value, 1) == ESP_OK
               ? ESP_OK
               : ESP_FAIL;
}

static void haptic_task(void *arg)
{
    (void)arg;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (write_register(DRV2605_REG_GO, 1) != ESP_OK) {
            ESP_LOGW(TAG, "Haptic trigger failed");
        }
    }
}

esp_err_t radar_haptics_init(void)
{
    if (!drv2605_dev_handle) return ESP_ERR_INVALID_STATE;

    uint8_t status = 0;
    if (i2c_read_buff(drv2605_dev_handle, DRV2605_REG_STATUS, &status, 1) != ESP_OK) {
        ESP_LOGW(TAG, "DRV2605 not detected; knob haptics disabled");
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = write_register(DRV2605_REG_MODE, DRV2605_MODE_INTTRIG);
    if (err == ESP_OK) err = write_register(DRV2605_REG_LIBRARY, DRV2605_LIBRARY_ERM_E);
    if (err == ESP_OK) err = write_register(DRV2605_REG_WAVESEQ1, DRV2605_EFFECT_DETENT);
    if (err == ESP_OK) err = write_register(DRV2605_REG_WAVESEQ2, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DRV2605 configuration failed; knob haptics disabled");
        return err;
    }

    BaseType_t created = xTaskCreate(haptic_task, "radar_haptic", 2048, NULL, 3,
                                     &haptic_task_handle);
    if (created != pdPASS) return ESP_ERR_NO_MEM;

    haptics_ready = true;
    ESP_LOGI(TAG, "DRV2605 ready (effect 5: sharp click 60%%)");
    return ESP_OK;
}

void radar_haptics_play_detent(void)
{
    if (haptics_ready && haptic_task_handle) xTaskNotifyGive(haptic_task_handle);
}
