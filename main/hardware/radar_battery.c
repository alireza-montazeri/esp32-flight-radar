#include "radar_battery.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0
#define BATTERY_SAMPLE_COUNT 8
#define BATTERY_DIVIDER_RATIO 2.0f

typedef struct {
    float voltage;
    int percentage;
} battery_curve_point_t;

static const battery_curve_point_t battery_curve[] = {
    {3.30f, 0},  {3.45f, 5},  {3.55f, 10}, {3.62f, 20},
    {3.70f, 35}, {3.76f, 50}, {3.83f, 65}, {3.92f, 80},
    {4.02f, 90}, {4.12f, 97}, {4.20f, 100},
};

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t calibration_handle;
static bool calibration_available;

static int percentage_from_voltage(float voltage)
{
    if (voltage <= battery_curve[0].voltage) return 0;
    const size_t point_count = sizeof(battery_curve) / sizeof(battery_curve[0]);
    if (voltage >= battery_curve[point_count - 1].voltage) return 100;

    for (size_t i = 1; i < point_count; ++i) {
        if (voltage > battery_curve[i].voltage) continue;
        const battery_curve_point_t *low = &battery_curve[i - 1];
        const battery_curve_point_t *high = &battery_curve[i];
        const float position = (voltage - low->voltage) /
                               (high->voltage - low->voltage);
        return (int)lroundf(low->percentage +
                            position * (high->percentage - low->percentage));
    }
    return 100;
}

esp_err_t radar_battery_init(void)
{
    const adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_config, &adc_handle);
    if (err != ESP_OK) return err;

    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_oneshot_config_channel(adc_handle, BATTERY_ADC_CHANNEL,
                                     &channel_config);
    if (err != ESP_OK) return err;

    const adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = ADC_UNIT_1,
        .chan = BATTERY_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    calibration_available =
        adc_cali_create_scheme_curve_fitting(&calibration_config,
                                             &calibration_handle) == ESP_OK;
    return ESP_OK;
}

esp_err_t radar_battery_read_percentage(int *percentage)
{
    if (!adc_handle || !percentage) return ESP_ERR_INVALID_STATE;

    int64_t millivolt_sum = 0;
    for (int i = 0; i < BATTERY_SAMPLE_COUNT; ++i) {
        int raw = 0;
        esp_err_t err = adc_oneshot_read(adc_handle, BATTERY_ADC_CHANNEL, &raw);
        if (err != ESP_OK) return err;

        int millivolts = 0;
        if (calibration_available) {
            err = adc_cali_raw_to_voltage(calibration_handle, raw, &millivolts);
            if (err != ESP_OK) return err;
        } else {
            millivolts = (int)lroundf(raw * 3300.0f / 4095.0f);
        }
        millivolt_sum += millivolts;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    const float voltage = (millivolt_sum / (float)BATTERY_SAMPLE_COUNT) *
                          BATTERY_DIVIDER_RATIO / 1000.0f;
    *percentage = percentage_from_voltage(voltage);
    return ESP_OK;
}
