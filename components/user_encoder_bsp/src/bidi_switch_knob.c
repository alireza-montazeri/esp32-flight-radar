/*
 * SPDX-FileCopyrightText: 2016-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *
 *
 * Modified by planevina 2025-01-20
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bidi_switch_knob.h"

static const char *TAG = "Knob";

#define TICKS_INTERVAL 3
#define DEBOUNCE_TICKS 2
#define IDLE_STABLE_TICKS 4
#define KNOB_TASK_STACK_BYTES 2048

#define KNOB_CHECK(a, str, ret_val)                               \
    if (!(a))                                                     \
    {                                                             \
        ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val);                                         \
    }

#define KNOB_CHECK_GOTO(a, str, label)                                         \
    if (!(a))                                                                  \
    {                                                                          \
        ESP_LOGE(TAG, "%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
        goto label;                                                            \
    }

#define CALL_EVENT_CB(ev) \
    if (knob->cb[ev])     \
    knob->cb[ev](knob, knob->usr_data[ev])

typedef struct Knob
{
    bool encoder_a_change;                          /*<! true means Encoder A phase Inverted*/
    bool encoder_b_change;                          /*<! true means Encoder B phase Inverted*/
    uint8_t debounce_a_cnt;                         /*!< Encoder A phase debounce count */
    uint8_t debounce_b_cnt;                         /*!< Encoder B phase debounce count */
    uint8_t encoder_a_level;                        /*!< Encoder A phase current level */
    uint8_t encoder_b_level;                        /*!< Encoder B phase current Level */
    knob_event_t event;                             /*!< Current event */
    int count_value;                                /*!< Knob count */
    uint8_t (*hal_knob_level)(void *hardware_data); /*!< Get current level */
    void *encoder_a;                                /*!< Encoder A phase gpio number */
    void *encoder_b;                                /*!< Encoder B phase gpio number */
    void *usr_data[KNOB_EVENT_MAX];                 /*!< User data for event */
    knob_cb_t cb[KNOB_EVENT_MAX];                   /*!< Event callback */
    struct Knob *next;                              /*!< Next pointer */
} knob_dev_t;

static knob_dev_t *s_head_handle = NULL;
static TaskHandle_t s_knob_task_handle;
static bool s_is_timer_running = false;

// 判定函数
static void process_knob_channel(uint8_t current_level, uint8_t *prev_level,
                                 uint8_t *debounce_cnt, int *count_value,
                                 knob_event_t event, bool is_increment, knob_dev_t *knob)
{
    if (current_level == 0)
    {
        if (current_level != *prev_level)
            *debounce_cnt = 0;
        else
            (*debounce_cnt)++;
    }
    else
    {
        if (current_level != *prev_level && ++(*debounce_cnt) >= DEBOUNCE_TICKS)
        {
            *debounce_cnt = 0;
            *count_value += is_increment ? 1 : -1;
            knob->event = event;
            CALL_EVENT_CB(event);
        }
        else
            *debounce_cnt = 0;
    }
    *prev_level = current_level;
}

static void knob_handler(knob_dev_t *knob)
{
    uint8_t pha_value = knob->hal_knob_level(knob->encoder_a);
    uint8_t phb_value = knob->hal_knob_level(knob->encoder_b);

    process_knob_channel(pha_value, &knob->encoder_a_level,
                         &knob->debounce_a_cnt, &knob->count_value,
                         KNOB_RIGHT, true, knob);

    process_knob_channel(phb_value, &knob->encoder_b_level,
                         &knob->debounce_b_cnt, &knob->count_value,
                         KNOB_LEFT, false, knob);
}

// 这是timer的回调函数，定期执行
static void knob_monitor_task(void *args)
{
    (void)args;
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        unsigned stable_ticks = 0;
        while (stable_ticks < IDLE_STABLE_TICKS && s_is_timer_running)
        {
            bool changed = false;
            for (knob_dev_t *target = s_head_handle; target; target = target->next)
            {
                const uint8_t previous_a = target->encoder_a_level;
                const uint8_t previous_b = target->encoder_b_level;
                knob_handler(target);
                changed |= previous_a != target->encoder_a_level ||
                           previous_b != target->encoder_b_level;
            }
            if (ulTaskNotifyTake(pdTRUE, 0) > 0)
                changed = true;
            stable_ticks = changed ? 0 : stable_ticks + 1;
            vTaskDelay(pdMS_TO_TICKS(TICKS_INTERVAL));
        }
    }
}

static void IRAM_ATTR knob_gpio_interrupt(void *arg)
{
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_knob_task_handle && s_is_timer_running)
        vTaskNotifyGiveFromISR(s_knob_task_handle, &higher_priority_task_woken);
    if (higher_priority_task_woken)
        portYIELD_FROM_ISR();
}

knob_handle_t iot_knob_create(const knob_config_t *config)
{
    KNOB_CHECK(NULL != config, "config pointer can't be NULL!", NULL)
    KNOB_CHECK(config->gpio_encoder_a != config->gpio_encoder_b, "encoder A can't be the same as encoder B", NULL);

    knob_dev_t *knob = (knob_dev_t *)calloc(1, sizeof(knob_dev_t));
    KNOB_CHECK(NULL != knob, "alloc knob failed", NULL);

    esp_err_t ret = ESP_OK;
    ret = knob_gpio_init(config->gpio_encoder_a);
    KNOB_CHECK(ESP_OK == ret, "encoder A gpio init failed", NULL);
    ret = knob_gpio_init(config->gpio_encoder_b);
    KNOB_CHECK_GOTO(ESP_OK == ret, "encoder B gpio init failed", _encoder_deinit);

    knob->hal_knob_level = knob_gpio_get_key_level;
    knob->encoder_a = (void *)(long)config->gpio_encoder_a;
    knob->encoder_b = (void *)(long)config->gpio_encoder_b;

    knob->encoder_a_level = knob->hal_knob_level(knob->encoder_a);
    knob->encoder_b_level = knob->hal_knob_level(knob->encoder_b);

    knob->event = KNOB_NONE;

    knob->next = s_head_handle;
    s_head_handle = knob;

    if (!s_knob_task_handle)
    {
        BaseType_t created = xTaskCreate(knob_monitor_task, "knob_monitor",
                                         KNOB_TASK_STACK_BYTES, NULL, 3,
                                         &s_knob_task_handle);
        KNOB_CHECK_GOTO(created == pdPASS, "knob task create failed", _encoder_deinit);
    }

    esp_err_t isr_err = gpio_install_isr_service(0);
    KNOB_CHECK_GOTO(isr_err == ESP_OK || isr_err == ESP_ERR_INVALID_STATE,
                    "GPIO ISR service install failed", _encoder_deinit);
    KNOB_CHECK_GOTO(gpio_isr_handler_add(config->gpio_encoder_a,
                                         knob_gpio_interrupt, knob) == ESP_OK,
                    "encoder A interrupt init failed", _encoder_deinit);
    KNOB_CHECK_GOTO(gpio_isr_handler_add(config->gpio_encoder_b,
                                         knob_gpio_interrupt, knob) == ESP_OK,
                    "encoder B interrupt init failed", _remove_a_interrupt);

    if (!s_is_timer_running)
    {
        s_is_timer_running = true;
    }

    ESP_LOGI(TAG, "Iot Knob Config Succeed, encoder A:%d, encoder B:%d", config->gpio_encoder_a, config->gpio_encoder_b);
    return (knob_handle_t)knob;

_remove_a_interrupt:
    gpio_isr_handler_remove(config->gpio_encoder_a);
_encoder_deinit:
    for (knob_dev_t **entry = &s_head_handle; *entry; entry = &(*entry)->next)
    {
        if (*entry == knob)
        {
            *entry = knob->next;
            break;
        }
    }
    if (!s_head_handle && s_knob_task_handle)
    {
        vTaskDelete(s_knob_task_handle);
        s_knob_task_handle = NULL;
    }
    knob_gpio_deinit(config->gpio_encoder_b);
    knob_gpio_deinit(config->gpio_encoder_a);
    free(knob);
    return NULL;
}

esp_err_t iot_knob_delete(knob_handle_t knob_handle)
{
    esp_err_t ret = ESP_OK;
    KNOB_CHECK(NULL != knob_handle, "Pointer of handle is invalid", ESP_ERR_INVALID_ARG);
    knob_dev_t *knob = (knob_dev_t *)knob_handle;
    gpio_isr_handler_remove((int)(long)knob->encoder_a);
    gpio_isr_handler_remove((int)(long)knob->encoder_b);
    ret = knob_gpio_deinit((int)(long)knob->encoder_a);
    KNOB_CHECK(ESP_OK == ret, "encoder A deinit failed", ESP_FAIL);
    ret = knob_gpio_deinit((int)(long)knob->encoder_b);
    KNOB_CHECK(ESP_OK == ret, "encoder B deinit failed", ESP_FAIL);
    knob_dev_t **curr;
    for (curr = &s_head_handle; *curr;)
    {
        knob_dev_t *entry = *curr;
        if (entry == knob)
        {
            *curr = entry->next;
            free(entry);
        }
        else
        {
            curr = &entry->next;
        }
    }

    uint16_t number = 0;
    knob_dev_t *target = s_head_handle;
    while (target)
    {
        target = target->next;
        number++;
    }
    ESP_LOGD(TAG, "remain knob number=%d", number);

    if (0 == number)
    {
        s_is_timer_running = false;
        if (s_knob_task_handle)
        {
            vTaskDelete(s_knob_task_handle);
            s_knob_task_handle = NULL;
        }
    }

    return ESP_OK;
}

esp_err_t iot_knob_register_cb(knob_handle_t knob_handle, knob_event_t event, knob_cb_t cb, void *usr_data)
{
    KNOB_CHECK(NULL != knob_handle, "Pointer of handle is invalid", ESP_ERR_INVALID_ARG);
    KNOB_CHECK(event < KNOB_EVENT_MAX, "event is invalid", ESP_ERR_INVALID_ARG);
    knob_dev_t *knob = (knob_dev_t *)knob_handle;
    knob->cb[event] = cb;
    knob->usr_data[event] = usr_data;
    return ESP_OK;
}

esp_err_t iot_knob_unregister_cb(knob_handle_t knob_handle, knob_event_t event)
{
    KNOB_CHECK(NULL != knob_handle, "Pointer of handle is invalid", ESP_ERR_INVALID_ARG);
    KNOB_CHECK(event < KNOB_EVENT_MAX, "event is invalid", ESP_ERR_INVALID_ARG);
    knob_dev_t *knob = (knob_dev_t *)knob_handle;
    knob->cb[event] = NULL;
    knob->usr_data[event] = NULL;
    return ESP_OK;
}

knob_event_t iot_knob_get_event(knob_handle_t knob_handle)
{
    KNOB_CHECK(NULL != knob_handle, "Pointer of handle is invalid", ESP_ERR_INVALID_ARG);
    knob_dev_t *knob = (knob_dev_t *)knob_handle;
    return knob->event;
}

int iot_knob_get_count_value(knob_handle_t knob_handle)
{
    KNOB_CHECK(NULL != knob_handle, "Pointer of handle is invalid", ESP_ERR_INVALID_ARG);
    knob_dev_t *knob = (knob_dev_t *)knob_handle;
    return knob->count_value;
}

esp_err_t iot_knob_clear_count_value(knob_handle_t knob_handle)
{
    KNOB_CHECK(NULL != knob_handle, "Pointer of handle is invalid", ESP_ERR_INVALID_ARG);
    knob_dev_t *knob = (knob_dev_t *)knob_handle;
    knob->count_value = 0;
    return ESP_OK;
}

esp_err_t iot_knob_resume(void)
{
    KNOB_CHECK(s_knob_task_handle, "knob task handle is invalid", ESP_ERR_INVALID_STATE);
    KNOB_CHECK(!s_is_timer_running, "knob monitor is already running", ESP_ERR_INVALID_STATE);

    for (knob_dev_t *target = s_head_handle; target; target = target->next)
    {
        gpio_intr_enable((int)(long)target->encoder_a);
        gpio_intr_enable((int)(long)target->encoder_b);
    }
    s_is_timer_running = true;
    return ESP_OK;
}

esp_err_t iot_knob_stop(void)
{
    KNOB_CHECK(s_knob_task_handle, "knob task handle is invalid", ESP_ERR_INVALID_STATE);
    KNOB_CHECK(s_is_timer_running, "knob monitor is not running", ESP_ERR_INVALID_STATE);

    s_is_timer_running = false;
    for (knob_dev_t *target = s_head_handle; target; target = target->next)
    {
        gpio_intr_disable((int)(long)target->encoder_a);
        gpio_intr_disable((int)(long)target->encoder_b);
    }
    xTaskNotifyGive(s_knob_task_handle);
    return ESP_OK;
}

esp_err_t knob_gpio_init(uint32_t gpio_num)
{
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = 1,
    };
    return gpio_config(&gpio_cfg);
}

esp_err_t knob_gpio_deinit(uint32_t gpio_num)
{
    return gpio_reset_pin(gpio_num);
}

uint8_t knob_gpio_get_key_level(void *gpio_num)
{
    return (uint8_t)gpio_get_level((uint32_t)gpio_num);
}
