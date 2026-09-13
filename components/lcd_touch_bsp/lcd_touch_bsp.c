#include <stdbool.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "lcd_touch_bsp.h"
#include "i2c_bsp.h"
#include "user_config.h"

static volatile uint32_t touch_data_pending = 1;
static bool touch_active;

static void IRAM_ATTR touch_interrupt(void *arg)
{
  (void)arg;
  __atomic_store_n(&touch_data_pending, 1, __ATOMIC_RELEASE);
}

void lcd_touch_init(void)
{
  const gpio_config_t interrupt_config = {
    .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_TOUCH_INT,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_NEGEDGE,
  };
  ESP_ERROR_CHECK(gpio_config(&interrupt_config));
  esp_err_t err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    ESP_ERROR_CHECK(err);
  ESP_ERROR_CHECK(gpio_isr_handler_add(EXAMPLE_PIN_NUM_TOUCH_INT,
                                      touch_interrupt, NULL));

  uint8_t data = 0x00;
  ESP_ERROR_CHECK(i2c_write_buff(disp_touch_dev_handle,0x00,&data,1)); //切换正常模式
}
uint8_t tpGetCoordinates(uint16_t *x,uint16_t *y)
{
  const bool pending = __atomic_exchange_n(&touch_data_pending, 0,
                                            __ATOMIC_ACQ_REL) != 0;
  if (!pending && !touch_active)
    return 0;

  uint8_t GetNum = 0;
  uint8_t data[7] = {0};
  if (i2c_read_buff(disp_touch_dev_handle,0x00,data,7) != ESP_OK)
  {
    touch_active = false;
    return 0;
  }
  GetNum = data[2];
  if(GetNum)
  {
    touch_active = true;
    *x = ((uint16_t)(data[3] & 0x0f)<<8) + (uint16_t)data[4];
    *y = ((uint16_t)(data[5] & 0x0f)<<8) + (uint16_t)data[6];
    return 1;
  }
  touch_active = false;
  return 0;
}
