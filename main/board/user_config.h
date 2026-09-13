#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"

/* GPIO19 (USB D-) and GPIO20 (USB D+) are reserved for native USB. */

/* Exact GPIO allocation from the supplied Waveshare schematic/demo. */
#define LCD_HOST                         SPI2_HOST
#define TOUCH_HOST                       I2C_NUM_0
#define ESP32_SCL_NUM                    GPIO_NUM_12
#define ESP32_SDA_NUM                    GPIO_NUM_11

#define EXAMPLE_LCD_H_RES                360
#define EXAMPLE_LCD_V_RES                360
#define EXAMPLE_LVGL_BUF_HEIGHT          (EXAMPLE_LCD_V_RES / 10)
#define EXAMPLE_PIN_NUM_LCD_CS           GPIO_NUM_14
#define EXAMPLE_PIN_NUM_LCD_PCLK         GPIO_NUM_13
#define EXAMPLE_PIN_NUM_LCD_DATA0        GPIO_NUM_15
#define EXAMPLE_PIN_NUM_LCD_DATA1        GPIO_NUM_16
#define EXAMPLE_PIN_NUM_LCD_DATA2        GPIO_NUM_17
#define EXAMPLE_PIN_NUM_LCD_DATA3        GPIO_NUM_18
#define EXAMPLE_PIN_NUM_LCD_RST          GPIO_NUM_21
#define EXAMPLE_PIN_NUM_BK_LIGHT         GPIO_NUM_47
#define EXAMPLE_TOUCH_ADDR               0x15
#define EXAMPLE_DRV2605_ADDR             0x5A
#define EXAMPLE_PIN_NUM_TOUCH_RST        GPIO_NUM_10
#define EXAMPLE_PIN_NUM_TOUCH_INT        GPIO_NUM_9
#define EXAMPLE_PIN_NUM_BATTERY_ADC      GPIO_NUM_1

#define EXAMPLE_ENCODER_ECA_PIN          8
#define EXAMPLE_ENCODER_ECB_PIN          7

#define EXAMPLE_LVGL_TICK_PERIOD_MS      2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS   100
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS   5
#define EXAMPLE_LVGL_TASK_STACK_SIZE     (6 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY       2
#define EXAMPLE_USE_TOUCH                1

#define SET_BIT(reg, bit)                ((reg) |= ((uint32_t)1U << (bit)))
#define CLEAR_BIT(reg, bit)              ((reg) &= ~((uint32_t)1U << (bit)))
#define READ_BIT(reg, bit)               (((uint32_t)(reg) >> (bit)) & 1U)
#define BIT_EVEN_ALL                     0x00ffffffU

#endif
