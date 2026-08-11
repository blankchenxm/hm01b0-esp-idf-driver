#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "hal/gpio_types.h"

/*
 * Board-specific HM01B0 wiring.
 *
 * These values describe this PCB, not the HM01B0 device itself. Keep them in
 * the application/board layer and pass the required pins to the driver through
 * hm01b0_config_t.
 */

/* Control interface used by the current probe milestone. */
#define BOARD_HM01B0_I2C_SCL_GPIO  GPIO_NUM_1
#define BOARD_HM01B0_I2C_SDA_GPIO  GPIO_NUM_2
#define BOARD_HM01B0_MCLK_GPIO     GPIO_NUM_5

/* 8-bit parallel camera interface reserved for the capture milestone. */
#define BOARD_HM01B0_PCLK_GPIO     GPIO_NUM_6
#define BOARD_HM01B0_VSYNC_GPIO    GPIO_NUM_7
#define BOARD_HM01B0_DE_GPIO       GPIO_NUM_8
#define BOARD_HM01B0_HSYNC_GPIO    BOARD_HM01B0_DE_GPIO
#define BOARD_HM01B0_D0_GPIO       GPIO_NUM_9
#define BOARD_HM01B0_D1_GPIO       GPIO_NUM_10
#define BOARD_HM01B0_D2_GPIO       GPIO_NUM_11
#define BOARD_HM01B0_D3_GPIO       GPIO_NUM_12
#define BOARD_HM01B0_D4_GPIO       GPIO_NUM_13
#define BOARD_HM01B0_D5_GPIO       GPIO_NUM_14
#define BOARD_HM01B0_D6_GPIO       GPIO_NUM_15
#define BOARD_HM01B0_D7_GPIO       GPIO_NUM_16

/* SPI ST7789 wiring used only by the sample application. */
#define BOARD_ST7789_SCLK_GPIO      GPIO_NUM_35
#define BOARD_ST7789_MOSI_GPIO      GPIO_NUM_36
#define BOARD_ST7789_RESET_GPIO     GPIO_NUM_37
#define BOARD_ST7789_DC_GPIO        GPIO_NUM_38
#define BOARD_ST7789_CS_GPIO        GPIO_NUM_39

#endif /* BOARD_CONFIG_H */
