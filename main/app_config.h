#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "hm01b0.h"

/* Sensor policy for the Stage 5 sample application. */
#define APP_HM01B0_MCLK_FREQUENCY_HZ 12000000U
#define APP_HM01B0_I2C_FREQUENCY_HZ    100000U
#define APP_HM01B0_MODE HM01B0_SENSOR_MODE_QVGA
#define APP_HM01B0_INTERFACE HM01B0_DATA_INTERFACE_8_BIT

/* DVP transport tuning remains the validated Stage 3 configuration. */
#define APP_CAPTURE_DMA_BURST_SIZE 64U

/* Optional startup preflight policy. */
#define APP_PREFLIGHT_WARMUP_FRAMES       5U
#define APP_PREFLIGHT_CAPTURE_TIMEOUT_MS 3000U
#define APP_PREFLIGHT_DISPLAY_TIME_MS    3000U

/* Shared 240x240 center crop for preflight snapshots and live display. */
#define APP_DISPLAY_CROP_X       42U
#define APP_DISPLAY_CROP_Y        2U
#define APP_DISPLAY_CROP_WIDTH  240U
#define APP_DISPLAY_CROP_HEIGHT 240U

/* SPI ST7789 configuration used by the esp_lcd transport. */
#define APP_ST7789_WIDTH          240U
#define APP_ST7789_HEIGHT         240U
#define APP_ST7789_CLOCK_HZ  40000000U

/* Real-image streaming policy after both startup preflights. */
#define APP_STREAM_WARMUP_FRAMES    5U
#define APP_STREAM_STATS_PERIOD_MS 1000U

#endif /* APP_CONFIG_H */
