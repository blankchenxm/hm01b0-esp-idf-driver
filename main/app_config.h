#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "hm01b0.h"

/* Sensor policy for the Stage 4.5 sample application. */
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
#define APP_PREFLIGHT_SNAPSHOT_X           42U
#define APP_PREFLIGHT_SNAPSHOT_Y            2U
#define APP_PREFLIGHT_SNAPSHOT_WIDTH      240U
#define APP_PREFLIGHT_SNAPSHOT_HEIGHT     240U

/* Current SPI ST7789 configuration; migration to esp_lcd is deferred. */
#define APP_ST7789_WIDTH          240U
#define APP_ST7789_HEIGHT         240U
#define APP_ST7789_CLOCK_HZ  40000000U

#endif /* APP_CONFIG_H */
