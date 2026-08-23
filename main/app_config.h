#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "hm01b0.h"

/* Sensor policy for the Stage 6 sample application. */
/* The driver permits 0.5% Sensor_Core tolerance for LEDC quantization. */
#define APP_HM01B0_MCLK_FREQUENCY_HZ 12000000U
#define APP_HM01B0_I2C_FREQUENCY_HZ    100000U
#define APP_HM01B0_MODE HM01B0_SENSOR_MODE_FULL
#define APP_HM01B0_INTERFACE HM01B0_DATA_INTERFACE_8_BIT
/*
 * Supported presets by mode:
 * FULL: 15/20/30/45; QVGA: 15/20/30/45/60;
 * QQVGA: 15/20/30/45/60/120.
 */
#define APP_HM01B0_FRAME_RATE HM01B0_FRAME_RATE_30

/* DVP transport tuning remains the validated Stage 3 configuration. */
#define APP_CAPTURE_DMA_BURST_SIZE 64U

/* Optional startup preflight policy. */
#define APP_PREFLIGHT_WARMUP_FRAMES       5U
#define APP_PREFLIGHT_DIAGNOSTIC_INTERVAL 60U
#define APP_PREFLIGHT_CAPTURE_TIMEOUT_MS 3000U
#define APP_PREFLIGHT_DISPLAY_TIME_MS    3000U

/* ST7789 geometry; sensor crop/destination are derived from the selected mode. */
#define APP_ST7789_WIDTH          240U
#define APP_ST7789_HEIGHT         240U
#define APP_ST7789_CLOCK_HZ  80000000U

/* Real-image streaming policy after both startup preflights. */
#define APP_STREAM_WARMUP_FRAMES    5U
#define APP_STREAM_STATS_PERIOD_MS 1000U

#endif /* APP_CONFIG_H */
