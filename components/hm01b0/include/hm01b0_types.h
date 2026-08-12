#ifndef HM01B0_TYPES_H
#define HM01B0_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_types.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hm01b0_dev hm01b0_handle_t;

typedef enum {
    HM01B0_STATE_UNINITIALIZED = 0,
    HM01B0_STATE_STANDBY,
    HM01B0_STATE_STREAMING,
} hm01b0_state_t;

typedef enum {
    HM01B0_SENSOR_MODE_FULL = 0,
    HM01B0_SENSOR_MODE_QVGA,
    HM01B0_SENSOR_MODE_QQVGA,
} hm01b0_mode_t;

typedef enum {
    HM01B0_DATA_INTERFACE_8_BIT = 0,
    HM01B0_DATA_INTERFACE_4_BIT,
    HM01B0_DATA_INTERFACE_1_BIT,
} hm01b0_interface_t;

typedef enum {
    HM01B0_FRAME_RATE_UNCONFIGURED = 0,
    HM01B0_FRAME_RATE_15 = 15,
    HM01B0_FRAME_RATE_20 = 20,
    HM01B0_FRAME_RATE_30 = 30,
    HM01B0_FRAME_RATE_45 = 45,
    HM01B0_FRAME_RATE_60 = 60,
    HM01B0_FRAME_RATE_120 = 120,
} hm01b0_frame_rate_t;

typedef enum {
    HM01B0_TEST_PATTERN_OFF = 0,
    HM01B0_TEST_PATTERN_COLOR_BAR,
    HM01B0_TEST_PATTERN_WALKING_1,
} hm01b0_test_pattern_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
} hm01b0_rect_t;

/** Geometry produced on the DVP bus for one sensor mode. */
typedef struct {
    uint16_t transport_width;
    uint16_t transport_height;
    hm01b0_rect_t sensor_valid;
    hm01b0_rect_t standard;
} hm01b0_mode_info_t;

typedef struct {
    uint16_t addr;
    uint8_t value;
    uint8_t mask;
    uint16_t delay_ms;
} hm01b0_regval_t;

typedef struct {
    gpio_num_t mclk_gpio;
    uint32_t mclk_freq_hz;

    i2c_port_num_t i2c_port;
    gpio_num_t i2c_sda_gpio;
    gpio_num_t i2c_scl_gpio;
    uint32_t i2c_freq_hz;
    bool enable_internal_i2c_pullups;

    hm01b0_mode_t initial_mode;
    hm01b0_interface_t data_interface;
    hm01b0_frame_rate_t frame_rate;
    hm01b0_test_pattern_t test_pattern;
} hm01b0_config_t;

#ifdef __cplusplus
}
#endif

#endif /* HM01B0_TYPES_H */
