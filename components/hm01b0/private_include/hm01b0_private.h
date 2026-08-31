#ifndef HM01B0_PRIVATE_H
#define HM01B0_PRIVATE_H

#include <stdbool.h>

#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "hm01b0.h"

struct hm01b0_dev {
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t i2c_device;
    ledc_mode_t mclk_speed_mode;
    ledc_timer_t mclk_timer;
    ledc_channel_t mclk_channel;
    bool mclk_started;
    uint32_t mclk_freq_hz;
    hm01b0_state_t state;
    hm01b0_variant_t variant;
    hm01b0_pixel_format_t pixel_format;
    uint8_t pixel_order;
    hm01b0_mode_t mode;
    hm01b0_interface_t interface;
    hm01b0_frame_rate_t frame_rate;
    hm01b0_test_pattern_t test_pattern;
};

#endif /* HM01B0_PRIVATE_H */
