#ifndef APP_MODE_PROFILE_H
#define APP_MODE_PROFILE_H

#include <stdint.h>

#include "esp_err.h"
#include "hm01b0.h"

typedef struct {
    const char *name;
    hm01b0_mode_info_t sensor;
    hm01b0_rect_t display_source;
    uint16_t display_x;
    uint16_t display_y;
} app_mode_profile_t;

/**
 * Build the capture/display geometry for one sensor mode.
 *
 * The sensor's standard valid area is centered and cropped only when it is
 * larger than the panel. A smaller image is kept at native resolution and
 * centered on the panel.
 */
esp_err_t app_mode_profile_build(hm01b0_mode_t mode,
                                 uint16_t display_width,
                                 uint16_t display_height,
                                 app_mode_profile_t *profile);

#endif /* APP_MODE_PROFILE_H */
