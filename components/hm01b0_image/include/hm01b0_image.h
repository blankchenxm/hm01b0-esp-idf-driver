#ifndef HM01B0_IMAGE_H
#define HM01B0_IMAGE_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "hm01b0_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const uint8_t *data;
    uint16_t width;
    uint16_t height;
    size_t stride;
    /** Absolute sensor coordinate represented by data[0]. */
    uint16_t origin_x;
    uint16_t origin_y;
    hm01b0_pixel_format_t pixel_format;
} hm01b0_raw8_image_t;

/**
 * Convert one RAW8 row directly to the panel's RGB565 MSB-first byte order.
 * Bayer input is reconstructed with a small bilinear Demosaic operation.
 */
esp_err_t hm01b0_image_convert_row_to_rgb565_be(
    const hm01b0_raw8_image_t *source,
    uint16_t source_x,
    uint16_t source_y,
    uint16_t width,
    uint8_t *destination,
    size_t destination_size);

/** Convert a RAW8 rectangle directly to packed RGB565 without RGB888 storage. */
esp_err_t hm01b0_image_convert_to_rgb565_be(
    const hm01b0_raw8_image_t *source,
    hm01b0_rect_t crop,
    uint8_t *destination,
    size_t destination_size);

#ifdef __cplusplus
}
#endif

#endif /* HM01B0_IMAGE_H */
