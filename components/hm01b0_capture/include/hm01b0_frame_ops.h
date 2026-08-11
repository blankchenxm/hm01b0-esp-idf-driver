#ifndef HM01B0_FRAME_OPS_H
#define HM01B0_FRAME_OPS_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
} hm01b0_frame_rect_t;

/**
 * Copy a RAW8 rectangle into a caller-owned tightly packed or strided buffer.
 * This helper performs no allocation and has no Camera/DMA dependency.
 */
esp_err_t hm01b0_frame_crop_raw8(const uint8_t *source,
                                 uint16_t source_width,
                                 uint16_t source_height,
                                 size_t source_stride,
                                 hm01b0_frame_rect_t crop,
                                 uint8_t *destination,
                                 size_t destination_size,
                                 size_t destination_stride);

#ifdef __cplusplus
}
#endif

#endif /* HM01B0_FRAME_OPS_H */
