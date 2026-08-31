#ifndef HM01B0_LIVE_DISPLAY_H
#define HM01B0_LIVE_DISPLAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "hm01b0_capture.h"
#include "hm01b0_types.h"
#include "st7789_display.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hm01b0_live_display hm01b0_live_display_handle_t;

typedef struct {
    st7789_display_handle_t *display;
    hm01b0_pixel_format_t pixel_format;
    uint16_t source_width;
    uint16_t source_height;
    size_t source_stride;
    hm01b0_rect_t source_crop;
    uint16_t destination_x;
    uint16_t destination_y;
} hm01b0_live_display_config_t;

typedef struct {
    size_t staging_buffer_size;
    uint32_t input_frames;
    uint32_t staged_frames;
    uint32_t staging_busy_drops;
    uint32_t rgb_busy_drops;
    uint32_t conversion_errors;
    uint32_t submission_errors;
    uint32_t submitted_frames;
    uint32_t input_fps_milli;
    uint32_t staged_fps_milli;
    uint32_t submitted_fps_milli;
    uint32_t last_copy_time_us;
    uint32_t max_copy_time_us;
    uint32_t last_conversion_time_us;
    uint32_t max_conversion_time_us;
} hm01b0_live_display_stats_t;

esp_err_t hm01b0_live_display_new(
    const hm01b0_live_display_config_t *config,
    hm01b0_live_display_handle_t **out_handle);

esp_err_t hm01b0_live_display_delete(
    hm01b0_live_display_handle_t *handle);

/**
 * Non-blocking Capture Task consumer.
 *
 * If the internal-SRAM staging buffer is busy, this display frame is dropped
 * and the function returns immediately so the caller can recycle its Camera
 * Buffer.
 */
void hm01b0_live_display_consume_frame(
    const hm01b0_capture_frame_t *frame,
    void *user_data);

void hm01b0_live_display_reset_stats(
    hm01b0_live_display_handle_t *handle);

esp_err_t hm01b0_live_display_get_stats(
    const hm01b0_live_display_handle_t *handle,
    hm01b0_live_display_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* HM01B0_LIVE_DISPLAY_H */
