#ifndef ST7789_DISPLAY_H
#define ST7789_DISPLAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct st7789_display st7789_display_handle_t;

typedef struct {
    spi_host_device_t spi_host;
    gpio_num_t clock_gpio;
    gpio_num_t mosi_gpio;
    gpio_num_t reset_gpio;
    gpio_num_t dc_gpio;
    gpio_num_t cs_gpio;
    uint32_t clock_speed_hz;
    uint16_t width;
    uint16_t height;
} st7789_display_config_t;

typedef struct {
    size_t frame_buffer_size;
    uint32_t submitted_frames;
    uint32_t completed_frames;
    uint32_t dropped_busy;
    uint32_t submit_errors;
    uint32_t last_convert_time_us;
    uint32_t max_convert_time_us;
    uint32_t last_submit_time_us;
    uint32_t max_submit_time_us;
    uint32_t last_dma_time_us;
    uint32_t max_dma_time_us;
    uint32_t fps_milli;
    bool busy;
} st7789_display_stats_t;

esp_err_t st7789_display_new(const st7789_display_config_t *config,
                             st7789_display_handle_t **out_display);

esp_err_t st7789_display_delete(st7789_display_handle_t *display);

/**
 * Borrow the first half of the reserved RGB565 frame workspace as one packed
 * RAW8 preflight snapshot. The pointer remains owned by the display component.
 */
esp_err_t st7789_display_get_preflight_buffer(
    st7789_display_handle_t *display,
    uint8_t **buffer,
    size_t *buffer_size);

/** Convert and synchronously display one packed grayscale preflight image. */
esp_err_t st7789_display_draw_gray8(st7789_display_handle_t *display,
                                    uint16_t x,
                                    uint16_t y,
                                    uint16_t width,
                                    uint16_t height,
                                    const uint8_t *data);

/** Fill the complete panel with one grayscale value during preflight. */
esp_err_t st7789_display_clear_gray8(st7789_display_handle_t *display,
                                     uint8_t gray);

/**
 * End preflight use of the shared workspace and enable full-frame streaming.
 */
esp_err_t st7789_display_prepare_stream(st7789_display_handle_t *display);

/**
 * Non-blocking live-frame submission.
 *
 * The source is valid only for this call. Crop and RAW8-to-RGB565 conversion
 * finish before the function returns. ESP_ERR_TIMEOUT means the single RGB565
 * buffer is still owned by SPI DMA and this display frame was intentionally
 * dropped.
 */
esp_err_t st7789_display_try_draw_gray8_frame(
    st7789_display_handle_t *display,
    const uint8_t *source,
    uint16_t source_width,
    uint16_t source_height,
    size_t source_stride,
    uint16_t source_x,
    uint16_t source_y,
    uint16_t width,
    uint16_t height,
    uint16_t destination_x,
    uint16_t destination_y);

/** Wait until the outstanding SPI color transaction has completed. */
esp_err_t st7789_display_wait_idle(st7789_display_handle_t *display,
                                  uint32_t timeout_ms);

void st7789_display_reset_stats(st7789_display_handle_t *display);

esp_err_t st7789_display_get_stats(const st7789_display_handle_t *display,
                                   st7789_display_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* ST7789_DISPLAY_H */
