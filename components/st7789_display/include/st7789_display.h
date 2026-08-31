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

typedef esp_err_t (*st7789_rgb565_row_fill_cb_t)(
    uint16_t row,
    uint8_t *destination,
    size_t destination_size,
    void *user_data);

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
 * Borrow the first half of the reserved RGB565 frame workspace as packed RAW8
 * preflight storage. A mode may use less than the returned capacity. The
 * pointer remains owned by the display component.
 */
esp_err_t st7789_display_get_preflight_buffer(
    st7789_display_handle_t *display,
    uint8_t **buffer,
    size_t *buffer_size);

/** Fill and synchronously submit an RGB565 preflight image one row at a time. */
esp_err_t st7789_display_draw_rgb565_rows(
    st7789_display_handle_t *display,
    uint16_t x,
    uint16_t y,
    uint16_t width,
    uint16_t height,
    st7789_rgb565_row_fill_cb_t fill_row,
    void *user_data);

/** Fill the complete panel with one RGB565 color during preflight. */
esp_err_t st7789_display_clear_rgb565(st7789_display_handle_t *display,
                                     uint16_t rgb565);

/**
 * End preflight use of the shared workspace and enable full-frame streaming.
 */
esp_err_t st7789_display_prepare_stream(st7789_display_handle_t *display);

/**
 * Non-blocking acquisition of the single complete RGB565 DMA workspace.
 * ESP_ERR_TIMEOUT means SPI DMA still owns it and the caller should drop the
 * display frame. A successful acquisition must be followed by submit/release.
 */
esp_err_t st7789_display_try_acquire_rgb565_frame(
    st7789_display_handle_t *display,
    uint8_t **buffer,
    size_t *buffer_size);

/** Submit the acquired RGB565 workspace as one SPI-DMA rectangle. */
esp_err_t st7789_display_submit_rgb565_frame(
    st7789_display_handle_t *display,
    uint16_t width,
    uint16_t height,
    uint16_t destination_x,
    uint16_t destination_y,
    uint32_t conversion_time_us);

/** Return an acquired workspace without submitting it. */
void st7789_display_release_rgb565_frame(
    st7789_display_handle_t *display);

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
