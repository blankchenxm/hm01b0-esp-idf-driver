#ifndef ST7789_DISPLAY_H
#define ST7789_DISPLAY_H

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

esp_err_t st7789_display_new(const st7789_display_config_t *config,
                             st7789_display_handle_t **out_display);

esp_err_t st7789_display_delete(st7789_display_handle_t *display);

/** Convert packed grayscale RAW8 to RGB565 chunks and send one image. */
esp_err_t st7789_display_draw_gray8(st7789_display_handle_t *display,
                                    uint16_t x,
                                    uint16_t y,
                                    uint16_t width,
                                    uint16_t height,
                                    const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* ST7789_DISPLAY_H */
