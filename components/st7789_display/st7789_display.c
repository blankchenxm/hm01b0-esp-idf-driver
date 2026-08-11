#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#include "esp_attr.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "st7789_display.h"

#define ST7789_NORON    0x13U
#define ST7789_FRMCTR2  0xB2U
#define ST7789_VMCTR1   0xC5U
#define ST7789_GMCTRP1  0xE0U
#define ST7789_GMCTRN1  0xE1U

#define ST7789_RGB565_BYTES_PER_PIXEL 2U
#define ST7789_SPI_QUEUE_DEPTH        1U

struct st7789_display {
    st7789_display_config_t config;
    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_handle_t panel;
    SemaphoreHandle_t buffer_free;
    StaticSemaphore_t buffer_free_state;
    portMUX_TYPE stats_lock;
    uint8_t *preflight_line;
    size_t preflight_line_size;
    uint8_t *rgb_frame;
    size_t rgb_frame_size;
    bool bus_initialized;
    bool stream_prepared;
    bool transfer_completes_frame;
    int64_t dma_start_us;
    int64_t stats_start_us;
    st7789_display_stats_t stats;
};

static const char *TAG = "st7789_display";

static bool IRAM_ATTR st7789_color_trans_done(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_io_event_data_t *event_data,
    void *user_ctx)
{
    (void)panel_io;
    (void)event_data;
    st7789_display_handle_t *display = user_ctx;
    const int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&display->stats_lock);
    if (display->transfer_completes_frame) {
        const uint32_t dma_time_us = (uint32_t)(now_us - display->dma_start_us);
        display->stats.completed_frames++;
        display->stats.last_dma_time_us = dma_time_us;
        if (dma_time_us > display->stats.max_dma_time_us) {
            display->stats.max_dma_time_us = dma_time_us;
        }
    }
    display->stats.busy = false;
    portEXIT_CRITICAL_ISR(&display->stats_lock);

    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(display->buffer_free, &task_woken);
    return task_woken == pdTRUE;
}

static esp_err_t st7789_send_tuning(st7789_display_handle_t *display)
{
    static const uint8_t porch[] = {0x0CU, 0x0CU, 0x00U, 0x33U, 0x33U};
    static const uint8_t power[] = {0xA4U, 0xA1U};
    static const uint8_t gamma_positive[] = {
        0xD0U, 0x04U, 0x0DU, 0x11U, 0x13U, 0x2BU, 0x3FU,
        0x54U, 0x4CU, 0x18U, 0x0DU, 0x0BU, 0x1FU, 0x23U,
    };
    static const uint8_t gamma_negative[] = {
        0xD0U, 0x04U, 0x0CU, 0x11U, 0x13U, 0x2CU, 0x3FU,
        0x44U, 0x51U, 0x2FU, 0x1FU, 0x1FU, 0x20U, 0x23U,
    };

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, ST7789_FRMCTR2,
                            porch, sizeof(porch)),
                        TAG, "porch configuration failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, 0xB7U,
                            (uint8_t[]) {0x35U}, 1U),
                        TAG, "gate-control configuration failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, ST7789_VMCTR1,
                            (uint8_t[]) {0x19U}, 1U),
                        TAG, "VCOM configuration failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, 0xC0U,
                            (uint8_t[]) {0x2CU}, 1U),
                        TAG, "LCM-control configuration failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, 0xC2U,
                            (uint8_t[]) {0x01U}, 1U),
                        TAG, "VDV/VRH configuration failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, 0xC3U,
                            (uint8_t[]) {0x12U}, 1U),
                        TAG, "VRH configuration failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, 0xC4U,
                            (uint8_t[]) {0x20U}, 1U),
                        TAG, "VDV configuration failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, 0xC6U,
                            (uint8_t[]) {0x0FU}, 1U),
                        TAG, "frame-rate configuration failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, 0xD0U,
                            power, sizeof(power)),
                        TAG, "power configuration failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, ST7789_GMCTRP1,
                            gamma_positive, sizeof(gamma_positive)),
                        TAG, "positive-gamma configuration failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, ST7789_GMCTRN1,
                            gamma_negative, sizeof(gamma_negative)),
                        TAG, "negative-gamma configuration failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(display->panel, true), TAG,
                        "color inversion failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(
                            display->io, ST7789_NORON, NULL, 0U),
                        TAG, "normal-mode command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(display->panel, true), TAG,
                        "display-on command failed");
    return ESP_OK;
}

static void st7789_gray8_to_rgb565_be(const uint8_t *source,
                                      uint8_t *destination,
                                      size_t pixel_count)
{
    for (size_t i = 0; i < pixel_count; ++i) {
        const uint8_t gray = source[i];
        const uint16_t rgb565 =
            (uint16_t)(((uint16_t)(gray >> 3U) << 11U) |
                       ((uint16_t)(gray >> 2U) << 5U) |
                       (gray >> 3U));
        destination[i * 2U] = (uint8_t)(rgb565 >> 8U);
        destination[i * 2U + 1U] = (uint8_t)rgb565;
    }
}

static esp_err_t st7789_take_buffer(st7789_display_handle_t *display,
                                    TickType_t wait_ticks)
{
    if (xSemaphoreTake(display->buffer_free, wait_ticks) != pdPASS) {
        return ESP_ERR_TIMEOUT;
    }
    portENTER_CRITICAL(&display->stats_lock);
    display->stats.busy = true;
    portEXIT_CRITICAL(&display->stats_lock);
    return ESP_OK;
}

static void st7789_release_buffer(st7789_display_handle_t *display)
{
    portENTER_CRITICAL(&display->stats_lock);
    display->stats.busy = false;
    portEXIT_CRITICAL(&display->stats_lock);
    (void)xSemaphoreGive(display->buffer_free);
}

static esp_err_t st7789_submit(st7789_display_handle_t *display,
                               uint16_t x,
                               uint16_t y,
                               uint16_t width,
                               uint16_t height,
                               const uint8_t *rgb565,
                               bool completes_frame)
{
    const int64_t submit_start_us = esp_timer_get_time();
    portENTER_CRITICAL(&display->stats_lock);
    display->transfer_completes_frame = completes_frame;
    if (completes_frame) {
        display->dma_start_us = submit_start_us;
    }
    portEXIT_CRITICAL(&display->stats_lock);
    const esp_err_t ret = esp_lcd_panel_draw_bitmap(
        display->panel, x, y, x + width, y + height, rgb565);
    const uint32_t submit_time_us = (uint32_t)(
        esp_timer_get_time() - submit_start_us);

    portENTER_CRITICAL(&display->stats_lock);
    display->stats.last_submit_time_us = submit_time_us;
    if (submit_time_us > display->stats.max_submit_time_us) {
        display->stats.max_submit_time_us = submit_time_us;
    }
    if (ret != ESP_OK) {
        display->stats.submit_errors++;
    }
    portEXIT_CRITICAL(&display->stats_lock);

    if (ret != ESP_OK) {
        st7789_release_buffer(display);
    }
    return ret;
}

esp_err_t st7789_display_new(const st7789_display_config_t *config,
                             st7789_display_handle_t **out_display)
{
    ESP_RETURN_ON_FALSE(config != NULL && out_display != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(config->width > 0U && config->height > 0U &&
                        config->clock_speed_hz > 0U,
                        ESP_ERR_INVALID_ARG, TAG,
                        "invalid display geometry or clock");
    *out_display = NULL;

    st7789_display_handle_t *display = calloc(1, sizeof(*display));
    ESP_RETURN_ON_FALSE(display != NULL, ESP_ERR_NO_MEM, TAG,
                        "failed to allocate display context");
    display->config = *config;
    display->stats_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    display->rgb_frame_size = (size_t)config->width * config->height *
                              ST7789_RGB565_BYTES_PER_PIXEL;
    display->preflight_line_size =
        (size_t)config->width * ST7789_RGB565_BYTES_PER_PIXEL;
    display->buffer_free = xSemaphoreCreateBinaryStatic(
        &display->buffer_free_state);
    if (display->buffer_free == NULL) {
        free(display);
        return ESP_ERR_NO_MEM;
    }
    (void)xSemaphoreGive(display->buffer_free);

    display->preflight_line = heap_caps_malloc(
        display->preflight_line_size,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (display->preflight_line == NULL) {
        free(display);
        return ESP_ERR_NO_MEM;
    }

    const uint32_t dma_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA |
                              MALLOC_CAP_8BIT;
    const size_t free_before = heap_caps_get_free_size(dma_caps);
    const size_t largest_before =
        heap_caps_get_largest_free_block(dma_caps);
    display->rgb_frame = heap_caps_malloc(display->rgb_frame_size, dma_caps);
    if (display->rgb_frame == NULL) {
        ESP_LOGE(TAG,
                 "failed to reserve %u-byte internal DMA RGB565 workspace; "
                 "free=%u largest=%u",
                 (unsigned)display->rgb_frame_size,
                 (unsigned)free_before, (unsigned)largest_before);
        heap_caps_free(display->preflight_line);
        free(display);
        return ESP_ERR_NO_MEM;
    }
    const size_t free_after = heap_caps_get_free_size(dma_caps);
    const size_t largest_after =
        heap_caps_get_largest_free_block(dma_caps);
    ESP_LOGI(TAG,
             "reserved RGB565 workspace=%p internal DMA, bytes=%u; heap "
             "before free=%u largest=%u, after free=%u largest=%u",
             display->rgb_frame, (unsigned)display->rgb_frame_size,
             (unsigned)free_before, (unsigned)largest_before,
             (unsigned)free_after, (unsigned)largest_after);

    const spi_bus_config_t bus_config = {
        .mosi_io_num = config->mosi_gpio,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = config->clock_gpio,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = display->rgb_frame_size,
    };
    esp_err_t ret = spi_bus_initialize(
        config->spi_host, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        goto fail;
    }
    display->bus_initialized = true;

    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = config->cs_gpio,
        .dc_gpio_num = config->dc_gpio,
        .spi_mode = 0,
        .pclk_hz = config->clock_speed_hz,
        .trans_queue_depth = ST7789_SPI_QUEUE_DEPTH,
        .on_color_trans_done = st7789_color_trans_done,
        .user_ctx = display,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ret = esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)config->spi_host,
        &io_config, &display->io);
    if (ret != ESP_OK) {
        goto fail;
    }

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = config->reset_gpio,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 16,
    };
    ret = esp_lcd_new_panel_st7789(
        display->io, &panel_config, &display->panel);
    if (ret != ESP_OK) {
        goto fail;
    }
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(display->panel), fail, TAG,
                      "panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(display->panel), fail, TAG,
                      "panel initialization failed");
    ESP_GOTO_ON_ERROR(st7789_send_tuning(display), fail, TAG,
                      "panel tuning failed");

    st7789_display_reset_stats(display);
    ESP_LOGI(TAG,
             "ST7789 initialized with esp_lcd (%ux%u RGB565, SPI=%" PRIu32
             " Hz, max_transfer=%u)",
             config->width, config->height, config->clock_speed_hz,
             (unsigned)display->rgb_frame_size);
    *out_display = display;
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "ST7789 initialization failed: %s", esp_err_to_name(ret));
    (void)st7789_display_delete(display);
    return ret;
}

esp_err_t st7789_display_delete(st7789_display_handle_t *display)
{
    if (display == NULL) {
        return ESP_OK;
    }
    esp_err_t result = st7789_display_wait_idle(display, 1000U);
    if (display->panel != NULL) {
        const esp_err_t ret = esp_lcd_panel_del(display->panel);
        if (result == ESP_OK) {
            result = ret;
        }
    }
    if (display->io != NULL) {
        const esp_err_t ret = esp_lcd_panel_io_del(display->io);
        if (result == ESP_OK) {
            result = ret;
        }
    }
    if (display->bus_initialized) {
        const esp_err_t ret = spi_bus_free(display->config.spi_host);
        if (result == ESP_OK) {
            result = ret;
        }
    }
    heap_caps_free(display->preflight_line);
    heap_caps_free(display->rgb_frame);
    free(display);
    return result;
}

esp_err_t st7789_display_draw_gray8(st7789_display_handle_t *display,
                                    uint16_t x,
                                    uint16_t y,
                                    uint16_t width,
                                    uint16_t height,
                                    const uint8_t *data)
{
    ESP_RETURN_ON_FALSE(display != NULL && data != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(display->preflight_line != NULL,
                        ESP_ERR_INVALID_STATE, TAG,
                        "preflight line buffer has been released");
    ESP_RETURN_ON_FALSE(width > 0U && height > 0U &&
                        x < display->config.width &&
                        y < display->config.height &&
                        width <= display->config.width - x &&
                        height <= display->config.height - y,
                        ESP_ERR_INVALID_ARG, TAG,
                        "image is outside the display");

    portENTER_CRITICAL(&display->stats_lock);
    display->stats.submitted_frames++;
    portEXIT_CRITICAL(&display->stats_lock);

    for (uint16_t row = 0; row < height; ++row) {
        ESP_RETURN_ON_ERROR(st7789_take_buffer(display, portMAX_DELAY), TAG,
                            "failed to acquire preflight line buffer");
        st7789_gray8_to_rgb565_be(
            data + (size_t)row * width,
            display->preflight_line, width);
        const esp_err_t ret = st7789_submit(
            display, x, (uint16_t)(y + row), width, 1U,
            display->preflight_line, row == height - 1U);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return st7789_display_wait_idle(display, 1000U);
}

esp_err_t st7789_display_get_preflight_buffer(
    st7789_display_handle_t *display,
    uint8_t **buffer,
    size_t *buffer_size)
{
    ESP_RETURN_ON_FALSE(display != NULL && buffer != NULL &&
                            buffer_size != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(!display->stream_prepared &&
                            display->rgb_frame != NULL,
                        ESP_ERR_INVALID_STATE, TAG,
                        "preflight workspace is unavailable");
    *buffer = display->rgb_frame;
    *buffer_size = (size_t)display->config.width * display->config.height;
    return ESP_OK;
}

esp_err_t st7789_display_prepare_stream(st7789_display_handle_t *display)
{
    ESP_RETURN_ON_FALSE(display != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "display handle is NULL");
    ESP_RETURN_ON_FALSE(!display->stream_prepared,
                        ESP_ERR_INVALID_STATE, TAG,
                        "streaming is already prepared");
    ESP_RETURN_ON_ERROR(st7789_display_wait_idle(display, 1000U), TAG,
                        "display did not become idle");

    heap_caps_free(display->preflight_line);
    display->preflight_line = NULL;
    display->preflight_line_size = 0U;
    display->stream_prepared = true;
    ESP_LOGI(TAG,
             "RGB565 workspace switched from %u-byte RAW8 preflight use to "
             "%u-byte full-frame SPI DMA use",
             (unsigned)((size_t)display->config.width * display->config.height),
             (unsigned)display->rgb_frame_size);
    st7789_display_reset_stats(display);
    return ESP_OK;
}

esp_err_t st7789_display_try_draw_gray8_frame(
    st7789_display_handle_t *display,
    const uint8_t *source,
    uint16_t source_width,
    uint16_t source_height,
    size_t source_stride,
    uint16_t crop_x,
    uint16_t crop_y)
{
    ESP_RETURN_ON_FALSE(display != NULL && source != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(display->stream_prepared &&
                            display->rgb_frame != NULL,
                        ESP_ERR_INVALID_STATE, TAG,
                        "stream buffer is not prepared");
    ESP_RETURN_ON_FALSE(source_width > 0U && source_height > 0U &&
                        source_stride >= source_width &&
                        crop_x < source_width && crop_y < source_height &&
                        display->config.width <= source_width - crop_x &&
                        display->config.height <= source_height - crop_y,
                        ESP_ERR_INVALID_ARG, TAG, "invalid source crop");

    if (st7789_take_buffer(display, 0U) != ESP_OK) {
        portENTER_CRITICAL(&display->stats_lock);
        display->stats.dropped_busy++;
        portEXIT_CRITICAL(&display->stats_lock);
        return ESP_ERR_TIMEOUT;
    }

    const int64_t convert_start_us = esp_timer_get_time();
    for (uint16_t row = 0; row < display->config.height; ++row) {
        const uint8_t *source_row =
            source + (size_t)(crop_y + row) * source_stride + crop_x;
        uint8_t *destination_row =
            display->rgb_frame +
            (size_t)row * display->config.width *
                ST7789_RGB565_BYTES_PER_PIXEL;
        st7789_gray8_to_rgb565_be(
            source_row, destination_row, display->config.width);
    }
    const uint32_t convert_time_us = (uint32_t)(
        esp_timer_get_time() - convert_start_us);

    portENTER_CRITICAL(&display->stats_lock);
    display->stats.last_convert_time_us = convert_time_us;
    if (convert_time_us > display->stats.max_convert_time_us) {
        display->stats.max_convert_time_us = convert_time_us;
    }
    portEXIT_CRITICAL(&display->stats_lock);

    const esp_err_t ret = st7789_submit(
        display, 0U, 0U, display->config.width, display->config.height,
        display->rgb_frame, true);
    if (ret == ESP_OK) {
        portENTER_CRITICAL(&display->stats_lock);
        display->stats.submitted_frames++;
        portEXIT_CRITICAL(&display->stats_lock);
    }
    return ret;
}

esp_err_t st7789_display_wait_idle(st7789_display_handle_t *display,
                                  uint32_t timeout_ms)
{
    if (display == NULL || display->buffer_free == NULL) {
        return ESP_OK;
    }
    const TickType_t wait_ticks = timeout_ms == UINT32_MAX
                                      ? portMAX_DELAY
                                      : pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(display->buffer_free, wait_ticks) != pdPASS) {
        return ESP_ERR_TIMEOUT;
    }
    (void)xSemaphoreGive(display->buffer_free);
    return ESP_OK;
}

void st7789_display_reset_stats(st7789_display_handle_t *display)
{
    if (display == NULL) {
        return;
    }
    portENTER_CRITICAL(&display->stats_lock);
    const bool busy = display->stats.busy;
    const size_t frame_buffer_size = display->rgb_frame != NULL
                                         ? display->rgb_frame_size
                                         : 0U;
    display->stats = (st7789_display_stats_t) {
        .frame_buffer_size = frame_buffer_size,
        .busy = busy,
    };
    display->stats_start_us = esp_timer_get_time();
    portEXIT_CRITICAL(&display->stats_lock);
}

esp_err_t st7789_display_get_stats(const st7789_display_handle_t *display,
                                   st7789_display_stats_t *stats)
{
    ESP_RETURN_ON_FALSE(display != NULL && stats != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    st7789_display_handle_t *mutable_display =
        (st7789_display_handle_t *)display;
    portENTER_CRITICAL(&mutable_display->stats_lock);
    *stats = display->stats;
    const int64_t stats_start_us = display->stats_start_us;
    portEXIT_CRITICAL(&mutable_display->stats_lock);

    const int64_t elapsed_us = esp_timer_get_time() - stats_start_us;
    if (elapsed_us > 0) {
        stats->fps_milli = (uint32_t)(
            ((uint64_t)stats->completed_frames * 1000000000ULL) /
            (uint64_t)elapsed_us);
    }
    return ESP_OK;
}
