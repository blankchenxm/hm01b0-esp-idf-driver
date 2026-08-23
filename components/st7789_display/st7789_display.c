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
#define ST7789_GRAY8_LUT_SIZE         256U
#define ST7789_SPI_QUEUE_DEPTH        2U
#define ST7789_STREAM_STRIP_COUNT     2U
#define ST7789_STREAM_STRIP_HEIGHT    60U
#define ST7789_PREFLIGHT_BUFFER_ID    UINT8_MAX

typedef struct {
    SemaphoreHandle_t free;
    StaticSemaphore_t free_state;
    uint8_t *data;
} st7789_strip_buffer_t;

typedef struct {
    uint8_t buffer_id;
    bool completes_frame;
} st7789_pending_transfer_t;

struct st7789_display {
    st7789_display_config_t config;
    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_handle_t panel;
    SemaphoreHandle_t preflight_free;
    StaticSemaphore_t preflight_free_state;
    st7789_strip_buffer_t strips[ST7789_STREAM_STRIP_COUNT];
    portMUX_TYPE stats_lock;
    uint8_t *preflight_line;
    size_t preflight_line_size;
    uint8_t *workspace;
    size_t workspace_size;
    size_t preflight_capacity;
    size_t strip_buffer_size;
    st7789_pending_transfer_t pending[ST7789_SPI_QUEUE_DEPTH];
    uint8_t pending_head;
    uint8_t pending_tail;
    uint8_t pending_count;
    uint16_t rgb565_table[ST7789_GRAY8_LUT_SIZE];
    bool bus_initialized;
    bool stream_prepared;
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
    st7789_pending_transfer_t completed = {0};
    bool have_completed = false;

    portENTER_CRITICAL_ISR(&display->stats_lock);
    if (display->pending_count > 0U) {
        completed = display->pending[display->pending_head];
        display->pending_head = (uint8_t)(
            (display->pending_head + 1U) % ST7789_SPI_QUEUE_DEPTH);
        display->pending_count--;
        have_completed = true;
    } else {
        display->stats.submit_errors++;
    }
    if (have_completed && completed.completes_frame) {
        const uint32_t dma_time_us = (uint32_t)(now_us - display->dma_start_us);
        display->stats.completed_frames++;
        display->stats.last_dma_time_us = dma_time_us;
        if (dma_time_us > display->stats.max_dma_time_us) {
            display->stats.max_dma_time_us = dma_time_us;
        }
        display->stats.busy = false;
    }
    portEXIT_CRITICAL_ISR(&display->stats_lock);

    BaseType_t task_woken = pdFALSE;
    if (have_completed) {
        SemaphoreHandle_t free = completed.buffer_id ==
                                         ST7789_PREFLIGHT_BUFFER_ID
                                     ? display->preflight_free
                                     : display->strips[completed.buffer_id].free;
        xSemaphoreGiveFromISR(free, &task_woken);
    }
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

static void st7789_init_rgb565_table(st7789_display_handle_t *display)
{
    for (uint16_t gray = 0U; gray < ST7789_GRAY8_LUT_SIZE; ++gray) {
        const uint16_t rgb565 =
            (uint16_t)(((gray >> 3U) << 11U) |
                       ((gray >> 2U) << 5U) |
                       (gray >> 3U));

        /*
         * ESP32-S3 stores uint16_t little-endian, while the validated panel
         * byte stream is RGB565 MSB first. Store the byte-swapped word once
         * so the hot conversion loop needs only a lookup and a 16-bit store.
         */
        display->rgb565_table[gray] =
            (uint16_t)((rgb565 >> 8U) | (rgb565 << 8U));
    }
}

static void st7789_gray8_to_rgb565_be(const uint16_t *rgb565_table,
                                      const uint8_t *source,
                                      uint8_t *destination,
                                      size_t pixel_count)
{
    uint16_t *destination_pixels = (uint16_t *)destination;
    for (size_t i = 0; i < pixel_count; ++i) {
        destination_pixels[i] = rgb565_table[source[i]];
    }
}

static SemaphoreHandle_t st7789_get_buffer_semaphore(
    st7789_display_handle_t *display,
    uint8_t buffer_id)
{
    if (buffer_id == ST7789_PREFLIGHT_BUFFER_ID) {
        return display->preflight_free;
    }
    return buffer_id < ST7789_STREAM_STRIP_COUNT
               ? display->strips[buffer_id].free
               : NULL;
}

static esp_err_t st7789_take_buffer(st7789_display_handle_t *display,
                                    uint8_t buffer_id,
                                    TickType_t wait_ticks)
{
    SemaphoreHandle_t free = st7789_get_buffer_semaphore(display, buffer_id);
    if (free == NULL || xSemaphoreTake(free, wait_ticks) != pdPASS) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void st7789_release_buffer(st7789_display_handle_t *display,
                                  uint8_t buffer_id)
{
    SemaphoreHandle_t free = st7789_get_buffer_semaphore(display, buffer_id);
    if (free != NULL) {
        (void)xSemaphoreGive(free);
    }
}

static esp_err_t st7789_queue_pending_transfer(
    st7789_display_handle_t *display,
    uint8_t buffer_id,
    bool completes_frame)
{
    esp_err_t ret = ESP_OK;
    portENTER_CRITICAL(&display->stats_lock);
    if (display->pending_count >= ST7789_SPI_QUEUE_DEPTH) {
        ret = ESP_ERR_INVALID_STATE;
    } else {
        display->pending[display->pending_tail] =
            (st7789_pending_transfer_t) {
                .buffer_id = buffer_id,
                .completes_frame = completes_frame,
            };
        display->pending_tail = (uint8_t)(
            (display->pending_tail + 1U) % ST7789_SPI_QUEUE_DEPTH);
        display->pending_count++;
    }
    portEXIT_CRITICAL(&display->stats_lock);
    return ret;
}

static bool st7789_rollback_pending_transfer(
    st7789_display_handle_t *display,
    uint8_t buffer_id)
{
    bool rolled_back = false;
    portENTER_CRITICAL(&display->stats_lock);
    if (display->pending_count > 0U) {
        const uint8_t last = (uint8_t)(
            (display->pending_tail + ST7789_SPI_QUEUE_DEPTH - 1U) %
            ST7789_SPI_QUEUE_DEPTH);
        if (display->pending[last].buffer_id == buffer_id) {
            display->pending_tail = last;
            display->pending_count--;
            rolled_back = true;
        }
    }
    portEXIT_CRITICAL(&display->stats_lock);
    return rolled_back;
}

static esp_err_t st7789_submit(st7789_display_handle_t *display,
                               uint16_t x,
                               uint16_t y,
                               uint16_t width,
                               uint16_t height,
                               const uint8_t *rgb565,
                               uint8_t buffer_id,
                               bool starts_frame,
                               bool completes_frame,
                               uint32_t *elapsed_us)
{
    const int64_t submit_start_us = esp_timer_get_time();
    portENTER_CRITICAL(&display->stats_lock);
    if (starts_frame) {
        display->dma_start_us = submit_start_us;
    }
    portEXIT_CRITICAL(&display->stats_lock);

    esp_err_t ret = st7789_queue_pending_transfer(
        display, buffer_id, completes_frame);
    if (ret != ESP_OK) {
        portENTER_CRITICAL(&display->stats_lock);
        display->stats.submit_errors++;
        portEXIT_CRITICAL(&display->stats_lock);
        st7789_release_buffer(display, buffer_id);
        return ret;
    }
    ret = esp_lcd_panel_draw_bitmap(
        display->panel, x, y, x + width, y + height, rgb565);
    const uint32_t submit_time_us = (uint32_t)(
        esp_timer_get_time() - submit_start_us);
    if (elapsed_us != NULL) {
        *elapsed_us = submit_time_us;
    }

    portENTER_CRITICAL(&display->stats_lock);
    if (ret != ESP_OK) {
        display->stats.submit_errors++;
    }
    portEXIT_CRITICAL(&display->stats_lock);

    if (ret != ESP_OK) {
        if (st7789_rollback_pending_transfer(display, buffer_id)) {
            st7789_release_buffer(display, buffer_id);
        }
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
    st7789_init_rgb565_table(display);
    display->preflight_capacity = (size_t)config->width * config->height;
    display->strip_buffer_size =
        (size_t)config->width * ST7789_STREAM_STRIP_HEIGHT *
        ST7789_RGB565_BYTES_PER_PIXEL;
    const size_t strip_workspace_size =
        display->strip_buffer_size * ST7789_STREAM_STRIP_COUNT;
    display->workspace_size = display->preflight_capacity >
                                      strip_workspace_size
                                  ? display->preflight_capacity
                                  : strip_workspace_size;
    display->preflight_line_size =
        (size_t)config->width * ST7789_RGB565_BYTES_PER_PIXEL;
    display->preflight_free = xSemaphoreCreateBinaryStatic(
        &display->preflight_free_state);
    if (display->preflight_free == NULL) {
        free(display);
        return ESP_ERR_NO_MEM;
    }
    (void)xSemaphoreGive(display->preflight_free);
    for (size_t i = 0U; i < ST7789_STREAM_STRIP_COUNT; ++i) {
        display->strips[i].free = xSemaphoreCreateBinaryStatic(
            &display->strips[i].free_state);
        if (display->strips[i].free == NULL) {
            free(display);
            return ESP_ERR_NO_MEM;
        }
        (void)xSemaphoreGive(display->strips[i].free);
    }

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
    display->workspace = heap_caps_malloc(display->workspace_size, dma_caps);
    if (display->workspace == NULL) {
        ESP_LOGE(TAG,
                 "failed to reserve %u-byte internal DMA shared workspace; "
                 "free=%u largest=%u",
                 (unsigned)display->workspace_size,
                 (unsigned)free_before, (unsigned)largest_before);
        heap_caps_free(display->preflight_line);
        free(display);
        return ESP_ERR_NO_MEM;
    }
    for (size_t i = 0U; i < ST7789_STREAM_STRIP_COUNT; ++i) {
        display->strips[i].data = display->workspace +
                                  i * display->strip_buffer_size;
    }
    const size_t free_after = heap_caps_get_free_size(dma_caps);
    const size_t largest_after =
        heap_caps_get_largest_free_block(dma_caps);
    ESP_LOGI(TAG,
             "reserved shared workspace=%p internal DMA, bytes=%u; "
             "preflight RAW8 capacity=%u, stream strips=%u x %u bytes "
             "(%ux%u RGB565); heap before free=%u largest=%u, after "
             "free=%u largest=%u",
             display->workspace, (unsigned)display->workspace_size,
             (unsigned)display->preflight_capacity,
             (unsigned)ST7789_STREAM_STRIP_COUNT,
             (unsigned)display->strip_buffer_size,
             (unsigned)config->width,
             (unsigned)ST7789_STREAM_STRIP_HEIGHT,
             (unsigned)free_before, (unsigned)largest_before,
             (unsigned)free_after, (unsigned)largest_after);

    const spi_bus_config_t bus_config = {
        .mosi_io_num = config->mosi_gpio,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = config->clock_gpio,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = display->strip_buffer_size,
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
             " Hz, queue=%u, max_transfer=%u, gray_lut=%u bytes)",
             config->width, config->height, config->clock_speed_hz,
             (unsigned)ST7789_SPI_QUEUE_DEPTH,
             (unsigned)display->strip_buffer_size,
             (unsigned)sizeof(display->rgb565_table));
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
    heap_caps_free(display->workspace);
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
        ESP_RETURN_ON_ERROR(st7789_take_buffer(
                                display, ST7789_PREFLIGHT_BUFFER_ID,
                                portMAX_DELAY),
                            TAG,
                            "failed to acquire preflight line buffer");
        st7789_gray8_to_rgb565_be(
            display->rgb565_table,
            data + (size_t)row * width,
            display->preflight_line, width);
        const esp_err_t ret = st7789_submit(
            display, x, (uint16_t)(y + row), width, 1U,
            display->preflight_line, ST7789_PREFLIGHT_BUFFER_ID,
            row == 0U, row == height - 1U, NULL);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return st7789_display_wait_idle(display, 1000U);
}

esp_err_t st7789_display_clear_gray8(st7789_display_handle_t *display,
                                     uint8_t gray)
{
    ESP_RETURN_ON_FALSE(display != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "display handle is NULL");
    ESP_RETURN_ON_FALSE(display->preflight_line != NULL,
                        ESP_ERR_INVALID_STATE, TAG,
                        "preflight line buffer has been released");

    uint16_t *pixels = (uint16_t *)display->preflight_line;
    const uint16_t color = display->rgb565_table[gray];
    for (uint16_t x = 0U; x < display->config.width; ++x) {
        pixels[x] = color;
    }

    for (uint16_t row = 0U; row < display->config.height; ++row) {
        ESP_RETURN_ON_ERROR(st7789_take_buffer(
                                display, ST7789_PREFLIGHT_BUFFER_ID,
                                portMAX_DELAY),
                            TAG,
                            "failed to acquire clear line buffer");
        const esp_err_t ret = st7789_submit(
            display, 0U, row, display->config.width, 1U,
            display->preflight_line, ST7789_PREFLIGHT_BUFFER_ID,
            row == 0U, row == display->config.height - 1U, NULL);
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
                            display->workspace != NULL,
                        ESP_ERR_INVALID_STATE, TAG,
                        "preflight workspace is unavailable");
    *buffer = display->workspace;
    *buffer_size = display->preflight_capacity;
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
             "shared workspace ended RAW8 preflight use (capacity=%u bytes); "
             "streaming uses Strip A=%p and Strip B=%p, each=%u bytes "
             "(%ux%u RGB565)",
             (unsigned)display->preflight_capacity,
             display->strips[0].data, display->strips[1].data,
             (unsigned)display->strip_buffer_size,
             (unsigned)display->config.width,
             (unsigned)ST7789_STREAM_STRIP_HEIGHT);
    st7789_display_reset_stats(display);
    return ESP_OK;
}

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
    uint16_t destination_y)
{
    ESP_RETURN_ON_FALSE(display != NULL && source != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(display->stream_prepared &&
                            display->workspace != NULL,
                        ESP_ERR_INVALID_STATE, TAG,
                        "stream buffer is not prepared");
    ESP_RETURN_ON_FALSE(
        source_width > 0U && source_height > 0U &&
            source_stride >= source_width && width > 0U && height > 0U &&
            source_x < source_width && source_y < source_height &&
            width <= source_width - source_x &&
            height <= source_height - source_y &&
            destination_x < display->config.width &&
            destination_y < display->config.height &&
            width <= display->config.width - destination_x &&
            height <= display->config.height - destination_y,
        ESP_ERR_INVALID_ARG, TAG, "invalid source/destination rectangle");

    const uint16_t transfer_count = (uint16_t)(
        (height + ST7789_STREAM_STRIP_HEIGHT - 1U) /
        ST7789_STREAM_STRIP_HEIGHT);
    const uint8_t initial_buffer_count =
        transfer_count < ST7789_STREAM_STRIP_COUNT
            ? (uint8_t)transfer_count
            : ST7789_STREAM_STRIP_COUNT;
    bool buffer_held[ST7789_STREAM_STRIP_COUNT] = {false};
    for (uint8_t i = 0U; i < initial_buffer_count; ++i) {
        if (st7789_take_buffer(display, i, 0U) != ESP_OK) {
            for (uint8_t acquired = 0U; acquired < i; ++acquired) {
                st7789_release_buffer(display, acquired);
            }
            portENTER_CRITICAL(&display->stats_lock);
            display->stats.dropped_busy++;
            portEXIT_CRITICAL(&display->stats_lock);
            return ESP_ERR_TIMEOUT;
        }
        buffer_held[i] = true;
    }

    portENTER_CRITICAL(&display->stats_lock);
    display->stats.busy = true;
    portEXIT_CRITICAL(&display->stats_lock);

    uint32_t total_convert_time_us = 0U;
    uint32_t total_submit_time_us = 0U;
    esp_err_t ret = ESP_OK;
    /*
     * A submitted Strip belongs to SPI DMA until its callback gives the
     * matching semaphore. CPU conversion then proceeds in the other Strip.
     * Taking the same Strip before its next use is the ownership hand-off that
     * prevents overwriting bytes still being transmitted.
     */
    for (uint16_t transfer = 0U; transfer < transfer_count; ++transfer) {
        const uint8_t buffer_id = (uint8_t)(
            transfer % ST7789_STREAM_STRIP_COUNT);
        if (!buffer_held[buffer_id]) {
            ret = st7789_take_buffer(display, buffer_id, portMAX_DELAY);
            if (ret != ESP_OK) {
                break;
            }
            buffer_held[buffer_id] = true;
        }

        const uint16_t first_row = (uint16_t)(
            transfer * ST7789_STREAM_STRIP_HEIGHT);
        const uint16_t remaining_rows = (uint16_t)(height - first_row);
        const uint16_t rows = remaining_rows < ST7789_STREAM_STRIP_HEIGHT
                                  ? remaining_rows
                                  : ST7789_STREAM_STRIP_HEIGHT;

        const int64_t convert_start_us = esp_timer_get_time();
        for (uint16_t row = 0U; row < rows; ++row) {
            const uint8_t *source_row =
                source + (size_t)(source_y + first_row + row) *
                             source_stride +
                source_x;
            uint8_t *destination_row =
                display->strips[buffer_id].data +
                (size_t)row * width * ST7789_RGB565_BYTES_PER_PIXEL;
            st7789_gray8_to_rgb565_be(
                display->rgb565_table,
                source_row, destination_row, width);
        }
        total_convert_time_us += (uint32_t)(
            esp_timer_get_time() - convert_start_us);

        uint32_t submit_time_us = 0U;
        ret = st7789_submit(
            display, destination_x,
            (uint16_t)(destination_y + first_row), width, rows,
            display->strips[buffer_id].data, buffer_id,
            transfer == 0U, transfer == transfer_count - 1U,
            &submit_time_us);
        buffer_held[buffer_id] = false;
        total_submit_time_us += submit_time_us;
        if (ret != ESP_OK) {
            break;
        }
    }

    for (uint8_t i = 0U; i < ST7789_STREAM_STRIP_COUNT; ++i) {
        if (buffer_held[i]) {
            st7789_release_buffer(display, i);
        }
    }

    portENTER_CRITICAL(&display->stats_lock);
    display->stats.last_convert_time_us = total_convert_time_us;
    if (total_convert_time_us > display->stats.max_convert_time_us) {
        display->stats.max_convert_time_us = total_convert_time_us;
    }
    display->stats.last_submit_time_us = total_submit_time_us;
    if (total_submit_time_us > display->stats.max_submit_time_us) {
        display->stats.max_submit_time_us = total_submit_time_us;
    }
    if (ret == ESP_OK) {
        display->stats.submitted_frames++;
    } else {
        display->stats.busy = false;
    }
    portEXIT_CRITICAL(&display->stats_lock);
    return ret;
}

static esp_err_t st7789_wait_semaphore(SemaphoreHandle_t semaphore,
                                       TickType_t wait_ticks)
{
    if (semaphore == NULL) {
        return ESP_OK;
    }
    if (xSemaphoreTake(semaphore, wait_ticks) != pdPASS) {
        return ESP_ERR_TIMEOUT;
    }
    (void)xSemaphoreGive(semaphore);
    return ESP_OK;
}

esp_err_t st7789_display_wait_idle(st7789_display_handle_t *display,
                                  uint32_t timeout_ms)
{
    if (display == NULL) {
        return ESP_OK;
    }
    const TickType_t wait_ticks = timeout_ms == UINT32_MAX
                                      ? portMAX_DELAY
                                      : pdMS_TO_TICKS(timeout_ms);
    if (!display->stream_prepared) {
        return st7789_wait_semaphore(display->preflight_free, wait_ticks);
    }

    bool acquired[ST7789_STREAM_STRIP_COUNT] = {false};
    for (uint8_t i = 0U; i < ST7789_STREAM_STRIP_COUNT; ++i) {
        if (xSemaphoreTake(display->strips[i].free, wait_ticks) != pdPASS) {
            for (uint8_t release = 0U; release < i; ++release) {
                if (acquired[release]) {
                    (void)xSemaphoreGive(display->strips[release].free);
                }
            }
            return ESP_ERR_TIMEOUT;
        }
        acquired[i] = true;
    }
    for (uint8_t i = 0U; i < ST7789_STREAM_STRIP_COUNT; ++i) {
        (void)xSemaphoreGive(display->strips[i].free);
    }
    return ESP_OK;
}

void st7789_display_reset_stats(st7789_display_handle_t *display)
{
    if (display == NULL) {
        return;
    }
    portENTER_CRITICAL(&display->stats_lock);
    const bool busy = display->stats.busy;
    display->stats = (st7789_display_stats_t) {
        .frame_buffer_size = display->workspace != NULL
                                 ? display->workspace_size
                                 : 0U,
        .strip_buffer_size = display->strip_buffer_size,
        .strip_height = ST7789_STREAM_STRIP_HEIGHT,
        .strip_buffer_count = ST7789_STREAM_STRIP_COUNT,
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
