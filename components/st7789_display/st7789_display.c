#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "st7789_display.h"

#define ST7789_SWRESET  0x01U
#define ST7789_SLPOUT   0x11U
#define ST7789_NORON    0x13U
#define ST7789_INVON    0x21U
#define ST7789_DISPON   0x29U
#define ST7789_CASET    0x2AU
#define ST7789_RASET    0x2BU
#define ST7789_RAMWR    0x2CU
#define ST7789_MADCTL   0x36U
#define ST7789_COLMOD   0x3AU
#define ST7789_FRMCTR2  0xB2U
#define ST7789_VMCTR1   0xC5U
#define ST7789_GMCTRP1  0xE0U
#define ST7789_GMCTRN1  0xE1U

#define ST7789_MAX_TRANSFER_SIZE 1024U
#define ST7789_GRAY_CHUNK_SIZE    512U

struct st7789_display {
    st7789_display_config_t config;
    spi_device_handle_t spi;
    bool bus_initialized;
};

static const char *TAG = "st7789_display";

static esp_err_t st7789_write_command(st7789_display_handle_t *display,
                                      uint8_t command)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(display->config.dc_gpio, 0), TAG,
                        "failed to select command mode");
    spi_transaction_t transaction = {
        .length = 8U,
        .tx_buffer = &command,
    };
    return spi_device_polling_transmit(display->spi, &transaction);
}

static esp_err_t st7789_write_data(st7789_display_handle_t *display,
                                   const uint8_t *data,
                                   size_t size)
{
    if (size == 0U) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(gpio_set_level(display->config.dc_gpio, 1), TAG,
                        "failed to select data mode");
    spi_transaction_t transaction = {
        .length = size * 8U,
        .tx_buffer = data,
    };
    return spi_device_polling_transmit(display->spi, &transaction);
}

static esp_err_t st7789_write_byte(st7789_display_handle_t *display,
                                   uint8_t value)
{
    return st7789_write_data(display, &value, 1U);
}

static void st7789_hardware_reset(st7789_display_handle_t *display)
{
    (void)gpio_set_level(display->config.reset_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(10U));
    (void)gpio_set_level(display->config.reset_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(10U));
    (void)gpio_set_level(display->config.reset_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(120U));
}

static esp_err_t st7789_initialize_panel(st7789_display_handle_t *display)
{
    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_SWRESET), TAG,
                        "software reset command failed");
    vTaskDelay(pdMS_TO_TICKS(150U));
    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_SLPOUT), TAG,
                        "sleep-out command failed");
    vTaskDelay(pdMS_TO_TICKS(120U));

    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_COLMOD), TAG,
                        "color-mode command failed");
    ESP_RETURN_ON_ERROR(st7789_write_byte(display, 0x55U), TAG,
                        "color-mode data failed");
    vTaskDelay(pdMS_TO_TICKS(10U));

    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_MADCTL), TAG,
                        "MADCTL command failed");
    ESP_RETURN_ON_ERROR(st7789_write_byte(display, 0x00U), TAG,
                        "MADCTL data failed");

    static const uint8_t porch[] = {0x0CU, 0x0CU, 0x00U, 0x33U, 0x33U};
    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_FRMCTR2), TAG,
                        "porch command failed");
    ESP_RETURN_ON_ERROR(st7789_write_data(display, porch, sizeof(porch)), TAG,
                        "porch data failed");

    ESP_RETURN_ON_ERROR(st7789_write_command(display, 0xB7U), TAG,
                        "gate-control command failed");
    ESP_RETURN_ON_ERROR(st7789_write_byte(display, 0x35U), TAG,
                        "gate-control data failed");
    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_VMCTR1), TAG,
                        "VCOM command failed");
    ESP_RETURN_ON_ERROR(st7789_write_byte(display, 0x19U), TAG,
                        "VCOM data failed");
    ESP_RETURN_ON_ERROR(st7789_write_command(display, 0xC0U), TAG,
                        "LCM-control command failed");
    ESP_RETURN_ON_ERROR(st7789_write_byte(display, 0x2CU), TAG,
                        "LCM-control data failed");
    ESP_RETURN_ON_ERROR(st7789_write_command(display, 0xC2U), TAG,
                        "VDV/VRH command failed");
    ESP_RETURN_ON_ERROR(st7789_write_byte(display, 0x01U), TAG,
                        "VDV/VRH data failed");
    ESP_RETURN_ON_ERROR(st7789_write_command(display, 0xC3U), TAG,
                        "VRH command failed");
    ESP_RETURN_ON_ERROR(st7789_write_byte(display, 0x12U), TAG,
                        "VRH data failed");
    ESP_RETURN_ON_ERROR(st7789_write_command(display, 0xC4U), TAG,
                        "VDV command failed");
    ESP_RETURN_ON_ERROR(st7789_write_byte(display, 0x20U), TAG,
                        "VDV data failed");
    ESP_RETURN_ON_ERROR(st7789_write_command(display, 0xC6U), TAG,
                        "frame-rate command failed");
    ESP_RETURN_ON_ERROR(st7789_write_byte(display, 0x0FU), TAG,
                        "frame-rate data failed");

    static const uint8_t power[] = {0xA4U, 0xA1U};
    ESP_RETURN_ON_ERROR(st7789_write_command(display, 0xD0U), TAG,
                        "power command failed");
    ESP_RETURN_ON_ERROR(st7789_write_data(display, power, sizeof(power)), TAG,
                        "power data failed");

    static const uint8_t gamma_positive[] = {
        0xD0U, 0x04U, 0x0DU, 0x11U, 0x13U, 0x2BU, 0x3FU,
        0x54U, 0x4CU, 0x18U, 0x0DU, 0x0BU, 0x1FU, 0x23U,
    };
    static const uint8_t gamma_negative[] = {
        0xD0U, 0x04U, 0x0CU, 0x11U, 0x13U, 0x2CU, 0x3FU,
        0x44U, 0x51U, 0x2FU, 0x1FU, 0x1FU, 0x20U, 0x23U,
    };
    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_GMCTRP1), TAG,
                        "positive-gamma command failed");
    ESP_RETURN_ON_ERROR(st7789_write_data(display, gamma_positive,
                                           sizeof(gamma_positive)),
                        TAG, "positive-gamma data failed");
    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_GMCTRN1), TAG,
                        "negative-gamma command failed");
    ESP_RETURN_ON_ERROR(st7789_write_data(display, gamma_negative,
                                           sizeof(gamma_negative)),
                        TAG, "negative-gamma data failed");

    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_INVON), TAG,
                        "inversion command failed");
    vTaskDelay(pdMS_TO_TICKS(10U));
    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_NORON), TAG,
                        "normal-mode command failed");
    vTaskDelay(pdMS_TO_TICKS(10U));
    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_DISPON), TAG,
                        "display-on command failed");
    vTaskDelay(pdMS_TO_TICKS(10U));
    return ESP_OK;
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

    const gpio_config_t control_gpio = {
        .pin_bit_mask = (1ULL << config->dc_gpio) |
                        (1ULL << config->reset_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&control_gpio);
    if (ret != ESP_OK) {
        goto fail;
    }

    const spi_bus_config_t bus_config = {
        .mosi_io_num = config->mosi_gpio,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = config->clock_gpio,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = ST7789_MAX_TRANSFER_SIZE,
    };
    ret = spi_bus_initialize(config->spi_host, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        goto fail;
    }
    display->bus_initialized = true;

    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = (int)config->clock_speed_hz,
        .mode = 0,
        .spics_io_num = config->cs_gpio,
        .queue_size = 7,
        .flags = SPI_DEVICE_NO_DUMMY,
    };
    ret = spi_bus_add_device(config->spi_host, &device_config, &display->spi);
    if (ret != ESP_OK) {
        goto fail;
    }

    st7789_hardware_reset(display);
    ret = st7789_initialize_panel(display);
    if (ret != ESP_OK) {
        goto fail;
    }

    ESP_LOGI(TAG, "ST7789 initialized (%ux%u, SPI=%" PRIu32 " Hz)",
             config->width, config->height, config->clock_speed_hz);
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
    esp_err_t result = ESP_OK;
    if (display->spi != NULL) {
        result = spi_bus_remove_device(display->spi);
    }
    if (display->bus_initialized) {
        const esp_err_t ret = spi_bus_free(display->config.spi_host);
        if (result == ESP_OK) {
            result = ret;
        }
    }
    free(display);
    return result;
}

static esp_err_t st7789_set_window(st7789_display_handle_t *display,
                                   uint16_t x0,
                                   uint16_t y0,
                                   uint16_t x1,
                                   uint16_t y1)
{
    const uint8_t columns[] = {
        (uint8_t)(x0 >> 8), (uint8_t)x0,
        (uint8_t)(x1 >> 8), (uint8_t)x1,
    };
    const uint8_t rows[] = {
        (uint8_t)(y0 >> 8), (uint8_t)y0,
        (uint8_t)(y1 >> 8), (uint8_t)y1,
    };
    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_CASET), TAG,
                        "column command failed");
    ESP_RETURN_ON_ERROR(st7789_write_data(display, columns, sizeof(columns)),
                        TAG, "column data failed");
    ESP_RETURN_ON_ERROR(st7789_write_command(display, ST7789_RASET), TAG,
                        "row command failed");
    ESP_RETURN_ON_ERROR(st7789_write_data(display, rows, sizeof(rows)), TAG,
                        "row data failed");
    return st7789_write_command(display, ST7789_RAMWR);
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
    ESP_RETURN_ON_FALSE(width > 0U && height > 0U &&
                        x < display->config.width &&
                        y < display->config.height &&
                        width <= display->config.width - x &&
                        height <= display->config.height - y,
                        ESP_ERR_INVALID_ARG, TAG,
                        "image is outside the display");
    ESP_RETURN_ON_ERROR(st7789_set_window(
                            display, x, y,
                            (uint16_t)(x + width - 1U),
                            (uint16_t)(y + height - 1U)),
                        TAG, "failed to set image window");

    uint8_t chunk[ST7789_GRAY_CHUNK_SIZE];
    uint32_t remaining = (uint32_t)width * height;
    uint32_t source_index = 0U;
    while (remaining > 0U) {
        const uint32_t pixels =
            remaining > ST7789_GRAY_CHUNK_SIZE / 2U
                ? ST7789_GRAY_CHUNK_SIZE / 2U
                : remaining;
        for (uint32_t i = 0; i < pixels; ++i) {
            const uint8_t gray = data[source_index + i];
            const uint16_t rgb565 =
                (uint16_t)(((uint16_t)(gray >> 3U) << 11U) |
                           ((uint16_t)(gray >> 2U) << 5U) |
                           (gray >> 3U));
            chunk[i * 2U] = (uint8_t)(rgb565 >> 8U);
            chunk[i * 2U + 1U] = (uint8_t)rgb565;
        }
        ESP_RETURN_ON_ERROR(
            st7789_write_data(display, chunk, pixels * 2U), TAG,
            "image transfer failed");
        source_index += pixels;
        remaining -= pixels;
    }
    return ESP_OK;
}
