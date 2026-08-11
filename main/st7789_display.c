#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "st7789_display.h"

static const char *TAG = "ST7789";

// SPI句柄
static spi_device_handle_t spi_handle = NULL;

// =======================================================
// ST7789 命令定义
// =======================================================
#define ST7789_NOP       0x00
#define ST7789_SWRESET   0x01
#define ST7789_SLPIN     0x10
#define ST7789_SLPOUT    0x11
#define ST7789_NORON     0x13
#define ST7789_INVOFF    0x20
#define ST7789_INVON     0x21
#define ST7789_DISPOFF   0x28
#define ST7789_DISPON    0x29
#define ST7789_CASET     0x2A
#define ST7789_RASET     0x2B
#define ST7789_RAMWR     0x2C
#define ST7789_COLMOD    0x3A
#define ST7789_MADCTL    0x36
#define ST7789_FRMCTR2   0xB2
#define ST7789_VMCTR1    0xC5
#define ST7789_GMCTRP1   0xE0
#define ST7789_GMCTRN1   0xE1

// MADCTL参数
#define MADCTL_RGB 0x00
#define ST7789_MAX_TRANSFER_SIZE 1024

// =======================================================
// 底层SPI发送函数
// =======================================================
static void st7789_write_cmd(uint8_t cmd)
{
    gpio_set_level(ST7789_PIN_DC, 0);  // 命令模式
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_polling_transmit(spi_handle, &t);
}

static void st7789_write_data(const uint8_t *data, size_t len)
{
    if (len == 0) return;
    gpio_set_level(ST7789_PIN_DC, 1);  // 数据模式
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    spi_device_polling_transmit(spi_handle, &t);
}

static void st7789_write_data_byte(uint8_t data)
{
    st7789_write_data(&data, 1);
}

// =======================================================
// 硬件复位
// =======================================================
static void st7789_hw_reset(void)
{
    gpio_set_level(ST7789_PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(ST7789_PIN_RES, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(ST7789_PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

// =======================================================
// 初始化ST7789
// =======================================================
esp_err_t st7789_init(void)
{
    ESP_LOGI(TAG, "Initializing ST7789...");

    // 配置GPIO (DC和RES引脚，BLK已直接接3.3V)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ST7789_PIN_DC) | (1ULL << ST7789_PIN_RES),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure control GPIOs: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    // 配置SPI总线
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = ST7789_PIN_SDA,
        .miso_io_num = -1,
        .sclk_io_num = ST7789_PIN_SCL,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ST7789_MAX_TRANSFER_SIZE,
    };
    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize SPI bus: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    // 配置SPI设备
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 40 * 1000 * 1000,  // 40MHz
        .mode = 0,
        .spics_io_num = ST7789_PIN_CS,
        .queue_size = 7,
        .flags = SPI_DEVICE_NO_DUMMY,
    };
    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to add SPI device: %s", esp_err_to_name(ret));
        (void)spi_bus_free(SPI2_HOST);
        return ret;
    }

    // 硬件复位
    st7789_hw_reset();

    // 软件复位
    st7789_write_cmd(ST7789_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));

    // 退出睡眠模式
    st7789_write_cmd(ST7789_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));

    // 设置颜色格式: 16bit RGB565
    st7789_write_cmd(ST7789_COLMOD);
    st7789_write_data_byte(0x55);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 设置内存数据访问控制
    st7789_write_cmd(ST7789_MADCTL);
    st7789_write_data_byte(MADCTL_RGB);

    // Porch设置
    st7789_write_cmd(ST7789_FRMCTR2);
    uint8_t porch[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    st7789_write_data(porch, sizeof(porch));

    // Gate控制
    st7789_write_cmd(0xB7);
    st7789_write_data_byte(0x35);

    // VCOM设置
    st7789_write_cmd(ST7789_VMCTR1);
    st7789_write_data_byte(0x19);

    // LCM控制
    st7789_write_cmd(0xC0);
    st7789_write_data_byte(0x2C);

    // VDV和VRH命令使能
    st7789_write_cmd(0xC2);
    st7789_write_data_byte(0x01);

    // VRH设置
    st7789_write_cmd(0xC3);
    st7789_write_data_byte(0x12);

    // VDV设置
    st7789_write_cmd(0xC4);
    st7789_write_data_byte(0x20);

    // 帧率控制
    st7789_write_cmd(0xC6);
    st7789_write_data_byte(0x0F);  // 60Hz

    // 电源控制
    st7789_write_cmd(0xD0);
    uint8_t pwr[] = {0xA4, 0xA1};
    st7789_write_data(pwr, sizeof(pwr));

    // 正极性Gamma校正
    st7789_write_cmd(ST7789_GMCTRP1);
    uint8_t gamma_pos[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
    st7789_write_data(gamma_pos, sizeof(gamma_pos));

    // 负极性Gamma校正
    st7789_write_cmd(ST7789_GMCTRN1);
    uint8_t gamma_neg[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
    st7789_write_data(gamma_neg, sizeof(gamma_neg));

    // 反转显示开启 (部分ST7789模块需要)
    st7789_write_cmd(ST7789_INVON);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 正常显示模式
    st7789_write_cmd(ST7789_NORON);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 开启显示
    st7789_write_cmd(ST7789_DISPON);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "ST7789 initialized (240x240)");
    return ESP_OK;
}

// =======================================================
// 设置绘图窗口
// =======================================================
void st7789_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    // Column地址设置
    st7789_write_cmd(ST7789_CASET);
    uint8_t col_data[] = {x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF};
    st7789_write_data(col_data, 4);

    // Row地址设置
    st7789_write_cmd(ST7789_RASET);
    uint8_t row_data[] = {y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF};
    st7789_write_data(row_data, 4);

    // 写入RAM
    st7789_write_cmd(ST7789_RAMWR);
}

// =======================================================
// 填充整个屏幕
// =======================================================
void st7789_fill_screen(uint16_t color)
{
    st7789_fill_rect(0, 0, ST7789_WIDTH, ST7789_HEIGHT, color);
}

// =======================================================
// 画点
// =======================================================
void st7789_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= ST7789_WIDTH || y >= ST7789_HEIGHT) return;

    st7789_set_window(x, y, x, y);
    uint8_t data[] = {color >> 8, color & 0xFF};
    st7789_write_data(data, 2);
}

// =======================================================
// 填充矩形
// =======================================================
void st7789_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= ST7789_WIDTH || y >= ST7789_HEIGHT) return;
    if (x + w > ST7789_WIDTH) w = ST7789_WIDTH - x;
    if (y + h > ST7789_HEIGHT) h = ST7789_HEIGHT - y;

    st7789_set_window(x, y, x + w - 1, y + h - 1);

    uint32_t total_pixels = w * h;
    uint8_t color_hi = color >> 8;
    uint8_t color_lo = color & 0xFF;

    // 使用缓冲区批量发送
    #define FILL_BUF_SIZE 512
    uint8_t buf[FILL_BUF_SIZE];
    for (int i = 0; i < FILL_BUF_SIZE; i += 2) {
        buf[i] = color_hi;
        buf[i + 1] = color_lo;
    }

    gpio_set_level(ST7789_PIN_DC, 1);
    uint32_t remaining = total_pixels * 2;
    while (remaining > 0) {
        uint32_t chunk = (remaining > FILL_BUF_SIZE) ? FILL_BUF_SIZE : remaining;
        spi_transaction_t t = {
            .length = chunk * 8,
            .tx_buffer = buf,
        };
        spi_device_polling_transmit(spi_handle, &t);
        remaining -= chunk;
    }
}

// =======================================================
// 显示RGB565图像
// =======================================================
void st7789_draw_image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
    if (x >= ST7789_WIDTH || y >= ST7789_HEIGHT) return;
    if (x + w > ST7789_WIDTH) w = ST7789_WIDTH - x;
    if (y + h > ST7789_HEIGHT) h = ST7789_HEIGHT - y;

    st7789_set_window(x, y, x + w - 1, y + h - 1);

    // RGB565需要字节交换 (大端序)
    uint32_t total_bytes = w * h * 2;
    gpio_set_level(ST7789_PIN_DC, 1);

    // 分块发送
    #define IMG_BUF_SIZE 1024
    uint8_t buf[IMG_BUF_SIZE];
    const uint8_t *src = (const uint8_t *)data;
    uint32_t remaining = total_bytes;
    uint32_t offset = 0;

    while (remaining > 0) {
        uint32_t chunk = (remaining > IMG_BUF_SIZE) ? IMG_BUF_SIZE : remaining;
        // 字节交换
        for (uint32_t i = 0; i < chunk; i += 2) {
            buf[i] = src[offset + i + 1];
            buf[i + 1] = src[offset + i];
        }
        spi_transaction_t t = {
            .length = chunk * 8,
            .tx_buffer = buf,
        };
        spi_device_polling_transmit(spi_handle, &t);
        remaining -= chunk;
        offset += chunk;
    }
}

// =======================================================
// 显示8位灰度图像
// =======================================================
void st7789_draw_gray_image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *data)
{
    if (x >= ST7789_WIDTH || y >= ST7789_HEIGHT) return;
    if (x + w > ST7789_WIDTH) w = ST7789_WIDTH - x;
    if (y + h > ST7789_HEIGHT) h = ST7789_HEIGHT - y;

    st7789_set_window(x, y, x + w - 1, y + h - 1);

    uint32_t total_pixels = w * h;
    gpio_set_level(ST7789_PIN_DC, 1);

    // 分块转换并发送
    #define GRAY_BUF_SIZE 512
    uint8_t buf[GRAY_BUF_SIZE];
    uint32_t remaining = total_pixels;
    uint32_t src_idx = 0;

    while (remaining > 0) {
        uint32_t pixels = (remaining > GRAY_BUF_SIZE / 2) ? GRAY_BUF_SIZE / 2 : remaining;
        for (uint32_t i = 0; i < pixels; i++) {
            uint8_t gray = data[src_idx + i];
            // 灰度转RGB565: R5 G6 B5 都用灰度值
            uint16_t color = ((gray >> 3) << 11) | ((gray >> 2) << 5) | (gray >> 3);
            buf[i * 2] = color >> 8;
            buf[i * 2 + 1] = color & 0xFF;
        }
        spi_transaction_t t = {
            .length = pixels * 2 * 8,
            .tx_buffer = buf,
        };
        spi_device_polling_transmit(spi_handle, &t);
        remaining -= pixels;
        src_idx += pixels;
    }
}

// =======================================================
// RGB888转RGB565
// =======================================================
uint16_t st7789_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}
