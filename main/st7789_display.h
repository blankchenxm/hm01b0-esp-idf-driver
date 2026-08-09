#ifndef __ST7789_DISPLAY_H__
#define __ST7789_DISPLAY_H__

#include <stdint.h>
#include "esp_err.h"

// =======================================================
// 屏幕参数
// =======================================================
#define ST7789_WIDTH   240
#define ST7789_HEIGHT  240

// =======================================================
// 引脚定义 (避免与HM01B0冲突)
// =======================================================
#define ST7789_PIN_SCL   35   // SPI CLK
#define ST7789_PIN_SDA   36   // SPI MOSI
#define ST7789_PIN_RES   37   // Reset
#define ST7789_PIN_DC    38   // Data/Command
#define ST7789_PIN_CS    39   // Chip Select
// BLK (背光) 直接接3.3V常亮

// =======================================================
// 常用颜色定义 (RGB565格式)
// =======================================================
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_ORANGE  0xFD20
#define COLOR_GRAY    0x8410

// =======================================================
// 函数声明
// =======================================================

/**
 * @brief 初始化ST7789显示屏
 */
esp_err_t st7789_init(void);

/**
 * @brief 填充整个屏幕为单一颜色
 * @param color RGB565颜色值
 */
void st7789_fill_screen(uint16_t color);

/**
 * @brief 设置绘图窗口
 * @param x0, y0 起始坐标
 * @param x1, y1 结束坐标
 */
void st7789_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/**
 * @brief 画一个像素点
 * @param x, y 坐标
 * @param color RGB565颜色值
 */
void st7789_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief 填充矩形区域
 * @param x, y 起始坐标
 * @param w, h 宽度和高度
 * @param color RGB565颜色值
 */
void st7789_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/**
 * @brief 显示RGB565图像数据
 * @param x, y 起始坐标
 * @param w, h 图像宽高
 * @param data RGB565图像数据
 */
void st7789_draw_image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);

/**
 * @brief 显示8位灰度图像 (转换为RGB565显示)
 * @param x, y 起始坐标
 * @param w, h 图像宽高
 * @param data 8位灰度数据
 */
void st7789_draw_gray_image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *data);

/**
 * @brief RGB888转RGB565
 */
uint16_t st7789_rgb565(uint8_t r, uint8_t g, uint8_t b);

#endif // __ST7789_DISPLAY_H__
