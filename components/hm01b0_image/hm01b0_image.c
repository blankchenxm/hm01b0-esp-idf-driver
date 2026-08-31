#include "hm01b0_image.h"

#include <stdbool.h>

#include "esp_check.h"

#define HM01B0_RGB565_BYTES_PER_PIXEL 2U

static const char *TAG = "hm01b0_image";
static uint16_t s_gray_rgb565[256];
static bool s_gray_rgb565_ready;

typedef enum {
    HM01B0_CFA_RED = 0,
    HM01B0_CFA_GREEN,
    HM01B0_CFA_BLUE,
} hm01b0_cfa_color_t;

static bool hm01b0_image_is_bayer(hm01b0_pixel_format_t format)
{
    return format >= HM01B0_PIXEL_FORMAT_BAYER_GRBG8 &&
           format <= HM01B0_PIXEL_FORMAT_BAYER_GBRG8;
}

static inline hm01b0_cfa_color_t hm01b0_cfa_color(
    hm01b0_pixel_format_t format,
    uint16_t x,
    uint16_t y)
{
    const bool odd_x = (x & 1U) != 0U;
    const bool odd_y = (y & 1U) != 0U;
    switch (format) {
    case HM01B0_PIXEL_FORMAT_BAYER_GRBG8:
        return odd_y ? (odd_x ? HM01B0_CFA_GREEN : HM01B0_CFA_BLUE)
                     : (odd_x ? HM01B0_CFA_RED : HM01B0_CFA_GREEN);
    case HM01B0_PIXEL_FORMAT_BAYER_RGGB8:
        return odd_y ? (odd_x ? HM01B0_CFA_BLUE : HM01B0_CFA_GREEN)
                     : (odd_x ? HM01B0_CFA_GREEN : HM01B0_CFA_RED);
    case HM01B0_PIXEL_FORMAT_BAYER_BGGR8:
        return odd_y ? (odd_x ? HM01B0_CFA_RED : HM01B0_CFA_GREEN)
                     : (odd_x ? HM01B0_CFA_GREEN : HM01B0_CFA_BLUE);
    case HM01B0_PIXEL_FORMAT_BAYER_GBRG8:
        return odd_y ? (odd_x ? HM01B0_CFA_GREEN : HM01B0_CFA_RED)
                     : (odd_x ? HM01B0_CFA_BLUE : HM01B0_CFA_GREEN);
    default:
        return HM01B0_CFA_GREEN;
    }
}

static inline uint8_t hm01b0_average_2(uint8_t first, uint8_t second)
{
    return (uint8_t)(((uint16_t)first + second + 1U) >> 1U);
}

static inline uint8_t hm01b0_average_4(uint8_t first,
                                       uint8_t second,
                                       uint8_t third,
                                       uint8_t fourth)
{
    return (uint8_t)(((uint16_t)first + second + third + fourth + 2U) >> 2U);
}

/* Fast path for pixels with a complete one-pixel Bayer neighborhood. */
static inline void hm01b0_demosaic_interior_pixel(
    const hm01b0_raw8_image_t *source,
    uint16_t x,
    uint16_t y,
    uint8_t *red,
    uint8_t *green,
    uint8_t *blue)
{
    const uint8_t *previous = source->data + (size_t)(y - 1U) * source->stride;
    const uint8_t *current = source->data + (size_t)y * source->stride;
    const uint8_t *next = source->data + (size_t)(y + 1U) * source->stride;
    const uint16_t absolute_x = (uint16_t)(source->origin_x + x);
    const uint16_t absolute_y = (uint16_t)(source->origin_y + y);
    const hm01b0_cfa_color_t color = hm01b0_cfa_color(
        source->pixel_format, absolute_x, absolute_y);

    if (color == HM01B0_CFA_RED) {
        *red = current[x];
        *green = hm01b0_average_4(current[x - 1U], current[x + 1U],
                                  previous[x], next[x]);
        *blue = hm01b0_average_4(previous[x - 1U], previous[x + 1U],
                                 next[x - 1U], next[x + 1U]);
        return;
    }
    if (color == HM01B0_CFA_BLUE) {
        *blue = current[x];
        *green = hm01b0_average_4(current[x - 1U], current[x + 1U],
                                  previous[x], next[x]);
        *red = hm01b0_average_4(previous[x - 1U], previous[x + 1U],
                                next[x - 1U], next[x + 1U]);
        return;
    }

    *green = current[x];
    const hm01b0_cfa_color_t horizontal_color = hm01b0_cfa_color(
        source->pixel_format, (uint16_t)(absolute_x - 1U), absolute_y);
    if (horizontal_color == HM01B0_CFA_RED) {
        *red = hm01b0_average_2(current[x - 1U], current[x + 1U]);
        *blue = hm01b0_average_2(previous[x], next[x]);
    } else {
        *blue = hm01b0_average_2(current[x - 1U], current[x + 1U]);
        *red = hm01b0_average_2(previous[x], next[x]);
    }
}

static uint8_t hm01b0_average_color(
    const hm01b0_raw8_image_t *source,
    int32_t x,
    int32_t y,
    hm01b0_cfa_color_t wanted,
    bool diagonal)
{
    static const int8_t axial_offsets[4][2] = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    };
    static const int8_t diagonal_offsets[4][2] = {
        {-1, -1}, {1, -1}, {-1, 1}, {1, 1},
    };
    const int8_t (*offsets)[2] = diagonal ? diagonal_offsets : axial_offsets;
    uint32_t sum = 0U;
    uint32_t count = 0U;

    for (size_t i = 0U; i < 4U; ++i) {
        const int32_t sample_x = x + offsets[i][0];
        const int32_t sample_y = y + offsets[i][1];
        if (sample_x < 0 || sample_y < 0 ||
            sample_x >= source->width || sample_y >= source->height) {
            continue;
        }
        const hm01b0_cfa_color_t color = hm01b0_cfa_color(
            source->pixel_format,
            (uint16_t)(source->origin_x + sample_x),
            (uint16_t)(source->origin_y + sample_y));
        if (color == wanted) {
            sum += source->data[(size_t)sample_y * source->stride + sample_x];
            count++;
        }
    }
    return count > 0U ? (uint8_t)((sum + count / 2U) / count)
                      : source->data[(size_t)y * source->stride + x];
}

static void hm01b0_demosaic_pixel(const hm01b0_raw8_image_t *source,
                                  uint16_t x,
                                  uint16_t y,
                                  uint8_t *red,
                                  uint8_t *green,
                                  uint8_t *blue)
{
    const uint8_t value = source->data[(size_t)y * source->stride + x];
    const hm01b0_cfa_color_t color = hm01b0_cfa_color(
        source->pixel_format,
        (uint16_t)(source->origin_x + x),
        (uint16_t)(source->origin_y + y));

    if (color == HM01B0_CFA_RED) {
        *red = value;
        *green = hm01b0_average_color(source, x, y, HM01B0_CFA_GREEN,
                                      false);
        *blue = hm01b0_average_color(source, x, y, HM01B0_CFA_BLUE, true);
    } else if (color == HM01B0_CFA_BLUE) {
        *blue = value;
        *green = hm01b0_average_color(source, x, y, HM01B0_CFA_GREEN,
                                      false);
        *red = hm01b0_average_color(source, x, y, HM01B0_CFA_RED, true);
    } else {
        *green = value;
        *red = hm01b0_average_color(source, x, y, HM01B0_CFA_RED, false);
        *blue = hm01b0_average_color(source, x, y, HM01B0_CFA_BLUE, false);
    }
}

static void hm01b0_store_rgb565_be(uint8_t *destination,
                                   uint8_t red,
                                   uint8_t green,
                                   uint8_t blue)
{
    const uint16_t rgb565 = (uint16_t)(((uint16_t)(red >> 3U) << 11U) |
                                      ((uint16_t)(green >> 2U) << 5U) |
                                      (blue >> 3U));
    destination[0] = (uint8_t)(rgb565 >> 8U);
    destination[1] = (uint8_t)rgb565;
}

static void hm01b0_init_gray_rgb565(void)
{
    if (s_gray_rgb565_ready) {
        return;
    }
    for (uint16_t gray = 0U; gray < 256U; ++gray) {
        s_gray_rgb565[gray] =
            (uint16_t)(((gray >> 3U) << 11U) |
                       ((gray >> 2U) << 5U) |
                       (gray >> 3U));
    }
    s_gray_rgb565_ready = true;
}

static esp_err_t hm01b0_image_validate(const hm01b0_raw8_image_t *source)
{
    ESP_RETURN_ON_FALSE(source != NULL && source->data != NULL &&
                            source->width > 0U && source->height > 0U &&
                            source->stride >= source->width,
                        ESP_ERR_INVALID_ARG, TAG, "invalid RAW8 source");
    ESP_RETURN_ON_FALSE(
        source->pixel_format == HM01B0_PIXEL_FORMAT_MONO8 ||
            hm01b0_image_is_bayer(source->pixel_format),
        ESP_ERR_NOT_SUPPORTED, TAG, "unsupported RAW8 pixel format");
    return ESP_OK;
}

static void hm01b0_image_convert_row_unchecked(
    const hm01b0_raw8_image_t *source,
    uint16_t source_x,
    uint16_t source_y,
    uint16_t width,
    uint8_t *destination)
{
    if (source->pixel_format == HM01B0_PIXEL_FORMAT_MONO8) {
        hm01b0_init_gray_rgb565();
    }

    const bool fast_bayer_row =
        source->pixel_format != HM01B0_PIXEL_FORMAT_MONO8 &&
        source_y > 0U && source_y + 1U < source->height;
    for (uint16_t column = 0U; column < width; ++column) {
        const uint16_t x = (uint16_t)(source_x + column);
        if (source->pixel_format == HM01B0_PIXEL_FORMAT_MONO8) {
            const uint16_t rgb565 = s_gray_rgb565[
                source->data[(size_t)source_y * source->stride + x]];
            destination[(size_t)column * 2U] = (uint8_t)(rgb565 >> 8U);
            destination[(size_t)column * 2U + 1U] = (uint8_t)rgb565;
            continue;
        }

        uint8_t red = 0U;
        uint8_t green = 0U;
        uint8_t blue = 0U;
        if (fast_bayer_row && x > 0U && x + 1U < source->width) {
            hm01b0_demosaic_interior_pixel(source, x, source_y,
                                            &red, &green, &blue);
        } else {
            hm01b0_demosaic_pixel(source, x, source_y,
                                  &red, &green, &blue);
        }
        hm01b0_store_rgb565_be(destination + (size_t)column * 2U,
                               red, green, blue);
    }
}

esp_err_t hm01b0_image_convert_row_to_rgb565_be(
    const hm01b0_raw8_image_t *source,
    uint16_t source_x,
    uint16_t source_y,
    uint16_t width,
    uint8_t *destination,
    size_t destination_size)
{
    ESP_RETURN_ON_ERROR(hm01b0_image_validate(source), TAG,
                        "invalid conversion source");
    ESP_RETURN_ON_FALSE(destination != NULL && width > 0U &&
                            source_x < source->width &&
                            source_y < source->height &&
                            width <= source->width - source_x &&
                            destination_size >=
                                (size_t)width *
                                    HM01B0_RGB565_BYTES_PER_PIXEL,
                        ESP_ERR_INVALID_ARG, TAG,
                        "invalid conversion row or destination");

    hm01b0_image_convert_row_unchecked(
        source, source_x, source_y, width, destination);
    return ESP_OK;
}

esp_err_t hm01b0_image_convert_to_rgb565_be(
    const hm01b0_raw8_image_t *source,
    hm01b0_rect_t crop,
    uint8_t *destination,
    size_t destination_size)
{
    ESP_RETURN_ON_ERROR(hm01b0_image_validate(source), TAG,
                        "invalid conversion source");
    const size_t required = (size_t)crop.width * crop.height *
                            HM01B0_RGB565_BYTES_PER_PIXEL;
    ESP_RETURN_ON_FALSE(destination != NULL && crop.width > 0U &&
                            crop.height > 0U && crop.x < source->width &&
                            crop.y < source->height &&
                            crop.width <= source->width - crop.x &&
                            crop.height <= source->height - crop.y &&
                            destination_size >= required,
                        ESP_ERR_INVALID_ARG, TAG,
                        "invalid conversion crop or destination");

    for (uint16_t row = 0U; row < crop.height; ++row) {
        hm01b0_image_convert_row_unchecked(
            source, crop.x, (uint16_t)(crop.y + row), crop.width,
            destination + (size_t)row * crop.width * 2U);
    }
    return ESP_OK;
}
