#include <string.h>

#include "esp_check.h"
#include "hm01b0_frame_ops.h"

static const char *TAG = "hm01b0_frame_ops";

esp_err_t hm01b0_frame_crop_raw8(const uint8_t *source,
                                 uint16_t source_width,
                                 uint16_t source_height,
                                 size_t source_stride,
                                 hm01b0_frame_rect_t crop,
                                 uint8_t *destination,
                                 size_t destination_size,
                                 size_t destination_stride)
{
    ESP_RETURN_ON_FALSE(source != NULL && destination != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "frame buffer is NULL");
    ESP_RETURN_ON_FALSE(source_width > 0U && source_height > 0U &&
                        source_stride >= source_width,
                        ESP_ERR_INVALID_ARG, TAG, "invalid source geometry");
    ESP_RETURN_ON_FALSE(crop.width > 0U && crop.height > 0U &&
                        crop.x < source_width && crop.y < source_height &&
                        crop.width <= source_width - crop.x &&
                        crop.height <= source_height - crop.y,
                        ESP_ERR_INVALID_ARG, TAG, "crop is outside source");
    ESP_RETURN_ON_FALSE(destination_stride >= crop.width,
                        ESP_ERR_INVALID_ARG, TAG,
                        "destination stride is too small");

    const size_t required =
        (size_t)(crop.height - 1U) * destination_stride + crop.width;
    ESP_RETURN_ON_FALSE(destination_size >= required,
                        ESP_ERR_INVALID_SIZE, TAG,
                        "destination buffer is too small");

    for (uint16_t y = 0; y < crop.height; ++y) {
        const size_t source_offset =
            (size_t)(crop.y + y) * source_stride + crop.x;
        const size_t destination_offset = (size_t)y * destination_stride;
        memcpy(destination + destination_offset,
               source + source_offset, crop.width);
    }
    return ESP_OK;
}
