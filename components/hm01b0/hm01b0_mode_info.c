#include "esp_check.h"
#include "hm01b0.h"

static const char *TAG = "hm01b0_mode_info";

esp_err_t hm01b0_get_mode_info(hm01b0_mode_t mode,
                               hm01b0_mode_info_t *info)
{
    ESP_RETURN_ON_FALSE(info != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "mode information output is NULL");

    switch (mode) {
    case HM01B0_SENSOR_MODE_FULL:
        *info = (hm01b0_mode_info_t) {
            .transport_width = 324U,
            .transport_height = 324U,
            .sensor_valid = { .x = 2U, .y = 2U,
                              .width = 320U, .height = 320U },
            .standard = { .x = 2U, .y = 2U,
                          .width = 320U, .height = 320U },
        };
        break;
    case HM01B0_SENSOR_MODE_QVGA:
        *info = (hm01b0_mode_info_t) {
            .transport_width = 324U,
            .transport_height = 244U,
            .sensor_valid = { .x = 2U, .y = 0U,
                              .width = 320U, .height = 244U },
            .standard = { .x = 2U, .y = 2U,
                          .width = 320U, .height = 240U },
        };
        break;
    case HM01B0_SENSOR_MODE_QQVGA:
        *info = (hm01b0_mode_info_t) {
            .transport_width = 162U,
            .transport_height = 122U,
            .sensor_valid = { .x = 1U, .y = 0U,
                              .width = 160U, .height = 122U },
            .standard = { .x = 1U, .y = 1U,
                          .width = 160U, .height = 120U },
        };
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}
