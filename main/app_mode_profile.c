#include "app_mode_profile.h"

#include "esp_check.h"

static const char *TAG = "app_mode_profile";

static const char *app_mode_name(hm01b0_mode_t mode)
{
    switch (mode) {
    case HM01B0_SENSOR_MODE_FULL:
        return "FULL";
    case HM01B0_SENSOR_MODE_QVGA:
        return "QVGA";
    case HM01B0_SENSOR_MODE_QQVGA:
        return "QQVGA";
    default:
        return NULL;
    }
}

esp_err_t app_mode_profile_build(hm01b0_mode_t mode,
                                 uint16_t display_width,
                                 uint16_t display_height,
                                 app_mode_profile_t *profile)
{
    ESP_RETURN_ON_FALSE(profile != NULL && display_width > 0U &&
                            display_height > 0U,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    const char *name = app_mode_name(mode);
    ESP_RETURN_ON_FALSE(name != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "invalid sensor mode");

    hm01b0_mode_info_t sensor = {0};
    ESP_RETURN_ON_ERROR(hm01b0_get_mode_info(mode, &sensor), TAG,
                        "failed to obtain sensor geometry");
    ESP_RETURN_ON_FALSE(
        sensor.standard.width > 0U && sensor.standard.height > 0U &&
            sensor.standard.x < sensor.transport_width &&
            sensor.standard.y < sensor.transport_height &&
            sensor.standard.width <=
                sensor.transport_width - sensor.standard.x &&
            sensor.standard.height <=
                sensor.transport_height - sensor.standard.y,
        ESP_ERR_INVALID_SIZE, TAG, "invalid standard sensor area");

    const uint16_t image_width = sensor.standard.width < display_width
                                     ? sensor.standard.width
                                     : display_width;
    const uint16_t image_height = sensor.standard.height < display_height
                                      ? sensor.standard.height
                                      : display_height;

    *profile = (app_mode_profile_t) {
        .name = name,
        .sensor = sensor,
        .display_source = {
            .x = (uint16_t)(sensor.standard.x +
                            (sensor.standard.width - image_width) / 2U),
            .y = (uint16_t)(sensor.standard.y +
                            (sensor.standard.height - image_height) / 2U),
            .width = image_width,
            .height = image_height,
        },
        .display_x = (uint16_t)((display_width - image_width) / 2U),
        .display_y = (uint16_t)((display_height - image_height) / 2U),
    };
    return ESP_OK;
}
