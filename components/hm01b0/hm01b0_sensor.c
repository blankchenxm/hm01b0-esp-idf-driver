#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "hm01b0.h"
#include "hm01b0_private.h"
#include "hm01b0_regs.h"
#include "hm01b0_tables.h"

#define HM01B0_RESET_RECOVERY_US 1000U

static const char *TAG = "hm01b0_sensor";

static esp_err_t hm01b0_require_standby(const hm01b0_handle_t *dev)
{
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "device handle is NULL");
    ESP_RETURN_ON_FALSE(dev->state == HM01B0_STATE_STANDBY,
                        ESP_ERR_INVALID_STATE, TAG,
                        "configuration is only allowed in standby");
    return ESP_OK;
}

esp_err_t hm01b0_probe(hm01b0_handle_t *dev, uint16_t *model_id)
{
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "device handle is NULL");

    uint8_t id_high = 0;
    uint8_t id_low = 0;
    ESP_RETURN_ON_ERROR(hm01b0_reg_read(dev, HM01B0_REG_MODEL_ID_H,
                                        &id_high),
                        TAG, "failed to read MODEL_ID_H");
    ESP_RETURN_ON_ERROR(hm01b0_reg_read(dev, HM01B0_REG_MODEL_ID_L,
                                        &id_low),
                        TAG, "failed to read MODEL_ID_L");

    const uint16_t detected_id = ((uint16_t)id_high << 8) | id_low;
    if (model_id != NULL) {
        *model_id = detected_id;
    }

    ESP_LOGI(TAG, "MODEL_ID_H=0x%02X, MODEL_ID_L=0x%02X, MODEL_ID=0x%04X",
             id_high, id_low, detected_id);
    if (detected_id != HM01B0_EXPECTED_MODEL_ID) {
        ESP_LOGE(TAG, "unexpected model ID (expected 0x%04X)",
                 HM01B0_EXPECTED_MODEL_ID);
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

esp_err_t hm01b0_reset(hm01b0_handle_t *dev)
{
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "device handle is NULL");
    ESP_RETURN_ON_ERROR(hm01b0_reg_write(dev, HM01B0_REG_SW_RESET,
                                         HM01B0_SOFTWARE_RESET),
                        TAG, "software reset failed");

    esp_rom_delay_us(HM01B0_RESET_RECOVERY_US);
    dev->state = HM01B0_STATE_STANDBY;
    ESP_LOGI(TAG, "software reset complete; state=STANDBY");
    return ESP_OK;
}

esp_err_t hm01b0_standby(hm01b0_handle_t *dev)
{
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "device handle is NULL");
    ESP_RETURN_ON_ERROR(hm01b0_reg_update_bits(dev,
                                               HM01B0_REG_MODE_SELECT,
                                               HM01B0_MODE_SELECT_MASK,
                                               HM01B0_MODE_STANDBY),
                        TAG, "failed to enter standby");

    dev->state = HM01B0_STATE_STANDBY;
    ESP_LOGI(TAG, "state=STANDBY");
    return ESP_OK;
}

esp_err_t hm01b0_start(hm01b0_handle_t *dev)
{
    ESP_RETURN_ON_ERROR(hm01b0_require_standby(dev), TAG,
                        "cannot start streaming");
    ESP_RETURN_ON_ERROR(hm01b0_reg_update_bits(dev,
                                               HM01B0_REG_MODE_SELECT,
                                               HM01B0_MODE_SELECT_MASK,
                                               HM01B0_MODE_STREAMING),
                        TAG, "failed to start streaming");

    dev->state = HM01B0_STATE_STREAMING;
    ESP_LOGI(TAG, "state=STREAMING");
    return ESP_OK;
}

esp_err_t hm01b0_stop(hm01b0_handle_t *dev)
{
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "device handle is NULL");
    ESP_RETURN_ON_FALSE(dev->state != HM01B0_STATE_UNINITIALIZED,
                        ESP_ERR_INVALID_STATE, TAG,
                        "sensor has not been initialized");

    if (dev->state == HM01B0_STATE_STANDBY) {
        return ESP_OK;
    }
    return hm01b0_standby(dev);
}

esp_err_t hm01b0_set_mode(hm01b0_handle_t *dev, hm01b0_mode_t mode)
{
    ESP_RETURN_ON_ERROR(hm01b0_require_standby(dev), TAG,
                        "cannot configure sensor mode");

    const hm01b0_regval_t *table = NULL;
    size_t count = 0;
    const char *name = NULL;

    switch (mode) {
    case HM01B0_SENSOR_MODE_FULL:
        table = hm01b0_mode_full;
        count = hm01b0_mode_full_count;
        name = "FULL";
        break;
    case HM01B0_SENSOR_MODE_QVGA:
        table = hm01b0_mode_qvga;
        count = hm01b0_mode_qvga_count;
        name = "QVGA";
        break;
    case HM01B0_SENSOR_MODE_QQVGA:
        table = hm01b0_mode_qqvga;
        count = hm01b0_mode_qqvga_count;
        name = "QQVGA";
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(hm01b0_write_table(dev, table, count), TAG,
                        "failed to apply %s mode table", name);
    dev->mode = mode;
    ESP_LOGI(TAG, "mode=%s", name);
    return ESP_OK;
}

esp_err_t hm01b0_set_interface(hm01b0_handle_t *dev,
                               hm01b0_interface_t interface)
{
    ESP_RETURN_ON_ERROR(hm01b0_require_standby(dev), TAG,
                        "cannot configure data interface");

    const hm01b0_regval_t *table = NULL;
    size_t count = 0;
    const char *name = NULL;

    switch (interface) {
    case HM01B0_DATA_INTERFACE_8_BIT:
        table = hm01b0_interface_8bit;
        count = hm01b0_interface_8bit_count;
        name = "8-bit";
        break;
    case HM01B0_DATA_INTERFACE_4_BIT:
        table = hm01b0_interface_4bit;
        count = hm01b0_interface_4bit_count;
        name = "4-bit";
        break;
    case HM01B0_DATA_INTERFACE_1_BIT:
        table = hm01b0_interface_1bit;
        count = hm01b0_interface_1bit_count;
        name = "1-bit";
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(hm01b0_write_table(dev, table, count), TAG,
                        "failed to apply %s interface table", name);
    dev->interface = interface;
    ESP_LOGI(TAG, "interface=%s", name);
    return ESP_OK;
}

esp_err_t hm01b0_set_test_pattern(hm01b0_handle_t *dev,
                                  hm01b0_test_pattern_t pattern)
{
    ESP_RETURN_ON_ERROR(hm01b0_require_standby(dev), TAG,
                        "cannot configure test pattern");

    const hm01b0_regval_t *table = NULL;
    size_t count = 0;
    const char *name = NULL;

    switch (pattern) {
    case HM01B0_TEST_PATTERN_OFF:
        table = hm01b0_test_pattern_off;
        count = hm01b0_test_pattern_off_count;
        name = "OFF";
        break;
    case HM01B0_TEST_PATTERN_COLOR_BAR:
        table = hm01b0_test_pattern_color_bar;
        count = hm01b0_test_pattern_color_bar_count;
        name = "COLOR_BAR";
        break;
    case HM01B0_TEST_PATTERN_WALKING_1:
        table = hm01b0_test_pattern_walking_1;
        count = hm01b0_test_pattern_walking_1_count;
        name = "WALKING_1";
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(hm01b0_write_table(dev, table, count), TAG,
                        "failed to apply %s test-pattern table", name);
    dev->test_pattern = pattern;
    ESP_LOGI(TAG, "test_pattern=%s", name);
    return ESP_OK;
}

hm01b0_state_t hm01b0_get_state(const hm01b0_handle_t *dev)
{
    return dev == NULL ? HM01B0_STATE_UNINITIALIZED : dev->state;
}
