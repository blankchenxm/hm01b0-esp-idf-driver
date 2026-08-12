#include <inttypes.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "hm01b0.h"
#include "hm01b0_private.h"
#include "hm01b0_regs.h"
#include "hm01b0_tables.h"

#define HM01B0_RESET_RECOVERY_US 1000U
#define HM01B0_SENSOR_CORE_MAX_HZ 6000000U

static const char *TAG = "hm01b0_sensor";

typedef struct {
    uint16_t min_line_length_pck;
    uint16_t min_frame_length_lines;
    uint16_t max_frame_rate;
} hm01b0_timing_constraints_t;

static esp_err_t hm01b0_get_timing_constraints(
    hm01b0_mode_t mode,
    hm01b0_timing_constraints_t *constraints)
{
    ESP_RETURN_ON_FALSE(constraints != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "timing constraints are NULL");

    switch (mode) {
    case HM01B0_SENSOR_MODE_FULL:
        *constraints = (hm01b0_timing_constraints_t) {
            .min_line_length_pck = 0x0178U,
            .min_frame_length_lines = 0x0158U,
            .max_frame_rate = 45U,
        };
        break;
    case HM01B0_SENSOR_MODE_QVGA:
        *constraints = (hm01b0_timing_constraints_t) {
            .min_line_length_pck = 0x0178U,
            .min_frame_length_lines = 0x0104U,
            .max_frame_rate = 60U,
        };
        break;
    case HM01B0_SENSOR_MODE_QQVGA:
        *constraints = (hm01b0_timing_constraints_t) {
            .min_line_length_pck = 0x00D7U,
            .min_frame_length_lines = 0x0080U,
            .max_frame_rate = 120U,
        };
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static uint32_t hm01b0_core_divider(hm01b0_interface_t interface)
{
    switch (interface) {
    case HM01B0_DATA_INTERFACE_8_BIT:
    case HM01B0_DATA_INTERFACE_4_BIT:
        return 2U;
    case HM01B0_DATA_INTERFACE_1_BIT:
        return 8U;
    default:
        return 0U;
    }
}

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

esp_err_t hm01b0_get_frame_count(hm01b0_handle_t *dev,
                                 uint8_t *frame_count)
{
    ESP_RETURN_ON_FALSE(dev != NULL && frame_count != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    return hm01b0_reg_read(dev, HM01B0_REG_FRAME_COUNT, frame_count);
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

esp_err_t hm01b0_stream_start(hm01b0_handle_t *dev)
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

esp_err_t hm01b0_stream_stop(hm01b0_handle_t *dev)
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
    ESP_RETURN_ON_ERROR(
        hm01b0_reg_write(dev, HM01B0_REG_GROUP_PARAMETER_HOLD,
                         HM01B0_GROUP_PARAMETER_APPLY),
        TAG, "failed to latch %s mode CMU parameters", name);
    dev->mode = mode;
    dev->frame_rate = HM01B0_FRAME_RATE_UNCONFIGURED;
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
    dev->frame_rate = HM01B0_FRAME_RATE_UNCONFIGURED;
    ESP_LOGI(TAG, "interface=%s", name);
    return ESP_OK;
}

static esp_err_t hm01b0_read_u16(hm01b0_handle_t *dev,
                                 uint16_t high_addr,
                                 uint16_t low_addr,
                                 uint16_t *value)
{
    ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "16-bit register result is NULL");

    uint8_t high = 0U;
    uint8_t low = 0U;
    ESP_RETURN_ON_ERROR(hm01b0_reg_read(dev, high_addr, &high), TAG,
                        "failed to read register 0x%04X", high_addr);
    ESP_RETURN_ON_ERROR(hm01b0_reg_read(dev, low_addr, &low), TAG,
                        "failed to read register 0x%04X", low_addr);
    *value = ((uint16_t)high << 8U) | low;
    return ESP_OK;
}

esp_err_t hm01b0_set_frame_rate(hm01b0_handle_t *dev,
                                hm01b0_frame_rate_t frame_rate)
{
    ESP_RETURN_ON_ERROR(hm01b0_require_standby(dev), TAG,
                        "cannot configure frame rate");

    switch (frame_rate) {
    case HM01B0_FRAME_RATE_15:
    case HM01B0_FRAME_RATE_20:
    case HM01B0_FRAME_RATE_30:
    case HM01B0_FRAME_RATE_45:
    case HM01B0_FRAME_RATE_60:
    case HM01B0_FRAME_RATE_120:
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    hm01b0_timing_constraints_t constraints = {0};
    ESP_RETURN_ON_ERROR(
        hm01b0_get_timing_constraints(dev->mode, &constraints), TAG,
        "failed to get timing constraints");

    const uint32_t target_fps = (uint32_t)frame_rate;
    ESP_RETURN_ON_FALSE(target_fps <= constraints.max_frame_rate,
                        ESP_ERR_NOT_SUPPORTED, TAG,
                        "%" PRIu32 " FPS exceeds mode maximum %u FPS",
                        target_fps,
                        (unsigned)constraints.max_frame_rate);

    const uint32_t core_divider = hm01b0_core_divider(dev->interface);
    ESP_RETURN_ON_FALSE(core_divider > 0U && dev->mclk_freq_hz > 0U,
                        ESP_ERR_INVALID_STATE, TAG,
                        "sensor clock/interface is not configured");
    const uint32_t sensor_core_hz = dev->mclk_freq_hz / core_divider;
    ESP_RETURN_ON_FALSE(
        sensor_core_hz > 0U &&
            sensor_core_hz <= HM01B0_SENSOR_CORE_MAX_HZ,
        ESP_ERR_NOT_SUPPORTED, TAG,
        "Sensor_Core=%" PRIu32 " Hz is outside supported range 1..%u Hz",
        sensor_core_hz, (unsigned)HM01B0_SENSOR_CORE_MAX_HZ);

    const uint16_t line_length_pck = constraints.min_line_length_pck;
    const uint64_t frame_denominator =
        (uint64_t)line_length_pck * target_fps;
    const uint64_t calculated_frame_length =
        ((uint64_t)sensor_core_hz + frame_denominator - 1U) /
        frame_denominator;
    ESP_RETURN_ON_FALSE(
        calculated_frame_length >= constraints.min_frame_length_lines &&
            calculated_frame_length <= UINT16_MAX,
        ESP_ERR_NOT_SUPPORTED, TAG,
        "%" PRIu32 " FPS cannot be reached: Sensor_Core=%" PRIu32
        " Hz, required frame length=%" PRIu64 ", allowed=%u..%u",
        target_fps, sensor_core_hz, calculated_frame_length,
        (unsigned)constraints.min_frame_length_lines,
        (unsigned)UINT16_MAX);

    const uint16_t frame_length_lines =
        (uint16_t)calculated_frame_length;
    const uint16_t max_integration = frame_length_lines - 2U;
    const hm01b0_regval_t timing_table[] = {
        {
            .addr = HM01B0_REG_LINE_LENGTH_PCK_H,
            .value = (uint8_t)(line_length_pck >> 8U),
            .mask = UINT8_MAX,
            .delay_ms = 0U,
        },
        {
            .addr = HM01B0_REG_LINE_LENGTH_PCK_L,
            .value = (uint8_t)line_length_pck,
            .mask = UINT8_MAX,
            .delay_ms = 0U,
        },
        {
            .addr = HM01B0_REG_FRAME_LENGTH_LINES_H,
            .value = (uint8_t)(frame_length_lines >> 8U),
            .mask = UINT8_MAX,
            .delay_ms = 0U,
        },
        {
            .addr = HM01B0_REG_FRAME_LENGTH_LINES_L,
            .value = (uint8_t)frame_length_lines,
            .mask = UINT8_MAX,
            .delay_ms = 0U,
        },
        {
            .addr = HM01B0_REG_MAX_INTEGRATION_H,
            .value = (uint8_t)(max_integration >> 8U),
            .mask = UINT8_MAX,
            .delay_ms = 0U,
        },
        {
            .addr = HM01B0_REG_MAX_INTEGRATION_L,
            .value = (uint8_t)max_integration,
            .mask = UINT8_MAX,
            .delay_ms = 0U,
        },
        {
            /* Latch the pending CMU timing values before streaming. */
            .addr = HM01B0_REG_GROUP_PARAMETER_HOLD,
            .value = HM01B0_GROUP_PARAMETER_APPLY,
            .mask = UINT8_MAX,
            .delay_ms = 0U,
        },
    };
    ESP_RETURN_ON_ERROR(
        hm01b0_write_table(dev, timing_table,
                           sizeof(timing_table) / sizeof(timing_table[0])),
        TAG, "failed to apply frame timing");

    uint16_t readback_line_length = 0U;
    uint16_t readback_frame_length = 0U;
    uint16_t readback_max_integration = 0U;
    ESP_RETURN_ON_ERROR(
        hm01b0_read_u16(dev, HM01B0_REG_LINE_LENGTH_PCK_H,
                        HM01B0_REG_LINE_LENGTH_PCK_L,
                        &readback_line_length),
        TAG, "failed to read back line length");
    ESP_RETURN_ON_ERROR(
        hm01b0_read_u16(dev, HM01B0_REG_FRAME_LENGTH_LINES_H,
                        HM01B0_REG_FRAME_LENGTH_LINES_L,
                        &readback_frame_length),
        TAG, "failed to read back frame length");
    ESP_RETURN_ON_ERROR(
        hm01b0_read_u16(dev, HM01B0_REG_MAX_INTEGRATION_H,
                        HM01B0_REG_MAX_INTEGRATION_L,
                        &readback_max_integration),
        TAG, "failed to read back maximum integration");
    ESP_RETURN_ON_FALSE(
        readback_line_length == line_length_pck &&
            readback_frame_length == frame_length_lines &&
            readback_max_integration == max_integration,
        ESP_ERR_INVALID_RESPONSE, TAG,
        "frame timing readback mismatch: expected line=%u frame=%u "
        "max_integration=%u, read line=%u frame=%u max_integration=%u",
        (unsigned)line_length_pck, (unsigned)frame_length_lines,
        (unsigned)max_integration, (unsigned)readback_line_length,
        (unsigned)readback_frame_length,
        (unsigned)readback_max_integration);

    dev->frame_rate = frame_rate;
    const uint32_t actual_fps_milli = (uint32_t)(
        ((uint64_t)sensor_core_hz * 1000U) /
        ((uint64_t)line_length_pck * frame_length_lines));
    ESP_LOGI(TAG,
             "frame_rate target=%" PRIu32 " FPS, calculated=%" PRIu32
             ".%03" PRIu32 " FPS, Sensor_Core=%" PRIu32
             " Hz, line_length=%u, frame_length=%u, max_integration=%u",
             target_fps, actual_fps_milli / 1000U,
             actual_fps_milli % 1000U, sensor_core_hz,
             (unsigned)line_length_pck,
             (unsigned)frame_length_lines,
             (unsigned)max_integration);
    ESP_LOGI(TAG,
             "frame timing readback verified: line_length=%u, "
             "frame_length=%u, max_integration=%u, CMU apply=0x%02X",
             (unsigned)readback_line_length,
             (unsigned)readback_frame_length,
             (unsigned)readback_max_integration,
             HM01B0_GROUP_PARAMETER_APPLY);
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
    uint8_t expected_register = 0U;

    switch (pattern) {
    case HM01B0_TEST_PATTERN_OFF:
        table = hm01b0_test_pattern_off;
        count = hm01b0_test_pattern_off_count;
        name = "OFF";
        expected_register = HM01B0_TEST_PATTERN_REG_DISABLED;
        break;
    case HM01B0_TEST_PATTERN_COLOR_BAR:
        table = hm01b0_test_pattern_color_bar;
        count = hm01b0_test_pattern_color_bar_count;
        name = "COLOR_BAR";
        expected_register = HM01B0_TEST_PATTERN_REG_COLOR_BAR;
        break;
    case HM01B0_TEST_PATTERN_WALKING_1:
        table = hm01b0_test_pattern_walking_1;
        count = hm01b0_test_pattern_walking_1_count;
        name = "WALKING_1";
        expected_register = HM01B0_TEST_PATTERN_REG_WALKING_1;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(hm01b0_write_table(dev, table, count), TAG,
                        "failed to apply %s test-pattern table", name);

    uint8_t register_value = 0U;
    ESP_RETURN_ON_ERROR(hm01b0_reg_read(dev, HM01B0_REG_TEST_PATTERN_MODE,
                                        &register_value),
                        TAG, "failed to read back %s test-pattern mode", name);
    const uint8_t mode_mask = HM01B0_TEST_PATTERN_ENABLE_MASK |
                              HM01B0_TEST_PATTERN_SELECT_MASK;
    ESP_RETURN_ON_FALSE((register_value & mode_mask) == expected_register,
                        ESP_ERR_INVALID_RESPONSE, TAG,
                        "%s test-pattern readback mismatch: expected=0x%02X "
                        "actual=0x%02X",
                        name, expected_register, register_value);

    dev->test_pattern = pattern;
    ESP_LOGI(TAG, "test_pattern=%s, register 0x0601=0x%02X", name,
             register_value);
    return ESP_OK;
}

hm01b0_state_t hm01b0_get_state(const hm01b0_handle_t *dev)
{
    return dev == NULL ? HM01B0_STATE_UNINITIALIZED : dev->state;
}
