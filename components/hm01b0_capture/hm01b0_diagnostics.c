#include <limits.h>
#include <string.h>

#include "esp_check.h"
#include "esp_crc.h"
#include "hm01b0_diagnostics.h"

#define HM01B0_COLOR_BAR_STRONG_DELTA 8U

static const char *TAG = "hm01b0_diagnostics";

const char *hm01b0_diagnostic_pattern_name(
    hm01b0_diagnostic_pattern_t pattern)
{
    switch (pattern) {
    case HM01B0_DIAGNOSTIC_PATTERN_NONE:
        return "none";
    case HM01B0_DIAGNOSTIC_PATTERN_WALKING_1:
        return "Walking-1";
    case HM01B0_DIAGNOSTIC_PATTERN_COLOR_BAR:
        return "Color-Bar";
    default:
        return "invalid";
    }
}

const char *hm01b0_diagnostic_status_name(
    hm01b0_diagnostic_status_t status)
{
    switch (status) {
    case HM01B0_DIAGNOSTIC_STATUS_NOT_RUN:
        return "NOT_RUN";
    case HM01B0_DIAGNOSTIC_STATUS_PASS:
        return "PASS";
    case HM01B0_DIAGNOSTIC_STATUS_WARNING:
        return "WARNING";
    default:
        return "INVALID";
    }
}

static bool hm01b0_diagnostics_valid_area(uint16_t width,
                                          uint16_t height,
                                          size_t stride,
                                          hm01b0_frame_rect_t area)
{
    return width > 0U && height > 0U && stride >= width &&
           area.width > 0U && area.height > 0U &&
           area.x < width && area.y < height &&
           area.width <= width - area.x &&
           area.height <= height - area.y;
}

static size_t hm01b0_select_sample_rows(
    hm01b0_frame_rect_t area,
    uint16_t rows[HM01B0_DIAGNOSTIC_SAMPLE_ROW_COUNT])
{
    const uint16_t offsets[HM01B0_DIAGNOSTIC_SAMPLE_ROW_COUNT] = {
        0U,
        area.height > 1U ? 1U : 0U,
        (uint16_t)(area.height / 2U),
        (uint16_t)(area.height - 1U),
    };
    size_t count = 0U;

    for (size_t i = 0; i < HM01B0_DIAGNOSTIC_SAMPLE_ROW_COUNT; ++i) {
        const uint16_t row = (uint16_t)(area.y + offsets[i]);
        bool duplicate = false;
        for (size_t j = 0; j < count; ++j) {
            if (rows[j] == row) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            rows[count++] = row;
        }
    }
    return count;
}

static bool hm01b0_is_one_hot(uint8_t value)
{
    return value != 0U && (value & (uint8_t)(value - 1U)) == 0U;
}

static uint32_t hm01b0_active_crc(const uint8_t *data,
                                  size_t stride,
                                  hm01b0_frame_rect_t area)
{
    uint32_t crc = 0U;
    for (uint16_t y = 0; y < area.height; ++y) {
        const size_t offset = (size_t)(area.y + y) * stride + area.x;
        crc = esp_crc32_le(crc, data + offset, area.width);
    }
    return crc;
}

static hm01b0_walking_1_result_t hm01b0_analyze_walking_1(
    const uint8_t *data,
    size_t stride,
    hm01b0_frame_rect_t area)
{
    hm01b0_walking_1_result_t result = {0};
    uint32_t seen_values[8] = {0};
    uint16_t rows[HM01B0_DIAGNOSTIC_SAMPLE_ROW_COUNT] = {0};
    const size_t row_count = hm01b0_select_sample_rows(area, rows);
    result.sampled_rows = (uint32_t)row_count;

    const uint8_t *reference = data + (size_t)rows[0] * stride + area.x;
    for (uint16_t x = 1U; x < area.width; ++x) {
        if (reference[x] != reference[x - 1U]) {
            result.horizontal_transitions++;
        }
    }

    for (size_t row_index = 0; row_index < row_count; ++row_index) {
        const uint8_t *row = data + (size_t)rows[row_index] * stride + area.x;
        bool equal = true;
        for (uint16_t x = 0; x < area.width; ++x) {
            const uint8_t value = row[x];
            uint32_t *word = &seen_values[value >> 5U];
            const uint32_t bit = 1UL << (value & 31U);
            if ((*word & bit) == 0U) {
                *word |= bit;
                result.unique_values++;
            }
            if (value == 0U) {
                result.zero_values++;
            } else if (hm01b0_is_one_hot(value)) {
                result.one_hot_values++;
            } else {
                result.other_values++;
            }
            if (row_index > 0U && value != reference[x]) {
                result.vertical_mismatches++;
                equal = false;
            }
        }
        if (row_index > 0U) {
            result.rows_compared++;
            if (equal) {
                result.rows_equal++;
            }
        }
    }
    return result;
}

static hm01b0_color_bar_result_t hm01b0_analyze_color_bar(
    const uint8_t *data,
    size_t stride,
    hm01b0_frame_rect_t area,
    hm01b0_pixel_format_t pixel_format)
{
    hm01b0_color_bar_result_t result = { .min_value = UINT8_MAX };
    const bool is_bayer = pixel_format != HM01B0_PIXEL_FORMAT_MONO8;
    const uint16_t sample_step = is_bayer ? 2U : 1U;
    result.cfa_aware = is_bayer;
    uint32_t seen_values[8] = {0};
    uint16_t rows[HM01B0_DIAGNOSTIC_SAMPLE_ROW_COUNT] = {0};
    uint16_t centers[HM01B0_DIAGNOSTIC_COLOR_BAR_COUNT] = {0};
    const size_t row_count = hm01b0_select_sample_rows(area, rows);
    result.sampled_rows = (uint32_t)row_count;

    const uint8_t *reference = data + (size_t)rows[0] * stride + area.x;
    for (uint16_t x = sample_step; x < area.width; ++x) {
        if (reference[x] != reference[x - sample_step]) {
            result.horizontal_transitions++;
        }
        const unsigned delta = reference[x] > reference[x - sample_step]
                                   ? reference[x] - reference[x - sample_step]
                                   : reference[x - sample_step] - reference[x];
        if (delta >= HM01B0_COLOR_BAR_STRONG_DELTA) {
            result.strong_transitions++;
        }
    }

    for (size_t bar = 0; bar < HM01B0_DIAGNOSTIC_COLOR_BAR_COUNT; ++bar) {
        uint16_t center = (uint16_t)(
            ((uint32_t)(2U * bar + 1U) * area.width) /
            (2U * HM01B0_DIAGNOSTIC_COLOR_BAR_COUNT));
        if (center >= area.width) {
            center = (uint16_t)(area.width - 1U);
        }
        centers[bar] = center;
        result.center_values[bar] = reference[center];
        if (bar > 0U &&
            result.center_values[bar] != result.center_values[bar - 1U]) {
            result.center_changes++;
        }
    }

    for (size_t row_index = 0; row_index < row_count; ++row_index) {
        const uint8_t *row = data + (size_t)rows[row_index] * stride + area.x;
        bool equal = true;
        for (uint16_t x = 0; x < area.width; ++x) {
            const uint8_t value = row[x];
            uint32_t *word = &seen_values[value >> 5U];
            const uint32_t bit = 1UL << (value & 31U);
            if ((*word & bit) == 0U) {
                *word |= bit;
                result.unique_values++;
            }
            if (value < result.min_value) {
                result.min_value = value;
            }
            if (value > result.max_value) {
                result.max_value = value;
            }
            if (row_index > 0U &&
                (!is_bayer || ((rows[row_index] - rows[0]) & 1U) == 0U) &&
                value != reference[x]) {
                result.vertical_mismatches++;
                equal = false;
            }
        }
        if (row_index > 0U &&
            (!is_bayer || ((rows[row_index] - rows[0]) & 1U) == 0U)) {
            result.rows_compared++;
            if (equal) {
                result.rows_equal++;
            }
            for (size_t bar = 0; bar < HM01B0_DIAGNOSTIC_COLOR_BAR_COUNT;
                 ++bar) {
                if (row[centers[bar]] != result.center_values[bar]) {
                    result.center_vertical_mismatches++;
                }
            }
        }
    }
    return result;
}

esp_err_t hm01b0_diagnostics_analyze_frame(
    hm01b0_diagnostic_pattern_t pattern,
    hm01b0_pixel_format_t pixel_format,
    const uint8_t *data,
    uint16_t width,
    uint16_t height,
    size_t stride,
    hm01b0_frame_rect_t area,
    hm01b0_diagnostic_report_t *report)
{
    ESP_RETURN_ON_FALSE(data != NULL && report != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(
        pattern == HM01B0_DIAGNOSTIC_PATTERN_WALKING_1 ||
        pattern == HM01B0_DIAGNOSTIC_PATTERN_COLOR_BAR,
        ESP_ERR_INVALID_ARG, TAG, "diagnostic pattern is disabled or invalid");
    ESP_RETURN_ON_FALSE(hm01b0_diagnostics_valid_area(width, height, stride,
                                                       area),
                        ESP_ERR_INVALID_ARG, TAG, "invalid analysis area");

    const uint32_t raw_crc = esp_crc32_le(
        0U, data, (uint32_t)((size_t)width * height));
    const uint32_t active_crc = hm01b0_active_crc(data, stride, area);
    const uint32_t prior_frames = report->analyzed_frames;
    const uint32_t prior_active_crc = report->last_active_crc;

    report->pattern = pattern;
    report->last_raw_crc = raw_crc;
    report->last_active_crc = active_crc;
    if (prior_frames > 0U) {
        if (active_crc != prior_active_crc) {
            report->active_crc_frame_changes++;
        }
    }

    if (pattern == HM01B0_DIAGNOSTIC_PATTERN_WALKING_1) {
        report->result.walking_1 = hm01b0_analyze_walking_1(
            data, stride, area);
        const hm01b0_walking_1_result_t *result =
            &report->result.walking_1;
        report->status = result->horizontal_transitions > 0U &&
                                 result->unique_values > 1U
                             ? HM01B0_DIAGNOSTIC_STATUS_PASS
                             : HM01B0_DIAGNOSTIC_STATUS_WARNING;
    } else {
        report->result.color_bar = hm01b0_analyze_color_bar(
            data, stride, area, pixel_format);
        const hm01b0_color_bar_result_t *result =
            &report->result.color_bar;
        report->status = result->strong_transitions > 0U &&
                                 result->unique_values > 1U &&
                                 result->min_value < result->max_value
                             ? HM01B0_DIAGNOSTIC_STATUS_PASS
                             : HM01B0_DIAGNOSTIC_STATUS_WARNING;
    }
    report->analyzed_frames++;
    return ESP_OK;
}
