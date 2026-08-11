#ifndef HM01B0_DIAGNOSTICS_H
#define HM01B0_DIAGNOSTICS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "hm01b0_frame_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HM01B0_DIAGNOSTIC_SAMPLE_ROW_COUNT 4U
#define HM01B0_DIAGNOSTIC_COLOR_BAR_COUNT  6U

typedef enum {
    HM01B0_DIAGNOSTIC_PATTERN_NONE = 0,
    HM01B0_DIAGNOSTIC_PATTERN_WALKING_1,
    HM01B0_DIAGNOSTIC_PATTERN_COLOR_BAR,
} hm01b0_diagnostic_pattern_t;

typedef enum {
    HM01B0_DIAGNOSTIC_STATUS_NOT_RUN = 0,
    HM01B0_DIAGNOSTIC_STATUS_PASS,
    HM01B0_DIAGNOSTIC_STATUS_WARNING,
} hm01b0_diagnostic_status_t;

typedef struct {
    hm01b0_diagnostic_pattern_t pattern;
    hm01b0_frame_rect_t area;
    uint32_t warmup_frames;
} hm01b0_diagnostic_config_t;

typedef struct {
    uint32_t sampled_rows;
    uint32_t rows_equal;
    uint32_t rows_compared;
    uint32_t vertical_mismatches;
    uint32_t horizontal_transitions;
    uint32_t unique_values;
    uint32_t zero_values;
    uint32_t one_hot_values;
    uint32_t other_values;
} hm01b0_walking_1_result_t;

typedef struct {
    uint32_t sampled_rows;
    uint32_t rows_equal;
    uint32_t rows_compared;
    uint32_t vertical_mismatches;
    uint32_t horizontal_transitions;
    uint32_t strong_transitions;
    uint32_t unique_values;
    uint32_t center_changes;
    uint32_t center_vertical_mismatches;
    uint8_t min_value;
    uint8_t max_value;
    uint8_t center_values[HM01B0_DIAGNOSTIC_COLOR_BAR_COUNT];
} hm01b0_color_bar_result_t;

typedef struct {
    hm01b0_diagnostic_pattern_t pattern;
    hm01b0_diagnostic_status_t status;
    uint32_t analyzed_frames;
    uint32_t warmup_frames;
    uint32_t last_raw_crc;
    uint32_t last_active_crc;
    uint32_t raw_crc_changes;
    uint32_t active_crc_changes;
    uint32_t active_crc_frame_changes;
    union {
        hm01b0_walking_1_result_t walking_1;
        hm01b0_color_bar_result_t color_bar;
    } result;
} hm01b0_diagnostic_report_t;

const char *hm01b0_diagnostic_pattern_name(
    hm01b0_diagnostic_pattern_t pattern);
const char *hm01b0_diagnostic_status_name(
    hm01b0_diagnostic_status_t status);

/** Analyze one complete RAW8 frame without retaining its buffer. */
esp_err_t hm01b0_diagnostics_analyze_frame(
    hm01b0_diagnostic_pattern_t pattern,
    const uint8_t *data,
    uint16_t width,
    uint16_t height,
    size_t stride,
    hm01b0_frame_rect_t area,
    hm01b0_diagnostic_report_t *report);

#ifdef __cplusplus
}
#endif

#endif /* HM01B0_DIAGNOSTICS_H */
