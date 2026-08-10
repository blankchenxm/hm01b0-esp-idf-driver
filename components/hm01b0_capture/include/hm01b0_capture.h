#ifndef HM01B0_CAPTURE_H
#define HM01B0_CAPTURE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HM01B0_CAPTURE_BUFFER_COUNT 2U

typedef struct hm01b0_capture hm01b0_capture_handle_t;

typedef struct {
    gpio_num_t data_gpio[8];
    gpio_num_t pclk_gpio;
    gpio_num_t vsync_gpio;
    gpio_num_t de_gpio;

    uint16_t raw_width;
    uint16_t raw_height;

    /* Analysis ROI inside the complete RAW8 DMA frame; no pixel copy/crop. */
    uint16_t active_x;
    uint16_t active_y;
    uint16_t active_width;
    uint16_t active_height;
    uint32_t dma_burst_size;

    uint32_t task_stack_size;
    uint32_t task_priority;
    uint32_t stats_period_ms;
    /* Zero selects the component default of five startup frames. */
    uint32_t warmup_frames;
    /* Observe Walking-1 geometry; this is not an exact byte-sequence test. */
    bool analyze_walking_1;
} hm01b0_capture_config_t;

typedef struct {
    uint16_t raw_width;
    uint16_t raw_height;
    uint16_t raw_stride;
    uint16_t active_x;
    uint16_t active_y;
    uint16_t active_width;
    uint16_t active_height;

    size_t payload_size;
    size_t active_payload_size;
    size_t buffer_capacity;
    size_t received_size;

    uint32_t frames_received;
    uint32_t transport_valid_frames;
    uint32_t warmup_frames;
    uint32_t size_errors;
    uint32_t received_size_changes;
    uint32_t raw_crc_changes;
    uint32_t active_crc_changes;
    uint32_t active_crc_frame_changes;
    uint32_t no_free_buffer;
    uint32_t ready_queue_overflows;
    uint32_t free_queue_errors;

    uint32_t last_raw_crc;
    uint32_t last_active_crc;
    uint32_t walking_analysis_frames;
    uint32_t walking_rows_equal;
    uint32_t walking_rows_compared;
    uint32_t walking_vertical_mismatches;
    uint32_t walking_horizontal_transitions;
    uint32_t walking_unique_values;
    uint32_t walking_zero_values;
    uint32_t walking_one_hot_values;
    uint32_t walking_other_values;
    uint32_t last_processing_time_us;
    uint32_t max_processing_time_us;
    uint32_t fps_milli;
} hm01b0_capture_stats_t;

/**
 * @brief Create the DVP controller, two internal DMA buffers, queues and the
 * analysis task. The receiver remains disabled and stopped.
 */
esp_err_t hm01b0_capture_new(const hm01b0_capture_config_t *config,
                             hm01b0_capture_handle_t **out_handle);

/** Enable and start Camera RX. Start the HM01B0 sensor only after this call. */
esp_err_t hm01b0_capture_start(hm01b0_capture_handle_t *handle);

/** Stop and disable Camera RX. Stop the HM01B0 sensor before this call. */
esp_err_t hm01b0_capture_stop(hm01b0_capture_handle_t *handle);

/** Stop the receiver if needed and release all capture resources. */
esp_err_t hm01b0_capture_delete(hm01b0_capture_handle_t *handle);

/** Copy a non-atomic diagnostic snapshot suitable for status reporting. */
esp_err_t hm01b0_capture_get_stats(const hm01b0_capture_handle_t *handle,
                                   hm01b0_capture_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* HM01B0_CAPTURE_H */
