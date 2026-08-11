#ifndef HM01B0_CAPTURE_H
#define HM01B0_CAPTURE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "hal/gpio_types.h"
#include "hm01b0_diagnostics.h"
#include "hm01b0_frame_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HM01B0_CAPTURE_BUFFER_COUNT 2U

typedef struct hm01b0_capture hm01b0_capture_handle_t;

/** Metadata for one application-visible Camera transaction buffer. */
typedef struct {
    uint8_t *data;
    size_t capacity;
    size_t received_size;
    uint32_t sequence;
    int64_t timestamp_us;
} hm01b0_capture_frame_t;

/** Stable DVP transport configuration; application policy is kept elsewhere. */
typedef struct {
    gpio_num_t data_gpio[8];
    gpio_num_t pclk_gpio;
    gpio_num_t vsync_gpio;
    gpio_num_t de_gpio;
    uint16_t frame_width;
    uint16_t frame_height;
    uint32_t dma_burst_size;
} hm01b0_capture_config_t;

typedef struct {
    uint16_t frame_width;
    uint16_t frame_height;
    uint16_t frame_stride;
    size_t payload_size;
    size_t buffer_capacity;
    size_t last_received_size;
    uint32_t frames_received;
    uint32_t valid_frames;
    uint32_t size_errors;
    uint32_t received_size_changes;
    uint32_t no_free_buffer;
    uint32_t ready_queue_overflows;
    uint32_t free_queue_errors;
    uint32_t last_processing_time_us;
    uint32_t max_processing_time_us;
    uint32_t last_buffer_hold_time_us;
    uint32_t max_buffer_hold_time_us;
    uint32_t fps_milli;
} hm01b0_capture_transport_stats_t;

typedef struct {
    uint32_t sequence;
    size_t size;
    uint32_t copy_time_us;
    uint32_t buffer_hold_time_us;
} hm01b0_snapshot_result_t;

typedef void (*hm01b0_snapshot_ready_cb_t)(
    const hm01b0_snapshot_result_t *result,
    void *user_data);

typedef struct {
    uint8_t *buffer;
    size_t buffer_size;
    hm01b0_frame_rect_t crop;
    uint32_t skip_frames;
    hm01b0_snapshot_ready_cb_t on_ready;
    void *user_data;
} hm01b0_snapshot_request_t;

/** Create DVP, two internal DMA buffers, queues, and the frame task. */
esp_err_t hm01b0_capture_new(const hm01b0_capture_config_t *config,
                             hm01b0_capture_handle_t **out_handle);

/** Configure optional test-pattern diagnostics while Camera RX is stopped. */
esp_err_t hm01b0_capture_set_diagnostics(
    hm01b0_capture_handle_t *handle,
    const hm01b0_diagnostic_config_t *config);

/** Queue or replace one snapshot request while Camera RX is stopped. */
esp_err_t hm01b0_capture_request_snapshot(
    hm01b0_capture_handle_t *handle,
    const hm01b0_snapshot_request_t *request);

/** Enable/start the ESP32-S3 Camera receiver. Start sensor streaming next. */
esp_err_t hm01b0_capture_rx_start(hm01b0_capture_handle_t *handle);

/** Stop/disable Camera RX. Stop sensor streaming before this call. */
esp_err_t hm01b0_capture_rx_stop(hm01b0_capture_handle_t *handle);

/** Stop the receiver if needed and release all capture resources. */
esp_err_t hm01b0_capture_delete(hm01b0_capture_handle_t *handle);

esp_err_t hm01b0_capture_get_transport_stats(
    const hm01b0_capture_handle_t *handle,
    hm01b0_capture_transport_stats_t *stats);

esp_err_t hm01b0_capture_get_diagnostic_report(
    const hm01b0_capture_handle_t *handle,
    hm01b0_diagnostic_report_t *report);

esp_err_t hm01b0_capture_get_snapshot_result(
    const hm01b0_capture_handle_t *handle,
    hm01b0_snapshot_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* HM01B0_CAPTURE_H */
