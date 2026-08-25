#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hm01b0_capture.h"

#define HM01B0_CAPTURE_TASK_STACK_SIZE      4096U
#define HM01B0_CAPTURE_TASK_PRIORITY        5U
#define HM01B0_CAPTURE_STATS_PERIOD_MS      1000U
#define HM01B0_CAPTURE_FIRST_SAMPLE_SIZE    32U
#define HM01B0_CAPTURE_QUEUE_WAIT_MS        100U
#define HM01B0_CAPTURE_TASK_STOP_TIMEOUT_MS 500U

struct hm01b0_capture {
    hm01b0_capture_config_t config;
    esp_cam_ctlr_handle_t cam_handle;

    hm01b0_capture_frame_t frames[HM01B0_CAPTURE_BUFFER_COUNT];
    QueueHandle_t free_queue;
    QueueHandle_t ready_queue;
    StaticQueue_t free_queue_state;
    StaticQueue_t ready_queue_state;
    uint8_t free_queue_storage[HM01B0_CAPTURE_BUFFER_COUNT *
                               sizeof(hm01b0_capture_frame_t *)];
    uint8_t ready_queue_storage[HM01B0_CAPTURE_BUFFER_COUNT *
                                sizeof(hm01b0_capture_frame_t *)];

    TaskHandle_t task_handle;
    volatile bool task_should_exit;
    volatile bool processing_frame;
    bool controller_enabled;
    bool controller_started;

    volatile uint32_t isr_sequence;
    volatile uint32_t isr_frames_received;
    volatile uint32_t isr_no_free_buffer;
    volatile uint32_t isr_ready_queue_overflows;
    volatile uint32_t isr_free_queue_errors;
    uint32_t task_free_queue_errors;
    uint32_t processed_frames;
    bool baseline_received_size_valid;
    size_t baseline_received_size;

    hm01b0_capture_transport_stats_t transport;
    hm01b0_diagnostic_config_t diagnostic_config;
    hm01b0_diagnostic_report_t diagnostic_report;
    bool diagnostic_enabled;
    bool diagnostic_baseline_valid;
    uint32_t diagnostic_baseline_raw_crc;
    uint32_t diagnostic_baseline_active_crc;
    bool first_diagnostic_sample_valid;
    uint32_t first_diagnostic_sequence;
    size_t first_diagnostic_received_size;
    uint16_t first_diagnostic_raw_count;
    uint16_t first_diagnostic_active_count;
    uint8_t first_diagnostic_raw[HM01B0_CAPTURE_FIRST_SAMPLE_SIZE];
    uint8_t first_diagnostic_active[HM01B0_CAPTURE_FIRST_SAMPLE_SIZE];

    hm01b0_snapshot_request_t snapshot_request;
    hm01b0_snapshot_result_t snapshot_result;
    bool snapshot_pending;

    hm01b0_capture_frame_consumer_t frame_consumer;
    void *frame_consumer_user_data;

    int64_t last_stats_time_us;
    uint32_t last_stats_frame_count;
    int64_t last_error_log_time_us;

    size_t dma_heap_free_before;
    size_t dma_heap_largest_before;
    size_t dma_heap_free_after;
    size_t dma_heap_largest_after;
};

static const char *TAG = "hm01b0_capture";

static bool IRAM_ATTR hm01b0_capture_on_get_new_trans(
    esp_cam_ctlr_handle_t cam_handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data);
static bool IRAM_ATTR hm01b0_capture_on_trans_finished(
    esp_cam_ctlr_handle_t cam_handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data);
static void hm01b0_capture_task(void *arg);

static bool hm01b0_capture_valid_dma_burst(uint32_t burst_size)
{
    return burst_size == 0U ||
           (burst_size >= 4U && burst_size <= 128U &&
            (burst_size & (burst_size - 1U)) == 0U);
}

static const char *hm01b0_capture_buffer_memory_name(
    hm01b0_capture_buffer_memory_t memory)
{
    return memory == HM01B0_CAPTURE_BUFFER_PSRAM
               ? "PSRAM DMA"
               : "internal DMA";
}

static uint32_t hm01b0_capture_buffer_caps(
    hm01b0_capture_buffer_memory_t memory)
{
    return memory == HM01B0_CAPTURE_BUFFER_PSRAM
               ? MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA
               : MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA;
}

static bool hm01b0_capture_rect_is_valid(
    const hm01b0_capture_handle_t *handle,
    hm01b0_frame_rect_t rect)
{
    return rect.width > 0U && rect.height > 0U &&
           rect.x < handle->config.frame_width &&
           rect.y < handle->config.frame_height &&
           rect.width <= handle->config.frame_width - rect.x &&
           rect.height <= handle->config.frame_height - rect.y;
}

static bool hm01b0_capture_frame_size_is_valid(
    const hm01b0_capture_handle_t *handle,
    const hm01b0_capture_frame_t *frame)
{
    /*
     * RAW8 pixels occupy the first payload_size bytes. esp_driver_cam may
     * align its non-JPEG DMA transaction to a cache/DMA boundary and report
     * the resulting capacity as received_size. Bytes after payload_size are
     * trailing transport padding, not pixels and not part of the row stride.
     */
    return frame->received_size >= handle->transport.payload_size &&
           frame->received_size <= handle->transport.buffer_capacity;
}

static esp_err_t hm01b0_capture_validate_config(
    const hm01b0_capture_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "configuration is NULL");
    ESP_RETURN_ON_FALSE(config->frame_width > 0U &&
                        config->frame_height > 0U,
                        ESP_ERR_INVALID_ARG, TAG, "invalid frame geometry");
    ESP_RETURN_ON_FALSE(
        config->buffer_memory == HM01B0_CAPTURE_BUFFER_INTERNAL ||
            config->buffer_memory == HM01B0_CAPTURE_BUFFER_PSRAM,
        ESP_ERR_INVALID_ARG, TAG, "invalid Camera buffer memory selection");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(config->pclk_gpio) &&
                        GPIO_IS_VALID_GPIO(config->vsync_gpio) &&
                        GPIO_IS_VALID_GPIO(config->de_gpio),
                        ESP_ERR_INVALID_ARG, TAG, "invalid DVP control GPIO");
    for (size_t i = 0; i < 8U; ++i) {
        ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(config->data_gpio[i]),
                            ESP_ERR_INVALID_ARG, TAG,
                            "invalid DVP data GPIO at index %u", (unsigned)i);
    }
    ESP_RETURN_ON_FALSE(hm01b0_capture_valid_dma_burst(
                            config->dma_burst_size),
                        ESP_ERR_INVALID_ARG, TAG, "invalid DMA burst size");
    return ESP_OK;
}

static esp_err_t hm01b0_capture_reset_queues(
    hm01b0_capture_handle_t *handle)
{
    xQueueReset(handle->free_queue);
    xQueueReset(handle->ready_queue);
    for (size_t i = 0; i < HM01B0_CAPTURE_BUFFER_COUNT; ++i) {
        hm01b0_capture_frame_t *frame = &handle->frames[i];
        frame->received_size = 0U;
        frame->sequence = 0U;
        frame->timestamp_us = 0;
        if (xQueueSend(handle->free_queue, &frame, 0) != pdPASS) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

static void hm01b0_capture_reset_session(hm01b0_capture_handle_t *handle)
{
    const size_t capacity = handle->transport.buffer_capacity;
    handle->transport = (hm01b0_capture_transport_stats_t) {
        .frame_width = handle->config.frame_width,
        .frame_height = handle->config.frame_height,
        .frame_stride = handle->config.frame_width,
        .payload_size = (size_t)handle->config.frame_width *
                        handle->config.frame_height,
        .buffer_capacity = capacity,
        .dma_padding_size =
            capacity - (size_t)handle->config.frame_width *
                           handle->config.frame_height,
    };
    handle->diagnostic_report = (hm01b0_diagnostic_report_t) {
        .pattern = handle->diagnostic_enabled
                       ? handle->diagnostic_config.pattern
                       : HM01B0_DIAGNOSTIC_PATTERN_NONE,
        .status = HM01B0_DIAGNOSTIC_STATUS_NOT_RUN,
    };
    handle->snapshot_result = (hm01b0_snapshot_result_t) {0};
    handle->isr_sequence = 0U;
    handle->isr_frames_received = 0U;
    handle->isr_no_free_buffer = 0U;
    handle->isr_ready_queue_overflows = 0U;
    handle->isr_free_queue_errors = 0U;
    handle->task_free_queue_errors = 0U;
    handle->processed_frames = 0U;
    handle->baseline_received_size_valid = false;
    handle->diagnostic_baseline_valid = false;
    handle->first_diagnostic_sample_valid = false;
    handle->first_diagnostic_sequence = 0U;
    handle->first_diagnostic_received_size = 0U;
    handle->first_diagnostic_raw_count = 0U;
    handle->first_diagnostic_active_count = 0U;
    handle->last_error_log_time_us = 0;
    handle->last_stats_time_us = esp_timer_get_time();
    handle->last_stats_frame_count = 0U;
}

static hm01b0_capture_frame_t *IRAM_ATTR hm01b0_capture_find_frame(
    hm01b0_capture_handle_t *handle,
    const void *buffer)
{
    for (size_t i = 0; i < HM01B0_CAPTURE_BUFFER_COUNT; ++i) {
        if (handle->frames[i].data == buffer) {
            return &handle->frames[i];
        }
    }
    return NULL;
}

static bool IRAM_ATTR hm01b0_capture_on_get_new_trans(
    esp_cam_ctlr_handle_t cam_handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data)
{
    (void)cam_handle;
    hm01b0_capture_handle_t *handle = user_data;
    hm01b0_capture_frame_t *frame = NULL;
    BaseType_t task_woken = pdFALSE;
    BaseType_t result;

    if (xPortInIsrContext()) {
        result = xQueueReceiveFromISR(handle->free_queue, &frame, &task_woken);
    } else {
        result = xQueueReceive(handle->free_queue, &frame, 0);
    }
    if (result != pdPASS || frame == NULL) {
        handle->isr_no_free_buffer++;
        return task_woken == pdTRUE;
    }

    trans->buffer = frame->data;
    trans->buflen = frame->capacity;
    return task_woken == pdTRUE;
}

static bool IRAM_ATTR hm01b0_capture_on_trans_finished(
    esp_cam_ctlr_handle_t cam_handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data)
{
    (void)cam_handle;
    hm01b0_capture_handle_t *handle = user_data;
    hm01b0_capture_frame_t *frame = hm01b0_capture_find_frame(
        handle, trans->buffer);
    BaseType_t task_woken = pdFALSE;

    if (frame == NULL) {
        handle->isr_free_queue_errors++;
        return false;
    }

    frame->received_size = trans->received_size;
    frame->sequence = ++handle->isr_sequence;
    /* Timestamp in task context to keep the ISR callback minimal. */
    frame->timestamp_us = 0;
    handle->isr_frames_received++;

    if (xQueueSendFromISR(handle->ready_queue, &frame, &task_woken) != pdPASS) {
        handle->isr_ready_queue_overflows++;
        if (xQueueSendFromISR(handle->free_queue, &frame, &task_woken) !=
            pdPASS) {
            handle->isr_free_queue_errors++;
        }
    }
    return task_woken == pdTRUE;
}

static bool hm01b0_capture_error_log_allowed(
    hm01b0_capture_handle_t *handle,
    int64_t now_us)
{
    if (handle->last_error_log_time_us == 0 ||
        now_us - handle->last_error_log_time_us >= 1000000LL) {
        handle->last_error_log_time_us = now_us;
        return true;
    }
    return false;
}

static void hm01b0_capture_update_isr_stats(
    hm01b0_capture_handle_t *handle)
{
    handle->transport.frames_received = handle->isr_frames_received;
    handle->transport.no_free_buffer = handle->isr_no_free_buffer;
    handle->transport.ready_queue_overflows =
        handle->isr_ready_queue_overflows;
    handle->transport.free_queue_errors = handle->isr_free_queue_errors +
                                          handle->task_free_queue_errors;
}

static void hm01b0_capture_log_diagnostic(
    const hm01b0_diagnostic_report_t *report)
{
    if (report->analyzed_frames == 0U) {
        return;
    }
    ESP_LOGI(TAG,
             "%s preflight=%s analyzed=%" PRIu32
             " warmup=%" PRIu32 " raw_crc=%08" PRIX32
             " active_crc=%08" PRIX32,
             hm01b0_diagnostic_pattern_name(report->pattern),
             hm01b0_diagnostic_status_name(report->status),
             report->analyzed_frames, report->warmup_frames,
             report->last_raw_crc, report->last_active_crc);

    if (report->pattern == HM01B0_DIAGNOSTIC_PATTERN_WALKING_1) {
        const hm01b0_walking_1_result_t *result = &report->result.walking_1;
        ESP_LOGI(TAG,
                 "Walking-1 rows=%" PRIu32 " equal=%" PRIu32 "/%" PRIu32
                 " vertical_mismatch=%" PRIu32 " transitions=%" PRIu32
                 " unique=%" PRIu32 " values(zero=%" PRIu32
                 ",one_hot=%" PRIu32 ",other=%" PRIu32 ")",
                 result->sampled_rows, result->rows_equal,
                 result->rows_compared, result->vertical_mismatches,
                 result->horizontal_transitions, result->unique_values,
                 result->zero_values, result->one_hot_values,
                 result->other_values);
    } else if (report->pattern == HM01B0_DIAGNOSTIC_PATTERN_COLOR_BAR) {
        const hm01b0_color_bar_result_t *result = &report->result.color_bar;
        ESP_LOGI(TAG,
                 "Color-Bar rows=%" PRIu32 " equal=%" PRIu32 "/%" PRIu32
                 " transitions=%" PRIu32 " strong=%" PRIu32
                 " unique=%" PRIu32 " range=%02X..%02X",
                 result->sampled_rows, result->rows_equal,
                 result->rows_compared, result->horizontal_transitions,
                 result->strong_transitions, result->unique_values,
                 (unsigned)result->min_value, (unsigned)result->max_value);
        ESP_LOGI(TAG,
                 "Color-Bar centers=[%02X %02X %02X %02X %02X %02X] "
                 "changes=%" PRIu32 "/5 center_vertical_mismatch=%" PRIu32,
                 (unsigned)result->center_values[0],
                 (unsigned)result->center_values[1],
                 (unsigned)result->center_values[2],
                 (unsigned)result->center_values[3],
                 (unsigned)result->center_values[4],
                 (unsigned)result->center_values[5],
                 result->center_changes,
                 result->center_vertical_mismatches);
    }
}

static void hm01b0_capture_log_stats(hm01b0_capture_handle_t *handle,
                                     int64_t now_us)
{
    hm01b0_capture_update_isr_stats(handle);
    const int64_t elapsed_us = now_us - handle->last_stats_time_us;
    if (elapsed_us <= 0) {
        return;
    }
    const uint32_t frames_now = handle->transport.frames_received;
    const uint32_t frames_delta = frames_now - handle->last_stats_frame_count;
    handle->transport.fps_milli = (uint32_t)(
        ((uint64_t)frames_delta * 1000000000ULL) / (uint64_t)elapsed_us);
    handle->last_stats_time_us = now_us;
    handle->last_stats_frame_count = frames_now;

    ESP_LOGI(TAG,
             "fps=%" PRIu32 ".%03" PRIu32 " received=%" PRIu32
             " valid=%" PRIu32 " size_err=%" PRIu32
             " size_change=%" PRIu32
             " no_buffer=%" PRIu32 " ready_overflow=%" PRIu32
             " free_err=%" PRIu32,
             handle->transport.fps_milli / 1000U,
             handle->transport.fps_milli % 1000U,
             handle->transport.frames_received,
             handle->transport.valid_frames,
             handle->transport.size_errors,
             handle->transport.received_size_changes,
             handle->transport.no_free_buffer,
             handle->transport.ready_queue_overflows,
             handle->transport.free_queue_errors);
    ESP_LOGI(TAG,
             "process_last=%" PRIu32 "us process_max=%" PRIu32
             "us hold_last=%" PRIu32 "us hold_max=%" PRIu32
             "us queues(free=%u,ready=%u)",
             handle->transport.last_processing_time_us,
             handle->transport.max_processing_time_us,
             handle->transport.last_buffer_hold_time_us,
             handle->transport.max_buffer_hold_time_us,
             (unsigned)uxQueueMessagesWaiting(handle->free_queue),
             (unsigned)uxQueueMessagesWaiting(handle->ready_queue));
    if (!handle->controller_started) {
        hm01b0_capture_log_diagnostic(&handle->diagnostic_report);
    }
}

static void hm01b0_capture_save_first_sample(
    hm01b0_capture_handle_t *handle,
    const hm01b0_capture_frame_t *frame)
{
    const size_t raw_count =
        handle->config.frame_width < HM01B0_CAPTURE_FIRST_SAMPLE_SIZE
            ? handle->config.frame_width
            : HM01B0_CAPTURE_FIRST_SAMPLE_SIZE;
    const size_t active_count =
        handle->diagnostic_config.area.width < HM01B0_CAPTURE_FIRST_SAMPLE_SIZE
            ? handle->diagnostic_config.area.width
            : HM01B0_CAPTURE_FIRST_SAMPLE_SIZE;
    const size_t active_offset =
        (size_t)handle->diagnostic_config.area.y *
            handle->config.frame_width +
        handle->diagnostic_config.area.x;

    memcpy(handle->first_diagnostic_raw, frame->data, raw_count);
    memcpy(handle->first_diagnostic_active,
           frame->data + active_offset, active_count);
    handle->first_diagnostic_sequence = frame->sequence;
    handle->first_diagnostic_received_size = frame->received_size;
    handle->first_diagnostic_raw_count = (uint16_t)raw_count;
    handle->first_diagnostic_active_count = (uint16_t)active_count;
    handle->first_diagnostic_sample_valid = true;
}

static void hm01b0_capture_log_saved_first_sample(
    const hm01b0_capture_handle_t *handle)
{
    if (!handle->first_diagnostic_sample_valid) {
        return;
    }

    ESP_LOGI(TAG,
             "saved first diagnostic frame=%" PRIu32 " received=%u; "
             "printed after Camera RX stopped",
             handle->first_diagnostic_sequence,
             (unsigned)handle->first_diagnostic_received_size);
    ESP_LOGI(TAG, "raw row y=0 x=0..%u",
             (unsigned)(handle->first_diagnostic_raw_count - 1U));
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, handle->first_diagnostic_raw,
                             handle->first_diagnostic_raw_count,
                             ESP_LOG_INFO);
    ESP_LOGI(TAG, "analysis row y=%u x=%u..%u",
             (unsigned)handle->diagnostic_config.area.y,
             (unsigned)handle->diagnostic_config.area.x,
             (unsigned)(handle->diagnostic_config.area.x +
                        handle->first_diagnostic_active_count - 1U));
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, handle->first_diagnostic_active,
                             handle->first_diagnostic_active_count,
                             ESP_LOG_INFO);
}

static void hm01b0_capture_process_transport(
    hm01b0_capture_handle_t *handle,
    const hm01b0_capture_frame_t *frame,
    int64_t now_us,
    bool *size_valid)
{
    handle->transport.last_received_size = frame->received_size;
    *size_valid = hm01b0_capture_frame_size_is_valid(handle, frame);

    if (!handle->baseline_received_size_valid) {
        handle->baseline_received_size = frame->received_size;
        handle->baseline_received_size_valid = true;
    } else if (frame->received_size != handle->baseline_received_size) {
        handle->transport.received_size_changes++;
    }

    if (*size_valid) {
        handle->transport.valid_frames++;
    } else {
        handle->transport.size_errors++;
        if (hm01b0_capture_error_log_allowed(handle, now_us)) {
            ESP_LOGE(TAG,
                     "frame=%" PRIu32
                     " transaction size error: received=%u "
                     "logical_payload=%u dma_capacity=%u",
                     frame->sequence, (unsigned)frame->received_size,
                     (unsigned)handle->transport.payload_size,
                     (unsigned)handle->transport.buffer_capacity);
        }
    }
}

static void hm01b0_capture_process_diagnostics(
    hm01b0_capture_handle_t *handle,
    const hm01b0_capture_frame_t *frame)
{
    if (!handle->diagnostic_enabled) {
        return;
    }
    if (handle->processed_frames <= handle->diagnostic_config.warmup_frames) {
        handle->diagnostic_report.warmup_frames++;
        return;
    }

    const uint32_t post_warmup_index =
        handle->processed_frames -
        handle->diagnostic_config.warmup_frames - 1U;
    if (post_warmup_index %
            handle->diagnostic_config.sample_interval_frames != 0U) {
        return;
    }

    if (hm01b0_diagnostics_analyze_frame(
            handle->diagnostic_config.pattern,
            frame->data,
            handle->config.frame_width,
            handle->config.frame_height,
            handle->config.frame_width,
            handle->diagnostic_config.area,
            &handle->diagnostic_report) != ESP_OK) {
        handle->diagnostic_report.status = HM01B0_DIAGNOSTIC_STATUS_WARNING;
        return;
    }

    if (!handle->diagnostic_baseline_valid) {
        handle->diagnostic_baseline_raw_crc =
            handle->diagnostic_report.last_raw_crc;
        handle->diagnostic_baseline_active_crc =
            handle->diagnostic_report.last_active_crc;
        handle->diagnostic_baseline_valid = true;
    } else {
        if (handle->diagnostic_report.last_raw_crc !=
            handle->diagnostic_baseline_raw_crc) {
            handle->diagnostic_report.raw_crc_changes++;
        }
        if (handle->diagnostic_report.last_active_crc !=
            handle->diagnostic_baseline_active_crc) {
            handle->diagnostic_report.active_crc_changes++;
        }
    }

    if (!handle->first_diagnostic_sample_valid) {
        hm01b0_capture_save_first_sample(handle, frame);
    }
}

static bool hm01b0_capture_process_snapshot(
    hm01b0_capture_handle_t *handle,
    const hm01b0_capture_frame_t *frame)
{
    if (!handle->snapshot_pending ||
        handle->processed_frames <= handle->snapshot_request.skip_frames) {
        return false;
    }

    const int64_t copy_start_us = esp_timer_get_time();
    const esp_err_t ret = hm01b0_frame_crop_raw8(
        frame->data,
        handle->config.frame_width,
        handle->config.frame_height,
        handle->config.frame_width,
        handle->snapshot_request.crop,
        handle->snapshot_request.buffer,
        handle->snapshot_request.buffer_size,
        handle->snapshot_request.crop.width);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "snapshot copy failed: %s", esp_err_to_name(ret));
        handle->snapshot_pending = false;
        return false;
    }

    handle->snapshot_result.sequence = frame->sequence;
    handle->snapshot_result.size =
        (size_t)handle->snapshot_request.crop.width *
        handle->snapshot_request.crop.height;
    handle->snapshot_result.copy_time_us = (uint32_t)(
        esp_timer_get_time() - copy_start_us);
    handle->snapshot_pending = false;
    return true;
}

static void hm01b0_capture_task(void *arg)
{
    hm01b0_capture_handle_t *handle = arg;

    while (!handle->task_should_exit) {
        hm01b0_capture_frame_t *frame = NULL;
        if (xQueueReceive(handle->ready_queue, &frame,
                          pdMS_TO_TICKS(HM01B0_CAPTURE_QUEUE_WAIT_MS)) !=
            pdPASS) {
            const int64_t now_us = esp_timer_get_time();
            if (handle->controller_started &&
                now_us - handle->last_stats_time_us >=
                    (int64_t)HM01B0_CAPTURE_STATS_PERIOD_MS * 1000LL) {
                hm01b0_capture_log_stats(handle, now_us);
            }
            continue;
        }
        if (frame == NULL) {
            continue;
        }

        const int64_t hold_start_us = esp_timer_get_time();
        frame->timestamp_us = hold_start_us;
        const int64_t process_start_us = hold_start_us;
        handle->processing_frame = true;
        handle->processed_frames++;

        bool size_valid = false;
        hm01b0_capture_process_transport(handle, frame, process_start_us,
                                         &size_valid);
        bool snapshot_ready = false;
        if (size_valid) {
            hm01b0_capture_process_diagnostics(handle, frame);
            snapshot_ready = hm01b0_capture_process_snapshot(handle, frame);
            if (handle->frame_consumer != NULL) {
                handle->frame_consumer(
                    frame, handle->frame_consumer_user_data);
            }
        }

        const uint32_t processing_time_us = (uint32_t)(
            esp_timer_get_time() - process_start_us);
        handle->transport.last_processing_time_us = processing_time_us;
        if (processing_time_us > handle->transport.max_processing_time_us) {
            handle->transport.max_processing_time_us = processing_time_us;
        }

        const bool frame_returned =
            xQueueSend(handle->free_queue, &frame, 0) == pdPASS;
        if (!frame_returned) {
            handle->task_free_queue_errors++;
        }
        const uint32_t hold_time_us = (uint32_t)(
            esp_timer_get_time() - hold_start_us);
        handle->transport.last_buffer_hold_time_us = hold_time_us;
        if (hold_time_us > handle->transport.max_buffer_hold_time_us) {
            handle->transport.max_buffer_hold_time_us = hold_time_us;
        }
        if (snapshot_ready) {
            handle->snapshot_result.buffer_hold_time_us = hold_time_us;
        }
        handle->processing_frame = false;

        if (snapshot_ready && frame_returned) {
            const hm01b0_frame_rect_t crop = handle->snapshot_request.crop;
            ESP_LOGI(TAG,
                     "snapshot ready: frame=%" PRIu32
                     " crop=(%u,%u %ux%u), bytes=%u, copy=%" PRIu32
                     "us, buffer_hold=%" PRIu32 "us",
                     handle->snapshot_result.sequence,
                     (unsigned)crop.x, (unsigned)crop.y,
                     (unsigned)crop.width, (unsigned)crop.height,
                     (unsigned)handle->snapshot_result.size,
                     handle->snapshot_result.copy_time_us,
                     handle->snapshot_result.buffer_hold_time_us);
            handle->snapshot_request.on_ready(
                &handle->snapshot_result,
                handle->snapshot_request.user_data);
        } else if (snapshot_ready) {
            ESP_LOGE(TAG,
                     "snapshot Camera buffer was not returned; notification "
                     "suppressed");
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us - handle->last_stats_time_us >=
            (int64_t)HM01B0_CAPTURE_STATS_PERIOD_MS * 1000LL) {
            hm01b0_capture_log_stats(handle, now_us);
        }
    }

    handle->task_handle = NULL;
    vTaskDelete(NULL);
}

static void hm01b0_capture_log_memory(const hm01b0_capture_handle_t *handle)
{
    ESP_LOGI(TAG,
             "%s heap before: free=%u largest=%u; after: free=%u largest=%u",
             hm01b0_capture_buffer_memory_name(
                 handle->config.buffer_memory),
             (unsigned)handle->dma_heap_free_before,
             (unsigned)handle->dma_heap_largest_before,
             (unsigned)handle->dma_heap_free_after,
             (unsigned)handle->dma_heap_largest_after);
    ESP_LOGI(TAG,
             "buffers: A=%p (%s), B=%p (%s), each=%u bytes, "
             "logical_payload=%u bytes, dma_padding=%u bytes",
             handle->frames[0].data,
             esp_ptr_internal(handle->frames[0].data) ? "internal" : "external",
             handle->frames[1].data,
             esp_ptr_internal(handle->frames[1].data) ? "internal" : "external",
             (unsigned)handle->transport.buffer_capacity,
             (unsigned)handle->transport.payload_size,
             (unsigned)handle->transport.dma_padding_size);
}

esp_err_t hm01b0_capture_new(const hm01b0_capture_config_t *config,
                             hm01b0_capture_handle_t **out_handle)
{
    ESP_RETURN_ON_ERROR(hm01b0_capture_validate_config(config), TAG,
                        "invalid capture configuration");
    ESP_RETURN_ON_FALSE(out_handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "output handle is NULL");
    *out_handle = NULL;

    hm01b0_capture_handle_t *handle = heap_caps_calloc(
        1, sizeof(*handle), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG,
                        "failed to allocate capture context");
    handle->config = *config;

    const uint32_t dma_caps = hm01b0_capture_buffer_caps(
        config->buffer_memory);
    const char *buffer_memory_name = hm01b0_capture_buffer_memory_name(
        config->buffer_memory);
    esp_err_t ret = ESP_OK;
    if (config->buffer_memory == HM01B0_CAPTURE_BUFFER_PSRAM &&
        heap_caps_get_total_size(dma_caps) == 0U) {
        ESP_LOGE(TAG,
                 "PSRAM DMA heap is unavailable; enable PSRAM and expose it "
                 "through the capability allocator");
        ret = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }

    const esp_cam_ctlr_dvp_pin_config_t pin_config = {
        .data_width = CAM_CTLR_DATA_WIDTH_8,
        .data_io = {
            config->data_gpio[0], config->data_gpio[1],
            config->data_gpio[2], config->data_gpio[3],
            config->data_gpio[4], config->data_gpio[5],
            config->data_gpio[6], config->data_gpio[7],
        },
        .vsync_io = config->vsync_gpio,
        .de_io = config->de_gpio,
        .pclk_io = config->pclk_gpio,
        .xclk_io = GPIO_NUM_NC,
    };
    const esp_cam_ctlr_dvp_config_t dvp_config = {
        .ctlr_id = 0,
        .clk_src = CAM_CLK_SRC_DEFAULT,
        .h_res = config->frame_width,
        .v_res = config->frame_height,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RAW8,
        .cam_data_width = 8U,
        .bit_swap_en = 0,
        .byte_swap_en = 0,
        .bk_buffer_dis = 1,
        .pic_format_jpeg = 0,
        .external_xtal = 1,
        .dma_burst_size = config->dma_burst_size,
        .xclk_freq = 0,
        .pin = &pin_config,
    };

    ret = esp_cam_new_dvp_ctlr(&dvp_config, &handle->cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to create DVP controller: %s",
                 esp_err_to_name(ret));
        goto fail;
    }

    ret = esp_cam_ctlr_get_frame_buffer_len(
        handle->cam_handle, &handle->transport.buffer_capacity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to query DMA buffer length: %s",
                 esp_err_to_name(ret));
        goto fail;
    }
    handle->transport.frame_width = config->frame_width;
    handle->transport.frame_height = config->frame_height;
    handle->transport.frame_stride = config->frame_width;
    handle->transport.payload_size = (size_t)config->frame_width *
                                     config->frame_height;
    if (handle->transport.buffer_capacity < handle->transport.payload_size) {
        ESP_LOGE(TAG, "driver buffer length is smaller than RAW8 payload");
        ret = ESP_ERR_INVALID_SIZE;
        goto fail;
    }
    handle->transport.dma_padding_size =
        handle->transport.buffer_capacity - handle->transport.payload_size;

    handle->dma_heap_free_before = heap_caps_get_free_size(dma_caps);
    handle->dma_heap_largest_before =
        heap_caps_get_largest_free_block(dma_caps);
    ESP_LOGI(TAG,
             "allocating %u Camera Buffers, each=%u bytes in %s; "
             "heap free=%u largest=%u",
             (unsigned)HM01B0_CAPTURE_BUFFER_COUNT,
             (unsigned)handle->transport.buffer_capacity,
             buffer_memory_name,
             (unsigned)handle->dma_heap_free_before,
             (unsigned)handle->dma_heap_largest_before);

    for (size_t i = 0; i < HM01B0_CAPTURE_BUFFER_COUNT; ++i) {
        const size_t free_before_buffer =
            heap_caps_get_free_size(dma_caps);
        const size_t largest_before_buffer =
            heap_caps_get_largest_free_block(dma_caps);
        handle->frames[i].data = esp_cam_ctlr_alloc_buffer(
            handle->cam_handle, handle->transport.buffer_capacity, dma_caps);
        if (handle->frames[i].data == NULL) {
            ESP_LOGE(TAG,
                     "failed to allocate %s Buffer %c: "
                     "required=%u free=%u largest=%u",
                     buffer_memory_name,
                     (int)('A' + i),
                     (unsigned)handle->transport.buffer_capacity,
                     (unsigned)free_before_buffer,
                     (unsigned)largest_before_buffer);
            ret = ESP_ERR_NO_MEM;
            goto fail;
        }
        const bool location_valid =
            config->buffer_memory == HM01B0_CAPTURE_BUFFER_PSRAM
                ? esp_ptr_external_ram(handle->frames[i].data)
                : esp_ptr_internal(handle->frames[i].data);
        if (!location_valid) {
            ESP_LOGE(TAG, "Buffer %c was not allocated in %s",
                     (int)('A' + i), buffer_memory_name);
            ret = ESP_ERR_INVALID_STATE;
            goto fail;
        }
        handle->frames[i].payload_size = handle->transport.payload_size;
        handle->frames[i].capacity = handle->transport.buffer_capacity;
        ESP_LOGI(TAG,
                 "Buffer %c allocated at %p: required=%u; heap now free=%u "
                 "largest=%u",
                 (int)('A' + i), handle->frames[i].data,
                 (unsigned)handle->transport.buffer_capacity,
                 (unsigned)heap_caps_get_free_size(dma_caps),
                 (unsigned)heap_caps_get_largest_free_block(dma_caps));
    }

    handle->dma_heap_free_after = heap_caps_get_free_size(dma_caps);
    handle->dma_heap_largest_after =
        heap_caps_get_largest_free_block(dma_caps);

    handle->free_queue = xQueueCreateStatic(
        HM01B0_CAPTURE_BUFFER_COUNT, sizeof(hm01b0_capture_frame_t *),
        handle->free_queue_storage, &handle->free_queue_state);
    handle->ready_queue = xQueueCreateStatic(
        HM01B0_CAPTURE_BUFFER_COUNT, sizeof(hm01b0_capture_frame_t *),
        handle->ready_queue_storage, &handle->ready_queue_state);
    if (handle->free_queue == NULL || handle->ready_queue == NULL) {
        ESP_LOGE(TAG, "failed to create frame queues");
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    ESP_GOTO_ON_ERROR(hm01b0_capture_reset_queues(handle), fail, TAG,
                      "failed to seed free queue");

    const esp_cam_ctlr_evt_cbs_t callbacks = {
        .on_get_new_trans = hm01b0_capture_on_get_new_trans,
        .on_trans_finished = hm01b0_capture_on_trans_finished,
    };
    ret = esp_cam_ctlr_register_event_callbacks(handle->cam_handle,
                                                 &callbacks, handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to register Camera callbacks: %s",
                 esp_err_to_name(ret));
        goto fail;
    }

    if (xTaskCreate(hm01b0_capture_task, "hm01b0_frame",
                    HM01B0_CAPTURE_TASK_STACK_SIZE, handle,
                    HM01B0_CAPTURE_TASK_PRIORITY,
                    &handle->task_handle) != pdPASS) {
        ESP_LOGE(TAG, "failed to create frame task");
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }

    ESP_LOGI(TAG,
             "DVP ready: RAW8 %ux%u, logical_payload=%u, dma_capacity=%u, "
             "dma_padding=%u, burst=%" PRIu32
             ", buffers=2, camera_memory=%s, backup=disabled",
             config->frame_width, config->frame_height,
             (unsigned)handle->transport.payload_size,
             (unsigned)handle->transport.buffer_capacity,
             (unsigned)handle->transport.dma_padding_size,
             config->dma_burst_size, buffer_memory_name);
    hm01b0_capture_log_memory(handle);
    *out_handle = handle;
    return ESP_OK;

fail:
    (void)hm01b0_capture_delete(handle);
    return ret;
}

esp_err_t hm01b0_capture_set_diagnostics(
    hm01b0_capture_handle_t *handle,
    const hm01b0_diagnostic_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "capture handle is NULL");
    ESP_RETURN_ON_FALSE(!handle->controller_started,
                        ESP_ERR_INVALID_STATE, TAG,
                        "stop Camera RX before changing diagnostics");
    if (config == NULL || config->pattern == HM01B0_DIAGNOSTIC_PATTERN_NONE) {
        handle->diagnostic_enabled = false;
        handle->diagnostic_config = (hm01b0_diagnostic_config_t) {0};
        return ESP_OK;
    }
    ESP_RETURN_ON_FALSE(
        config->pattern == HM01B0_DIAGNOSTIC_PATTERN_WALKING_1 ||
        config->pattern == HM01B0_DIAGNOSTIC_PATTERN_COLOR_BAR,
        ESP_ERR_INVALID_ARG, TAG, "invalid diagnostic pattern");
    ESP_RETURN_ON_FALSE(hm01b0_capture_rect_is_valid(handle, config->area),
                        ESP_ERR_INVALID_ARG, TAG,
                        "diagnostic area is outside the frame");
    ESP_RETURN_ON_FALSE(config->sample_interval_frames > 0U,
                        ESP_ERR_INVALID_ARG, TAG,
                        "diagnostic sample interval must be greater than 0");
    handle->diagnostic_config = *config;
    handle->diagnostic_enabled = true;
    ESP_LOGI(TAG, "diagnostics configured: pattern=%s area=(%u,%u %ux%u) "
                  "warmup=%" PRIu32 " sample_interval=%" PRIu32,
             hm01b0_diagnostic_pattern_name(config->pattern),
             (unsigned)config->area.x, (unsigned)config->area.y,
             (unsigned)config->area.width, (unsigned)config->area.height,
             config->warmup_frames, config->sample_interval_frames);
    return ESP_OK;
}

esp_err_t hm01b0_capture_request_snapshot(
    hm01b0_capture_handle_t *handle,
    const hm01b0_snapshot_request_t *request)
{
    ESP_RETURN_ON_FALSE(handle != NULL && request != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid snapshot request");
    ESP_RETURN_ON_FALSE(!handle->controller_started,
                        ESP_ERR_INVALID_STATE, TAG,
                        "stop Camera RX before requesting a snapshot");
    ESP_RETURN_ON_FALSE(request->buffer != NULL && request->on_ready != NULL,
                        ESP_ERR_INVALID_ARG, TAG,
                        "snapshot buffer or callback is NULL");
    ESP_RETURN_ON_FALSE(hm01b0_capture_rect_is_valid(handle, request->crop),
                        ESP_ERR_INVALID_ARG, TAG,
                        "snapshot crop is outside the frame");
    const size_t required = (size_t)request->crop.width * request->crop.height;
    ESP_RETURN_ON_FALSE(request->buffer_size >= required,
                        ESP_ERR_INVALID_SIZE, TAG,
                        "snapshot buffer is too small");
    handle->snapshot_request = *request;
    handle->snapshot_pending = true;
    handle->snapshot_result = (hm01b0_snapshot_result_t) {0};
    ESP_LOGI(TAG,
             "snapshot requested: crop=(%u,%u %ux%u), buffer=%p, bytes=%u, "
             "skip=%" PRIu32,
             (unsigned)request->crop.x, (unsigned)request->crop.y,
             (unsigned)request->crop.width, (unsigned)request->crop.height,
             request->buffer, (unsigned)required, request->skip_frames);
    return ESP_OK;
}

esp_err_t hm01b0_capture_set_frame_consumer(
    hm01b0_capture_handle_t *handle,
    hm01b0_capture_frame_consumer_t consumer,
    void *user_data)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "capture handle is NULL");
    ESP_RETURN_ON_FALSE(!handle->controller_started,
                        ESP_ERR_INVALID_STATE, TAG,
                        "stop Camera RX before changing frame consumer");
    ESP_RETURN_ON_FALSE(!handle->processing_frame,
                        ESP_ERR_INVALID_STATE, TAG,
                        "a frame is still being processed");
    handle->frame_consumer = consumer;
    handle->frame_consumer_user_data = user_data;
    ESP_LOGI(TAG, "live frame consumer %s",
             consumer != NULL ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t hm01b0_capture_rx_start(hm01b0_capture_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "capture handle is NULL");
    ESP_RETURN_ON_FALSE(!handle->controller_started,
                        ESP_ERR_INVALID_STATE, TAG,
                        "Camera RX is already started");
    ESP_RETURN_ON_FALSE(!handle->processing_frame,
                        ESP_ERR_INVALID_STATE, TAG,
                        "a frame is still being processed");

    hm01b0_capture_reset_session(handle);
    ESP_RETURN_ON_ERROR(hm01b0_capture_reset_queues(handle), TAG,
                        "failed to prepare frame queues");

    esp_err_t ret = esp_cam_ctlr_enable(handle->cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to enable Camera RX: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    handle->controller_enabled = true;

    ret = esp_cam_ctlr_start(handle->cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to start Camera RX: %s", esp_err_to_name(ret));
        (void)esp_cam_ctlr_disable(handle->cam_handle);
        handle->controller_enabled = false;
        return ret;
    }
    handle->controller_started = true;
    ESP_LOGI(TAG, "Camera RX started and waiting for HM01B0 FVLD/PCLK");
    return ESP_OK;
}

esp_err_t hm01b0_capture_rx_stop(hm01b0_capture_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_OK;
    }

    esp_err_t result = ESP_OK;
    if (handle->controller_started) {
        const esp_err_t ret = esp_cam_ctlr_stop(handle->cam_handle);
        if (ret != ESP_OK) {
            result = ret;
        } else {
            handle->controller_started = false;
        }
    }
    if (handle->controller_enabled) {
        const esp_err_t ret = esp_cam_ctlr_disable(handle->cam_handle);
        if (result == ESP_OK) {
            result = ret;
        }
        if (ret == ESP_OK) {
            handle->controller_enabled = false;
        }
    }

    if (result == ESP_OK && handle->free_queue != NULL &&
        handle->ready_queue != NULL) {
        const TickType_t wait_ticks = pdMS_TO_TICKS(10U);
        const uint32_t wait_count =
            HM01B0_CAPTURE_TASK_STOP_TIMEOUT_MS / 10U;
        uint32_t i = 0U;
        while ((handle->processing_frame ||
                uxQueueMessagesWaiting(handle->ready_queue) != 0U) &&
               i < wait_count) {
            vTaskDelay(wait_ticks);
            ++i;
        }
        if (handle->processing_frame ||
            uxQueueMessagesWaiting(handle->ready_queue) != 0U) {
            ESP_LOGE(TAG, "timed out waiting for frame task to become idle");
            result = ESP_ERR_TIMEOUT;
        } else {
            result = hm01b0_capture_reset_queues(handle);
        }
    }
    if (result == ESP_OK) {
        if (handle->free_queue != NULL && handle->ready_queue != NULL &&
            handle->last_stats_time_us != 0) {
            hm01b0_capture_log_stats(handle, esp_timer_get_time());
        }
        hm01b0_capture_log_saved_first_sample(handle);
        ESP_LOGI(TAG, "Camera RX stopped and disabled");
    }
    return result;
}

esp_err_t hm01b0_capture_delete(hm01b0_capture_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_OK;
    }

    esp_err_t result = hm01b0_capture_rx_stop(handle);
    if (handle->controller_started || handle->controller_enabled) {
        ESP_LOGE(TAG, "capture resources remain owned because Camera RX "
                      "could not be stopped safely");
        return result == ESP_OK ? ESP_ERR_INVALID_STATE : result;
    }

    handle->task_should_exit = true;
    const TickType_t wait_ticks = pdMS_TO_TICKS(10U);
    const uint32_t wait_count = HM01B0_CAPTURE_TASK_STOP_TIMEOUT_MS / 10U;
    for (uint32_t i = 0; handle->task_handle != NULL && i < wait_count; ++i) {
        vTaskDelay(wait_ticks);
    }
    if (handle->task_handle != NULL) {
        vTaskDelete(handle->task_handle);
        handle->task_handle = NULL;
    }

    if (handle->cam_handle != NULL) {
        const esp_err_t ret = esp_cam_ctlr_del(handle->cam_handle);
        if (result == ESP_OK) {
            result = ret;
        }
    }
    for (size_t i = 0; i < HM01B0_CAPTURE_BUFFER_COUNT; ++i) {
        heap_caps_free(handle->frames[i].data);
    }
    if (handle->free_queue != NULL) {
        vQueueDelete(handle->free_queue);
    }
    if (handle->ready_queue != NULL) {
        vQueueDelete(handle->ready_queue);
    }
    heap_caps_free(handle);
    return result;
}

esp_err_t hm01b0_capture_get_transport_stats(
    const hm01b0_capture_handle_t *handle,
    hm01b0_capture_transport_stats_t *stats)
{
    ESP_RETURN_ON_FALSE(handle != NULL && stats != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    *stats = handle->transport;
    stats->frames_received = handle->isr_frames_received;
    stats->no_free_buffer = handle->isr_no_free_buffer;
    stats->ready_queue_overflows = handle->isr_ready_queue_overflows;
    stats->free_queue_errors = handle->isr_free_queue_errors +
                               handle->task_free_queue_errors;
    return ESP_OK;
}

esp_err_t hm01b0_capture_get_diagnostic_report(
    const hm01b0_capture_handle_t *handle,
    hm01b0_diagnostic_report_t *report)
{
    ESP_RETURN_ON_FALSE(handle != NULL && report != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    *report = handle->diagnostic_report;
    return ESP_OK;
}

esp_err_t hm01b0_capture_get_snapshot_result(
    const hm01b0_capture_handle_t *handle,
    hm01b0_snapshot_result_t *result)
{
    ESP_RETURN_ON_FALSE(handle != NULL && result != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(handle->snapshot_result.size > 0U,
                        ESP_ERR_NOT_FOUND, TAG,
                        "no completed snapshot is available");
    *result = handle->snapshot_result;
    return ESP_OK;
}
