#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_check.h"
#include "esp_crc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hm01b0_capture.h"

#define HM01B0_CAPTURE_DEFAULT_TASK_STACK_SIZE 4096U
#define HM01B0_CAPTURE_DEFAULT_TASK_PRIORITY   5U
#define HM01B0_CAPTURE_DEFAULT_STATS_PERIOD_MS 1000U
#define HM01B0_CAPTURE_DEFAULT_WARMUP_FRAMES   5U
#define HM01B0_CAPTURE_FIRST_SAMPLE_SIZE       32U
#define HM01B0_CAPTURE_QUEUE_WAIT_MS           100U
#define HM01B0_CAPTURE_TASK_STOP_TIMEOUT_MS    500U

typedef struct {
    uint8_t *data;
    size_t received_size;
    uint32_t sequence;
} hm01b0_capture_frame_t;

typedef struct {
    uint32_t rows_equal;
    uint32_t rows_compared;
    uint32_t vertical_mismatches;
    uint32_t horizontal_transitions;
    uint32_t unique_values;
    uint32_t zero_values;
    uint32_t one_hot_values;
    uint32_t other_values;
} hm01b0_walking_analysis_t;

typedef struct {
    uint32_t sequence;
    uint32_t warmup_frames;
    size_t received_size;
    uint32_t raw_crc;
    uint32_t active_crc;
    size_t raw_sample_size;
    size_t active_sample_size;
    uint16_t active_x;
    uint16_t active_rows[3];
    uint8_t raw[HM01B0_CAPTURE_FIRST_SAMPLE_SIZE];
    uint8_t active[3][HM01B0_CAPTURE_FIRST_SAMPLE_SIZE];
} hm01b0_analysis_sample_t;

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
    hm01b0_capture_stats_t stats;

    bool baseline_crc_valid;
    uint32_t baseline_raw_crc;
    uint32_t baseline_active_crc;
    bool previous_active_crc_valid;
    uint32_t previous_active_crc;
    bool baseline_received_size_valid;
    size_t baseline_received_size;
    uint32_t processed_frames;
    bool first_analysis_logged;
    int64_t last_stats_time_us;
    int64_t last_error_log_time_us;
    uint32_t last_stats_frame_count;

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

static esp_err_t hm01b0_capture_reset_queues(
    hm01b0_capture_handle_t *handle)
{
    xQueueReset(handle->free_queue);
    xQueueReset(handle->ready_queue);
    for (size_t i = 0; i < HM01B0_CAPTURE_BUFFER_COUNT; ++i) {
        hm01b0_capture_frame_t *frame = &handle->frames[i];
        frame->received_size = 0U;
        if (xQueueSend(handle->free_queue, &frame, 0) != pdPASS) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

static void hm01b0_capture_reset_diagnostics(
    hm01b0_capture_handle_t *handle)
{
    const size_t buffer_capacity = handle->stats.buffer_capacity;
    handle->stats = (hm01b0_capture_stats_t) {
        .raw_width = handle->config.raw_width,
        .raw_height = handle->config.raw_height,
        .raw_stride = handle->config.raw_width,
        .active_x = handle->config.active_x,
        .active_y = handle->config.active_y,
        .active_width = handle->config.active_width,
        .active_height = handle->config.active_height,
        .payload_size = (size_t)handle->config.raw_width *
                        handle->config.raw_height,
        .active_payload_size = (size_t)handle->config.active_width *
                               handle->config.active_height,
        .buffer_capacity = buffer_capacity,
    };
    handle->isr_sequence = 0U;
    handle->isr_frames_received = 0U;
    handle->isr_no_free_buffer = 0U;
    handle->isr_ready_queue_overflows = 0U;
    handle->isr_free_queue_errors = 0U;
    handle->task_free_queue_errors = 0U;
    handle->baseline_crc_valid = false;
    handle->previous_active_crc_valid = false;
    handle->baseline_received_size_valid = false;
    handle->processed_frames = 0U;
    handle->first_analysis_logged = false;
    handle->last_error_log_time_us = 0;
    handle->last_stats_time_us = esp_timer_get_time();
    handle->last_stats_frame_count = 0U;
}

static bool hm01b0_capture_valid_dma_burst(uint32_t burst_size)
{
    return burst_size == 0U ||
           (burst_size >= 4U && burst_size <= 128U &&
            (burst_size & (burst_size - 1U)) == 0U);
}

static esp_err_t hm01b0_capture_validate_config(
    const hm01b0_capture_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "configuration is NULL");
    ESP_RETURN_ON_FALSE(config->raw_width > 0U && config->raw_height > 0U,
                        ESP_ERR_INVALID_ARG, TAG, "invalid raw geometry");
    ESP_RETURN_ON_FALSE(config->active_width > 0U &&
                        config->active_height > 0U &&
                        config->active_x < config->raw_width &&
                        config->active_y < config->raw_height &&
                        config->active_width <=
                            config->raw_width - config->active_x &&
                        config->active_height <=
                            config->raw_height - config->active_y,
                        ESP_ERR_INVALID_ARG, TAG,
                        "active area is outside the raw frame");
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
    ESP_RETURN_ON_FALSE(config->task_priority < configMAX_PRIORITIES,
                        ESP_ERR_INVALID_ARG, TAG, "invalid task priority");
    return ESP_OK;
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
    trans->buflen = handle->stats.buffer_capacity;
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

static bool hm01b0_is_one_hot(uint8_t value)
{
    return value != 0U && (value & (uint8_t)(value - 1U)) == 0U;
}

static uint32_t hm01b0_capture_active_crc(
    const hm01b0_capture_handle_t *handle,
    const uint8_t *data)
{
    uint32_t crc = 0U;
    const size_t stride = handle->config.raw_width;
    for (uint16_t y = 0; y < handle->config.active_height; ++y) {
        const size_t offset =
            (size_t)(handle->config.active_y + y) * stride +
            handle->config.active_x;
        crc = esp_crc32_le(crc, data + offset,
                           handle->config.active_width);
    }
    return crc;
}

static hm01b0_walking_analysis_t hm01b0_analyze_walking_one(
    const uint8_t *data,
    uint16_t stride,
    uint16_t active_x,
    uint16_t active_y,
    uint16_t active_width,
    uint16_t active_height)
{
    hm01b0_walking_analysis_t result = {0};
    uint32_t seen_values[8] = {0};

    if (data == NULL || stride == 0U || active_width == 0U ||
        active_height == 0U) {
        return result;
    }

    const uint8_t *reference = data + (size_t)active_y * stride + active_x;
    for (uint16_t x = 0; x < active_width; ++x) {
        if (x > 0U && reference[x] != reference[x - 1U]) {
            result.horizontal_transitions++;
        }
    }

    for (uint16_t y = 0; y < active_height; ++y) {
        const uint8_t *row = data +
                             (size_t)(active_y + y) * stride + active_x;
        bool row_equal = true;
        for (uint16_t x = 0; x < active_width; ++x) {
            const uint8_t value = row[x];
            const uint32_t value_bit = 1UL << (value & 31U);
            uint32_t *const seen_word = &seen_values[value >> 5U];
            if ((*seen_word & value_bit) == 0U) {
                *seen_word |= value_bit;
                result.unique_values++;
            }
            if (value == 0U) {
                result.zero_values++;
            } else if (hm01b0_is_one_hot(value)) {
                result.one_hot_values++;
            } else {
                result.other_values++;
            }
            if (y > 0U && value != reference[x]) {
                result.vertical_mismatches++;
                row_equal = false;
            }
        }
        if (y > 0U) {
            result.rows_compared++;
            if (row_equal) {
                result.rows_equal++;
            }
        }
    }

    return result;
}

static bool hm01b0_capture_error_log_allowed(hm01b0_capture_handle_t *handle,
                                              int64_t now_us)
{
    if (handle->last_error_log_time_us == 0 ||
        now_us - handle->last_error_log_time_us >= 1000000LL) {
        handle->last_error_log_time_us = now_us;
        return true;
    }
    return false;
}

static void hm01b0_capture_log_stats(hm01b0_capture_handle_t *handle,
                                     int64_t now_us)
{
    handle->stats.frames_received = handle->isr_frames_received;
    handle->stats.no_free_buffer = handle->isr_no_free_buffer;
    handle->stats.ready_queue_overflows =
        handle->isr_ready_queue_overflows;
    handle->stats.free_queue_errors = handle->isr_free_queue_errors +
                                      handle->task_free_queue_errors;
    const int64_t elapsed_us = now_us - handle->last_stats_time_us;
    const uint32_t frames_now = handle->stats.frames_received;
    const uint32_t frames_delta = frames_now - handle->last_stats_frame_count;

    if (elapsed_us <= 0) {
        return;
    }

    handle->stats.fps_milli = (uint32_t)(
        ((uint64_t)frames_delta * 1000000000ULL) / (uint64_t)elapsed_us);
    handle->last_stats_time_us = now_us;
    handle->last_stats_frame_count = frames_now;

    ESP_LOGI(TAG,
             "fps=%" PRIu32 ".%03" PRIu32 " received=%" PRIu32
             " transport_ok=%" PRIu32 " warmup=%" PRIu32
             " size_err=%" PRIu32 " size_change=%" PRIu32,
             handle->stats.fps_milli / 1000U,
             handle->stats.fps_milli % 1000U,
             handle->stats.frames_received,
             handle->stats.transport_valid_frames,
             handle->stats.warmup_frames,
             handle->stats.size_errors,
             handle->stats.received_size_changes);
    ESP_LOGI(TAG,
             "crc raw=%08" PRIX32 " active=%08" PRIX32
             " changes(raw_base=%" PRIu32 ",active_base=%" PRIu32
             ",active_prev=%" PRIu32 ")",
             handle->stats.last_raw_crc,
             handle->stats.last_active_crc,
             handle->stats.raw_crc_changes,
             handle->stats.active_crc_changes,
             handle->stats.active_crc_frame_changes);
    if (handle->config.analyze_walking_1 &&
        handle->stats.walking_analysis_frames > 0U) {
        ESP_LOGI(TAG,
                 "Walking-1 observe rows_equal=%" PRIu32 "/%" PRIu32
                 " vertical_mismatch=%" PRIu32
                 " transitions=%" PRIu32 " unique=%" PRIu32
                 " values(zero=%" PRIu32 ",one_hot=%" PRIu32
                 ",other=%" PRIu32 ")",
                 handle->stats.walking_rows_equal,
                 handle->stats.walking_rows_compared,
                 handle->stats.walking_vertical_mismatches,
                 handle->stats.walking_horizontal_transitions,
                 handle->stats.walking_unique_values,
                 handle->stats.walking_zero_values,
                 handle->stats.walking_one_hot_values,
                 handle->stats.walking_other_values);
    }
    ESP_LOGI(TAG,
             "no_buffer=%" PRIu32 " ready_overflow=%" PRIu32
             " free_err=%" PRIu32
             " process_last=%" PRIu32 "us process_max=%" PRIu32
             "us queues(free=%u,ready=%u)",
             handle->stats.no_free_buffer,
             handle->stats.ready_queue_overflows,
             handle->stats.free_queue_errors,
             handle->stats.last_processing_time_us,
             handle->stats.max_processing_time_us,
             (unsigned)uxQueueMessagesWaiting(handle->free_queue),
             (unsigned)uxQueueMessagesWaiting(handle->ready_queue));
}

static void hm01b0_capture_process_frame(hm01b0_capture_handle_t *handle,
                                         hm01b0_capture_frame_t *frame)
{
    const int64_t start_us = esp_timer_get_time();
    handle->stats.received_size = frame->received_size;
    handle->processed_frames++;
    bool size_valid = frame->received_size == handle->stats.payload_size;

    if (!handle->baseline_received_size_valid) {
        handle->baseline_received_size = frame->received_size;
        handle->baseline_received_size_valid = true;
    } else if (frame->received_size != handle->baseline_received_size) {
        handle->stats.received_size_changes++;
    }

    if (!size_valid) {
        handle->stats.size_errors++;
        if (hm01b0_capture_error_log_allowed(handle, start_us)) {
            ESP_LOGE(TAG,
                     "frame=%" PRIu32 " size error: received=%u payload=%u "
                     "capacity=%u",
                     frame->sequence,
                     (unsigned)frame->received_size,
                     (unsigned)handle->stats.payload_size,
                     (unsigned)handle->stats.buffer_capacity);
        }
    }

    if (size_valid) {
        handle->stats.transport_valid_frames++;
    }

    if (!size_valid) {
        goto finish;
    }

    if (handle->processed_frames <= handle->config.warmup_frames) {
        handle->stats.warmup_frames++;
        goto finish;
    }

    const uint32_t raw_crc = esp_crc32_le(
        0U, frame->data, (uint32_t)handle->stats.payload_size);
    const uint32_t active_crc = hm01b0_capture_active_crc(handle,
                                                          frame->data);
    handle->stats.last_raw_crc = raw_crc;
    handle->stats.last_active_crc = active_crc;

    if (!handle->baseline_crc_valid) {
        handle->baseline_raw_crc = raw_crc;
        handle->baseline_active_crc = active_crc;
        handle->baseline_crc_valid = true;
    } else {
        if (raw_crc != handle->baseline_raw_crc) {
            handle->stats.raw_crc_changes++;
        }
        if (active_crc != handle->baseline_active_crc) {
            handle->stats.active_crc_changes++;
        }
    }
    if (handle->previous_active_crc_valid &&
        active_crc != handle->previous_active_crc) {
        handle->stats.active_crc_frame_changes++;
    }
    handle->previous_active_crc = active_crc;
    handle->previous_active_crc_valid = true;

    if (handle->config.analyze_walking_1) {
        const hm01b0_walking_analysis_t analysis =
            hm01b0_analyze_walking_one(
                frame->data, handle->config.raw_width,
                handle->config.active_x, handle->config.active_y,
                handle->config.active_width, handle->config.active_height);
        handle->stats.walking_analysis_frames++;
        handle->stats.walking_rows_equal = analysis.rows_equal;
        handle->stats.walking_rows_compared = analysis.rows_compared;
        handle->stats.walking_vertical_mismatches =
            analysis.vertical_mismatches;
        handle->stats.walking_horizontal_transitions =
            analysis.horizontal_transitions;
        handle->stats.walking_unique_values = analysis.unique_values;
        handle->stats.walking_zero_values = analysis.zero_values;
        handle->stats.walking_one_hot_values = analysis.one_hot_values;
        handle->stats.walking_other_values = analysis.other_values;
    }

finish:
    ;
    const int64_t end_us = esp_timer_get_time();
    const uint32_t processing_time = (uint32_t)(end_us - start_us);
    handle->stats.last_processing_time_us = processing_time;
    if (processing_time > handle->stats.max_processing_time_us) {
        handle->stats.max_processing_time_us = processing_time;
    }
}

static void hm01b0_capture_take_analysis_sample(
    const hm01b0_capture_handle_t *handle,
    const hm01b0_capture_frame_t *frame,
    hm01b0_analysis_sample_t *sample)
{
    sample->sequence = frame->sequence;
    sample->warmup_frames = handle->config.warmup_frames;
    sample->received_size = frame->received_size;
    sample->raw_crc = handle->stats.last_raw_crc;
    sample->active_crc = handle->stats.last_active_crc;
    sample->raw_sample_size =
        handle->config.raw_width < HM01B0_CAPTURE_FIRST_SAMPLE_SIZE
            ? handle->config.raw_width
            : HM01B0_CAPTURE_FIRST_SAMPLE_SIZE;
    sample->active_sample_size =
        handle->config.active_width < HM01B0_CAPTURE_FIRST_SAMPLE_SIZE
            ? handle->config.active_width
            : HM01B0_CAPTURE_FIRST_SAMPLE_SIZE;
    sample->active_x = handle->config.active_x;
    sample->active_rows[0] = handle->config.active_y;
    sample->active_rows[1] =
        (uint16_t)(handle->config.active_y +
                   handle->config.active_height / 2U);
    sample->active_rows[2] =
        (uint16_t)(handle->config.active_y +
                   handle->config.active_height - 1U);

    memcpy(sample->raw, frame->data, sample->raw_sample_size);
    for (size_t i = 0; i < 3U; ++i) {
        const size_t offset = (size_t)sample->active_rows[i] *
                                  handle->config.raw_width +
                              handle->config.active_x;
        memcpy(sample->active[i], frame->data + offset,
               sample->active_sample_size);
    }
}

static void hm01b0_capture_log_first_analysis(
    const hm01b0_analysis_sample_t *sample)
{
    ESP_LOGI(TAG, "warm-up complete: skipped=%" PRIu32 " frames",
             sample->warmup_frames);
    ESP_LOGI(TAG,
             "first analyzed frame: sequence=%" PRIu32
             " received=%u raw_crc=%08" PRIX32
             " active_crc=%08" PRIX32,
             sample->sequence, (unsigned)sample->received_size,
             sample->raw_crc, sample->active_crc);
    ESP_LOGI(TAG, "raw row y=0 x=0..%u",
             (unsigned)(sample->raw_sample_size - 1U));
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, sample->raw, sample->raw_sample_size,
                             ESP_LOG_INFO);

    for (size_t i = 0; i < 3U; ++i) {
        ESP_LOGI(TAG, "active row y=%u x=%u..%u",
                 (unsigned)sample->active_rows[i],
                 (unsigned)sample->active_x,
                 (unsigned)(sample->active_x +
                            sample->active_sample_size - 1U));
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, sample->active[i],
                                 sample->active_sample_size, ESP_LOG_INFO);
    }
}

static void hm01b0_capture_task(void *arg)
{
    hm01b0_capture_handle_t *handle = arg;
    handle->last_stats_time_us = esp_timer_get_time();
    handle->last_stats_frame_count = handle->isr_frames_received;

    while (!handle->task_should_exit) {
        hm01b0_capture_frame_t *frame = NULL;
        if (xQueueReceive(handle->ready_queue, &frame,
                          pdMS_TO_TICKS(HM01B0_CAPTURE_QUEUE_WAIT_MS)) !=
            pdPASS) {
            const int64_t now_us = esp_timer_get_time();
            if (now_us - handle->last_stats_time_us >=
                (int64_t)handle->config.stats_period_ms * 1000LL) {
                hm01b0_capture_log_stats(handle, now_us);
            }
            continue;
        }
        if (frame == NULL) {
            continue;
        }
        handle->processing_frame = true;

        hm01b0_analysis_sample_t first_sample;
        bool log_first_analysis = false;
        const uint32_t analyses_before =
            handle->stats.walking_analysis_frames;
        hm01b0_capture_process_frame(handle, frame);
        if (!handle->first_analysis_logged &&
            handle->stats.walking_analysis_frames > analyses_before) {
            hm01b0_capture_take_analysis_sample(handle, frame,
                                                 &first_sample);
            handle->first_analysis_logged = true;
            log_first_analysis = true;
        }

        if (xQueueSend(handle->free_queue, &frame, 0) != pdPASS) {
            handle->task_free_queue_errors++;
        }
        handle->processing_frame = false;

        if (log_first_analysis) {
            hm01b0_capture_log_first_analysis(&first_sample);
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us - handle->last_stats_time_us >=
            (int64_t)handle->config.stats_period_ms * 1000LL) {
            hm01b0_capture_log_stats(handle, now_us);
        }
    }

    handle->task_handle = NULL;
    vTaskDelete(NULL);
}

static void hm01b0_capture_log_memory(const hm01b0_capture_handle_t *handle)
{
    ESP_LOGI(TAG,
             "internal DMA heap before: free=%u largest=%u; after: free=%u "
             "largest=%u",
             (unsigned)handle->dma_heap_free_before,
             (unsigned)handle->dma_heap_largest_before,
             (unsigned)handle->dma_heap_free_after,
             (unsigned)handle->dma_heap_largest_after);
    ESP_LOGI(TAG,
             "buffers: A=%p (%s), B=%p (%s), each=%u bytes, "
             "payload=%u bytes",
             handle->frames[0].data,
             esp_ptr_internal(handle->frames[0].data) ? "internal" : "external",
             handle->frames[1].data,
             esp_ptr_internal(handle->frames[1].data) ? "internal" : "external",
             (unsigned)handle->stats.buffer_capacity,
             (unsigned)handle->stats.payload_size);
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
    if (handle->config.task_stack_size == 0U) {
        handle->config.task_stack_size =
            HM01B0_CAPTURE_DEFAULT_TASK_STACK_SIZE;
    }
    if (handle->config.task_priority == 0U) {
        handle->config.task_priority = HM01B0_CAPTURE_DEFAULT_TASK_PRIORITY;
    }
    if (handle->config.stats_period_ms == 0U) {
        handle->config.stats_period_ms =
            HM01B0_CAPTURE_DEFAULT_STATS_PERIOD_MS;
    }
    if (handle->config.warmup_frames == 0U) {
        handle->config.warmup_frames =
            HM01B0_CAPTURE_DEFAULT_WARMUP_FRAMES;
    }

    const uint32_t dma_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA;

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
        .h_res = config->raw_width,
        .v_res = config->raw_height,
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

    esp_err_t ret = esp_cam_new_dvp_ctlr(&dvp_config, &handle->cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to create DVP controller: %s",
                 esp_err_to_name(ret));
        goto fail;
    }

    ret = esp_cam_ctlr_get_frame_buffer_len(handle->cam_handle,
                                             &handle->stats.buffer_capacity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to query DMA buffer length: %s",
                 esp_err_to_name(ret));
        goto fail;
    }

    handle->stats.raw_width = config->raw_width;
    handle->stats.raw_height = config->raw_height;
    handle->stats.raw_stride = config->raw_width;
    handle->stats.active_x = config->active_x;
    handle->stats.active_y = config->active_y;
    handle->stats.active_width = config->active_width;
    handle->stats.active_height = config->active_height;
    handle->stats.payload_size = (size_t)config->raw_width *
                                 config->raw_height;
    handle->stats.active_payload_size =
        (size_t)config->active_width * config->active_height;
    if (handle->stats.buffer_capacity < handle->stats.payload_size) {
        ESP_LOGE(TAG, "driver buffer length is smaller than RAW8 payload");
        ret = ESP_ERR_INVALID_SIZE;
        goto fail;
    }

    handle->dma_heap_free_before = heap_caps_get_free_size(dma_caps);
    handle->dma_heap_largest_before =
        heap_caps_get_largest_free_block(dma_caps);

    for (size_t i = 0; i < HM01B0_CAPTURE_BUFFER_COUNT; ++i) {
        handle->frames[i].data = esp_cam_ctlr_alloc_buffer(
            handle->cam_handle, handle->stats.buffer_capacity, dma_caps);
        if (handle->frames[i].data == NULL) {
            ESP_LOGE(TAG, "failed to allocate internal DMA Buffer %c",
                     (int)('A' + i));
            ret = ESP_ERR_NO_MEM;
            goto fail;
        }
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

    ret = hm01b0_capture_reset_queues(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to seed free queue");
        goto fail;
    }

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
                    handle->config.task_stack_size, handle,
                    (UBaseType_t)handle->config.task_priority,
                    &handle->task_handle) != pdPASS) {
        ESP_LOGE(TAG, "failed to create frame analysis task");
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }

    ESP_LOGI(TAG,
             "DVP ready: RAW8 %ux%u, payload=%u, capacity=%u, "
             "burst=%" PRIu32 ", buffers=2, backup=disabled",
             config->raw_width, config->raw_height,
             (unsigned)handle->stats.payload_size,
             (unsigned)handle->stats.buffer_capacity,
             config->dma_burst_size);
    ESP_LOGI(TAG,
             "analysis area=(%u,%u %ux%u), active_payload=%u, warmup=%" PRIu32
             "; DMA still captures the complete raw frame without cropping",
             config->active_x, config->active_y,
             config->active_width, config->active_height,
             (unsigned)handle->stats.active_payload_size,
             handle->config.warmup_frames);
    hm01b0_capture_log_memory(handle);

    *out_handle = handle;
    return ESP_OK;

fail:
    (void)hm01b0_capture_delete(handle);
    return ret;
}

esp_err_t hm01b0_capture_start(hm01b0_capture_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "capture handle is NULL");
    ESP_RETURN_ON_FALSE(!handle->controller_started,
                        ESP_ERR_INVALID_STATE, TAG,
                        "Camera RX is already started");
    ESP_RETURN_ON_FALSE(!handle->processing_frame,
                        ESP_ERR_INVALID_STATE, TAG,
                        "a frame is still being processed");

    hm01b0_capture_reset_diagnostics(handle);
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

esp_err_t hm01b0_capture_stop(hm01b0_capture_handle_t *handle)
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
        uint32_t i = 0;
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
            const esp_err_t ret = hm01b0_capture_reset_queues(handle);
            if (ret != ESP_OK) {
                result = ret;
            }
        }
    }
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Camera RX stopped and disabled");
    }
    return result;
}

esp_err_t hm01b0_capture_delete(hm01b0_capture_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_OK;
    }

    esp_err_t result = hm01b0_capture_stop(handle);
    if (handle->controller_started || handle->controller_enabled) {
        ESP_LOGE(TAG,
                 "capture resources remain owned because Camera RX could not "
                 "be stopped safely");
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
        handle->cam_handle = NULL;
    }

    for (size_t i = 0; i < HM01B0_CAPTURE_BUFFER_COUNT; ++i) {
        heap_caps_free(handle->frames[i].data);
        handle->frames[i].data = NULL;
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

esp_err_t hm01b0_capture_get_stats(const hm01b0_capture_handle_t *handle,
                                   hm01b0_capture_stats_t *stats)
{
    ESP_RETURN_ON_FALSE(handle != NULL && stats != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    *stats = handle->stats;
    stats->frames_received = handle->isr_frames_received;
    stats->no_free_buffer = handle->isr_no_free_buffer;
    stats->ready_queue_overflows = handle->isr_ready_queue_overflows;
    stats->free_queue_errors = handle->isr_free_queue_errors +
                               handle->task_free_queue_errors;
    return ESP_OK;
}
