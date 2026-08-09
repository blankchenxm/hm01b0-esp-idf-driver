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
#define HM01B0_CAPTURE_FIRST_SAMPLE_SIZE       32U
#define HM01B0_CAPTURE_QUEUE_WAIT_MS           100U
#define HM01B0_CAPTURE_TASK_STOP_TIMEOUT_MS    500U

typedef struct {
    uint8_t *data;
    size_t received_size;
    uint32_t sequence;
} hm01b0_capture_frame_t;

typedef struct {
    bool valid;
    uint16_t x;
    uint16_t y;
    uint8_t expected;
    uint8_t actual;
    int direction;
} hm01b0_pattern_result_t;

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
    uint32_t baseline_crc;
    bool baseline_received_size_valid;
    size_t baseline_received_size;
    bool pattern_description_logged;
    bool first_frame_logged;
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

static uint8_t hm01b0_rotate_left_one(uint8_t value)
{
    return (uint8_t)((value << 1U) | (value >> 7U));
}

static uint8_t hm01b0_rotate_right_one(uint8_t value)
{
    return (uint8_t)((value >> 1U) | (value << 7U));
}

static bool hm01b0_is_one_hot(uint8_t value)
{
    return value != 0U && (value & (uint8_t)(value - 1U)) == 0U;
}

static hm01b0_pattern_result_t hm01b0_validate_walking_one(
    const uint8_t *data,
    uint16_t width,
    uint16_t height)
{
    hm01b0_pattern_result_t result = {
        .valid = false,
        .direction = 0,
    };

    if (data == NULL || width < 2U || height == 0U) {
        return result;
    }

    if (!hm01b0_is_one_hot(data[0])) {
        result.actual = data[0];
        return result;
    }

    const uint8_t left = hm01b0_rotate_left_one(data[0]);
    const uint8_t right = hm01b0_rotate_right_one(data[0]);
    if (data[1] == left) {
        result.direction = 1;
    } else if (data[1] == right) {
        result.direction = -1;
    } else {
        result.x = 1U;
        result.expected = left;
        result.actual = data[1];
        return result;
    }

    for (uint16_t y = 0; y < height; ++y) {
        uint8_t expected = data[0];
        const size_t row_offset = (size_t)y * width;
        for (uint16_t x = 0; x < width; ++x) {
            const uint8_t actual = data[row_offset + x];
            if (!hm01b0_is_one_hot(actual) || actual != expected) {
                result.x = x;
                result.y = y;
                result.expected = expected;
                result.actual = actual;
                return result;
            }
            expected = result.direction > 0
                           ? hm01b0_rotate_left_one(expected)
                           : hm01b0_rotate_right_one(expected);
        }
    }

    result.valid = true;
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
             " valid=%" PRIu32 " size_err=%" PRIu32
             " size_change=%" PRIu32 " pattern_err=%" PRIu32
             " crc_change=%" PRIu32,
             handle->stats.fps_milli / 1000U,
             handle->stats.fps_milli % 1000U,
             handle->stats.frames_received,
             handle->stats.frames_valid,
             handle->stats.size_errors,
             handle->stats.received_size_changes,
             handle->stats.pattern_errors,
             handle->stats.crc_changes);
    ESP_LOGI(TAG,
             "no_buffer=%" PRIu32 " ready_overflow=%" PRIu32
             " free_err=%" PRIu32 " crc=%08" PRIX32
             " process_last=%" PRIu32 "us process_max=%" PRIu32
             "us queues(free=%u,ready=%u)",
             handle->stats.no_free_buffer,
             handle->stats.ready_queue_overflows,
             handle->stats.free_queue_errors,
             handle->stats.last_crc,
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
    bool size_valid =
        frame->received_size >= handle->stats.payload_size &&
        frame->received_size <= handle->stats.buffer_capacity;
    bool pattern_valid = true;
    bool crc_valid = true;
    hm01b0_pattern_result_t pattern_result = {.valid = true};

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
    } else if (!handle->baseline_received_size_valid) {
        handle->baseline_received_size = frame->received_size;
        handle->baseline_received_size_valid = true;
    } else if (frame->received_size != handle->baseline_received_size) {
        size_valid = false;
        handle->stats.size_errors++;
        handle->stats.received_size_changes++;
        if (hm01b0_capture_error_log_allowed(handle, start_us)) {
            ESP_LOGE(TAG,
                     "frame=%" PRIu32
                     " received size changed: baseline=%u actual=%u",
                     frame->sequence,
                     (unsigned)handle->baseline_received_size,
                     (unsigned)frame->received_size);
        }
    }

    const uint32_t crc = esp_crc32_le(0U, frame->data,
                                      (uint32_t)handle->stats.payload_size);
    handle->stats.last_crc = crc;
    if (!handle->baseline_crc_valid) {
        handle->baseline_crc = crc;
        handle->baseline_crc_valid = true;
    } else if (crc != handle->baseline_crc) {
        crc_valid = false;
        handle->stats.crc_changes++;
        if (hm01b0_capture_error_log_allowed(handle, start_us)) {
            ESP_LOGE(TAG,
                     "frame=%" PRIu32 " CRC changed: baseline=%08" PRIX32
                     " actual=%08" PRIX32,
                     frame->sequence, handle->baseline_crc, crc);
        }
    }

    if (handle->config.validate_walking_1) {
        pattern_result = hm01b0_validate_walking_one(
            frame->data, handle->config.raw_width,
            handle->config.raw_height);
        pattern_valid = pattern_result.valid;
        if (!pattern_valid) {
            handle->stats.pattern_errors++;
            if (hm01b0_capture_error_log_allowed(handle, start_us)) {
                ESP_LOGE(TAG,
                         "frame=%" PRIu32 " Walking-1 mismatch at (%u,%u): "
                         "expected=0x%02X actual=0x%02X",
                         frame->sequence, pattern_result.x, pattern_result.y,
                         pattern_result.expected, pattern_result.actual);
            }
        } else if (!handle->pattern_description_logged) {
            ESP_LOGI(TAG,
                     "Walking-1 detected: phase=0x%02X direction=%s; "
                     "absolute phase is not specified by the datasheet",
                     frame->data[0],
                     pattern_result.direction > 0 ? "bit-left" : "bit-right");
            handle->pattern_description_logged = true;
        }
    }

    if (size_valid && pattern_valid && crc_valid) {
        handle->stats.frames_valid++;
    }

    const int64_t end_us = esp_timer_get_time();
    const uint32_t processing_time = (uint32_t)(end_us - start_us);
    handle->stats.last_processing_time_us = processing_time;
    if (processing_time > handle->stats.max_processing_time_us) {
        handle->stats.max_processing_time_us = processing_time;
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

        uint8_t first_sample[HM01B0_CAPTURE_FIRST_SAMPLE_SIZE] = {0};
        const bool log_first_frame = !handle->first_frame_logged;
        if (log_first_frame) {
            memcpy(first_sample, frame->data, sizeof(first_sample));
        }

        hm01b0_capture_process_frame(handle, frame);

        if (xQueueSend(handle->free_queue, &frame, 0) != pdPASS) {
            handle->task_free_queue_errors++;
        }
        handle->processing_frame = false;

        if (log_first_frame) {
            ESP_LOGI(TAG,
                     "first frame: sequence=%" PRIu32
                     " received=%u payload=%u capacity=%u crc=%08" PRIX32,
                     frame->sequence,
                     (unsigned)frame->received_size,
                     (unsigned)handle->stats.payload_size,
                     (unsigned)handle->stats.buffer_capacity,
                     handle->stats.last_crc);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, first_sample, sizeof(first_sample),
                                     ESP_LOG_INFO);
            handle->first_frame_logged = true;
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
    handle->stats.payload_size = (size_t)config->raw_width *
                                 config->raw_height;
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
        ESP_LOGE(TAG, "failed to create frame validation task");
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
             "sensor-valid crop=(2,0 320x244), optional standard crop=(2,2 "
             "320x240); Stage 3 captures without cropping");
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
