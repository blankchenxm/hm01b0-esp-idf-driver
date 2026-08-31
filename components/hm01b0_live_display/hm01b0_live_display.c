#include "hm01b0_live_display.h"

#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hm01b0_image.h"

#define HM01B0_LIVE_DISPLAY_TASK_STACK_SIZE 4096U
#define HM01B0_LIVE_DISPLAY_TASK_PRIORITY   4U
#define HM01B0_LIVE_DISPLAY_QUEUE_LENGTH    1U
#define HM01B0_LIVE_DISPLAY_QUEUE_WAIT_MS   20U
#define HM01B0_LIVE_DISPLAY_RGB_WAIT_MS     20U
#define HM01B0_LIVE_DISPLAY_STOP_TIMEOUT_MS 5000U
#define HM01B0_LIVE_DISPLAY_BAYER_HALO      1U

typedef struct {
    uint32_t sequence;
} hm01b0_live_display_item_t;

struct hm01b0_live_display {
    hm01b0_live_display_config_t config;
    uint8_t *staging_buffer;
    uint16_t staging_x;
    uint16_t staging_y;
    uint16_t staging_width;
    uint16_t staging_height;
    uint16_t output_x;
    uint16_t output_y;
    size_t staging_size;

    QueueHandle_t free_queue;
    QueueHandle_t ready_queue;
    StaticQueue_t free_queue_state;
    StaticQueue_t ready_queue_state;
    uint8_t free_queue_storage[sizeof(uint8_t)];
    uint8_t ready_queue_storage[sizeof(hm01b0_live_display_item_t)];

    TaskHandle_t task_handle;
    SemaphoreHandle_t task_stopped;
    StaticSemaphore_t task_stopped_state;
    atomic_bool task_should_exit;

    portMUX_TYPE stats_lock;
    int64_t stats_start_us;
    hm01b0_live_display_stats_t stats;
};

static const char *TAG = "hm01b0_live_display";

static bool hm01b0_live_display_is_bayer(hm01b0_pixel_format_t format)
{
    return format >= HM01B0_PIXEL_FORMAT_BAYER_GRBG8 &&
           format <= HM01B0_PIXEL_FORMAT_BAYER_GBRG8;
}

static bool hm01b0_live_display_rect_is_valid(
    const hm01b0_live_display_config_t *config)
{
    return config->source_width > 0U && config->source_height > 0U &&
           config->source_stride >= config->source_width &&
           config->source_crop.width > 0U &&
           config->source_crop.height > 0U &&
           config->source_crop.x < config->source_width &&
           config->source_crop.y < config->source_height &&
           config->source_crop.width <=
               config->source_width - config->source_crop.x &&
           config->source_crop.height <=
               config->source_height - config->source_crop.y;
}

static void hm01b0_live_display_return_staging(
    hm01b0_live_display_handle_t *handle)
{
    const uint8_t token = 1U;
    (void)xQueueSend(handle->free_queue, &token, 0U);
}

static void hm01b0_live_display_task(void *arg)
{
    hm01b0_live_display_handle_t *handle = arg;
    while (!atomic_load(&handle->task_should_exit)) {
        hm01b0_live_display_item_t item = {0};
        if (xQueueReceive(handle->ready_queue, &item,
                          pdMS_TO_TICKS(HM01B0_LIVE_DISPLAY_QUEUE_WAIT_MS)) !=
            pdPASS) {
            continue;
        }
        (void)item;
        if (atomic_load(&handle->task_should_exit)) {
            hm01b0_live_display_return_staging(handle);
            break;
        }

        uint8_t *rgb565 = NULL;
        size_t rgb565_size = 0U;
        esp_err_t ret = st7789_display_wait_idle(
            handle->config.display, HM01B0_LIVE_DISPLAY_RGB_WAIT_MS);
        if (ret == ESP_OK) {
            ret = st7789_display_try_acquire_rgb565_frame(
                handle->config.display, &rgb565, &rgb565_size);
        }
        if (ret == ESP_ERR_TIMEOUT) {
            portENTER_CRITICAL(&handle->stats_lock);
            handle->stats.rgb_busy_drops++;
            portEXIT_CRITICAL(&handle->stats_lock);
            hm01b0_live_display_return_staging(handle);
            continue;
        }
        if (ret != ESP_OK) {
            portENTER_CRITICAL(&handle->stats_lock);
            handle->stats.submission_errors++;
            portEXIT_CRITICAL(&handle->stats_lock);
            hm01b0_live_display_return_staging(handle);
            continue;
        }

        const hm01b0_raw8_image_t source = {
            .data = handle->staging_buffer,
            .width = handle->staging_width,
            .height = handle->staging_height,
            .stride = handle->staging_width,
            .origin_x = handle->staging_x,
            .origin_y = handle->staging_y,
            .pixel_format = handle->config.pixel_format,
        };
        const hm01b0_rect_t crop = {
            .x = handle->output_x,
            .y = handle->output_y,
            .width = handle->config.source_crop.width,
            .height = handle->config.source_crop.height,
        };
        const int64_t conversion_start_us = esp_timer_get_time();
        ret = hm01b0_image_convert_to_rgb565_be(
            &source, crop, rgb565, rgb565_size);
        const uint32_t conversion_time_us = (uint32_t)(
            esp_timer_get_time() - conversion_start_us);

        /* Conversion no longer reads RAW8, so Capture may refill staging. */
        hm01b0_live_display_return_staging(handle);

        portENTER_CRITICAL(&handle->stats_lock);
        handle->stats.last_conversion_time_us = conversion_time_us;
        if (conversion_time_us > handle->stats.max_conversion_time_us) {
            handle->stats.max_conversion_time_us = conversion_time_us;
        }
        if (ret != ESP_OK) {
            handle->stats.conversion_errors++;
        }
        portEXIT_CRITICAL(&handle->stats_lock);
        if (ret != ESP_OK) {
            st7789_display_release_rgb565_frame(handle->config.display);
            continue;
        }

        ret = st7789_display_submit_rgb565_frame(
            handle->config.display,
            handle->config.source_crop.width,
            handle->config.source_crop.height,
            handle->config.destination_x,
            handle->config.destination_y,
            conversion_time_us);
        portENTER_CRITICAL(&handle->stats_lock);
        if (ret == ESP_OK) {
            handle->stats.submitted_frames++;
        } else {
            handle->stats.submission_errors++;
        }
        portEXIT_CRITICAL(&handle->stats_lock);
    }

    handle->task_handle = NULL;
    (void)xSemaphoreGive(handle->task_stopped);
    vTaskDelete(NULL);
}

esp_err_t hm01b0_live_display_new(
    const hm01b0_live_display_config_t *config,
    hm01b0_live_display_handle_t **out_handle)
{
    ESP_RETURN_ON_FALSE(config != NULL && out_handle != NULL &&
                            config->display != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(hm01b0_live_display_rect_is_valid(config),
                        ESP_ERR_INVALID_ARG, TAG, "invalid source geometry");
    ESP_RETURN_ON_FALSE(
        config->pixel_format >= HM01B0_PIXEL_FORMAT_MONO8 &&
            config->pixel_format <= HM01B0_PIXEL_FORMAT_BAYER_GBRG8,
        ESP_ERR_INVALID_ARG, TAG, "invalid pixel format");
    *out_handle = NULL;

    hm01b0_live_display_handle_t *handle = calloc(1U, sizeof(*handle));
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_NO_MEM, TAG,
                        "failed to allocate pipeline handle");
    handle->config = *config;
    atomic_init(&handle->task_should_exit, false);
    handle->stats_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    esp_err_t ret = ESP_OK;

    const uint16_t halo = hm01b0_live_display_is_bayer(config->pixel_format)
                              ? HM01B0_LIVE_DISPLAY_BAYER_HALO
                              : 0U;
    ESP_GOTO_ON_FALSE(
        config->source_crop.x >= halo && config->source_crop.y >= halo &&
            (uint32_t)config->source_crop.x + config->source_crop.width +
                    halo <=
                config->source_width &&
            (uint32_t)config->source_crop.y + config->source_crop.height +
                    halo <=
                config->source_height,
        ESP_ERR_INVALID_ARG, fail, TAG,
        "Bayer crop requires a one-pixel source halo");

    handle->staging_x = (uint16_t)(config->source_crop.x - halo);
    handle->staging_y = (uint16_t)(config->source_crop.y - halo);
    handle->staging_width = (uint16_t)(config->source_crop.width + 2U * halo);
    handle->staging_height =
        (uint16_t)(config->source_crop.height + 2U * halo);
    handle->output_x = halo;
    handle->output_y = halo;
    handle->staging_size =
        (size_t)handle->staging_width * handle->staging_height;

    /* CPU-only image: internal SRAM avoids cached PSRAM neighbor reads. */
    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    const size_t free_before = heap_caps_get_free_size(caps);
    const size_t largest_before = heap_caps_get_largest_free_block(caps);
    handle->staging_buffer = heap_caps_malloc(handle->staging_size, caps);
    ESP_GOTO_ON_FALSE(handle->staging_buffer != NULL, ESP_ERR_NO_MEM, fail,
                      TAG, "failed to allocate internal RAW8 staging buffer");
    ESP_LOGI(TAG,
             "RAW8 staging=%p internal SRAM, source=(%u,%u %ux%u), "
             "output=(%u,%u "
             "%ux%u), bytes=%u; heap before free=%u largest=%u, after "
             "free=%u largest=%u",
             handle->staging_buffer,
             (unsigned)handle->staging_x, (unsigned)handle->staging_y,
             (unsigned)handle->staging_width,
             (unsigned)handle->staging_height,
             (unsigned)handle->output_x, (unsigned)handle->output_y,
             (unsigned)config->source_crop.width,
             (unsigned)config->source_crop.height,
             (unsigned)handle->staging_size,
             (unsigned)free_before, (unsigned)largest_before,
             (unsigned)heap_caps_get_free_size(caps),
             (unsigned)heap_caps_get_largest_free_block(caps));

    handle->free_queue = xQueueCreateStatic(
        HM01B0_LIVE_DISPLAY_QUEUE_LENGTH, sizeof(uint8_t),
        handle->free_queue_storage, &handle->free_queue_state);
    handle->ready_queue = xQueueCreateStatic(
        HM01B0_LIVE_DISPLAY_QUEUE_LENGTH,
        sizeof(hm01b0_live_display_item_t),
        handle->ready_queue_storage, &handle->ready_queue_state);
    handle->task_stopped = xSemaphoreCreateBinaryStatic(
        &handle->task_stopped_state);
    ESP_GOTO_ON_FALSE(handle->free_queue != NULL &&
                          handle->ready_queue != NULL &&
                          handle->task_stopped != NULL,
                      ESP_ERR_NO_MEM, fail, TAG,
                      "failed to create pipeline synchronization objects");
    hm01b0_live_display_return_staging(handle);
    hm01b0_live_display_reset_stats(handle);

    const BaseType_t task_created = xTaskCreate(
        hm01b0_live_display_task, "hm01b0_image",
        HM01B0_LIVE_DISPLAY_TASK_STACK_SIZE, handle,
        HM01B0_LIVE_DISPLAY_TASK_PRIORITY, &handle->task_handle);
    ESP_GOTO_ON_FALSE(task_created == pdPASS, ESP_ERR_NO_MEM, fail, TAG,
                      "failed to create Image/Display Task");
    ESP_LOGI(TAG, "Image/Display Task started: stack=%u priority=%u",
             (unsigned)HM01B0_LIVE_DISPLAY_TASK_STACK_SIZE,
             (unsigned)HM01B0_LIVE_DISPLAY_TASK_PRIORITY);
    *out_handle = handle;
    return ESP_OK;

fail:
    heap_caps_free(handle->staging_buffer);
    free(handle);
    return ret;
}

esp_err_t hm01b0_live_display_delete(
    hm01b0_live_display_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_OK;
    }
    atomic_store(&handle->task_should_exit, true);
    if (handle->task_handle != NULL &&
        xSemaphoreTake(handle->task_stopped,
                       pdMS_TO_TICKS(HM01B0_LIVE_DISPLAY_STOP_TIMEOUT_MS)) !=
            pdPASS) {
        ESP_LOGE(TAG, "Image/Display Task did not stop");
        return ESP_ERR_TIMEOUT;
    }
    heap_caps_free(handle->staging_buffer);
    free(handle);
    return ESP_OK;
}

void hm01b0_live_display_consume_frame(
    const hm01b0_capture_frame_t *frame,
    void *user_data)
{
    hm01b0_live_display_handle_t *handle = user_data;
    if (handle == NULL || frame == NULL ||
        atomic_load(&handle->task_should_exit)) {
        return;
    }

    portENTER_CRITICAL(&handle->stats_lock);
    handle->stats.input_frames++;
    portEXIT_CRITICAL(&handle->stats_lock);

    uint8_t token = 0U;
    if (xQueueReceive(handle->free_queue, &token, 0U) != pdPASS) {
        portENTER_CRITICAL(&handle->stats_lock);
        handle->stats.staging_busy_drops++;
        portEXIT_CRITICAL(&handle->stats_lock);
        return;
    }

    const int64_t copy_start_us = esp_timer_get_time();
    for (uint16_t row = 0U; row < handle->staging_height; ++row) {
        const size_t source_offset =
            (size_t)(handle->staging_y + row) * handle->config.source_stride +
            handle->staging_x;
        memcpy(handle->staging_buffer + (size_t)row * handle->staging_width,
               frame->data + source_offset, handle->staging_width);
    }
    const uint32_t copy_time_us = (uint32_t)(
        esp_timer_get_time() - copy_start_us);

    const hm01b0_live_display_item_t item = {
        .sequence = frame->sequence,
    };
    if (xQueueSend(handle->ready_queue, &item, 0U) != pdPASS) {
        hm01b0_live_display_return_staging(handle);
        portENTER_CRITICAL(&handle->stats_lock);
        handle->stats.staging_busy_drops++;
        portEXIT_CRITICAL(&handle->stats_lock);
        return;
    }

    portENTER_CRITICAL(&handle->stats_lock);
    handle->stats.staged_frames++;
    handle->stats.last_copy_time_us = copy_time_us;
    if (copy_time_us > handle->stats.max_copy_time_us) {
        handle->stats.max_copy_time_us = copy_time_us;
    }
    portEXIT_CRITICAL(&handle->stats_lock);
}

void hm01b0_live_display_reset_stats(
    hm01b0_live_display_handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    portENTER_CRITICAL(&handle->stats_lock);
    handle->stats = (hm01b0_live_display_stats_t) {
        .staging_buffer_size = handle->staging_size,
    };
    handle->stats_start_us = esp_timer_get_time();
    portEXIT_CRITICAL(&handle->stats_lock);
}

esp_err_t hm01b0_live_display_get_stats(
    const hm01b0_live_display_handle_t *handle,
    hm01b0_live_display_stats_t *stats)
{
    ESP_RETURN_ON_FALSE(handle != NULL && stats != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    hm01b0_live_display_handle_t *mutable_handle =
        (hm01b0_live_display_handle_t *)handle;
    portENTER_CRITICAL(&mutable_handle->stats_lock);
    *stats = handle->stats;
    const int64_t start_us = handle->stats_start_us;
    portEXIT_CRITICAL(&mutable_handle->stats_lock);

    const int64_t elapsed_us = esp_timer_get_time() - start_us;
    if (elapsed_us > 0) {
        stats->input_fps_milli = (uint32_t)(
            ((uint64_t)stats->input_frames * 1000000000ULL) /
            (uint64_t)elapsed_us);
        stats->staged_fps_milli = (uint32_t)(
            ((uint64_t)stats->staged_frames * 1000000000ULL) /
            (uint64_t)elapsed_us);
        stats->submitted_fps_milli = (uint32_t)(
            ((uint64_t)stats->submitted_frames * 1000000000ULL) /
            (uint64_t)elapsed_us);
    }
    return ESP_OK;
}
