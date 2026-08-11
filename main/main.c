#include <inttypes.h>

#include "app_config.h"
#include "board_config.h"
#include "driver/i2c_types.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hm01b0.h"
#include "hm01b0_capture.h"
#include "st7789_display.h"

static const char *TAG = "hm01b0_stage4_5";

static hm01b0_handle_t *s_sensor;
static hm01b0_capture_handle_t *s_capture;
static st7789_display_handle_t *s_display;
static uint8_t *s_snapshot;
static TaskHandle_t s_app_task;
static hm01b0_snapshot_result_t s_snapshot_result;

static void hm01b0_snapshot_ready(
    const hm01b0_snapshot_result_t *result,
    void *user_data)
{
    s_snapshot_result = *result;
    xTaskNotifyGive((TaskHandle_t)user_data);
}

static hm01b0_frame_rect_t hm01b0_to_frame_rect(hm01b0_rect_t rect)
{
    return (hm01b0_frame_rect_t) {
        .x = rect.x,
        .y = rect.y,
        .width = rect.width,
        .height = rect.height,
    };
}

static void hm01b0_log_preflight_result(
    const char *name,
    const hm01b0_capture_transport_stats_t *transport,
    const hm01b0_diagnostic_report_t *diagnostic)
{
    const bool transport_pass =
        transport->valid_frames > 0U &&
        transport->size_errors == 0U &&
        transport->no_free_buffer == 0U &&
        transport->ready_queue_overflows == 0U &&
        transport->free_queue_errors == 0U;

    ESP_LOGI(TAG,
             "%s transport=%s received=%" PRIu32 " valid=%" PRIu32
             " size_err=%" PRIu32 " no_buffer=%" PRIu32
             " ready_overflow=%" PRIu32 " free_err=%" PRIu32,
             name, transport_pass ? "PASS" : "FAIL",
             transport->frames_received, transport->valid_frames,
             transport->size_errors, transport->no_free_buffer,
             transport->ready_queue_overflows,
             transport->free_queue_errors);

    if (diagnostic->status == HM01B0_DIAGNOSTIC_STATUS_PASS) {
        ESP_LOGI(TAG,
                 "%s content=%s analyzed=%" PRIu32
                 " raw_crc=%08" PRIX32 " active_crc=%08" PRIX32
                 "; structural observation only, exact RAW8 values are "
                 "not specified by the datasheet",
                 name,
                 hm01b0_diagnostic_status_name(diagnostic->status),
                 diagnostic->analyzed_frames,
                 diagnostic->last_raw_crc,
                 diagnostic->last_active_crc);
    } else {
        ESP_LOGW(TAG,
                 "%s content=%s analyzed=%" PRIu32
                 " raw_crc=%08" PRIX32 " active_crc=%08" PRIX32
                 "; structural observation only, exact RAW8 values are "
                 "not specified by the datasheet",
                 name,
                 hm01b0_diagnostic_status_name(diagnostic->status),
                 diagnostic->analyzed_frames,
                 diagnostic->last_raw_crc,
                 diagnostic->last_active_crc);
    }
}

static esp_err_t hm01b0_run_preflight(
    hm01b0_test_pattern_t sensor_pattern,
    hm01b0_diagnostic_pattern_t diagnostic_pattern,
    const char *name,
    hm01b0_frame_rect_t analysis_area)
{
    ESP_LOGI(TAG, "preflight begin: %s", name);
    esp_err_t ret = hm01b0_set_test_pattern(s_sensor, sensor_pattern);
    if (ret != ESP_OK) {
        return ret;
    }

    const hm01b0_diagnostic_config_t diagnostic_config = {
        .pattern = diagnostic_pattern,
        .area = analysis_area,
        .warmup_frames = APP_PREFLIGHT_WARMUP_FRAMES,
    };
    ret = hm01b0_capture_set_diagnostics(s_capture, &diagnostic_config);
    if (ret != ESP_OK) {
        return ret;
    }

    const hm01b0_snapshot_request_t snapshot_request = {
        .buffer = s_snapshot,
        .buffer_size = APP_PREFLIGHT_SNAPSHOT_WIDTH *
                       APP_PREFLIGHT_SNAPSHOT_HEIGHT,
        .crop = {
            .x = APP_PREFLIGHT_SNAPSHOT_X,
            .y = APP_PREFLIGHT_SNAPSHOT_Y,
            .width = APP_PREFLIGHT_SNAPSHOT_WIDTH,
            .height = APP_PREFLIGHT_SNAPSHOT_HEIGHT,
        },
        .skip_frames = APP_PREFLIGHT_WARMUP_FRAMES,
        .on_ready = hm01b0_snapshot_ready,
        .user_data = s_app_task,
    };
    ret = hm01b0_capture_request_snapshot(s_capture, &snapshot_request);
    if (ret != ESP_OK) {
        return ret;
    }

    (void)ulTaskNotifyTake(pdTRUE, 0U);
    ret = hm01b0_capture_rx_start(s_capture);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = hm01b0_stream_start(s_sensor);
    if (ret != ESP_OK) {
        (void)hm01b0_capture_rx_stop(s_capture);
        return ret;
    }

    const uint32_t notification = ulTaskNotifyTake(
        pdTRUE, pdMS_TO_TICKS(APP_PREFLIGHT_CAPTURE_TIMEOUT_MS));
    if (notification == 0U) {
        ESP_LOGE(TAG, "%s preflight timed out waiting for snapshot", name);
        (void)hm01b0_stream_stop(s_sensor);
        (void)hm01b0_capture_rx_stop(s_capture);
        return ESP_ERR_TIMEOUT;
    }

    ret = hm01b0_capture_get_snapshot_result(s_capture, &s_snapshot_result);
    if (ret != ESP_OK) {
        (void)hm01b0_stream_stop(s_sensor);
        (void)hm01b0_capture_rx_stop(s_capture);
        return ret;
    }

    const int64_t display_start_us = esp_timer_get_time();
    ret = st7789_display_draw_gray8(
        s_display, 0U, 0U,
        APP_PREFLIGHT_SNAPSHOT_WIDTH,
        APP_PREFLIGHT_SNAPSHOT_HEIGHT,
        s_snapshot);
    if (ret != ESP_OK) {
        (void)hm01b0_stream_stop(s_sensor);
        (void)hm01b0_capture_rx_stop(s_capture);
        return ret;
    }
    const uint32_t display_time_us = (uint32_t)(
        esp_timer_get_time() - display_start_us);
    ESP_LOGI(TAG,
             "%s displayed: source_frame=%" PRIu32
             " snapshot=%u bytes copy=%" PRIu32 "us hold=%" PRIu32
             "us display=%" PRIu32 "us; holding for %u ms while Camera RX "
             "and diagnostics continue",
             name, s_snapshot_result.sequence,
             (unsigned)s_snapshot_result.size,
             s_snapshot_result.copy_time_us,
             s_snapshot_result.buffer_hold_time_us,
             display_time_us,
             (unsigned)APP_PREFLIGHT_DISPLAY_TIME_MS);
    vTaskDelay(pdMS_TO_TICKS(APP_PREFLIGHT_DISPLAY_TIME_MS));

    const esp_err_t sensor_stop_result = hm01b0_stream_stop(s_sensor);
    const esp_err_t rx_stop_result = hm01b0_capture_rx_stop(s_capture);
    if (sensor_stop_result != ESP_OK) {
        return sensor_stop_result;
    }
    if (rx_stop_result != ESP_OK) {
        return rx_stop_result;
    }

    hm01b0_capture_transport_stats_t transport = {0};
    hm01b0_diagnostic_report_t diagnostic = {0};
    ret = hm01b0_capture_get_transport_stats(s_capture, &transport);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = hm01b0_capture_get_diagnostic_report(s_capture, &diagnostic);
    if (ret != ESP_OK) {
        return ret;
    }

    hm01b0_log_preflight_result(name, &transport, &diagnostic);
    const bool transport_pass =
        transport.valid_frames > 0U &&
        transport.size_errors == 0U &&
        transport.no_free_buffer == 0U &&
        transport.ready_queue_overflows == 0U &&
        transport.free_queue_errors == 0U;
    if (!transport_pass) {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "preflight complete: %s", name);
    return ESP_OK;
}

static void hm01b0_cleanup(void)
{
    if (s_sensor != NULL) {
        (void)hm01b0_stream_stop(s_sensor);
        (void)hm01b0_set_test_pattern(s_sensor, HM01B0_TEST_PATTERN_OFF);
    }
    (void)hm01b0_capture_rx_stop(s_capture);
    (void)hm01b0_capture_delete(s_capture);
    s_capture = NULL;
    (void)st7789_display_delete(s_display);
    s_display = NULL;
    heap_caps_free(s_snapshot);
    s_snapshot = NULL;
    (void)hm01b0_delete(s_sensor);
    s_sensor = NULL;
}

void app_main(void)
{
    s_app_task = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "Stage 4.5 startup: initialize, preflight both patterns, "
                  "then disable test output");
    ESP_LOGI(TAG, "Control pins: MCLK=%d SDA=%d SCL=%d",
             BOARD_HM01B0_MCLK_GPIO,
             BOARD_HM01B0_I2C_SDA_GPIO,
             BOARD_HM01B0_I2C_SCL_GPIO);
    ESP_LOGI(TAG, "DVP: PCLK=%d FVLD=%d LVLD/DE=%d D0..D7="
                  "%d,%d,%d,%d,%d,%d,%d,%d",
             BOARD_HM01B0_PCLK_GPIO, BOARD_HM01B0_VSYNC_GPIO,
             BOARD_HM01B0_DE_GPIO,
             BOARD_HM01B0_D0_GPIO, BOARD_HM01B0_D1_GPIO,
             BOARD_HM01B0_D2_GPIO, BOARD_HM01B0_D3_GPIO,
             BOARD_HM01B0_D4_GPIO, BOARD_HM01B0_D5_GPIO,
             BOARD_HM01B0_D6_GPIO, BOARD_HM01B0_D7_GPIO);

    hm01b0_mode_info_t mode_info = {0};
    esp_err_t ret = hm01b0_get_mode_info(APP_HM01B0_MODE, &mode_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to obtain sensor mode geometry: %s",
                 esp_err_to_name(ret));
        return;
    }

    const hm01b0_config_t sensor_config = {
        .mclk_gpio = BOARD_HM01B0_MCLK_GPIO,
        .mclk_freq_hz = APP_HM01B0_MCLK_FREQUENCY_HZ,
        .i2c_port = I2C_NUM_0,
        .i2c_sda_gpio = BOARD_HM01B0_I2C_SDA_GPIO,
        .i2c_scl_gpio = BOARD_HM01B0_I2C_SCL_GPIO,
        .i2c_freq_hz = APP_HM01B0_I2C_FREQUENCY_HZ,
        .enable_internal_i2c_pullups = true,
        .initial_mode = APP_HM01B0_MODE,
        .data_interface = APP_HM01B0_INTERFACE,
        .test_pattern = HM01B0_TEST_PATTERN_OFF,
    };
    ret = hm01b0_new(&sensor_config, &s_sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HM01B0 initialization failed: %s",
                 esp_err_to_name(ret));
        goto fail;
    }

    uint16_t model_id = 0U;
    ret = hm01b0_probe(s_sensor, &model_id);
    if (ret != ESP_OK ||
        hm01b0_get_state(s_sensor) != HM01B0_STATE_STANDBY) {
        ESP_LOGE(TAG, "HM01B0 probe/state check failed: %s",
                 esp_err_to_name(ret));
        goto fail;
    }
    ESP_LOGI(TAG, "HM01B0 ready in STANDBY, MODEL_ID=0x%04X", model_id);

    const size_t snapshot_size =
        APP_PREFLIGHT_SNAPSHOT_WIDTH * APP_PREFLIGHT_SNAPSHOT_HEIGHT;
    s_snapshot = heap_caps_malloc(snapshot_size,
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_snapshot == NULL) {
        ret = ESP_ERR_NO_MEM;
        ESP_LOGE(TAG, "failed to allocate %u-byte preflight snapshot",
                 (unsigned)snapshot_size);
        goto fail;
    }

    const st7789_display_config_t display_config = {
        .spi_host = SPI2_HOST,
        .clock_gpio = BOARD_ST7789_SCLK_GPIO,
        .mosi_gpio = BOARD_ST7789_MOSI_GPIO,
        .reset_gpio = BOARD_ST7789_RESET_GPIO,
        .dc_gpio = BOARD_ST7789_DC_GPIO,
        .cs_gpio = BOARD_ST7789_CS_GPIO,
        .clock_speed_hz = APP_ST7789_CLOCK_HZ,
        .width = APP_ST7789_WIDTH,
        .height = APP_ST7789_HEIGHT,
    };
    ret = st7789_display_new(&display_config, &s_display);
    if (ret != ESP_OK) {
        goto fail;
    }

    const hm01b0_capture_config_t capture_config = {
        .data_gpio = {
            BOARD_HM01B0_D0_GPIO, BOARD_HM01B0_D1_GPIO,
            BOARD_HM01B0_D2_GPIO, BOARD_HM01B0_D3_GPIO,
            BOARD_HM01B0_D4_GPIO, BOARD_HM01B0_D5_GPIO,
            BOARD_HM01B0_D6_GPIO, BOARD_HM01B0_D7_GPIO,
        },
        .pclk_gpio = BOARD_HM01B0_PCLK_GPIO,
        .vsync_gpio = BOARD_HM01B0_VSYNC_GPIO,
        .de_gpio = BOARD_HM01B0_DE_GPIO,
        .frame_width = mode_info.transport_width,
        .frame_height = mode_info.transport_height,
        .dma_burst_size = APP_CAPTURE_DMA_BURST_SIZE,
    };
    ret = hm01b0_capture_new(&capture_config, &s_capture);
    if (ret != ESP_OK) {
        goto fail;
    }

    const hm01b0_frame_rect_t analysis_area =
        hm01b0_to_frame_rect(mode_info.sensor_valid);
    ret = hm01b0_run_preflight(
        HM01B0_TEST_PATTERN_WALKING_1,
        HM01B0_DIAGNOSTIC_PATTERN_WALKING_1,
        "Walking-1", analysis_area);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Walking-1 preflight failed: %s",
                 esp_err_to_name(ret));
        goto fail;
    }

    ret = hm01b0_run_preflight(
        HM01B0_TEST_PATTERN_COLOR_BAR,
        HM01B0_DIAGNOSTIC_PATTERN_COLOR_BAR,
        "Color-Bar", analysis_area);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Color-Bar preflight failed: %s",
                 esp_err_to_name(ret));
        goto fail;
    }

    ret = hm01b0_set_test_pattern(s_sensor, HM01B0_TEST_PATTERN_OFF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to disable test pattern: %s",
                 esp_err_to_name(ret));
        goto fail;
    }
    (void)hm01b0_capture_set_diagnostics(s_capture, NULL);
    ESP_LOGI(TAG,
             "Stage 4.5 complete: both preflights finished; HM01B0 is in "
             "STANDBY, Camera RX is stopped, test pattern is OFF. Real-image "
             "streaming is reserved for Stage 5.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }

fail:
    hm01b0_cleanup();
    ESP_LOGE(TAG, "Stage 4.5 stopped: %s", esp_err_to_name(ret));
}
