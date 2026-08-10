#include <inttypes.h>

#include "driver/i2c_types.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "board_config.h"
#include "hm01b0.h"
#include "hm01b0_capture.h"
#include "st7789_display.h"

#define HM01B0_MCLK_FREQUENCY_HZ 12000000U
#define HM01B0_I2C_FREQUENCY_HZ    100000U

#define HM01B0_QVGA_RAW_WIDTH        324U
#define HM01B0_QVGA_RAW_HEIGHT       244U
#define HM01B0_QVGA_ACTIVE_X           2U
#define HM01B0_QVGA_ACTIVE_Y           0U
#define HM01B0_QVGA_ACTIVE_WIDTH     320U
#define HM01B0_QVGA_ACTIVE_HEIGHT    244U
#define HM01B0_CAPTURE_DMA_BURST       64U
#define HM01B0_CAPTURE_TASK_STACK     4096U
#define HM01B0_CAPTURE_TASK_PRIORITY     5U
#define HM01B0_CAPTURE_STATS_PERIOD_MS 1000U
#define HM01B0_CAPTURE_WARMUP_FRAMES      5U

#define HM01B0_SNAPSHOT_X                 42U
#define HM01B0_SNAPSHOT_Y                  2U
#define HM01B0_SNAPSHOT_WIDTH            240U
#define HM01B0_SNAPSHOT_HEIGHT           240U
#define HM01B0_SNAPSHOT_SIZE             \
    (HM01B0_SNAPSHOT_WIDTH * HM01B0_SNAPSHOT_HEIGHT)

static const char *TAG = "hm01b0_probe";
static hm01b0_handle_t *s_sensor;
static hm01b0_capture_handle_t *s_capture;
static uint8_t *s_snapshot;
static TaskHandle_t s_app_task;
static uint32_t s_snapshot_sequence;

static void hm01b0_snapshot_ready(const uint8_t *buffer,
                                  uint16_t width,
                                  uint16_t height,
                                  uint32_t sequence,
                                  void *user_data)
{
    (void)buffer;
    (void)width;
    (void)height;
    s_snapshot_sequence = sequence;
    xTaskNotifyGive((TaskHandle_t)user_data);
}

void app_main(void)
{
    s_app_task = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "HM01B0 model ID probe starting");
    ESP_LOGI(TAG, "Control pins: MCLK=GPIO%d, SDA=GPIO%d, SCL=GPIO%d",
             BOARD_HM01B0_MCLK_GPIO, BOARD_HM01B0_I2C_SDA_GPIO,
             BOARD_HM01B0_I2C_SCL_GPIO);
    ESP_LOGI(TAG, "8-bit DVP bus: PCLK=%d FVLD=%d LVLD/DE=%d "
                  "D0..D7=%d,%d,%d,%d,%d,%d,%d,%d",
             BOARD_HM01B0_PCLK_GPIO, BOARD_HM01B0_VSYNC_GPIO,
             BOARD_HM01B0_DE_GPIO, BOARD_HM01B0_D0_GPIO,
             BOARD_HM01B0_D1_GPIO, BOARD_HM01B0_D2_GPIO,
             BOARD_HM01B0_D3_GPIO, BOARD_HM01B0_D4_GPIO,
             BOARD_HM01B0_D5_GPIO, BOARD_HM01B0_D6_GPIO,
             BOARD_HM01B0_D7_GPIO);

    const hm01b0_config_t config = {
        .mclk_gpio = BOARD_HM01B0_MCLK_GPIO,
        .mclk_freq_hz = HM01B0_MCLK_FREQUENCY_HZ,
        .i2c_port = I2C_NUM_0,
        .i2c_sda_gpio = BOARD_HM01B0_I2C_SDA_GPIO,
        .i2c_scl_gpio = BOARD_HM01B0_I2C_SCL_GPIO,
        .i2c_freq_hz = HM01B0_I2C_FREQUENCY_HZ,
        .enable_internal_i2c_pullups = true,
        .initial_mode = HM01B0_SENSOR_MODE_QVGA,
        .data_interface = HM01B0_DATA_INTERFACE_8_BIT,
        .test_pattern = HM01B0_TEST_PATTERN_WALKING_1,
    };

    esp_err_t ret = hm01b0_new(&config, &s_sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: HM01B0 initialization failed: %s",
                 esp_err_to_name(ret));
        return;
    }

    uint16_t model_id = 0;
    ret = hm01b0_probe(s_sensor, &model_id);
    if (ret != ESP_OK ||
        hm01b0_get_state(s_sensor) != HM01B0_STATE_STANDBY) {
        ESP_LOGE(TAG, "FAIL: ID read=%s, state=%d",
                 esp_err_to_name(ret), (int)hm01b0_get_state(s_sensor));
        (void)hm01b0_delete(s_sensor);
        s_sensor = NULL;
        return;
    }

    ESP_LOGI(TAG, "PASS: HM01B0 initialized in STANDBY, MODEL_ID=0x%04X",
             model_id);

    s_snapshot = heap_caps_malloc(HM01B0_SNAPSHOT_SIZE,
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_snapshot == NULL) {
        ESP_LOGE(TAG, "FAIL: unable to allocate %u-byte RAW8 snapshot",
                 (unsigned)HM01B0_SNAPSHOT_SIZE);
        (void)hm01b0_delete(s_sensor);
        s_sensor = NULL;
        return;
    }
    ESP_LOGI(TAG,
             "Stage 4 snapshot allocated: buffer=%p, crop=(%u,%u %ux%u), "
             "bytes=%u",
             s_snapshot, HM01B0_SNAPSHOT_X, HM01B0_SNAPSHOT_Y,
             HM01B0_SNAPSHOT_WIDTH, HM01B0_SNAPSHOT_HEIGHT,
             (unsigned)HM01B0_SNAPSHOT_SIZE);

    ret = st7789_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: ST7789 initialization failed: %s",
                 esp_err_to_name(ret));
        heap_caps_free(s_snapshot);
        s_snapshot = NULL;
        (void)hm01b0_delete(s_sensor);
        s_sensor = NULL;
        return;
    }

    const hm01b0_capture_config_t capture_config = {
        .data_gpio = {
            BOARD_HM01B0_D0_GPIO,
            BOARD_HM01B0_D1_GPIO,
            BOARD_HM01B0_D2_GPIO,
            BOARD_HM01B0_D3_GPIO,
            BOARD_HM01B0_D4_GPIO,
            BOARD_HM01B0_D5_GPIO,
            BOARD_HM01B0_D6_GPIO,
            BOARD_HM01B0_D7_GPIO,
        },
        .pclk_gpio = BOARD_HM01B0_PCLK_GPIO,
        .vsync_gpio = BOARD_HM01B0_VSYNC_GPIO,
        .de_gpio = BOARD_HM01B0_DE_GPIO,
        .raw_width = HM01B0_QVGA_RAW_WIDTH,
        .raw_height = HM01B0_QVGA_RAW_HEIGHT,
        .active_x = HM01B0_QVGA_ACTIVE_X,
        .active_y = HM01B0_QVGA_ACTIVE_Y,
        .active_width = HM01B0_QVGA_ACTIVE_WIDTH,
        .active_height = HM01B0_QVGA_ACTIVE_HEIGHT,
        .dma_burst_size = HM01B0_CAPTURE_DMA_BURST,
        .task_stack_size = HM01B0_CAPTURE_TASK_STACK,
        .task_priority = HM01B0_CAPTURE_TASK_PRIORITY,
        .stats_period_ms = HM01B0_CAPTURE_STATS_PERIOD_MS,
        .warmup_frames = HM01B0_CAPTURE_WARMUP_FRAMES,
        .analyze_walking_1 = true,
        .snapshot_buffer = s_snapshot,
        .snapshot_buffer_size = HM01B0_SNAPSHOT_SIZE,
        .snapshot_x = HM01B0_SNAPSHOT_X,
        .snapshot_y = HM01B0_SNAPSHOT_Y,
        .snapshot_width = HM01B0_SNAPSHOT_WIDTH,
        .snapshot_height = HM01B0_SNAPSHOT_HEIGHT,
        .on_snapshot_ready = hm01b0_snapshot_ready,
        .snapshot_user_data = s_app_task,
    };

    ret = hm01b0_capture_new(&capture_config, &s_capture);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: DVP capture initialization failed: %s",
                 esp_err_to_name(ret));
        heap_caps_free(s_snapshot);
        s_snapshot = NULL;
        (void)hm01b0_delete(s_sensor);
        s_sensor = NULL;
        return;
    }

    ret = hm01b0_capture_start(s_capture);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: Camera RX start failed: %s",
                 esp_err_to_name(ret));
        (void)hm01b0_capture_delete(s_capture);
        s_capture = NULL;
        heap_caps_free(s_snapshot);
        s_snapshot = NULL;
        (void)hm01b0_delete(s_sensor);
        s_sensor = NULL;
        return;
    }

    ret = hm01b0_start(s_sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: HM01B0 streaming start failed: %s",
                 esp_err_to_name(ret));
        (void)hm01b0_capture_stop(s_capture);
        (void)hm01b0_capture_delete(s_capture);
        s_capture = NULL;
        heap_caps_free(s_snapshot);
        s_snapshot = NULL;
        (void)hm01b0_delete(s_sensor);
        s_sensor = NULL;
        return;
    }

    ESP_LOGI(TAG,
             "Stage 4 running: waiting for one post-warm-up Walking-1 "
             "snapshot while Camera RX continues");

    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    const int64_t display_start_us = esp_timer_get_time();
    st7789_draw_gray_image(0U, 0U,
                           HM01B0_SNAPSHOT_WIDTH,
                           HM01B0_SNAPSHOT_HEIGHT,
                           s_snapshot);
    const uint32_t display_time_us = (uint32_t)(
        esp_timer_get_time() - display_start_us);
    ESP_LOGI(TAG,
             "Stage 4 static frame displayed: source_frame=%" PRIu32
             ", destination=(0,0 240x240), display=%" PRIu32 "us",
             s_snapshot_sequence, display_time_us);

    /* ST7789 GRAM retains the snapshot while Camera RX keeps running. */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
