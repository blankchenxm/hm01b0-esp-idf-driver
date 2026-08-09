#include "driver/i2c_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "board_config.h"
#include "hm01b0.h"
#include "hm01b0_capture.h"

#define HM01B0_MCLK_FREQUENCY_HZ 12000000U
#define HM01B0_I2C_FREQUENCY_HZ    100000U

#define HM01B0_QVGA_RAW_WIDTH        324U
#define HM01B0_QVGA_RAW_HEIGHT       244U
#define HM01B0_CAPTURE_DMA_BURST       64U
#define HM01B0_CAPTURE_TASK_STACK     4096U
#define HM01B0_CAPTURE_TASK_PRIORITY     5U
#define HM01B0_CAPTURE_STATS_PERIOD_MS 1000U

static const char *TAG = "hm01b0_probe";
static hm01b0_handle_t *s_sensor;
static hm01b0_capture_handle_t *s_capture;

void app_main(void)
{
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
        .dma_burst_size = HM01B0_CAPTURE_DMA_BURST,
        .task_stack_size = HM01B0_CAPTURE_TASK_STACK,
        .task_priority = HM01B0_CAPTURE_TASK_PRIORITY,
        .stats_period_ms = HM01B0_CAPTURE_STATS_PERIOD_MS,
        .validate_walking_1 = true,
    };

    ret = hm01b0_capture_new(&capture_config, &s_capture);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FAIL: DVP capture initialization failed: %s",
                 esp_err_to_name(ret));
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
        (void)hm01b0_delete(s_sensor);
        s_sensor = NULL;
        return;
    }

    ESP_LOGI(TAG,
             "Stage 3 running: HM01B0 QVGA RAW8 324x244 Walking-1; "
             "ST7789 output and cropping are disabled");

    /* Keep the application owner task alive while the capture task validates. */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
