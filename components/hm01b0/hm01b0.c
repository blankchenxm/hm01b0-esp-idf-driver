#include <inttypes.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "hm01b0.h"
#include "hm01b0_private.h"
#include "hm01b0_tables.h"

#define HM01B0_POR_DELAY_US  50U
#define HM01B0_MCLK_DUTY     1U

static const char *TAG = "hm01b0";

static esp_err_t hm01b0_start_mclk(hm01b0_handle_t *dev,
                                   gpio_num_t gpio,
                                   uint32_t frequency_hz)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = dev->mclk_speed_mode,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num = dev->mclk_timer,
        .freq_hz = frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG,
                        "failed to configure MCLK timer");

    const ledc_channel_config_t channel_config = {
        .gpio_num = gpio,
        .speed_mode = dev->mclk_speed_mode,
        .channel = dev->mclk_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = dev->mclk_timer,
        .duty = HM01B0_MCLK_DUTY,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG,
                        "failed to route MCLK to GPIO%d", gpio);

    dev->mclk_started = true;
    const uint32_t actual_hz = ledc_get_freq(dev->mclk_speed_mode,
                                             dev->mclk_timer);
    ESP_LOGI(TAG, "MCLK started: GPIO%d, requested=%" PRIu32
                  " Hz, actual=%" PRIu32 " Hz",
             gpio, frequency_hz, actual_hz);
    return ESP_OK;
}

static void hm01b0_stop_mclk(hm01b0_handle_t *dev)
{
    if (dev->mclk_started) {
        (void)ledc_stop(dev->mclk_speed_mode, dev->mclk_channel, 0);
        dev->mclk_started = false;
    }
}

esp_err_t hm01b0_new(const hm01b0_config_t *config,
                     hm01b0_handle_t **out_handle)
{
    ESP_RETURN_ON_FALSE(config != NULL && out_handle != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(config->mclk_gpio) &&
                        GPIO_IS_VALID_GPIO(config->i2c_sda_gpio) &&
                        GPIO_IS_VALID_GPIO(config->i2c_scl_gpio) &&
                        config->mclk_freq_hz > 0 &&
                        config->i2c_freq_hz > 0 &&
                        config->initial_mode >= HM01B0_SENSOR_MODE_FULL &&
                        config->initial_mode <= HM01B0_SENSOR_MODE_QQVGA &&
                        config->data_interface >= HM01B0_DATA_INTERFACE_8_BIT &&
                        config->data_interface <= HM01B0_DATA_INTERFACE_1_BIT &&
                        config->test_pattern >= HM01B0_TEST_PATTERN_OFF &&
                        config->test_pattern <= HM01B0_TEST_PATTERN_WALKING_1,
                        ESP_ERR_INVALID_ARG, TAG, "invalid HM01B0 configuration");

    *out_handle = NULL;
    hm01b0_handle_t *dev = calloc(1, sizeof(*dev));
    ESP_RETURN_ON_FALSE(dev != NULL, ESP_ERR_NO_MEM, TAG,
                        "failed to allocate device handle");

    dev->mclk_speed_mode = LEDC_LOW_SPEED_MODE;
    dev->mclk_timer = LEDC_TIMER_0;
    dev->mclk_channel = LEDC_CHANNEL_0;
    dev->state = HM01B0_STATE_UNINITIALIZED;

    esp_err_t ret = hm01b0_start_mclk(dev, config->mclk_gpio,
                                      config->mclk_freq_hz);
    if (ret != ESP_OK) {
        free(dev);
        return ret;
    }

    /* MCLK must be present while the sensor completes POR. */
    esp_rom_delay_us(HM01B0_POR_DELAY_US);

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = config->i2c_port,
        .sda_io_num = config->i2c_sda_gpio,
        .scl_io_num = config->i2c_scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = config->enable_internal_i2c_pullups,
    };
    ret = i2c_new_master_bus(&bus_config, &dev->i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to create I2C bus: %s", esp_err_to_name(ret));
        hm01b0_stop_mclk(dev);
        free(dev);
        return ret;
    }

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = HM01B0_I2C_ADDRESS,
        .scl_speed_hz = config->i2c_freq_hz,
    };
    ret = i2c_master_bus_add_device(dev->i2c_bus, &device_config,
                                    &dev->i2c_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to add I2C device 0x%02X: %s",
                 HM01B0_I2C_ADDRESS, esp_err_to_name(ret));
        (void)i2c_del_master_bus(dev->i2c_bus);
        hm01b0_stop_mclk(dev);
        free(dev);
        return ret;
    }

    ESP_LOGI(TAG, "I2C ready: address=0x%02X, SDA=GPIO%d, SCL=GPIO%d, "
                  "frequency=%" PRIu32 " Hz",
             HM01B0_I2C_ADDRESS, config->i2c_sda_gpio,
             config->i2c_scl_gpio, config->i2c_freq_hz);

    uint16_t model_id = 0;
    ret = hm01b0_probe(dev, &model_id);
    if (ret != ESP_OK) {
        goto initialization_failed;
    }

    ret = hm01b0_reset(dev);
    if (ret != ESP_OK) {
        goto initialization_failed;
    }

    ret = hm01b0_standby(dev);
    if (ret != ESP_OK) {
        goto initialization_failed;
    }

    ret = hm01b0_write_table(dev, hm01b0_common_init,
                              hm01b0_common_init_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to apply common initialization table: %s",
                 esp_err_to_name(ret));
        goto initialization_failed;
    }

    ret = hm01b0_set_mode(dev, config->initial_mode);
    if (ret != ESP_OK) {
        goto initialization_failed;
    }

    ret = hm01b0_set_interface(dev, config->data_interface);
    if (ret != ESP_OK) {
        goto initialization_failed;
    }

    ret = hm01b0_set_test_pattern(dev, config->test_pattern);
    if (ret != ESP_OK) {
        goto initialization_failed;
    }

    ESP_LOGI(TAG, "initialization complete: MODEL_ID=0x%04X, state=STANDBY",
             model_id);

    *out_handle = dev;
    return ESP_OK;

initialization_failed:
    ESP_LOGE(TAG, "sensor initialization failed: %s", esp_err_to_name(ret));
    (void)hm01b0_delete(dev);
    return ret;
}

esp_err_t hm01b0_delete(hm01b0_handle_t *dev)
{
    if (dev == NULL) {
        return ESP_OK;
    }

    esp_err_t result = ESP_OK;
    if (dev->state == HM01B0_STATE_STREAMING) {
        result = hm01b0_stream_stop(dev);
    }
    if (dev->i2c_device != NULL) {
        const esp_err_t ret = i2c_master_bus_rm_device(dev->i2c_device);
        if (result == ESP_OK) {
            result = ret;
        }
    }
    if (dev->i2c_bus != NULL) {
        const esp_err_t ret = i2c_del_master_bus(dev->i2c_bus);
        if (result == ESP_OK) {
            result = ret;
        }
    }

    hm01b0_stop_mclk(dev);
    free(dev);
    return result;
}
