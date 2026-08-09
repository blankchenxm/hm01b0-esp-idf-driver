#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "hm01b0.h"
#include "hm01b0_private.h"

#define HM01B0_I2C_TIMEOUT_MS 100

static const char *TAG = "hm01b0_reg";

esp_err_t hm01b0_reg_read(hm01b0_handle_t *dev,
                          uint16_t addr,
                          uint8_t *value)
{
    ESP_RETURN_ON_FALSE(dev != NULL && dev->i2c_device != NULL &&
                        value != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    const uint8_t register_address[2] = {
        (uint8_t)(addr >> 8),
        (uint8_t)(addr & 0xFFU),
    };

    return i2c_master_transmit_receive(dev->i2c_device,
                                       register_address,
                                       sizeof(register_address),
                                       value,
                                       1,
                                       HM01B0_I2C_TIMEOUT_MS);
}

esp_err_t hm01b0_reg_write(hm01b0_handle_t *dev,
                           uint16_t addr,
                           uint8_t value)
{
    ESP_RETURN_ON_FALSE(dev != NULL && dev->i2c_device != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid device handle");

    const uint8_t transaction[3] = {
        (uint8_t)(addr >> 8),
        (uint8_t)(addr & 0xFFU),
        value,
    };

    return i2c_master_transmit(dev->i2c_device,
                               transaction,
                               sizeof(transaction),
                               HM01B0_I2C_TIMEOUT_MS);
}

esp_err_t hm01b0_reg_update_bits(hm01b0_handle_t *dev,
                                 uint16_t addr,
                                 uint8_t mask,
                                 uint8_t value)
{
    uint8_t current = 0;
    ESP_RETURN_ON_ERROR(hm01b0_reg_read(dev, addr, &current), TAG,
                        "failed to read register 0x%04X", addr);

    const uint8_t updated = (current & (uint8_t)~mask) | (value & mask);
    if (updated == current) {
        return ESP_OK;
    }

    return hm01b0_reg_write(dev, addr, updated);
}

esp_err_t hm01b0_write_table(hm01b0_handle_t *dev,
                             const hm01b0_regval_t *table,
                             size_t count)
{
    ESP_RETURN_ON_FALSE(dev != NULL && (table != NULL || count == 0),
                        ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(dev->state == HM01B0_STATE_STANDBY,
                        ESP_ERR_INVALID_STATE, TAG,
                        "register tables may only be applied in standby");

    for (size_t i = 0; i < count; ++i) {
        esp_err_t ret;
        if (table[i].mask == UINT8_MAX) {
            ret = hm01b0_reg_write(dev, table[i].addr, table[i].value);
        } else {
            ret = hm01b0_reg_update_bits(dev, table[i].addr,
                                         table[i].mask, table[i].value);
        }
        ESP_RETURN_ON_ERROR(ret, TAG,
                            "register table failed at index %u, address 0x%04X",
                            (unsigned)i, table[i].addr);

        if (table[i].delay_ms > 0) {
            TickType_t delay_ticks = pdMS_TO_TICKS(table[i].delay_ms);
            if (delay_ticks == 0) {
                delay_ticks = 1;
            }
            vTaskDelay(delay_ticks);
        }
    }

    return ESP_OK;
}
