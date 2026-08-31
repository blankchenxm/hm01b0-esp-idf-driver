#ifndef HM01B0_H
#define HM01B0_H

#include <stdint.h>

#include "esp_err.h"
#include "hm01b0_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HM01B0_I2C_ADDRESS       0x24U
#define HM01B0_EXPECTED_MODEL_ID 0x01B0U

/**
 * @brief Create and initialize an HM01B0, leaving it in software standby.
 *
 * Sequence: start MCLK, wait for POR, create I2C, probe, software reset,
 * standby, common initialization, mode, interface/timing, and test pattern.
 * The sensor power rails must already be stable before this function is called.
 */
esp_err_t hm01b0_new(const hm01b0_config_t *config,
                     hm01b0_handle_t **out_handle);

/** Stop MCLK, release the I2C resources, and destroy the handle. */
esp_err_t hm01b0_delete(hm01b0_handle_t *dev);

/** Read and verify MODEL_ID_H/MODEL_ID_L against 0x01B0. */
esp_err_t hm01b0_probe(hm01b0_handle_t *dev, uint16_t *model_id);

/** Software-reset the sensor. A successful reset always enters standby. */
esp_err_t hm01b0_reset(hm01b0_handle_t *dev);

/** Enter software standby. Configuration changes are allowed in this state. */
esp_err_t hm01b0_standby(hm01b0_handle_t *dev);

/** Start sensor pixel streaming from the standby state. */
esp_err_t hm01b0_stream_start(hm01b0_handle_t *dev);

/** Stop sensor pixel streaming and return to software standby. */
esp_err_t hm01b0_stream_stop(hm01b0_handle_t *dev);

/** Apply one of the Full/QVGA/QQVGA mode tables while in standby. */
esp_err_t hm01b0_set_mode(hm01b0_handle_t *dev, hm01b0_mode_t mode);

/** Apply one of the 8-bit/4-bit/1-bit interface tables while in standby. */
esp_err_t hm01b0_set_interface(hm01b0_handle_t *dev,
                               hm01b0_interface_t interface);

/**
 * @brief Configure frame timing for the current mode and interface.
 *
 * The driver keeps LINE_LENGTH_PCK at the datasheet minimum for the current
 * mode, derives FRAME_LENGTH_LINES from Sensor_Core and the requested preset,
 * and keeps MAX_INTEGRATION at FRAME_LENGTH_LINES - 2. Configuration is only
 * allowed in standby. A preset that cannot be reached by the current clock,
 * interface divider, and mode constraints returns ESP_ERR_NOT_SUPPORTED.
 */
esp_err_t hm01b0_set_frame_rate(hm01b0_handle_t *dev,
                                hm01b0_frame_rate_t frame_rate);

/** Select test-pattern off, color bar, or walking-1 while in standby. */
esp_err_t hm01b0_set_test_pattern(hm01b0_handle_t *dev,
                                  hm01b0_test_pattern_t pattern);

/** Return the transport and crop geometry associated with a sensor mode. */
esp_err_t hm01b0_get_mode_info(hm01b0_mode_t mode,
                               hm01b0_mode_info_t *info);

/** Return the configured physical sensor variant. */
hm01b0_variant_t hm01b0_get_variant(const hm01b0_handle_t *dev);

/** Return the RAW8 interpretation derived from variant and PIXEL_ORDER. */
hm01b0_pixel_format_t hm01b0_get_pixel_format(const hm01b0_handle_t *dev);

const char *hm01b0_variant_name(hm01b0_variant_t variant);
const char *hm01b0_pixel_format_name(hm01b0_pixel_format_t format);

/** Return the driver's current state without accessing the sensor bus. */
hm01b0_state_t hm01b0_get_state(const hm01b0_handle_t *dev);

esp_err_t hm01b0_reg_read(hm01b0_handle_t *dev,
                          uint16_t addr,
                          uint8_t *value);

esp_err_t hm01b0_reg_write(hm01b0_handle_t *dev,
                           uint16_t addr,
                           uint8_t value);

esp_err_t hm01b0_reg_update_bits(hm01b0_handle_t *dev,
                                 uint16_t addr,
                                 uint8_t mask,
                                 uint8_t value);

/**
 * @brief Apply a register table.
 *
 * A mask of 0xFF writes the value directly. Other masks perform a read-modify-
 * write. delay_ms is applied after the corresponding entry. Tables are
 * rejected unless the sensor is in software standby.
 */
esp_err_t hm01b0_write_table(hm01b0_handle_t *dev,
                             const hm01b0_regval_t *table,
                             size_t count);

#ifdef __cplusplus
}
#endif

#endif /* HM01B0_H */
