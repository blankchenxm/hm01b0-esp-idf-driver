#include <stddef.h>

#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_err.h"
#include "esp_memory_utils.h"

/*
 * ESP-IDF 6.0 esp_driver_cam unconditionally calls esp_cache_msync(M2C)
 * before starting a DVP transaction.  Internal ESP32-S3 SRAM is DMA-capable
 * but is not behind the data cache, so the real function returns
 * ESP_ERR_NOT_SUPPORTED.  For that one no-op case, success is the correct
 * result.  All cacheable buffers and all other error results remain governed
 * by the original ESP-IDF implementation.
 */
extern esp_err_t __real_esp_cache_msync(void *addr, size_t size, int flags);

esp_err_t IRAM_ATTR __wrap_esp_cache_msync(void *addr, size_t size, int flags)
{
    esp_err_t ret = __real_esp_cache_msync(addr, size, flags);

    if (ret == ESP_ERR_NOT_SUPPORTED &&
        (flags & ESP_CACHE_MSYNC_FLAG_DIR_M2C) != 0 &&
        esp_ptr_internal(addr)) {
        return ESP_OK;
    }

    return ret;
}
