idf_component_register(
    SRCS
        "hm01b0_capture.c"
        "hm01b0_cache_compat.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_cam
        esp_driver_gpio
        esp_hw_support
        esp_rom
        esp_timer
        freertos
        heap
        log
)

# ESP-IDF 6.0's DVP controller calls esp_cache_msync(M2C) for every frame
# buffer, including internal SRAM.  On ESP32-S3 internal SRAM is not cached,
# so esp_cache_msync() correctly reports ESP_ERR_NOT_SUPPORTED even though no
# synchronization is required.  Keep the workaround local to this component
# and leave the installed ESP-IDF tree untouched.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_cache_msync")
