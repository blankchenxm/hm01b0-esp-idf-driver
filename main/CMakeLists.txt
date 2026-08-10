idf_component_register(
    SRCS "main.c" "st7789_display.c"
    INCLUDE_DIRS "."
    REQUIRES
        hm01b0
        hm01b0_capture
        esp_driver_gpio
        esp_driver_spi
        esp_timer
        heap
)
