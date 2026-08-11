idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        hm01b0
        hm01b0_capture
        st7789_display
        esp_timer
        heap
)
