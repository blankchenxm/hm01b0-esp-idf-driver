idf_component_register(
    SRCS
        "main.c"
        "app_mode_profile.c"
    INCLUDE_DIRS "."
    REQUIRES
        hm01b0
        hm01b0_capture
        hm01b0_image
        st7789_display
        esp_timer
)
