idf_component_register(
    SRCS "src/kd_pixdriver.cpp"
         "src/pixel_effects.cpp"
         "src/rmt_led_encoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "driver" "nvs_flash" "esp_system" "log"
)
