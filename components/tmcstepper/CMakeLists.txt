idf_component_register(
    SRCS "src/uartbus.cpp" "src/tmc2209.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio freertos esp_common
)
