idf_component_register(
    SRCS "src/uartbus.cpp" "src/tmc2209.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver freertos esp_common
)
