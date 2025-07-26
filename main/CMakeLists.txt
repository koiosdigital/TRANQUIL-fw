file(GLOB_RECURSE NESTED_SRC
       **.cpp
)


list(APPEND INCLUDE_DIRS ".")
list(APPEND INCLUDE_DIRS "internet_time")
list(APPEND INCLUDE_DIRS "api")
list(APPEND INCLUDE_DIRS "sd")
list(APPEND INCLUDE_DIRS "RobotMotion")
list(APPEND INCLUDE_DIRS "stusb4500")

message(STATUS ${NESTED_SRC})

idf_component_register(
    SRCS ${NESTED_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES lwip esp_wifi heap json bootloader_support kd_common esp_http_client wifi_provisioning esp_driver_rmt mbedtls driver esp_http_server clock-app kd_pixdriver fatfs vfs sdmmc tmcstepper
)

idf_build_set_property(COMPILE_OPTIONS "-Wno-missing-field-initializers" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=parentheses" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-write-strings" APPEND)

idf_build_set_property(COMPILE_OPTIONS "-DDEVICE_NAME_PREFIX=\"TRANQUIL\"" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-DKD_COMMON_CRYPTO_DISABLE" APPEND)