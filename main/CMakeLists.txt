file(GLOB_RECURSE NESTED_SRC
       **.cpp
)


list(APPEND INCLUDE_DIRS ".")
list(APPEND INCLUDE_DIRS "api")
list(APPEND INCLUDE_DIRS "usb_pd")

message(STATUS ${NESTED_SRC})

idf_component_register(
    SRCS ${NESTED_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    PRIV_REQUIRES lwip esp_wifi heap bootloader_support kd_common wifi_provisioning mbedtls esp_http_server kd_pixdriver stusb4500 kd_ntp sd RobotMotion tranquil-app
)

idf_build_set_property(COMPILE_OPTIONS "-Wno-missing-field-initializers" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=parentheses" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-write-strings" APPEND)

idf_build_set_property(COMPILE_OPTIONS "-DDEVICE_NAME_PREFIX=\"TRANQUIL\"" APPEND)