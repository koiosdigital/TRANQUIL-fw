file(GLOB_RECURSE NESTED_SRC
       **.cpp
)

idf_component_register(
    SRCS ${NESTED_SRC}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_common log
    REQUIRES esp_driver_i2c
)
