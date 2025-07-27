file(GLOB_RECURSE NESTED_SRC
       **.cpp
)

idf_component_register(
    SRCS ${NESTED_SRC}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES vfs json
    REQUIRES esp_driver_sdmmc fatfs
)