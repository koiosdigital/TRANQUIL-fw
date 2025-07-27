file(GLOB_RECURSE NESTED_SRC
       **.cpp
)

idf_component_register(
    SRCS ${NESTED_SRC}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_http_client json esp_event mbedtls nvs_flash esp_netif esp_wifi
)