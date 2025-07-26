idf_component_register(
    SRCS "stusb4500.c" "stusb4500_example.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_common log
)
