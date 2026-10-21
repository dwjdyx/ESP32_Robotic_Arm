idf_component_register(
    SRCS
        "myiic.c"
        "OLED_Data.c"
        "oled.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)