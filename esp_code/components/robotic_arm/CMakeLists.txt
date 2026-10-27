idf_component_register(
    SRCS
        "myuart.c"
        "servo.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)