idf_component_register(
    SRCS "system.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver 
        nvs_flash
        robotic_arm
        websocket
        OLED
        BLE
        manipulate
        button
)