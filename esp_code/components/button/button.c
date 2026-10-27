#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "button.h"

void button_init(void)
{
    gpio_config_t gpio_struct_init = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ull << KEY_PIN,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    gpio_config(&gpio_struct_init);
}

uint8_t get_button_state(gpio_num_t button_pin)
{
    if(gpio_get_level(button_pin))
    {
        return PRESSED;
    }else return UNPRESSED;
}
