#ifndef _BUTTON_H_
#define _BUTTON_H_

#include "driver/gpio.h"
#include "stdint.h"

#define  KEY_PIN    GPIO_NUM_41

#define  PRESSED    1
#define  UNPRESSED  0

void button_init(void);
uint8_t get_button_state(gpio_num_t button_pin);

#endif
