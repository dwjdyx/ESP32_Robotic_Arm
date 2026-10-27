#include "myiic.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

void myiic_init(void)
{
    i2c_config_t myi2c_config_struct = {
        .clk_flags = 0,
        .master.clk_speed = 50000,
        .mode = I2C_MODE_MASTER,
        .scl_io_num = OLED_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,      //是否内部上拉
        .sda_io_num = OLED_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE
    };
    i2c_param_config(I2C_NUM_0, &myi2c_config_struct);
    //注册iic
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}
