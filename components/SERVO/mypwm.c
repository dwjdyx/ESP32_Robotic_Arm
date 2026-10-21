#include "mypwm.h"

//PWM通道初始化
static void mypwm_channel_init(ledc_channel_t channel, int gpio)
{
    ledc_channel_config_t channel_config = {
        .channel = channel,
        .duty = 0,
        .flags.output_invert = 0,
        .gpio_num = gpio,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_1
    };

    ledc_channel_config(&channel_config);
}

void mypwm_init(void)
{
    ledc_timer_config_t ledc_timer_init = {
                .clk_cfg = LEDC_AUTO_CLK,
                .duty_resolution = LEDC_TIMER_10_BIT,           //分辨率,把占空比表示成1024份
                .freq_hz = 50,                                //脉冲频率
                .speed_mode = LEDC_LOW_SPEED_MODE,              //esp用低速
                .timer_num = LEDC_TIMER_1                       //选择的定时器
    };
    ledc_timer_config(&ledc_timer_init);                        //  配置时钟源和计数器

    // 配置 8 路 PWM
    mypwm_channel_init(LEDC_CHANNEL_0, BASE); 
    mypwm_channel_init(LEDC_CHANNEL_1, SHOULDER);
    mypwm_channel_init(LEDC_CHANNEL_2, ELBOW);
    mypwm_channel_init(LEDC_CHANNEL_3, WRIST);
//    mypwm_channel_init(LEDC_CHANNEL_4, GRIPPER);
}

//修改占空比
void pwm_duty_set(ledc_channel_t channel, uint16_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

