#include "servo.h"

static void servo_angle_init(void)
{
    servo_angle_set(SERVO_BASE, SERVO_INIT_ANGLE);
    servo_angle_set(SERVO_SHOULDER, SERVO_INIT_ANGLE);
    servo_angle_set(SERVO_ELBOW, SERVO_INIT_ANGLE);
    servo_angle_set(SERVO_WRIST, SERVO_INIT_ANGLE);
//    servo_angle_set(SERVO_GRIPPER, SERVO_INIT_ANGLE);
}

void servo_init(void)
{
    mypwm_init();
    //角度初始化
    servo_angle_init();
}

//角度映射占空比
void servo_angle_set(servo_id_t id, float theta)
{
    if (id >= SERVO_MAX)
        return;

    ledc_channel_t channel = servo_pwm_channel[id];

    if(theta > 270)
    {
        theta = 270;
    }else if(theta < 0)
    {
        theta = 0;
    }
    // 0度0.5ms = 0.025f(0.5/20)
    uint16_t duty = ((float)theta / 2700.0f + 0.025f) * 1024;
    pwm_duty_set(channel, duty);
}
