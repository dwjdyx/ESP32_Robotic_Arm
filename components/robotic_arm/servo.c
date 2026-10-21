#include "servo.h"

servo_angle_t angle[SERVO_MAX];

void servo_angle_init(void)
{
    for(servo_id_t index = SERVO_BASE; index<SERVO_MAX; index++)
    {
        if(index == SERVO_SHOULDER)
        {
            angle[index].target_angle = SERVO2_INIT_ANGLE;
            angle[index].current_angle = SERVO2_INIT_ANGLE;
        }
        else
        {
            angle[index].target_angle = SERVO_INIT_ANGLE;
            angle[index].current_angle = SERVO_INIT_ANGLE;
        }
    }
}

// 计算占空比
uint16_t servo_pos_get(float theta)
{
    //角度限位
    theta = CLAMP(theta,ANGLE_MIN,ANGLE_MAX);

    // 0度0.5ms = 0.025f(0.5/20)
    uint16_t duty = ((float)theta / 270.0f ) * 2000 + 500;
    return duty;
}
