#include "servo.h"

// 计算占空比
uint16_t servo_pos_get(float theta)
{
    //角度限位
    theta = CLAMP(theta,ANGLE_MIN,ANGLE_MAX);

    // 0度0.5ms = 0.025f(0.5/20)
    uint16_t duty = ((float)theta / 270.0f ) * 2000 + 500;
    return duty;
}
