#ifndef _SERVO_H_
#define _SERVO_H_

#include "stdint.h"
#include "math.h"

// 把 x 的值限制在 [min, max] 之间
#define CLAMP(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

// 根据实际修改
#define ANGLE_MAX           270.0f      // 角度限位
#define ANGLE_MIN           0.0f
#define SERVO_INIT_ANGLE    135.0f      // 舵机初始角度
#define SERVO2_INIT_ANGLE   141.0f      // 舵机2初始角度

typedef struct
{
    float target_angle;
    float current_angle;
}servo_angle_t;

uint16_t servo_pos_get(float theta);

#endif
