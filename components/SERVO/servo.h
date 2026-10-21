#ifndef _SERVO_H_
#define _SERVO_H_

#include "stdint.h"
#include "mypwm.h"

// 根据实际修改
#define SERVO_INIT_ANGLE   135.0f      // 舵机初始角度

typedef enum
{
    SERVO_BASE = 0,      // 底座旋转 θ1
    SERVO_SHOULDER,      // 肩部 θ2
    SERVO_ELBOW,         // 肘部 θ3
    SERVO_WRIST,         // 腕部/爪子转向 θ4
//    SERVO_GRIPPER,        // 末端旋转/夹爪 θ5
    SERVO_MAX

} servo_id_t;


//舵机对应pwm通道
static const ledc_channel_t servo_pwm_channel[SERVO_MAX]=
{
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
//    LEDC_CHANNEL_4
};

void servo_init(void);
void servo_angle_set(servo_id_t id, float theta);

#endif
