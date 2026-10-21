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
#define SERVO_UPDATE_STEP   0.5f        // 舵机更新一次角度

typedef enum
{
    SERVO_BASE = 0,      // 底座旋转 θ1
    SERVO_SHOULDER,      // 肩部 θ2
    SERVO_ELBOW,         // 肘部 θ3
    SERVO_WRIST,         // 腕部/爪子转向 θ4
    SERVO_GRIPPER,        // 末端旋转/夹爪 θ5
    SERVO_MAX

} servo_id_t;

typedef struct
{
    float target_angle;
    float current_angle;
}servo_angle_t;

extern servo_angle_t angle[SERVO_MAX];

void servo_angle_init(void);
uint16_t servo_pos_get(float theta);

#endif
