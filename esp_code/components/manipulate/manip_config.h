#ifndef _MANIP_CONFIG_H_
#define _MANIP_CONFIG_H_

#include "math.h"
 
// 把 x 的值限制在 [min, max] 之间
#define CLAMP(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

// 根据实际修改
#define ANGLE_MAX           255.0f      // 角度限位
#define ANGLE_MIN           15.0f
#define SERVO_INIT_ANGLE    135.0f      // 舵机初始角度
#define SERVO2_INIT_ANGLE   141.0f      // 舵机2初始角度

typedef enum
{
    JOINT_1 = 0,
    JOINT_2,
    JOINT_3,
    JOINT_4,
    JOINT_5,
    JOINT_6,
    JOINT_MAX_NUM

} joint_id_t;

typedef struct
{
    float target_angle[JOINT_MAX_NUM];
    float initial_angle[JOINT_MAX_NUM];
}servo_angle_t;

typedef struct
{
    float x;
    float y;
    float z;
}pos_t;

typedef struct 
{
    pos_t target_pos;                   // 目标位置
    float last_joint_theta[JOINT_MAX_NUM];       // 各关节角度
    float cur_joint_theta[JOINT_MAX_NUM];
}manip_t;

typedef struct
{
    float offset;       // 舵机零位偏移
    float dir;          // 舵机方向系数

} joint_config_t;         //舵机映射配置参数

extern const joint_config_t joint_config[JOINT_MAX_NUM];
extern manip_t manipulator;

#endif
