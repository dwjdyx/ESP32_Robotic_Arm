#ifndef _MANIP_CONFIG_H_
#define _MANIP_CONFIG_H_

#include "stdbool.h"

typedef enum
{
    JOINT_1 = 0,
    JOINT_2,
    JOINT_3,
    JOINT_4,
    JOINT_5,
    JOINT_MAX_NUM

} joint_id_t;

typedef struct
{
    float x;
    float y;
    float z;
}pos_t;

typedef struct
{
    float last_joint_theta;     // 上一次关节角度
    float cur_joint_theta;          // 关节角度
}theta_t;     // 角度

typedef struct 
{
    pos_t target_pos;                   // 目标位置
    theta_t joint_theta[JOINT_MAX_NUM];       // 各关节角度
}manip_t;

typedef struct
{
    float offset;       // 舵机零位偏移
    float dir;          // 舵机方向系数

} joint_config_t;         //舵机映射配置参数

extern const joint_config_t joint_config[JOINT_MAX_NUM];
extern manip_t manipulator;

#endif
