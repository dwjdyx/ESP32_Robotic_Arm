#ifndef _TRAJ_CFG_H_
#define _TRAJ_CFG_H_

#include "manip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAX_WAYPOINT 4      // 最大轨迹点数量

typedef enum
{
    TRAJ_PHASE_1,
    TRAJ_PHASE_2,
    TRAJ_PHASE_3,
    TRAJ_PHASE_4,

    TRAJ_PHASE_MAX
} traj_phase_t;             // 每个动作固定四个阶段

typedef struct
{
    float duration;         // 整个轨迹持续时间，单位：ms
    float elapsed;          // 当前已经经过的时间，单位：ms
    TickType_t  last_tick;  // 上一次更新时间

} traj_time_t;

typedef struct
{
    float theta_start;       // 起始坐标极角
    float theta_end;         // 终止坐标极角
    float dtheta;            // 极角变化量

    float theta_xy;         // knee舵机移动xy形成的角度
    float radius_xy;        // hip舵机移动xy形成的半径

}traj_calc_t;

typedef struct
{
    traj_phase_t cur_phase;       //当前阶段
    traj_calc_t calc[TRAJ_PHASE_MAX];
    // 是否执行标志位 对应每个阶段
    bool initialized[TRAJ_PHASE_MAX];
} traj_phase_param_t;     // 动态阶段

typedef struct 
{
    pos_t point[MAX_WAYPOINT];    // 过程点坐标集
    traj_time_t time;
    traj_phase_param_t phase_param;
    bool running;           // 是否正在运行

}gait_traj_t;              //动态轨迹

extern gait_traj_t g_gait_traj;


#endif
