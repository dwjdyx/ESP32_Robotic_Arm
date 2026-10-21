#ifndef _TRAJ_H_
#define _TRAJ_H_

#include "traj_param.h"

//两个平面圆心坐标，根据实际修改
#define TRAJ_XY_CIRCLE_U    0.0f
#define TRAJ_XY_CIRCLE_V    0.0f

#define TRAJ_RZ_CIRCLE_U    40.0f
#define TRAJ_RZ_CIRCLE_V    0.0f

// 行走姿态 抬脚-平移-落地-回到起始点四个阶段时间比例
#define TRAJ_LIFT_RATIO   0.25f
#define TRAJ_MOVE_RATIO   0.25f
#define TRAJ_DOWN_RATIO   0.25f
#define TRAJ_BACK_RATIO   0.25f

void traj_task(void *param);
#endif
