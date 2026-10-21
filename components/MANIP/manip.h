#ifndef _MANIP_H_
#define _MANIP_H_

#include <stdbool.h>
#include "manip_str.h"

// ============== 定义关节软限位（根据机器蜘蛛实际结构调整）=============
#define THETA1_MIN  -180.0f
#define THETA1_MAX  180.0f

#define THETA2_MIN  -180.0f
#define THETA2_MAX  180.0f

#define THETA3_MIN  -180.0f
#define THETA3_MAX  180.0f

#define MAX_DELTA_ANGLE 15.0f // 单周期允许的最大角度变化量 (度)

// ================ 数学参数 =====================
#define PI 3.14159265358979323846f
#define RAD_TO_DEG  (180.0f / PI)   //弧度转角度
#define DEG_TO_RAD  (PI / 180.0f)   //角度转弧度

//================ DH参数 ====================
#define L0 0.0f
#define L1 10.25f
#define L2 104.0f
#define L3 0.0f
#define L4 16.0f

#define d1 0.0f
#define d2 21.0f
#define d3 21.0f
#define d4 94.0f
#define d5 0.0f

//======================== 函数 ==============================
bool check_pos_validity(const pos_t *pos);
bool manip_IK(const pos_t *target_pos, float *angle);
void manip_FK(pos_t *actual_pos, const float *angle);
bool manip_control_task(manip_t *manipulator);
void theta_to_angle(const float joint_angle, joint_id_t joint_id, float *servo_angle);

#endif
