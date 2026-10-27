#ifndef _MANIP_H_
#define _MANIP_H_

#include <stdbool.h>
#include "manip_config.h"
#include "servo.h"

// ============== 定义关节软限位（根据机器蜘蛛实际结构调整）=============
#define THETA1_MIN  -180.0f
#define THETA1_MAX  180.0f

#define THETA2_MIN  -180.0f
#define THETA2_MAX  180.0f

#define THETA3_MIN  -180.0f
#define THETA3_MAX  180.0f

#define THETA4_MIN  -180.0f
#define THETA4_MAX  180.0f

//#define MAX_DELTA_ANGLE 15.0f // 单周期允许的最大角度变化量 (度)
#define THETA_MAX_NUM   4       //解的数量
// ================ 数学参数 =====================
#define PI 3.14159265358979323846f
#define RAD_TO_DEG  (180.0f / PI)   //弧度转角度
#define DEG_TO_RAD  (PI / 180.0f)   //角度转弧度

//================ DH参数 ====================
#define L0  0.0f
#define L1  10.25f
#define L2  105.0f
#define L3  0.0f
#define L4  0.0f
    
#define d1  0.0f
#define d2  21.0f
#define d3  21.0f
#define d4  62.0f
#define d5  107.0f

//======================== 函数 ==============================
bool manip_IK(const pos_t *target_pos, float calc_theta[THETA_MAX_NUM][JOINT_5]);
void manip_FK(pos_t *actual_pos, const float *theta);
bool manip_control(manip_t *manipulator);
void theta_to_angle(theta_t *s_theta, servo_angle_t *s_angle);

#endif
