#include "manip.h"
#include "math.h"
#include "esp_log.h"

static const char *TAG_IK = "IK";
static const char *TAG_FK = "FK";

#define FK_EPSILON 0.5f // 允许最大 0.5mm 的位置偏差   FK验证

//检验IK角度合理性
static bool check_theta_validity(const float *theta) 
{
    if (theta == NULL) return false;

    // ================== 检查是否有任何一个角度变成了 NaN 或 Inf ================
    if (isnan(theta[JOINT_1]) || isnan(theta[JOINT_2]) || isnan(theta[JOINT_3]) || isnan(theta[JOINT_4]) ||
        isinf(theta[JOINT_1]) || isinf(theta[JOINT_2]) || isinf(theta[JOINT_3]) || isinf(theta[JOINT_4])) {
        ESP_LOGE(TAG_IK, "数学运算无效 (NaN 或 Inf)");
        return false;
    }

    // =============== 检查角度是否超出限制范围 ==================
    if (theta[JOINT_1] < THETA1_MIN || theta[JOINT_1] > THETA1_MAX) {
        ESP_LOGW(TAG_IK, "Theta1 超限: %.2f", theta[JOINT_1]);
        return false;
    }
    if (theta[JOINT_2] < THETA2_MIN || theta[JOINT_2] > THETA2_MAX) {
        ESP_LOGW(TAG_IK, "Theta2 超限: %.2f", theta[JOINT_2]);
        return false;
    }
    if (theta[JOINT_3] < THETA3_MIN || theta[JOINT_3] > THETA3_MAX) {
        ESP_LOGW(TAG_IK, "Theta3 超限: %.2f", theta[JOINT_3]);
        return false;
    }
    if (theta[JOINT_4] < THETA4_MIN || theta[JOINT_4] > THETA4_MAX) {
        ESP_LOGW(TAG_IK, "Theta4 超限: %.2f", theta[JOINT_4]);
        return false;
    }

    return true;
}
// // =============== 检查单周期允许的最大角度变化量 ====================
// static bool check_theta_delta(const float *theta,
//                        const theta_t *last_theta)
// {
//     if (theta == NULL || last_theta == NULL)
//         return false;

//     if (fabsf(theta[JOINT_1] - last_theta[JOINT_1].last_joint_theta) > MAX_DELTA_ANGLE ||
//         fabsf(theta[JOINT_2] - last_theta[JOINT_2].last_joint_theta) > MAX_DELTA_ANGLE ||
//         fabsf(theta[JOINT_3] - last_theta[JOINT_3].last_joint_theta) > MAX_DELTA_ANGLE) {

//         ESP_LOGW(TAG_IK, "单周期角度突变过大！");
//         return false;
//     }

//     return true;
// }

/**
 * @brief  腿部正运动学 (FK) 验证
 * @param  actual_pos 计算得到的实际末端坐标
 * @param  theta  当前输入的关节角度
 * @return
 */
void manip_FK(pos_t *actual_pos, const float *theta)
{
    if (actual_pos == NULL || theta == NULL) return;

    float t1 = theta[JOINT_1] * DEG_TO_RAD;      // 角度转弧度
    float t2 = theta[JOINT_2] * DEG_TO_RAD;
    float t3 = theta[JOINT_3] * DEG_TO_RAD;
    float t23 = t2 - t3;

    float c1 = cosf(t1);
    float s1 = sinf(t1);
    float c2 = cosf(t2);
    float s2 = sinf(t2);
    float c23 = cosf(t23);
    float s23 = sinf(t23);

    // 计算公式根据实际变换矩阵修改
    actual_pos->x = L1*c1 + L2*c1*c2 - (d2 - d3)*s1 + (d4 + d5)*c1*s23;
    actual_pos->y = L1*s1 + L2*s1*c2 + (d2 - d3)*c1 + (d4 + d5)*s1*s23;
    actual_pos->z = -L2*s2 + (d4 + d5)*c23;
}

// FK 结果精度检验
static bool check_fk_precision(const pos_t *target_pos, const float *theta) {
    if (target_pos == NULL || theta == NULL) return false;

    pos_t calc_pos;
    
    // 用 IK 解出的角度反算 FK 得到位置
    manip_FK(&calc_pos, theta);
    
    // 计算欧式距离平方: (dx^2 + dy^2 + dz^2)
    float dx = target_pos->x - calc_pos.x;
    float dy = target_pos->y - calc_pos.y;
    float dz = target_pos->z - calc_pos.z;
    float error_sq = dx * dx + dy * dy + dz * dz;
    
    // 偏差超过阈值平方
    if (error_sq > (FK_EPSILON * FK_EPSILON)) {
        ESP_LOGE(TAG_FK, "FK 验证偏差超过阀值: error_sq=%.2f", error_sq);
        return false;
    }
    
    return true;
}

/**
 * @brief  腿部逆运动学 (IK)
 * @param  target_pos       目标脚尖坐标 (x, y, z)
 * @param  calc_theta       输出关节角度
 * @return bool             解算成功返回 true，超工作空间返回 false
 */
bool manip_IK(const pos_t *target_pos, float calc_theta[THETA_MAX_NUM][JOINT_5])
{
    if (target_pos == NULL || calc_theta == NULL) return false;
    float theta[THETA_MAX_NUM][JOINT_4] = {
        {0.0f,0.0f,0.0f},
        {0.0f,0.0f,0.0f},
        {0.0f,0.0f,0.0f},
        {0.0f,0.0f,0.0f}
    };

    float x = target_pos->x;
    float y = target_pos->y;
    float z = target_pos->z;
    float xy_sq = x * x + y * y;
    
    if (!isfinite(x) || !isfinite(y) || !isfinite(z))
    {
        ESP_LOGE(TAG_IK, "目标坐标包含无效数值");
        return false;
    }

    float h = d2 - d3;
    float r = xy_sq - h*h;
    if(r < 0)
    {
        ESP_LOGE(TAG_IK, "目标点过近，根号内为负数");
        return false; 
    }
    float R_1 = sqrtf(r);
    float R_2 = -sqrtf(r);
    float L = d4 + d5;
    float P_1 = R_1 - L1;
    float P_2 = R_2 - L1;
    float alpha = atan2f(L,L3);
    float C_1 = (P_1*P_1 + z*z -L2*L2 - L*L)/(2.0f*L2*L);
    float C_2 = (P_2*P_2 + z*z -L2*L2 - L*L)/(2.0f*L2*L);
    if (C_1 < -1.0f || C_1 > 1.0f)
        ESP_LOGW(TAG_IK, "C_1 超出 acosf 定义域: %.6f", C_1);

    if (C_2 < -1.0f || C_2 > 1.0f)
        ESP_LOGW(TAG_IK, "C_2 超出 acosf 定义域: %.6f", C_2);
// ===================== 以下公式根据实际变换矩阵修改 ======================
    // 第一关节
    float t1_1 = atan2f(y, x) - atan2f(h, R_1);
    float t1_2 = atan2f(y, x) - atan2f(h, R_2);
    theta[1][JOINT_1] = theta[0][JOINT_1] = t1_1 * RAD_TO_DEG;
    theta[3][JOINT_1] = theta[2][JOINT_1] = t1_2 * RAD_TO_DEG;

    float t3_1_1 = acosf(C_1) - alpha;
    float t3_1_2 = -acosf(C_1) - alpha;
    float t3_2_1 = acosf(C_2) - alpha;
    float t3_2_2 = -acosf(C_2) - alpha;
    theta[0][JOINT_3] = t3_1_1 * RAD_TO_DEG;
    theta[1][JOINT_3] = t3_1_2 * RAD_TO_DEG;
    theta[2][JOINT_3] = t3_2_1 * RAD_TO_DEG;
    theta[3][JOINT_3] = t3_2_2 * RAD_TO_DEG;

    // 第二关节
    float t2_1_1 = atan2f(-z,P_1) + atan2f(L*sin(t3_1_1 + alpha), L2 + L*cos(t3_1_1 + alpha));
    float t2_1_2 = atan2f(-z,P_1) + atan2f(L*sin(t3_1_2 + alpha), L2 + L*cos(t3_1_2 + alpha));
    float t2_2_1 = atan2f(-z,P_2) + atan2f(L*sin(t3_2_1 + alpha), L2 + L*cos(t3_2_1 + alpha));
    float t2_2_2 = atan2f(-z,P_2) + atan2f(L*sin(t3_2_2 + alpha), L2 + L*cos(t3_2_2 + alpha));
    theta[0][JOINT_2] = t2_1_1 * RAD_TO_DEG;
    theta[1][JOINT_2] = t2_1_2 * RAD_TO_DEG;
    theta[2][JOINT_2] = t2_2_1 * RAD_TO_DEG;
    theta[3][JOINT_2] = t2_2_2 * RAD_TO_DEG;

    for (int i = 0; i < 4; i++)
    {
        for(joint_id_t j = JOINT_1; j<=JOINT_3; j++)
        {
            calc_theta[i][j] = theta[i][j];
        }
    }
    
    calc_theta[0][JOINT_4] = -90.0f;
    calc_theta[1][JOINT_4] = -90.0f;
    calc_theta[2][JOINT_4] = -90.0f;
    calc_theta[3][JOINT_4] = -90.0f;

    return true;
}

// 计算误差
int8_t mult_solutions_selection(float calc_theta[THETA_MAX_NUM][JOINT_5], 
                                const theta_t *g_theta, 
                                const bool flag[THETA_MAX_NUM])
{
    // ==================== 多解选择 ====================
    int8_t min_index = -1;
    float min_errors = INFINITY;
    for (int j = 0; j < 4; j++)
    {
        if (!flag[j])
        {
            continue;
        }
        float errors = 0.0f;
        for (joint_id_t i = JOINT_1; i <= JOINT_4; i++)
        {
            float current = g_theta[i].last_joint_theta;

            float diff = calc_theta[j][i] - current;

            errors += diff * diff;
        }
        if(errors < min_errors)
        {
            min_errors = errors;
            min_index = j;
        }
    }
    return min_index;
}

bool manip_control(manip_t *manipulator) 
{
    if (manipulator == NULL) return false;
    bool check_flag[THETA_MAX_NUM] = {true,true,true,true};

    // 使用局部临时变量接收 IK 解算结果（防止脏数据写入）
    float calc_theta[THETA_MAX_NUM][JOINT_5] = {{0.0f,0.0f,0.0f,0.0f},
                                    {0.0f,0.0f,0.0f,0.0f},
                                    {0.0f,0.0f,0.0f,0.0f},
                                    {0.0f,0.0f,0.0f,0.0f}};

    // 求解 IK
    if (!manip_IK(&manipulator->target_pos, calc_theta)) {
        ESP_LOGW("IK", "目标点超出工作空间，拒绝执行！");
        return false;
    }
    for (int i = 0; i < THETA_MAX_NUM; i++)
    {
        // 检查基础角度合法性
        if (!check_theta_validity(calc_theta[i])) {
            ESP_LOGW("TASK", "候选解[%d]IK 角度非法",i);
            check_flag[i] = false;
            continue;
        }

        // 检查FK精度
        if (!check_fk_precision(&manipulator->target_pos, calc_theta[i])) {
            ESP_LOGW("TASK", "候选解[%d]FK 精度校验不通过",i);
            check_flag[i] = false;
            continue;
        }
    }

    int8_t index = mult_solutions_selection(calc_theta, manipulator->joint_theta, &check_flag);
    if(index == -1)
    {
        ESP_LOGW("TASK", "全部候选解IK 角度非法");
        return false;
    }
    // 校验全部通过，才真正更新全局结构体的数据
    for(joint_id_t i = JOINT_1; i < JOINT_5; i++)
    {
        manipulator->joint_theta[i].cur_joint_theta = calc_theta[index][i];
    }
    
    return true;
}

//关节角转换舵机角度
void theta_to_angle(theta_t *s_theta, servo_angle_t *s_angle)
{
    if(s_angle == NULL || s_theta == NULL) return;
    const joint_config_t *cfg = &joint_config;

    for (joint_id_t i = JOINT_1; i < JOINT_MAX_NUM; i++)
    {

        float angle;
        angle = cfg[i].offset + cfg[i].dir * s_theta[i].cur_joint_theta;
        if (angle >= 360.0f)
        {
            angle -=360.0f;
        }
        
        //角度限位
        angle = CLAMP(angle,ANGLE_MIN,ANGLE_MAX);
        s_angle[i].target_angle = angle;
    }
    
}
