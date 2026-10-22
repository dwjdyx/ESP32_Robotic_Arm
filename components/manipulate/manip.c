#include "manip.h"
#include "math.h"
#include "esp_log.h"

static const char *TAG_IK = "IK";
static const char *TAG_FK = "FK";

#define FK_EPSILON 0.5f // 允许最大 0.5mm 的位置偏差   FK验证

//检验IK角度合理性
static bool check_angle_validity(const float *angle) 
{
    if (angle == NULL) return false;

    // ================== 检查是否有任何一个角度变成了 NaN 或 Inf ================
    if (isnan(angle[JOINT_1]) || isnan(angle[JOINT_2]) || isnan(angle[JOINT_3]) || isnan(angle[JOINT_4]) ||
        isinf(angle[JOINT_1]) || isinf(angle[JOINT_2]) || isinf(angle[JOINT_3]) || isinf(angle[JOINT_4])) {
        ESP_LOGE(TAG_IK, "数学运算无效 (NaN 或 Inf)");
        return false;
    }

    // =============== 检查角度是否超出限制范围 ==================
    if (angle[JOINT_1] < THETA1_MIN || angle[JOINT_1] > THETA1_MAX) {
        ESP_LOGW(TAG_IK, "Theta1 超限: %.2f", angle[JOINT_1]);
        return false;
    }
    if (angle[JOINT_2] < THETA2_MIN || angle[JOINT_2] > THETA2_MAX) {
        ESP_LOGW(TAG_IK, "Theta2 超限: %.2f", angle[JOINT_2]);
        return false;
    }
    if (angle[JOINT_3] < THETA3_MIN || angle[JOINT_3] > THETA3_MAX) {
        ESP_LOGW(TAG_IK, "Theta3 超限: %.2f", angle[JOINT_3]);
        return false;
    }
    if (angle[JOINT_4] < THETA4_MIN || angle[JOINT_4] > THETA4_MAX) {
        ESP_LOGW(TAG_IK, "Theta4 超限: %.2f", angle[JOINT_4]);
        return false;
    }

    return true;
}
// // =============== 检查单周期允许的最大角度变化量 ====================
// static bool check_angle_delta(const float *angle,
//                        const angle_t *last_angle)
// {
//     if (angle == NULL || last_angle == NULL)
//         return false;

//     if (fabsf(angle[JOINT_1] - last_angle[JOINT_1].last_joint_angle) > MAX_DELTA_ANGLE ||
//         fabsf(angle[JOINT_2] - last_angle[JOINT_2].last_joint_angle) > MAX_DELTA_ANGLE ||
//         fabsf(angle[JOINT_3] - last_angle[JOINT_3].last_joint_angle) > MAX_DELTA_ANGLE) {

//         ESP_LOGW(TAG_IK, "单周期角度突变过大！");
//         return false;
//     }

//     return true;
// }

/**
 * @brief  腿部正运动学 (FK) 验证
 * @param  actual_pos 计算得到的实际末端坐标
 * @param  angle  当前输入的关节角度
 * @return
 */
void manip_FK(pos_t *actual_pos, const float *angle)
{
    if (actual_pos == NULL || angle == NULL) return;

    float t1 = angle[JOINT_1] * DEG_TO_RAD;      // 角度转弧度
    float t2 = angle[JOINT_2] * DEG_TO_RAD;
    float t3 = angle[JOINT_3] * DEG_TO_RAD;
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
static bool check_fk_precision(const pos_t *target_pos, const float *angle) {
    if (target_pos == NULL || angle == NULL) return false;

    pos_t calc_pos;
    
    // 用 IK 解出的角度反算 FK 得到位置
    manip_FK(&calc_pos, angle);
    
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
 * @param  target_pos   目标脚尖坐标 (x, y, z)
 * @param  angle        输出关节角度
 * @return bool         解算成功返回 true，超工作空间返回 false
 */
bool manip_IK(const pos_t *target_pos, float *fir_angle, float *sec_angle)
{
    if (target_pos == NULL || fir_angle == NULL || sec_angle == NULL) return false;

    float angle_1[JOINT_4];
    float angle_2[JOINT_4];

    float x = target_pos->x;
    float y = target_pos->y;
    float z = target_pos->z;
    float xy_sq = x * x + y * y;
    
    float h = d2 - d3;
    float r = xy_sq - h*h;
    // 工作空间安全校验：根号内不能为负数
    if(r < 0)
    {
        ESP_LOGE(TAG_IK, "目标点过近，根号内为负数");
        return false; 
    }
    float R = sqrtf(r);
    float L = d4 + d5;
    float P = R - L1;
// ===================== 以下公式根据实际变换矩阵修改 ======================
    // 第一关节(因为当前h=0,后半部分为0，因此不考虑+情况)
    float t1 = atan2f(y, x) - atan2f(h, R);
    angle_1[JOINT_1] = angle_2[JOINT_1] =  t1 * RAD_TO_DEG;

    //第三关节(含两种解)
    float K = (L2 * L2 + L * L - P * P - z * z)
          / (2.0f * L2 * L);

    if (K < -1.0f - 1e-6f || K > 1.0f + 1e-6f) {
        ESP_LOGE(TAG_IK, "目标点超出工作空间");
        return false;
    }

    K = fmaxf(-1.0f, fminf(1.0f, K));

    float t3_1 = asinf(K);
    float t3_2 = PI - t3_1;
    angle_1[JOINT_3] = t3_1 * RAD_TO_DEG;
    angle_2[JOINT_3] = t3_2 * RAD_TO_DEG;

    // 第二关节
    float t2_1 = atan2f(L*cosf(t3_1),L2-L*sinf(t3_1)) - atan2f(z,P);
    float t2_2 = atan2f(L*cosf(t3_2),L2-L*sinf(t3_2)) - atan2f(z,P);
    angle_1[JOINT_2] = t2_1 * RAD_TO_DEG;
    angle_2[JOINT_2] = t2_2 * RAD_TO_DEG;

    for(joint_id_t i = JOINT_1; i<=JOINT_3; i++)
    {
        fir_angle[i] = angle_1[i];
        sec_angle[i] = angle_2[i];
    }

    //第四关节固定135.0f
    fir_angle[JOINT_4] = sec_angle[JOINT_4] = 135.0f;
    
    return true;
}

uint8_t mult_solutions_selection(float *angle1, float *angle2, const angle_t *g_angle)
{
    // ==================== 多解选择 ====================
    float sum0 = 0.0f;
    float sum1 = 0.0f;

    for (joint_id_t i = JOINT_1; i <= JOINT_4; i++)
    {
        float current = g_angle[i].last_joint_angle;

        float diff0 = angle1[i] - current;
        float diff1 = angle2[i] - current;

        sum0 += diff0 * diff0;
        sum1 += diff1 * diff1;
    }

    // 选择角度变化更小的候选解
    uint8_t flag = (sum0 <= sum1) ? 0 : 1;
    return flag;
}

bool manip_control_task(manip_t *manipulator) 
{
    if (manipulator == NULL) return false;
    uint8_t check_flag = 2;

    // 使用局部临时变量接收 IK 解算结果（防止脏数据写入）
    float calc_angle[2][JOINT_5] = {{0.0f,0.0f,0.0f,0.0f},
                                    {0.0f,0.0f,0.0f,0.0f}};

    // 求解 IK
    if (!manip_IK(&manipulator->target_pos, calc_angle[0], calc_angle[1])) {
        ESP_LOGW("IK", "目标点超出工作空间，拒绝执行！");
        return false;
    }
    for (int i = 0; i < 2; i++)
    {
        // 检查基础角度合法性
        if (!check_angle_validity(calc_angle[i])) {
            ESP_LOGW("TASK", "候选解[%d]IK 角度非法，阻断输出！",i);
            check_flag = i;
            continue;
        }

        // 检查FK精度
        if (!check_fk_precision(&manipulator->target_pos, calc_angle[i])) {
            ESP_LOGW("TASK", "候选解[%d]FK 精度校验不通过，阻断输出！",i);
            check_flag = i;
            continue;
        }
    }
    if (check_flag == 2)
    {
        uint8_t flag = mult_solutions_selection(calc_angle[0], calc_angle[1], manipulator->angle);
        // 校验全部通过，才真正更新全局结构体的数据
        for(joint_id_t i = JOINT_1; i < JOINT_5; i++)
        {
            manipulator->angle[i].joint_angle = calc_angle[flag][i];
        }
    }else if (check_flag == 0)
    {
        // 校验全部通过，才真正更新全局结构体的数据
        for(joint_id_t i = JOINT_1; i < JOINT_5; i++)
        {
            manipulator->angle[i].joint_angle = calc_angle[1][i];
        }
    }else if (check_flag == 1)
    {
        // 校验全部通过，才真正更新全局结构体的数据
        for(joint_id_t i = JOINT_1; i < JOINT_5; i++)
        {
            manipulator->angle[i].joint_angle = calc_angle[0][i];
        }
    }
    
    return true;
}

//关节角转换舵机角度
void theta_to_angle(const float joint_angle, joint_id_t joint_id, float *servo_angle)
{
    if (joint_id >= JOINT_MAX_NUM)
        return;

    const joint_config_t *cfg = &joint_config[joint_id];

// ==================== 以下转换公式根据实际修改 ======================
    *servo_angle = cfg->offset + cfg->dir * joint_angle;

    //角度限位
    *servo_angle = CLAMP(*servo_angle,ANGLE_MIN,ANGLE_MAX);
    
}
