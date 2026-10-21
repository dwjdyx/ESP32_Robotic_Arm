#include "servo.h"
#include "manip.h"
#include "math.h"
#include "esp_log.h"

static const char *TAG_IK = "IK";
static const char *TAG_FK = "FK";

#define FK_EPSILON 0.5f // 允许最大 0.5mm 的位置偏差   FK验证

//检验坐标合理性
bool check_pos_validity(const pos_t *pos)
{
    float R = sqrtf(pos->x * pos->x + pos->y * pos->y);
    // 以下公式根据实际修改
    float temp = L2 * L2 - (R - L1) * (R - L1);
    
    if (temp < 0.0f)
    {
        return false;
    }

    float h = sqrtf(temp);
    const float EPSILON = 0.15f;

    if (fabsf(fabsf(pos->z) - h) > EPSILON)
    {
        return false;
    }

    return true;
}

//检验IK角度合理性
static bool check_angle_validity(const float *angle) 
{
    if (angle == NULL) return false;

    // ================== 检查是否有任何一个角度变成了 NaN 或 Inf ================
    if (isnan(angle[JOINT_1]) || isnan(angle[JOINT_2]) || isnan(angle[JOINT_3]) ||
        isinf(angle[JOINT_1]) || isinf(angle[JOINT_2]) || isinf(angle[JOINT_3])) {
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
    if (angle[JOINT_3] < THETA2_MIN || angle[JOINT_3] > THETA2_MAX) {
        ESP_LOGW(TAG_IK, "Theta3 超限: %.2f", angle[JOINT_3]);
        return false;
    }

    return true;
}
// =============== 检查单周期允许的最大角度变化量 ====================
static bool check_angle_delta(const float *angle,
                       const angle_t *last_angle)
{
    if (angle == NULL || last_angle == NULL)
        return false;

    if (fabsf(angle[JOINT_1] - last_angle[JOINT_1].last_joint_angle) > MAX_DELTA_ANGLE ||
        fabsf(angle[JOINT_2] - last_angle[JOINT_2].last_joint_angle) > MAX_DELTA_ANGLE ||
        fabsf(angle[JOINT_3] - last_angle[JOINT_3].last_joint_angle) > MAX_DELTA_ANGLE) {

        ESP_LOGW(TAG_IK, "单周期角度突变过大！");
        return false;
    }

    return true;
}

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
    float c3 = cosf(t3);
    float s3 = sinf(t3);
    float c23 = cosf(t23);
    float s23 = sinf(t23);

    // 计算公式根据实际变换矩阵修改
    actual_pos->x = -8*sqrtf(2)*s1 + (d4*s23 + L2*c2 - 8*sqrtf(2)*c23 + L1)*c1;
    actual_pos->y = (d4*s23 + L2*c2 - 8*sqrtf(2)*c23 + L1)*s1 + 8*sqrtf(2)*c1;
    actual_pos->z = -L2*s2 + 8*sqrtf(2)*s23 + d4*c23;
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
bool manip_IK(const pos_t *target_pos, float *angle)
{
    if (target_pos == NULL || angle == NULL) return false;

    float x = target_pos->x;
    float y = target_pos->y;
    float z = target_pos->z;
    float xy_sq = x * x + y * y;
    
    float D = xy_sq - L4*L4/2;
    // 工作空间安全校验：根号内不能为负数
    if(D < 0)
    {
        ESP_LOGE(TAG_IK, "目标点过近，根号内为负数");
        return false; 
    }

    float A = sqrtf(D);
    float Q = A - L1;
    float L = sqrtf(Q*Q + z*z);
    float C = sqrtf(d4*d4 + L4*L4/2);

    float B = L4/sqrtf(2);
// ===================== 以下公式根据实际变换矩阵修改 ======================
    // 第一关节
    float t1 = atan2f(y, x) - atan2f(B, A);
    angle[JOINT_1] = t1 * RAD_TO_DEG;

    // 第二关节
    float cos_t2 = (L*L + L2*L2 - C*C) / (2.0f*L2*L);
    if (cos_t2 < -1.0f - 1e-6f || cos_t2 >  1.0f + 1e-6f)
    {
        ESP_LOGE(TAG_IK, "目标点超出工作空间");
        return false;
    }
    cos_t2 = fmaxf(-1.0f, fminf(1.0f, cos_t2));

    float t2 = -atan2f(z,Q) + acosf(cos_t2);
    angle[JOINT_2] = t2 * RAD_TO_DEG;

    //第三关节
    float t3 = t2 - atan2f(Q - L2*cos(t2),z + L2*sin(t2)) - atan2f(B, d4);
    angle[JOINT_3] = t3 * RAD_TO_DEG;

    //第四关节固定135.0f
    angle[JOINT_4] = 135.0f;

    return true;
}

//腿部控制任务
bool manip_control_task(manip_t *manipulator) 
{
    if (manipulator == NULL) return false;

    // 使用局部临时变量接收 IK 解算结果（防止脏数据写入）
    float calc_angle[JOINT_MAX_NUM] = {0.0f,0.0f,0.0f,0.0f,0.0f};

    //检验坐标合理性
    if(!check_pos_validity(&manipulator->target_pos))
    {
        ESP_LOGW("TASK", "目标点不合理，拒绝执行！");
        return false;
    }

    // 求解 IK
    if (!manip_IK(&manipulator->target_pos, &calc_angle)) {
        ESP_LOGW("IK", "目标点超出工作空间，拒绝执行！");
        return false;
    }

    // 检查基础角度合法性
    if (!check_angle_validity(&calc_angle)) {
        ESP_LOGW("TASK", "IK 角度非法，阻断输出！");
        return false;
    }

    // 第一次不检查角度突变
    if (manipulator->initialized)
    {
        if (!check_angle_delta(&calc_angle,
                               &manipulator->angle)) {

            ESP_LOGW("TASK", "IK 角度突变过大，阻断输出！");
            return false;
        }
    }
    // 检查FK精度
    if (!check_fk_precision(&manipulator->target_pos, &calc_angle)) {
        ESP_LOGW("TASK", "FK 精度校验不通过，阻断输出！");
        return false;
    }
    
    // 校验全部通过，才真正更新全局结构体的数据
    for(joint_id_t i = JOINT_1; i < JOINT_MAX_NUM; i++)
    {
        manipulator->angle[i].joint_angle = calc_angle[i];
        // 更新 last_joint_angle 供下一周期比对，并输出 PWM
        manipulator->angle[i].last_joint_angle = manipulator->angle[i].joint_angle;
    }

    manipulator->initialized = true;
    
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

    // 角度范围限制
    if(*servo_angle < 0) *servo_angle = 0;
    else if(*servo_angle > 270) *servo_angle = 270;
    
}
