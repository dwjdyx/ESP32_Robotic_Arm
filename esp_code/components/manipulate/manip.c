#include "manip.h"
#include "math.h"
#include "esp_log.h"

static const char *TAG_IK = "IK";
static const char *TAG_FK = "FK";

#define FK_EPSILON 0.5f // 允许最大 0.5mm 的位置偏差   FK验证
#define IK_EPSILON 1e-5f

//检验IK角度合理性
static bool check_theta_validity(const float *cur_theta) 
{
    if (cur_theta == NULL) return false;

    // ================== 检查是否有任何一个角度变成了 NaN 或 Inf ================
    if (!isfinite(cur_theta[JOINT_1]) ||
        !isfinite(cur_theta[JOINT_2]) ||
        !isfinite(cur_theta[JOINT_3]) ||
        !isfinite(cur_theta[JOINT_4])) {

        ESP_LOGE(TAG_IK, "数学运算无效 (NaN 或 Inf)");
        return false;
    }

    // // =============== 检查角度是否超出限制范围 ==================
    // if (cur_theta[JOINT_1] < THETA1_MIN || cur_theta[JOINT_1] > THETA1_MAX) {
    //     ESP_LOGW(TAG_IK, "Theta1 超限: %.2f", cur_theta[JOINT_1]);
    //     return false;
    // }
    // if (cur_theta[JOINT_2] < THETA2_MIN || cur_theta[JOINT_2] > THETA2_MAX) {
    //     ESP_LOGW(TAG_IK, "Theta2 超限: %.2f", cur_theta[JOINT_2]);
    //     return false;
    // }
    // if (cur_theta[JOINT_3] < THETA3_MIN || cur_theta[JOINT_3] > THETA3_MAX) {
    //     ESP_LOGW(TAG_IK, "Theta3 超限: %.2f", cur_theta[JOINT_3]);
    //     return false;
    // }
    // if (cur_theta[JOINT_4] < THETA4_MIN || cur_theta[JOINT_4] > THETA4_MAX) {
    //     ESP_LOGW(TAG_IK, "Theta4 超限: %.2f", cur_theta[JOINT_4]);
    //     return false;
    // }

    return true;
}

/**
 * @brief  腿部正运动学 (FK) 验证
 * @param  actual_pos 计算得到的实际末端坐标
 * @param  cur_theta  当前输入的关节角度
 * @return
 */
void manip_FK(pos_t *actual_pos, const float *cur_theta)
{
    if (actual_pos == NULL || cur_theta == NULL) return;

    float t1 = cur_theta[JOINT_1];
    float t2 = cur_theta[JOINT_2];
    float t3 = cur_theta[JOINT_3];
    float t4 = cur_theta[JOINT_4];
    float t23 = t2 - t3;
    float t234 = t23 - t4;

    float c1 = cosf(t1);
    float s1 = sinf(t1);
    float c2 = cosf(t2);
    float s2 = sinf(t2);
    float s23 = sinf(t23);
    float c23 = cosf(t23);
    float s234 = sinf(t234);
    float c234 = cosf(t234);

    // 计算公式根据实际变换矩阵修改
    actual_pos->x = c1*(L1 + L2*c2 + L3*c23 + (d5+d6)*s234);
    actual_pos->y = s1*(L1 + L2*c2 + L3*c23 + (d5+d6)*s234);
    actual_pos->z = -L2*s2 - L3*s23 + (d5+d6)*c234;
}

// FK 结果精度检验
static bool check_fk_precision(const pos_t *target_pos, const float *cur_theta) 
{
    if (target_pos == NULL || cur_theta == NULL) return false;

    pos_t calc_pos;
    
    // 用 IK 解出的角度反算 FK 得到位置
    manip_FK(&calc_pos, cur_theta);
    
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

// 计算当前解误差
float mult_solutions_selection(const float calc_theta[JOINT_5], 
                                const float *last_theta)
{
    float errors = 0.0f;
    // ==================== 多解选择 ====================
    for (joint_id_t i = JOINT_1; i <= JOINT_4; i++)
    {
        float diff = calc_theta[i] * RAD_TO_DEG - last_theta[i];

        errors += diff * diff;
    }
    return errors;
}

/**
 * @brief  腿部逆运动学 (IK)
 * @param  target_pos       目标脚尖坐标 (x, y, z)
 * @param  calc_theta       输出关节角度
 * @return bool             解算成功返回 true，超工作空间返回 false
 */
bool manip_IK(const pos_t *target_pos, const float *last_theta, float beta, 
            float temp_theta[4], float *min_error)
{
    if(target_pos == NULL || last_theta == NULL || min_error == NULL) return false;
    float x = target_pos->x;
    float y = target_pos->y;
    float z = target_pos->z;

    float P = sqrtf(x*x + y*y);

    float X[2] = {
        (P - L1 - (d5+d6)*sinf(beta)),
        (-P - L1 - (d5+d6)*sinf(beta))
    };
    float Y = -z + (d5+d6)*cosf(beta);
    float D[2] = {
        (X[0]*X[0] + Y*Y),
        (X[1]*X[1] + Y*Y),
    };

    // float l1 = (L3-L4);
    // float l2 = (L3+L4);
    // if(!((l1*l1 <=D[0] && D[0]<= l2*l2)
    //     || (l1*l1 <=D[1] && D[1]<= l2*l2)))
    // {
    //     ESP_LOGW("IK","D计算错误");
    //     return false;
    // }

    float Q[2] = {
        (D[0] - L2*L2 - L3*L3) / (2*L2*L3),
        (D[1] - L2*L2 - L3*L3) / (2*L2*L3),
    };
    bool q_valid[2] = {
        Q[0] >= -1.0f - IK_EPSILON && Q[0] <= 1.0f + IK_EPSILON,
        Q[1] >= -1.0f - IK_EPSILON && Q[1] <= 1.0f + IK_EPSILON
    };

    if (!q_valid[0] && !q_valid[1]) {
        ESP_LOGW("IK","Q越界");
        return false;
    }

    Q[0] = CLAMP(Q[0], -1.0f, 1.0f);
    Q[1] = CLAMP(Q[1], -1.0f, 1.0f);

    float t1[2] = {
        atan2f(y,x),
        atan2f(y,x) + PI
    };
    float t3[4] = {
        acosf(Q[0]),
        acosf(Q[1]),
        -acosf(Q[0]),
        -acosf(Q[1]),
    };
    float t2[4] = 
    {
        atan2f(Y,X[0]) - atan2f(-L3*sinf(t3[0]), L2+L3*cosf(t3[0])),
        atan2f(Y,X[1]) - atan2f(-L3*sinf(t3[1]), L2+L3*cosf(t3[1])),
        atan2f(Y,X[0]) - atan2f(-L3*sinf(t3[2]), L2+L3*cosf(t3[2])),
        atan2f(Y,X[1]) - atan2f(-L3*sinf(t3[3]), L2+L3*cosf(t3[3])),
    };
    float t4[4] = 
    {
        t2[0] - t3[0] - beta,
        t2[1] - t3[1] - beta,
        t2[2] - t3[2] - beta,
        t2[3] - t3[3] - beta,
    };

    float theta_set[4][JOINT_5] = {
        {t1[0],t2[0],t3[0],t4[0]},
        {t1[1],t2[1],t3[1],t4[1]},
        {t1[0],t2[2],t3[2],t4[2]},
        {t1[1],t2[3],t3[3],t4[3]},
    };
    float temp_min_errors = INFINITY;   // 当前解集最小偏差
    int8_t min_index = -1;              // 最小偏差索引
    // 寻找当前最小偏差解
    for(int i =0; i<4; i++)
    {
        // 0、2 对应第一支；1、3 对应第二支
        if (!q_valid[i % 2]) {
            continue;
        }
        // IK检验
        if(check_theta_validity(theta_set[i]))
        {
            // 检查FK精度
            if (check_fk_precision(target_pos, theta_set[i])) {
                // 计算偏差
                float error = mult_solutions_selection(theta_set[i],last_theta);
                if(error < temp_min_errors)
                {
                    temp_min_errors = error;
                    min_index = i;
                }
            }else ESP_LOGW("TASK", "FK 精度校验不通过");
        }
    }
    // 没找到解或者最小解大于上一次最小解
    if(min_index == -1 || *min_error <= temp_min_errors)
    {
        return false;
    }
    for (int i = 0; i < 4; i++)
    {
        temp_theta[i] = theta_set[min_index][i] * RAD_TO_DEG;
    }
    *min_error = temp_min_errors;

    return true;
}

// 逆解，多解选择
bool manipulate(const pos_t *target_pos, const float *last_theta, float *cur_theta)
{
    // 本次逆解值
    float temp_theta[JOINT_5] = {0.0,0.0,0.0,0.0};
    float min_error = INFINITY;      // 最小差值
    bool valid = false;                     // 是否有解

    // 专门检查 beta = 0
    if(manip_IK(target_pos, last_theta, 0.0f, temp_theta, &min_error))
    {
        valid = true;
    }
    // for (float beta = -PI; beta <= PI; beta += 0.01f)
    // {
    //     if(!manip_IK(target_pos, last_theta, beta, temp_theta, &min_error))
    //     {
    //         continue;
    //     }else valid = true;
    // }

    if(valid)
    {
        cur_theta[JOINT_1] = temp_theta[0];
        cur_theta[JOINT_2] = temp_theta[1];
        cur_theta[JOINT_3] = temp_theta[2];
        cur_theta[JOINT_4] = temp_theta[3];
        cur_theta[JOINT_5] = -90.0f;
        cur_theta[JOINT_6] = -90.0f;
    }else ESP_LOGW("MANIP","无有效解");
    return valid;
}

bool manip_control(manip_t *manipulator) 
{
    if (manipulator == NULL) return false;

    // 使用局部临时变量接收 IK 解算结果（防止脏数据写入）
    float calc_theta[JOINT_MAX_NUM] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

    // 求解 IK
    if (!manipulate(&manipulator->target_pos, manipulator->last_joint_theta, calc_theta)) {
        ESP_LOGW("IK", "目标点超出工作空间，拒绝执行！");
        return false;
    }

    // 校验全部通过，才真正更新全局结构体的数据
    for(joint_id_t i = JOINT_1; i < JOINT_5; i++)
    {
        manipulator->cur_joint_theta[i] = calc_theta[i];
    }
    
    return true;
}

//关节角转换舵机角度
void theta_to_angle(float *cur_theta, float *s_angle)
{
    if(s_angle == NULL || cur_theta == NULL) return;
    const joint_config_t *cfg = joint_config;

    for (joint_id_t i = JOINT_1; i < JOINT_MAX_NUM; i++)
    {
        float angle;
        angle = cfg[i].offset + cfg[i].dir * cur_theta[i];
        if (angle >= 360.0f)
        {
            angle -=360.0f;
        }
        
        //角度限位
        angle = CLAMP(angle,ANGLE_MIN,ANGLE_MAX);
        s_angle[i] = angle;
    }
}

// 计算占空比
uint16_t servo_pos_get(float theta)
{
    //角度限位
    theta = CLAMP(theta,ANGLE_MIN,ANGLE_MAX);

    // 0度0.5ms = 0.025f(0.5/20)
    uint16_t duty = ((float)theta / 270.0f ) * 2000 + 500;
    return duty;
}
