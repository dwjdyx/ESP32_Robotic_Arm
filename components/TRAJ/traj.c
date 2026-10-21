#include "traj.h"
#include "servo.h"
#include "math.h"
#include "esp_log.h"

// 用于ACT_WALK计算时间差
bool second_pair_started = false;
TickType_t gait_tick;

// ===================== 初始化 ====================

// ===================== 坐标更新 =========================

/**
 * @brief 计算两坐标极角
 *
 * @param start_pos       起始坐标
 * @param end_pos         终止坐标
 * @param theta           极角
 * @param circel          圆心坐标
 */
static void polar_angle_cal(const pos_t *start_pos, const pos_t *end_pos, 
                            float *theta_start, float *theta_end,
                            float circle_u, float circle_v, float n) // 圆心坐标
{
//============ 以下计算公式根据实际修改 ==============
    float start_u, start_v, end_u, end_v;
    if(n == 1)      // knee移动
    {
        start_u = sqrtf(start_pos->x * start_pos->x + start_pos->y * start_pos->y);
        start_v = start_pos->z;

        end_u = sqrtf(end_pos->x * end_pos->x + end_pos->y * end_pos->y);
        end_v = end_pos->z;
    }else if(n == 2)    // hip移动
    {
        start_u = start_pos->x;
        start_v= start_pos->y;

        end_u = end_pos->x;
        end_v = end_pos->y;
    }

    // 计算局部坐标
    float u0 = start_u - circle_u;
    float v0 = start_v - circle_v;

    float u1 = end_u - circle_u;
    float v1 = end_v - circle_v;

    // 计算起点终点局部极角
    *theta_start = atan2f(v0, u0);
    *theta_end = atan2f(v1, u1);
}

// 检查dtheta变化
/*static void check_traj_dtheta(float *dtheta)
{
    if(*dtheta > M_PI) *dtheta -= 2*M_PI;
    else if (*dtheta < -M_PI) *dtheta += 2*M_PI;
    else return;
}*/

/** 
 *  @brief 膝关节运动轨迹计算
 * 
 * 根据起点和终点的位置，计算腿部在 XZ 平面上的圆弧轨迹
 * 并根据当前轨迹进度 ratio 计算当前足端目标位置
 * 
 * @param start_pos     当前轨迹阶段的起点
 * @param end_pos       当前轨迹阶段的终点
 * @param manip           当前腿的数据
 * @param traj_calc    当前腿的轨迹参数 
 * @param init_index    当前轨迹阶段对应的初始化标志位索引
 */
static pos_t knee_move(const pos_t *start_pos, const pos_t *end_pos, 
                            traj_phase_param_t *param, float ratio)
{
    traj_phase_t *phase = &param->cur_phase;
    traj_calc_t *calc = &param->calc[*phase];

    if(param->initialized[*phase])
    {
        // 计算起点xy平面角度
        calc->theta_xy = atan2f(start_pos->y, start_pos->x);
        // 计算极角
        polar_angle_cal(start_pos, end_pos,
                        &calc->theta_start, &calc->theta_end, 
                        TRAJ_RZ_CIRCLE_U, TRAJ_RZ_CIRCLE_V, 1);
        // 计算极角变化量
        calc->dtheta = calc->theta_end - calc->theta_start;

        param->initialized[*phase] = false;  //标志位置false，以上仅初始化一次
    }
    
    float alpha = calc->theta_start + calc->dtheta * ratio;  // 计算角度变化量

    pos_t target_pos;

    float R = 40.0f + 53.0f * cosf(alpha);

    target_pos.z = 53.0f * sinf(alpha);
    target_pos.x = R * cosf(calc->theta_xy);
    target_pos.y = R * sinf(calc->theta_xy);

    return target_pos;
}

/** 
 * @brief 髋关节运动轨迹计算
 * 
 * 根据起点和终点的位置，计算足端在 XY 平面上的圆弧轨迹
 * 并根据当前轨迹进度 ratio 计算当前足端目标位置 
 * 
 * @param start_pos     当前轨迹阶段的起点
 * @param end_pos       当前轨迹阶段的终点
 * @param manip           当前腿的数据
 * @param traj_calc    当前腿的轨迹参数
 * @param init_index    当前轨迹阶段对应的初始化标志位索引 
 */
static pos_t hip_move(const pos_t *start_pos, const pos_t *end_pos, 
                            traj_phase_param_t *param, float ratio)
{
    traj_phase_t *phase = &param->cur_phase;
    traj_calc_t *calc = &param->calc[*phase];
    if(param->initialized[*phase])
    {
        calc->radius_xy = sqrtf(start_pos->x * start_pos->x + start_pos->y * start_pos->y);
        polar_angle_cal(start_pos, end_pos, 
                        &calc->theta_start, &calc->theta_end, 
                        TRAJ_XY_CIRCLE_U, TRAJ_XY_CIRCLE_V, 2);
        calc->dtheta = calc->theta_end - calc->theta_start;

        param->initialized[*phase] = false;
    }

    pos_t target_pos;

    float alpha = calc->theta_start + calc->dtheta * ratio;
    target_pos.x = calc->radius_xy * cosf(alpha);
    target_pos.y = calc->radius_xy * sinf(alpha);
    target_pos.z = end_pos->z;

    return target_pos;
}

// 更新phase
static void traj_phase_update(traj_phase_t *phase, float t)
{
    if (phase == NULL)
        return;

    switch (*phase)
    {
        case TRAJ_PHASE_1:
            if (t > TRAJ_LIFT_RATIO)
            {
                *phase = TRAJ_PHASE_2;
            }
            break;

        case TRAJ_PHASE_2:
            if (t > TRAJ_LIFT_RATIO + TRAJ_MOVE_RATIO)
            {
                *phase = TRAJ_PHASE_3;
            }
            break;

        case TRAJ_PHASE_3:
            if (t > TRAJ_LIFT_RATIO +
                    TRAJ_MOVE_RATIO +
                    TRAJ_DOWN_RATIO)
            {
                *phase = TRAJ_PHASE_4;
            }
            break;

        case TRAJ_PHASE_4:
            break;

        default:
            *phase = TRAJ_PHASE_1;
            break;
    }
}

// 计算时间差
static float traj_deltaT_cal(traj_time_t *time)
{
    if(time == NULL) return 0.0f;
    // 防止除零
    if (time->duration <= 0.0f) return 0.0f;

    TickType_t now = xTaskGetTickCount();

    float dt = (now - time->last_tick) * (portTICK_PERIOD_MS); //转为ms

    time->elapsed += dt;
    time->last_tick = now;

    // 计算当前轨迹进度 t：0.0 ~ 1.0
    float t = time->elapsed / time->duration;

    return t;
}

/**
 * @brief 更新足端轨迹坐标
 *
 * 根据当前轨迹运行时间，计算足端当前应该到达的坐标。
 * 当前采用三维空间中的线性插值。
 *
 * @param traj      轨迹参数
 * @param manip       需要更新的腿
 */
static void gait_traj_update(gait_traj_t *traj, manip_t *manip, joint_id_t id)
{
    if (traj == NULL || manip == NULL) return;
    // 轨迹未运行，直接返回
    if (!traj->running) return;
    // 防止除零
    if (traj->time.duration <= 0.0f) return;

//================== 计算时间差 ===================
    // 计算当前轨迹进度 t：0.0 ~ 1.0
    float t = traj_deltaT_cal(&traj->time);
    if(t < 0.0f) t = 0.0f;
    else if(t > 1.0f) t = 1.0f;
//==================== 更新phase =====================
    traj_phase_update(&traj->phase_param.cur_phase, t);

//==================== 分时间段计算坐标 =================
    float ratio;        // 时间差
    // 以下关系式/结果基本是根据实际预先算好的，不建议直接复用，根据实际修改
    switch (traj->phase_param.cur_phase)
    {
        case TRAJ_PHASE_1:
        {
            //阶段1:原地抬腿 start → lift_start
            ratio = t / TRAJ_LIFT_RATIO;  // 根据比例计算时间
            manip->target_pos = knee_move(&traj->point[0], &traj->point[1], &traj->phase_param, ratio);
            break;
        }
        case TRAJ_PHASE_2:
        {
            //阶段2:保持高度，水平移动 lift_start → lift_end
            ratio = (t - TRAJ_LIFT_RATIO) / TRAJ_MOVE_RATIO;
            manip->target_pos = hip_move(&traj->point[1], &traj->point[2], &traj->phase_param, ratio);
            break;
        }
        case TRAJ_PHASE_3:
        {
            //阶段3:下降落地 lift_end → end
            ratio =
                (t - TRAJ_LIFT_RATIO - TRAJ_MOVE_RATIO)
                / TRAJ_DOWN_RATIO;

            manip->target_pos = knee_move(&traj->point[2], &traj->point[3], &traj->phase_param, ratio);
            break;
        }
        case TRAJ_PHASE_4:
        {
            //阶段4:回到起始点 end → start
            ratio =
                (t - TRAJ_LIFT_RATIO - TRAJ_MOVE_RATIO - TRAJ_DOWN_RATIO)
                / TRAJ_BACK_RATIO;
            manip->target_pos = hip_move(&traj->point[3], &traj->point[0], &traj->phase_param, ratio);
            break;
        }
        default: break;
    }
//===================== 打印坐标验证坐标拆解合理性 =====================
//    ESP_LOGI("TRAJ",
//         "manip%d phase=%d t=%.3f pos=(%.6f, %.6f, %.6f)",
//         id,
//         traj->phase_param.cur_phase,
//         t,
//         manip->target_pos.x,
//         manip->target_pos.y,
//         manip->target_pos.z);
    
    //检验坐标合理性
//    if(!check_pos_validity(manip->target_pos.x, 
//                           manip->target_pos.y, 
//                           manip->target_pos.z))
//    {
//        ESP_LOGW("TASK", "目标点不合理，拒绝执行！");
//        
//    }


// ====================== 验证相位 ==================
//    ESP_LOGI("GAIT",
//         "t=%.2f elapsed=%.1f",
//         t,
//         traj->elapsed);

    // 到达终点
    if (traj->time.elapsed >= traj->time.duration)
    {
        traj->time.elapsed = 0.0f;
        traj->time.last_tick = xTaskGetTickCount();
        traj->phase_param.cur_phase = TRAJ_PHASE_1;
        // 进行循环
        for(int i = 0; i < TRAJ_PHASE_MAX; i++)
        {
            traj->phase_param.initialized[i] = true;
        }
    }
}

// ========================== 以下为gait_task调用函数，层级由上至下 ============================
// 2.更新关节角并转换成舵机角
static void gait_update_joint(void)
{
    // 计算关节角
    if (!manip_control_task(&manipulator))
    {
        ESP_LOGW("GAIT", "manip control failed");
    }

    for(joint_id_t i = JOINT_1; i<JOINT_MAX_NUM; i++)
    {
        // 角度转换
        theta_to_angle(manipulator.angle[i].joint_angle,
                    i,
                    &manipulator.angle[i].servo_angle);

    }
}
// 3.统一输入占空比
static void gait_update_servo(void)
{
    for (joint_id_t i = JOINT_1; i < JOINT_MAX_NUM; i++)
    {
        servo_angle_set(joint_config[i].servo_id,
                        manipulator.angle[i].servo_angle);
    }
}

// ====================== 任务函数 ===========================
void traj_task(void *param)
{
    // 记录当前时间
    TickType_t last_log = xTaskGetTickCount();

    while (1)
    {
// ======================= 执行任务 =====================
        // 2. 足端坐标 -> 关节角 -> 舵机角
        gait_update_joint();

        // 3. 舵机角 -> PWM
        gait_update_servo();
        
        //数据打印
        /*if (xTaskGetTickCount() - last_log >= pdMS_TO_TICKS(500))
        {
            last_log = xTaskGetTickCount();

            for (joint_id_t i = JOINT_1; i < JOINT_MAX_NUM; i++)
            {
                ESP_LOGI("GAIT",
                         "manip %d: target(%.1f, %.1f, %.1f), theta(%.1f, %.1f)",
                         i,
                         manip_array[i].target_pos.x,
                         manip_array[i].target_pos.y,
                         manip_array[i].target_pos.z,
                         manip_array[i].servo_angle.hip_angle,
                         manip_array[i].servo_angle.knee_angle);
            }
        }*/

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
