#include "traj.h"
#include "servo.h"
#include "math.h"
#include "esp_log.h"

// 用于ACT_WALK计算时间差
bool second_pair_started = false;
TickType_t gait_tick;

// 更新关节角并转换成舵机角
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

// ====================== 任务函数 ===========================
void traj_task(void *param)
{
    // 记录当前时间
    TickType_t last_log = xTaskGetTickCount();

    while (1)
    {

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
