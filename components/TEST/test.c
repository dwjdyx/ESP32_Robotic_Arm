#include "test.h"
#include "manip.h"
#include "servo.h"
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==================== 舵机转动方向测试 ====================
void servo_direction_test(void)
{
       for(servo_id_t i = SERVO_BASE; i < SERVO_MAX; i++)
       {
              for(int j = 30; j <= 270; j+=30)
              {
                     servo_angle_set(i, j);
                     vTaskDelay(pdMS_TO_TICKS(500));
              }
       }
}

// ==================== 运动学测试 ==========================
//测试运动学函数   正向测试
void manip_FKtest(void)
{
    float input [JOINT_MAX_NUM] = {61.83f,30.98f,33.06f,135.0f};

    pos_t pos;
    float output[JOINT_MAX_NUM];

    manip_FK(&pos,input);
    manip_IK(&pos,output);

    printf("input: %.2f %.2f %.2f %.2f\n", input[JOINT_1], input[JOINT_2],
                                                input[JOINT_3],input[JOINT_4]);

    printf("output: %.2f %.2f %.2f %.2f\n", output[JOINT_1],output[JOINT_2],
                                                 output[JOINT_3],output[JOINT_4]);

    printf("pos: %.2f %.2f %.2f\n", pos.x,pos.y,pos.z);
}
//逆向测试
void manip_IKtest(void)
{
    pos_t pos = {
        .x = 10.25f,
        .y = 16.0f,
        .z = 198.0f
    };

    pos_t test_pos;
    float output[JOINT_MAX_NUM];

    manip_IK(&pos,output);
    manip_FK(&test_pos,output);

    printf("test_pos: %.2f %.2f %.2f\n", test_pos.x,test_pos.y,test_pos.z);
           
    printf("pos: %.2f %.2f %.2f\n", pos.x,pos.y,pos.z);

    printf("output: %.2f %.2f %.2f %.2f\n", output[JOINT_1],output[JOINT_2],
                                                 output[JOINT_3],output[JOINT_4]);

}
