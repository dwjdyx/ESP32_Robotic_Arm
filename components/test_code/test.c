#include "test.h"
#include "manip.h"
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==================== 运动学测试 ==========================
//测试运动学函数   正向测试
void manip_FKtest(void)
{
    float input_angle [JOINT_MAX_NUM] = {0.0f,-83.76f,-79.87f,135.00f};

    pos_t pos;
    float output_angle[2][JOINT_MAX_NUM];

    manip_FK(&pos,input_angle);
    manip_IK(&pos,output_angle[0],output_angle[1]);

    printf("input_angle: %.2f %.2f %.2f %.2f\n", input_angle[JOINT_1], input_angle[JOINT_2],
                                                input_angle[JOINT_3],input_angle[JOINT_4]);

    printf("output_angle[0]: %.2f %.2f %.2f %.2f\n", output_angle[0][JOINT_1],output_angle[0][JOINT_2],
                                                output_angle[0][JOINT_3],output_angle[0][JOINT_4]);
    printf("output_angle[1]: %.2f %.2f %.2f %.2f\n", output_angle[1][JOINT_1],output_angle[1][JOINT_2],
                                                output_angle[1][JOINT_3],output_angle[1][JOINT_4]);

    printf("pos: %.2f %.2f %.2f\n", pos.x,pos.y,pos.z);
}
//逆向测试
void manip_IKtest(void)
{
    pos_t input_pos = {
        .x = 10.25f,
        .y = 0.0f,
        .z = 270.0f
    };

    pos_t output_pos[2];
    float angle[2][JOINT_MAX_NUM];

    manip_IK(&input_pos,angle[0],angle[1]);
    manip_FK(&output_pos[0],angle[0]);
    manip_FK(&output_pos[1],angle[1]);

    printf("output_pos[0]: %.2f %.2f %.2f\n", output_pos[0].x,output_pos[0].y,output_pos[0].z);
    printf("output_pos[1]: %.2f %.2f %.2f\n", output_pos[1].x,output_pos[1].y,output_pos[1].z);

    printf("input_pos: %.2f %.2f %.2f\n", input_pos.x,input_pos.y,input_pos.z);

    printf("angle[0]: %.2f %.2f %.2f %.2f\n", angle[0][JOINT_1],angle[0][JOINT_2],
                                                 angle[0][JOINT_3],angle[0][JOINT_4]);
    printf("angle[1]: %.2f %.2f %.2f %.2f\n", angle[1][JOINT_1],angle[1][JOINT_2],
                                                 angle[1][JOINT_3],angle[1][JOINT_4]);
}
