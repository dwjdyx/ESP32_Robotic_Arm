#include "test.h"
#include "manip.h"
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// ==================== 运动学测试 ==========================
//测试运动学函数   正向测试
void manip_FKtest(float joint1,float joint2, float joint3)
{
    float input_angle [JOINT_5] = {joint1,joint2,joint3,90.00f};

    pos_t pos;
    float output_angle[THETA_MAX_NUM][JOINT_5];

    manip_FK(&pos,input_angle);
    if(manip_IK(&pos,output_angle))
    {
        printf("FK Test  angle-pos-angle\n");
        printf("input_angle: %.2f %.2f %.2f\n", input_angle[JOINT_1], input_angle[JOINT_2],input_angle[JOINT_3]);

        printf("pos: %.2f %.2f %.2f\n", pos.x,pos.y,pos.z);

        printf("output_angle[0]: %.2f %.2f %.2f\n", output_angle[0][JOINT_1],output_angle[0][JOINT_2],output_angle[0][JOINT_3]);
        printf("output_angle[1]: %.2f %.2f %.2f\n", output_angle[1][JOINT_1],output_angle[1][JOINT_2],output_angle[1][JOINT_3]);
        printf("output_angle[2]: %.2f %.2f %.2f\n", output_angle[2][JOINT_1],output_angle[2][JOINT_2],output_angle[2][JOINT_3]);
        printf("output_angle[3]: %.2f %.2f %.2f\n", output_angle[3][JOINT_1],output_angle[3][JOINT_2],output_angle[3][JOINT_3]);
    }else  ESP_LOGW("IK", "目标点超出工作空间，拒绝执行！");

    printf("\n");
}
//逆向测试
void manip_IKtest(float x0, float y0, float z0)
{
    pos_t input_pos = {
        .x = x0,
        .y = y0,
        .z = z0
    };

    pos_t output_pos[THETA_MAX_NUM];
    float angle[THETA_MAX_NUM][JOINT_5];

    if(manip_IK(&input_pos,angle))
    {
        manip_FK(&output_pos[0],angle[0]);
        manip_FK(&output_pos[1],angle[1]);
        manip_FK(&output_pos[2],angle[2]);
        manip_FK(&output_pos[3],angle[3]);

        printf("IK Test pos-angle-pos\n");
        printf("input_pos: %.2f %.2f %.2f\n", input_pos.x,input_pos.y,input_pos.z);
        printf("output_pos[0]: %.2f %.2f %.2f\n", output_pos[0].x,output_pos[0].y,output_pos[0].z);
        printf("output_pos[1]: %.2f %.2f %.2f\n", output_pos[1].x,output_pos[1].y,output_pos[1].z);
        printf("output_pos[2]: %.2f %.2f %.2f\n", output_pos[2].x,output_pos[2].y,output_pos[2].z);
        printf("output_pos[3]: %.2f %.2f %.2f\n", output_pos[3].x,output_pos[3].y,output_pos[3].z);

        printf("angle[0]: %.2f %.2f %.2f\n", angle[0][JOINT_1],angle[0][JOINT_2],angle[0][JOINT_3]);
        printf("angle[1]: %.2f %.2f %.2f\n", angle[1][JOINT_1],angle[1][JOINT_2],angle[1][JOINT_3]);
        printf("angle[2]: %.2f %.2f %.2f\n", angle[2][JOINT_1],angle[2][JOINT_2],angle[2][JOINT_3]);
        printf("angle[3]: %.2f %.2f %.2f\n", angle[3][JOINT_1],angle[3][JOINT_2],angle[3][JOINT_3]);
        printf("\n");
    }
    else ESP_LOGW("IK", "目标点超出工作空间，拒绝执行！");
}

// ==================== theta映射angle====================
void theta_map_angle(void)
{
    float b = (L2 + d4 + d5) + 10.25f;
    float c = (L2 + d4 + d5) - 10.25f;
    float v = (L2 + d4 + d5) * cosf(PI/4);
    float u = v + 10.25f;
    float l = v - 10.25f;
    float p = u * cosf(PI/4);
    float q = l * cosf(PI/4);
    float n = c * cosf(PI/4);

    // float a = (d4 + d5) * cosf(PI/4);
    // float b = a + 10.25f;
    // float c = a - 10.25f;
    // float d = a + L2;
    // float e = b * cosf(PI/4);
    // float f = c * cosf(PI/4);
    // float g = (d4 + d5) + 10.25;
    // float h = g - 10.25 * 2;
    // float I = g * cosf(PI/4);
    // float J = h * cosf(PI/4);
// ================== 验证FKIK ======================
    manip_IKtest(10.25f,0.0f,274.0f);
    manip_IKtest(u,0.0f,v);
    manip_IKtest(b,0.0f,0.0f);
    manip_IKtest(-l,0.0f,v);
    manip_IKtest(-c,0.0f,0.0f);
    manip_IKtest(7.25f,7.25f,274.0f);
    manip_IKtest(p,p,v);
    manip_IKtest(-q,-q,v);

    // manip_IKtest(b,0,d);
    // manip_IKtest(g,0,L2);
    // manip_IKtest(-c,0,d);
    // manip_IKtest(-h,0,L2);
    // manip_IKtest(e,e,d);
    // manip_IKtest(I,I,L2);
    // manip_IKtest(-f,-f,d);
    // manip_IKtest(-J,-J,L2);

    // manip_FKtest(0.0f,-90.0f,-45.0f);
    // manip_FKtest(45.0f,-180.0f,-90.0f);
    // manip_FKtest(-135.0f,-24.58f,-51.76f);
    // manip_FKtest(-315.0f,-135.0f,-90.0f);
}
