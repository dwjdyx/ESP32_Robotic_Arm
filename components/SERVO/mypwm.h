#ifndef _MYPWM_H_
#define _MYPWM_H_

#include "stdint.h"
#include "driver/ledc.h"

//舵机引脚
#define   BASE        GPIO_NUM_4        // 底座旋转 θ1
#define   SHOULDER    GPIO_NUM_5        // 肩部 θ2
#define   ELBOW       GPIO_NUM_6        // 肘部 θ3
#define   WRIST       GPIO_NUM_7        // 腕部/爪子转向 θ4
//#define   GRIPPER     GPIO_NUM_15       // 末端旋转/夹爪 θ5

void mypwm_init(void);
void pwm_duty_set(ledc_channel_t channel, uint16_t duty);

#endif
