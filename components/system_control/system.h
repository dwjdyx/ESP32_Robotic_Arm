#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "stdint.h"
#include "servo.h"
#include "myuart.h"

#define  SERVO_STEP     2      // 角度增幅
#define  CMD            0x03
#define  CONTROL_TIME   100     //ms

void system_init(void);

void oled_show_task(void *param);
void ps2_task(void *param);
void send_data_task(void *param);

#endif
