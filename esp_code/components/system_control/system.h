#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "stdint.h"
#include "servo.h"
#include "myuart.h"

#define  SERVO_STEP     2      // 角度增幅
#define  CMD            0x03
#define  CONTROL_TIME   100     //ms

void system_init(void);

void key_task(void *arg);
void manip_control_task(void *arg);
void send_data_task(void *arg);
// bool parse_coord_frame(const uint8_t *data, uint16_t len, pos_t *pos);

#endif
