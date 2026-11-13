#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#define  SERVO_STEP     2      // 角度增幅
#define  RUN_CMD            0x03    //发送数据指令
#define  RETURN_CMD         0x08    //接收数据指令
#define  CONTROL_TIME   100     //ms

void system_init(void);

void key_task(void *arg);
void manip_control_task(void *arg);
void send_data_task(void *arg);

#endif
