#ifndef _BLE_H_
#define _BLE_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

//========== 帧头帧尾 =============
#define FRAME_HEADER_1  0xA1
#define FRAME_HEADER_2  0xB2
#define FRAME_TAIL      0xC3
#define FRAME_LEN       9

void ble_init(QueueHandle_t BleQueue);
void start_advertising(void);

#endif
