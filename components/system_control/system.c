#include "system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "websocket.h"
#include "nvs_flash.h"
#include "oled.h"

// 复制全局数据
static servo_angle_t local_angle[SERVO_MAX];

// 舵机对应控制板通道
static const uint8_t servo_channel[SERVO_MAX]=
{
    0x00,
    0x01,
    0x0C,
    0x0E,
    0x0D
};

// 系统使能标志位
static bool g_system_running = false;

// 声明互斥锁句柄
SemaphoreHandle_t xAngleMutex = NULL;
SemaphoreHandle_t xPosMutex = NULL;

// 包装数据
void txdata_package(servo_angle_t *angle, uint8_t* tx_buf)
{
    uint16_t pos;

    for(servo_id_t index = SERVO_BASE; index<SERVO_MAX; index++)
    {
        pos = servo_pos_get(angle[index].current_angle);
        uint8_t base_idx = (index * 3);
        *(tx_buf + base_idx) = servo_channel[index];
        *(tx_buf + base_idx + 1) = pos & 0xFF;
        *(tx_buf + base_idx + 2) = (pos >> 8) & 0xFF;
    }
}

void angle_init(void)
{
    uint8_t InitTxBuf[DATA_LENGTH];

    InitTxBuf[0] = InitTxBuf[1] = 0x55;
    InitTxBuf[2] = SERVO_MAX * 3 + 5;
    InitTxBuf[3] = CMD;
    InitTxBuf[4] = SERVO_MAX;
    InitTxBuf[5] = CONTROL_TIME & 0xFF;
    InitTxBuf[6] = (CONTROL_TIME >> 8) & 0xFF;
    
    servo_angle_init();
    txdata_package(&angle,&InitTxBuf[7]);
    uart_senddata(InitTxBuf);
}

void system_init(void)
{
    // 创建互斥锁
    xAngleMutex = xSemaphoreCreateMutex();
    xPosMutex = xSemaphoreCreateMutex();
    
    // 防护：检查创建是否成功
    if (xAngleMutex == NULL || xPosMutex == NULL) {
        // 创建失败处理
        ESP_LOGE("INIT", "Mutex creat fail!");
        return;
    }
    oled_init();
    myuart_init();
    angle_init();

    // // 初始化 NVS
    // esp_err_t ret = nvs_flash_init();

    // // NVS 分区版本不兼容或空间不足时，擦除后重新初始化
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
    //     ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }

    // ESP_ERROR_CHECK(ret);

    // // 初始化 WebSocket 服务器
    // ESP_ERROR_CHECK(websocket_server_init());
}

// // 数据解析函数
// bool parse_coord_frame(const uint8_t *data, uint16_t len, pos_t *pos)
// {
//     if (data == NULL || pos == NULL || len != FRAME_LEN) {
//         return false;
//     }

//     // 检查帧头和帧尾
//     if (data[0] != FRAME_HEADER_1 ||
//         data[1] != FRAME_HEADER_2 ||
//         data[8] != FRAME_TAIL) {
//         return false;
//     }

//     // 大端格式：高字节在前
//     int16_t x = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
//     int16_t y = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
//     int16_t z = (int16_t)(((uint16_t)data[6] << 8) | data[7]);

//     pos->x = (float)x;
//     pos->y = (float)y;
//     pos->z = (float)z;

//     return true;
// }

void oled_show_task(void *param)
{
    while (1)
    {
        OLED_Printf(0,0,OLED_6X8,"J1:%04.0f",local_angle[SERVO_BASE].target_angle);
        OLED_Printf(0,10,OLED_6X8,"J2:%04.0f",local_angle[SERVO_SHOULDER].target_angle);
        OLED_Printf(0,20,OLED_6X8,"J3:%04.0f",local_angle[SERVO_ELBOW].target_angle);
        OLED_Printf(0,30,OLED_6X8,"J4:%04.0f",local_angle[SERVO_WRIST].target_angle);
        OLED_Printf(0,40,OLED_6X8,"J5:%04.0f",local_angle[SERVO_GRIPPER].target_angle);
        OLED_Update();
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
}

// 机械臂控制任务函数
void send_data_task(void *param)
{
    uint8_t LobotTxBuf[DATA_LENGTH];
    LobotTxBuf[0] = LobotTxBuf[1] = 0x55;
    LobotTxBuf[2] = SERVO_MAX * 3 + 5;
    LobotTxBuf[3] = CMD;
    LobotTxBuf[4] = SERVO_MAX;
    LobotTxBuf[5] = CONTROL_TIME & 0xFF;
    LobotTxBuf[6] = (CONTROL_TIME >> 8) & 0xFF;
    while(1)
    {
        if(g_system_running)
        {
            // 上锁
            if (xSemaphoreTake(xAngleMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                
                //更新target_angle
                // 把全局数据深拷贝到局部数组里
                for (uint8_t i = SERVO_BASE; i < SERVO_MAX; i++) {
                    local_angle[i] = angle[i];
                }

                // 解锁
                xSemaphoreGive(xAngleMutex);
            }else continue;
            // float target[SERVO_MAX];
            // for(servo_id_t i = SERVO_BASE; i<SERVO_MAX; i++)
            // {
            //     target[i] = local_angle[i].target_angle;
            // }
            // ws_set_servo_angles(&target);
            txdata_package(local_angle, &LobotTxBuf[7]);
            uart_senddata(LobotTxBuf);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else{
            vTaskDelay(pdMS_TO_TICKS(20));
        }

    }
}
