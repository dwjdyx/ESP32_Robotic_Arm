#include "system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "websocket.h"
#include "nvs_flash.h"

#include "stdint.h"
#include "myuart.h"
#include "oled.h"
#include "ble.h"
#include "manip.h"
#include "button.h"

// 舵机对应控制板通道
static const uint8_t servo_channel[JOINT_MAX_NUM]=
{
    0x00,
    0x01,
    0x0C,
    0x0E,
    0x0D,
    0x0F
};

// 声明句柄
SemaphoreHandle_t xRunBinary = NULL;
QueueHandle_t xPosQueue = NULL;
QueueHandle_t xAngleQueue = NULL;

// 包装数据
void txdata_package(float *s_angle, uint8_t* tx_buf)
{
    uint16_t pos;

    for(joint_id_t index = JOINT_1; index<JOINT_MAX_NUM; index++)
    {
        pos = servo_pos_get(s_angle[index]);
        uint8_t base_idx = (index * 3);
        *(tx_buf + base_idx) = servo_channel[index];
        *(tx_buf + base_idx + 1) = pos & 0xFF;
        *(tx_buf + base_idx + 2) = (pos >> 8) & 0xFF;
    }
}

void angle_init(void)
{
    uint8_t InitTxBuf[DATA_LENGTH];
    float initial_angle[JOINT_MAX_NUM] = 
    {
        SERVO_INIT_ANGLE, 
        SERVO2_INIT_ANGLE,
        SERVO_INIT_ANGLE,
        SERVO_INIT_ANGLE,
        SERVO_INIT_ANGLE,
        SERVO_INIT_ANGLE
    };

    InitTxBuf[0] = InitTxBuf[1] = 0x55;
    InitTxBuf[2] = JOINT_MAX_NUM * 3 + 5;
    InitTxBuf[3] = RUN_CMD;
    InitTxBuf[4] = JOINT_MAX_NUM;
    InitTxBuf[5] = CONTROL_TIME & 0xFF;
    InitTxBuf[6] = (CONTROL_TIME >> 8) & 0xFF;
    
    txdata_package(initial_angle,&InitTxBuf[7]);
    uart_senddata(InitTxBuf);
}

void system_init(void)
{
    xRunBinary = xSemaphoreCreateBinary();      // 系统启动标志位
    xPosQueue = xQueueCreate( 1, sizeof( pos_t ) );
    xAngleQueue = xQueueCreate( 1, sizeof(float) * JOINT_MAX_NUM );

    // 防护：检查创建是否成功
    if (xAngleQueue == NULL || xPosQueue == NULL || xRunBinary == NULL) {
        // 创建失败处理
        ESP_LOGE("INIT", "Creat fail!");
        return;
    }

    // 初始化 NVS
    esp_err_t ret = nvs_flash_init();

    // NVS 分区版本不兼容或空间不足时，擦除后重新初始化
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化流程
    myuart_init();
    angle_init();
    ble_init(xPosQueue);

    // // 初始化 WebSocket 服务器
    // ESP_ERROR_CHECK(websocket_server_init());
}

// 按键控制任务启动
void key_task(void *arg)
{
    uint8_t last_key = UNPRESSED;
    uint8_t now_key = UNPRESSED;
    while(1)
    {
        now_key = get_button_state(KEY_PIN);
        if(now_key != last_key)
        {
            vTaskDelay(pdMS_TO_TICKS(20)); //消抖
            now_key = get_button_state(KEY_PIN);
            if(now_key != last_key)
            {
                if(now_key == PRESSED) //检测到按下
                {
                    xSemaphoreGive(xRunBinary); // 发事件通知
                }
                last_key = now_key;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void manip_control_task(void *arg)
{
    pos_t target;
    float target_angle[JOINT_MAX_NUM] = {0};
    while (1)
    {
        // 等待任务开始
        if(xSemaphoreTake(xRunBinary, portMAX_DELAY) == pdPASS)
        {
            // 阻塞等待目标坐标
            if (xQueueReceive(xPosQueue, &target, portMAX_DELAY) == pdPASS)
            {
                manipulator.target_pos = target;
                if(manip_control(&manipulator))
                {
                    theta_to_angle(manipulator.cur_joint_theta,target_angle);

                    if(xQueueSend(xAngleQueue, target_angle, pdMS_TO_TICKS(1)) == pdPASS)
                    {
                        ESP_LOGI("MANIP","%.2f %.2f %.2f %.2f %.2f 发送成功\n",
                            target_angle[JOINT_1],target_angle[JOINT_2],target_angle[JOINT_3],
                            target_angle[JOINT_3],target_angle[JOINT_5]);
                    }else ESP_LOGW("MANIP", "servo_target_angle 发送失败\n");
                }else{
                    ESP_LOGW("MANIP", "目标位置无效，取消本次动作\n");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// 机械臂控制任务函数
void send_data_task(void *arg)
{
    uint8_t LobotTxBuf[DATA_LENGTH];
    LobotTxBuf[0] = LobotTxBuf[1] = 0x55;
    LobotTxBuf[2] = JOINT_MAX_NUM * 3 + 5;
    LobotTxBuf[3] = RUN_CMD;
    LobotTxBuf[4] = JOINT_MAX_NUM;
    LobotTxBuf[5] = CONTROL_TIME & 0xFF;
    LobotTxBuf[6] = (CONTROL_TIME >> 8) & 0xFF;

    float local_angle[JOINT_MAX_NUM];
    while(1)
    {
        if (xQueueReceive(xAngleQueue, local_angle, portMAX_DELAY) == pdPASS){
            ESP_LOGI("ARM","local_angle 接收成功\n");

            txdata_package(local_angle, &LobotTxBuf[7]);
            uart_senddata(LobotTxBuf);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
 