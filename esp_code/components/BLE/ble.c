#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "ble.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "manip_config.h"

#include "nimble/nimble_port_freertos.h"
#include "nimble/nimble_port.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "manip_config.h"

#define TAG "NimBLE-Demo"
#define DEVICE_NAME "ESP32-NimBLE"

static bool ble_adv_active = false;
static QueueHandle_t s_queue = NULL;

// RX：手机写入数据
static uint16_t rx_value_handler;
// TX：手机读取数据
static uint16_t tx_value_handler;

// 数据解析函数
bool parse_coord_frame(const uint8_t *data, uint16_t len, pos_t *pos)
{
    if (data == NULL || pos == NULL ||len != FRAME_LEN) {
        return false;
    }

    // 检查帧头和帧尾
    if (data[0] != FRAME_HEADER_1 ||
        data[1] != FRAME_HEADER_2 ||
        data[8] != FRAME_TAIL) {
        return false;
    }

    // 大端格式：高字节在前
    int16_t x = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    int16_t y = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    int16_t z = (int16_t)(((uint16_t)data[6] << 8) | data[7]);

    pos->x = x;
    pos->y = y;
    pos->z = z;

    return true;
}

// BLE TX：向手机发送字符串
static void ble_send_string(const char *str)
{
    struct os_mbuf *om;

    if (str == NULL)
        return;

    om = ble_hs_mbuf_from_flat(str, strlen(str));
    if (om == NULL)
    {
        ESP_LOGE(TAG, "创建 BLE 发送缓冲区失败");
        return;
    }

    int rc = ble_gatts_notify_custom(0, tx_value_handler, om);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "BLE Notify 发送失败: %d", rc);
    }
}

//GATT 特征访问回调函数
static int gatt_event_handle(uint16_t conn_handle,
                             uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    // 手机写入坐标数据
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        if (attr_handle == rx_value_handler)
        {
            uint8_t frame[FRAME_LEN];
            pos_t target_pos = {0};
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);

            // 固定帧长度校验
            if (len != FRAME_LEN) {
                ESP_LOGW(TAG, "坐标帧长度错误: %u", len);
                ble_send_string("ERR:LEN");
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            // 复制完整数据，避免只读取第一个 mbuf 分片
            if (os_mbuf_copydata(ctxt->om, 0, FRAME_LEN, frame) != 0) {
                ESP_LOGW(TAG, "坐标帧读取失败");
                ble_send_string("ERR:DATA");
                return BLE_ATT_ERR_UNLIKELY;
            }
            if(parse_coord_frame(frame,FRAME_LEN,&target_pos))
            {
                if (s_queue == NULL ||xQueueSend(s_queue, &target_pos, 0) != pdPASS)
                {
                    ESP_LOGW(TAG, "坐标帧队列发送失败");
                    ble_send_string("ERR:QUEUE");
                    return BLE_ATT_ERR_UNLIKELY;
                }
                ble_send_string("OK");
            }else{
                ESP_LOGW(TAG, "坐标帧格式错误");
                ble_send_string("ERR:FRAME");
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
        }
    }
    // 手机读取数据
    else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        if (attr_handle == tx_value_handler)
        {
            const char *value = "hello";

            os_mbuf_append(ctxt->om,
                           value,
                           strlen(value));
        }
    }
    return 0;
}

//GAP 事件回调函数
static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    //BLE 连接事件
    if (event->type == BLE_GAP_EVENT_CONNECT)
    {
        if (event->connect.status == 0)
        {
            ESP_LOGI(TAG, "设备已连接");
            // 已经建立连接，不需要继续广播
            ble_adv_active = false;
        }
        else
        {
            ESP_LOGI(TAG, "连接失败,重新广播");

            if (!ble_adv_active)
            {
                start_advertising();
            }
        }
    }
    //BLE 断开事件
    else if (event->type == BLE_GAP_EVENT_DISCONNECT)
    {
        ESP_LOGI(TAG, "设备断开连接");
        //断开后重新广播
        if (!ble_adv_active)
        {
            start_advertising();
        }
    }

    return 0;
}


/*
 * GATT 数据库
 *
 * 用来描述：
 * ESP32 提供哪些 Service
 * Service 下面有哪些 Characteristic
 */
static const struct ble_gatt_svc_def gatt_svcs[] =
{
    {
        // 定义一个 Primary Service
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x00FF),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = BLE_UUID16_DECLARE(0xFF01),
                // 对该 Characteristic 的访问交给 gatt_event_handle()
                .access_cb = gatt_event_handle,
                // 允许手机 WRITE
                .flags = BLE_GATT_CHR_F_WRITE,
                .val_handle = &rx_value_handler,
                .arg = NULL
            },

            {
                .uuid = BLE_UUID16_DECLARE(0xFF02),
                .access_cb = gatt_event_handle,
                /*
                 * 允许：
                 * READ   → 手机主动读取
                 * NOTIFY → ESP32主动通知手机
                 */
                .flags = BLE_GATT_CHR_F_READ |
                         BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &tx_value_handler,
                .arg = NULL
            },

            //{0} 表示 Characteristic 数组结束
            {0}
        }
    },

    //{0} 表示 Service 数组结束
    {0}
};


//开启 BLE 广播
void start_advertising(void)
{
    //设置广播内容
    struct ble_hs_adv_fields fields = {0};

    // 广播中携带设备名称
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;

    /*
     * flags：
     * DISC_GEN      → 通用可发现
     * BREDR_UNSUP   → 不支持经典蓝牙 BR/EDR
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    // 使用自动发射功率
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.tx_pwr_lvl_is_present = 1;

    /*
     * 设置广播数据
     */
    int rc = ble_gap_adv_set_fields(&fields);

    if (rc != 0)
    {
        ESP_LOGE(TAG, "广播内容设置失败");
    }


    //BLE 广播参数
    struct ble_gap_adv_params adv_params =
    {
        // 可连接广播
        .conn_mode = BLE_GAP_CONN_MODE_UND,

        // 通用可发现模式
        .disc_mode = BLE_GAP_DISC_MODE_GEN,

        // 广播间隔
        .itvl_max = BLE_GAP_ADV_ITVL_MS(500),
        .itvl_min = BLE_GAP_ADV_ITVL_MS(200)
    };


    /*
     * 开始广播
     *
     * BLE_HS_FOREVER：
     * 一直广播，直到建立连接或者主动停止。
     *
     * gap_event_handler：
     * BLE GAP 事件发生时调用这个回调。
     */
    rc = ble_gap_adv_start(
        BLE_OWN_ADDR_PUBLIC,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        gap_event_handler,
        NULL
    );

    if (rc != 0)
    {
        ESP_LOGE(TAG, "广播启动失败");
    }
    else
    {
        ESP_LOGI(TAG, "广播已启动");
    }
}


/*
 * NimBLE Host 同步完成后的回调
 *
 * 当 NimBLE Host 初始化完成、可以正常工作后，
 * 会自动调用这里。
 */
static void on_sync(void)
{
    ESP_LOGI(TAG, "设备初始化完毕");

    // Host 准备完成后开始广播
    start_advertising();
}


/*
 * NimBLE Host 任务
 *
 * 这个任务负责持续运行 NimBLE 协议栈，
 * 接收和处理 BLE 事件。
 */
void host_task(void *arg)
{
    nimble_port_run();
}

//BLE 初始化
void ble_init(QueueHandle_t BleQueue)
{
    s_queue = BleQueue;
    if(s_queue == NULL)
    {
        return;
    }
    //初始化 NimBLE
    nimble_port_init();
    //初始化 GAP / GATT 服务
    ble_svc_gap_init();
    ble_svc_gatt_init();
    //置设备名称
    ble_svc_gap_device_name_set(DEVICE_NAME);

    /*
     * 告诉 NimBLE：
     * 我要注册这些 Service / Characteristic。
     */
    ble_gatts_count_cfg(gatt_svcs);

    //正式把 GATT Service 添加到 NimBLE。
    ble_gatts_add_svcs(gatt_svcs);

    /*
     * 设置 Host 同步完成后的回调。
     *
     * NimBLE 初始化完成后：
     * on_sync()
     *      ↓
     * start_advertising()
     */
    ble_hs_cfg.sync_cb = on_sync;


    /*
     * 创建并启动 NimBLE Host 任务。
     *
     * 从这里开始，BLE 协议栈在独立的 RTOS 任务中运行。
     */
    nimble_port_freertos_init(host_task);
}
