#include "websocket.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#define WIFI_AP_SSID       "Robot_Arm_AP"
#define WIFI_AP_PASSWORD   "12345678"
#define WIFI_AP_MAX_CONN   4

#define WS_PUSH_PERIOD_MS  100
#define WS_RX_MAX_LEN      256
#define SERVO_COUNT        5

static const char *TAG = "websocket_server";

static httpd_handle_t s_server = NULL;
static int s_client_fd = -1;

/* 软件中的目标角度，不是舵机实际反馈角度 */
static float s_servo_target_angles[SERVO_COUNT] = {0};

/* 保护客户端 FD 和目标角度数组 */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;


/* 更新网页要显示的目标角度 */
void ws_set_servo_angles(const float angles[SERVO_COUNT])
{
    if (angles == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_lock);

    for (int i = 0; i < SERVO_COUNT; i++) {
        s_servo_target_angles[i] = angles[i];
    }

    portEXIT_CRITICAL(&s_lock);
}


/* WebSocket 请求处理 */
static esp_err_t ws_handler(httpd_req_t *req)
{
    /* WebSocket 握手请求 */
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);

        portENTER_CRITICAL(&s_lock);
        s_client_fd = fd;
        portEXIT_CRITICAL(&s_lock);

        ESP_LOGI(TAG, "WebSocket connected, fd=%d", fd);
        return ESP_OK;
    }

    /* 接收并忽略网页发送的数据 */
    httpd_ws_frame_t pkt = {0};

    esp_err_t err = httpd_ws_recv_frame(req, &pkt, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Get frame length failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    if (pkt.len > WS_RX_MAX_LEN) {
        ESP_LOGW(TAG, "WebSocket frame too large");
        return ESP_FAIL;
    }

    if (pkt.len > 0) {
        uint8_t *payload = calloc(1, pkt.len + 1);

        if (payload == NULL) {
            return ESP_ERR_NO_MEM;
        }

        pkt.payload = payload;
        err = httpd_ws_recv_frame(req, &pkt, pkt.len);

        free(payload);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Receive frame failed: %s",
                     esp_err_to_name(err));
            return err;
        }
    }

    return ESP_OK;
}


/* 注册 WebSocket 接口 */
static const httpd_uri_t s_ws_uri = {
    .uri          = "/ws",
    .method       = HTTP_GET,
    .handler      = ws_handler,
    .user_ctx     = NULL,
    .is_websocket = true
};


/* 启动 HTTP/WebSocket 服务 */
static esp_err_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    esp_err_t err = httpd_start(&s_server, &config);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed: %s",
                 esp_err_to_name(err));
        s_server = NULL;
        return err;
    }

    err = httpd_register_uri_handler(s_server, &s_ws_uri);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Register WebSocket URI failed: %s",
                 esp_err_to_name(err));

        httpd_stop(s_server);
        s_server = NULL;
        return err;
    }

    ESP_LOGI(TAG, "WebSocket: ws://192.168.4.1/ws");

    return ESP_OK;
}

/* 每 100 ms 推送一次五个舵机的目标角度 */
static void ws_broadcast_task(void *arg)
{
    httpd_handle_t server = (httpd_handle_t)arg;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(WS_PUSH_PERIOD_MS)
        );

        int fd;
        float angles[SERVO_COUNT];

        /* 获取客户端 FD 和目标角度快照 */
        portENTER_CRITICAL(&s_lock);

        fd = s_client_fd;

        for (int i = 0; i < SERVO_COUNT; i++) {
            angles[i] = s_servo_target_angles[i];
        }

        portEXIT_CRITICAL(&s_lock);

        if (fd < 0) {
            continue;
        }

        /* 组装 JSON */
        char json[128];

        int len = snprintf(
            json,
            sizeof(json),
            "{\"j1\":%.1f,\"j2\":%.1f,\"j3\":%.1f,"
            "\"j4\":%.1f,\"j5\":%.1f}",
            (double)angles[0],
            (double)angles[1],
            (double)angles[2],
            (double)angles[3],
            (double)angles[4]
        );

        if (len < 0 || (size_t)len >= sizeof(json)) {
            ESP_LOGW(TAG, "JSON buffer too small");
            continue;
        }

        httpd_ws_frame_t pkt = {0};

        pkt.type = HTTPD_WS_TYPE_TEXT;
        pkt.payload = (uint8_t *)json;
        pkt.len = (size_t)len;

        /* 从独立任务发送 WebSocket 数据 */
        esp_err_t err =
            httpd_ws_send_frame_async(server, fd, &pkt);

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Send failed: %s",
                     esp_err_to_name(err));

            portENTER_CRITICAL(&s_lock);

            if (s_client_fd == fd) {
                s_client_fd = -1;
            }

            portEXIT_CRITICAL(&s_lock);
        }
    }
}


/* 初始化 Wi-Fi AP */
static esp_err_t wifi_init_softap(void)
{
    esp_err_t err = esp_netif_init();

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    if (esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "Create AP netif failed");
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    wifi_config_t wifi_config = {0};

    memcpy(
        wifi_config.ap.ssid,
        WIFI_AP_SSID,
        strlen(WIFI_AP_SSID)
    );

    wifi_config.ap.ssid_len = strlen(WIFI_AP_SSID);

    memcpy(
        wifi_config.ap.password,
        WIFI_AP_PASSWORD,
        strlen(WIFI_AP_PASSWORD)
    );

    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = WIFI_AP_MAX_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Wi-Fi AP started: %s", WIFI_AP_SSID);

    return ESP_OK;
}

esp_err_t websocket_server_init(void)
{
    if (s_server != NULL) {
        return ESP_OK;
    }

    esp_err_t err = wifi_init_softap();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi initialization failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = start_webserver();

    if (err != ESP_OK) {
        return err;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(
        ws_broadcast_task,
        "ws_broadcast",
        4096,
        (void *)s_server,
        1,
        NULL,
        0
    );

    if (ret != pdPASS) {
        httpd_stop(s_server);
        s_server = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
