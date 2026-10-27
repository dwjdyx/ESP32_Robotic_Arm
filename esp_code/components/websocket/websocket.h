#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include "esp_err.h"

esp_err_t websocket_server_init(void);
void ws_set_servo_angles(const float angles[5]);

#endif