/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "stdint.h"

#include "manip.h"
#include "traj.h"
#include "test.h"

void app_main(void)
{
//   system_init();
    manipulator.target_pos.x = 7.25f;
    manipulator.target_pos.y = 7.25f;
    manipulator.target_pos.z = 274.0f;

    manip_control_task(&manipulator);
    printf("%.2f %.2f %.2f\n",manipulator.angle[JOINT_1].joint_angle, manipulator.angle[JOINT_2].joint_angle,
                                manipulator.angle[JOINT_2].joint_angle);
// =====================================================

//    xTaskCreatePinnedToCore(traj_task, "traj_task", 4096, NULL, 1, NULL, 0 );
}
    