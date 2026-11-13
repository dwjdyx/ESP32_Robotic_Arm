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
#include "test.h"

void app_main(void)
{
//   system_init();
    // manipulator.target_pos.x = 126.75;
    // manipulator.target_pos.y =  126.75;
    // manipulator.target_pos.z =  105.00;
    
    // if(manip_control(&manipulator))
    // {
    //     printf("JOINT1:%.2f\n",manipulator.joint_theta[0].cur_joint_theta);
    //     printf("JOINT2:%.2f\n",manipulator.joint_theta[1].cur_joint_theta);
    //     printf("JOINT3:%.2f\n",manipulator.joint_theta[2].cur_joint_theta);
    //     printf("JOINT4:%.2f\n",manipulator.joint_theta[3].cur_joint_theta);
    // }else printf("woring");
    manip_FKtest(0.00,-90.0,0.0,-90.0);
// =====================================================

    // xTaskCreatePinnedToCore(key_task, "key_task", 4096, NULL, 5, NULL, 0 );
    // xTaskCreatePinnedToCore(manip_control_task, "manip_control_task", 4096, NULL, 3, NULL, 0 );
    // xTaskCreatePinnedToCore(send_data_task, "send_data_task", 4096, NULL, 1, NULL, 0 );

    
}
    