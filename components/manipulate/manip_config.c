#include "manip_config.h"

manip_t manipulator = {
    .target_pos = {0.0f, 0.0f, 131.0f},
    .angle = {
        [JOINT_1] = {
            .joint_angle = 0.0f,
            .last_joint_angle = 0.0f,
            .servo_angle = 0.0f
        },
        [JOINT_2] = {
            .joint_angle = 0.0f,
            .last_joint_angle = 0.0f,
            .servo_angle = 0.0f
        },
        [JOINT_3] = {
            .joint_angle = 0.0f,
            .last_joint_angle = 0.0f,
            .servo_angle = 0.0f
        },
        [JOINT_4] = {
            .joint_angle = 135.0f,
            .last_joint_angle = 135.0f,
            .servo_angle = 135.0f
        },
       [JOINT_5] = {
           .joint_angle = 0.0f,
           .last_joint_angle = 0.0f,
           .servo_angle = 0.0f
       },
    }
};

//映射表，根据实际情况修改offset和dir
const joint_config_t joint_config[JOINT_MAX_NUM] =
{
    [JOINT_1] = {
        .offset = 90.0f,
        .dir = 1.0f,
        .servo_id = SERVO_BASE
    },

    [JOINT_2] = {
        .offset = 90.0f,
        .dir = 1.0f,
        .servo_id = SERVO_SHOULDER
    },

    [JOINT_3] = {
        .offset = 90.0f,
        .dir = 1.0f,
        .servo_id = SERVO_ELBOW
    },

    [JOINT_4] = {
        .offset = 0.0f,
        .dir = 1.0f,
        .servo_id = SERVO_WRIST
    },

   [JOINT_5] = {
       .offset = 90.0f,
       .dir = 1.0f,
       .servo_id = SERVO_GRIPPER
   }
};
