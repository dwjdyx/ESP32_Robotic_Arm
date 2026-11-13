#include "manip_config.h"

manip_t manipulator = {
    .target_pos = {10.25f, 0.0f, 342.0f},
    
    .last_joint_theta= {0.0f, -90.0f, -90.0f, 
                        -90.0f, -90.0f, -90.0f},
    .cur_joint_theta = {0.0f, -90.0f, -90.0f, 
                        -90.0f, -90.0f, -90.0f},
};

//映射表，根据实际情况修改offset和dir
const joint_config_t joint_config[JOINT_MAX_NUM] =
{
    [JOINT_1] = {
        .offset = 135.0f,
        .dir = -1.0f
    },

    [JOINT_2] = {
        .offset = 231.0f,
        .dir = 1.0f
    },

    [JOINT_3] = {
        .offset = 45.0f,
        .dir = -1.0f
    },

    [JOINT_4] = {
        .offset = 225.0f,
        .dir = 1.0f
    },

   [JOINT_5] = {
       .offset = 225.0f,
       .dir = 1.0f
   },

   [JOINT_6] = {
        .offset = 225.0f,
        .dir = 1.0f
   }
};
