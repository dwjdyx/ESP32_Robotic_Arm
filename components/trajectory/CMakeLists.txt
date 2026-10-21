idf_component_register(
    SRCS "traj_cfg.c" "traj.c"
    INCLUDE_DIRS "."
    REQUIRES robotic_arm manipulate
)