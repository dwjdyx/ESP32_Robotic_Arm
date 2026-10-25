idf_component_register(SRCS "hello_world_main.c"
                    INCLUDE_DIRS ""
                    REQUIRES 
                        trajectory 
                        robotic_arm 
                        test_code
                        manipulate
                        )
