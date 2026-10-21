set(src_dirs
            TRAJ
            MANIP
            SERVO
            TEST)

set(include_dirs
            TRAJ
            MANIP
            SERVO
            TEST)

set(requires
            driver
            bt)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)