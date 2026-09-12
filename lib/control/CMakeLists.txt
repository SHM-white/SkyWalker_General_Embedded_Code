zephyr_library()

zephyr_library_sources(
    pid.c
    feedforward.c
    feedforward_pid.c
    slew_rate_limiter.c
    angle.c
    motor_velocity.c
    motor_position.c
)

if(CONFIG_SKYWALKER_LIB_MOTOR_CONTROL)
    zephyr_library_sources(motor_control.cpp)
    zephyr_library_sources_ifdef(CONFIG_SKYWALKER_MOTOR_DJI dji_motor_backend.cpp)
    zephyr_library_sources_ifdef(CONFIG_SKYWALKER_MOTOR_DM dm_motor_backend.cpp)
endif()
