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
endif()
