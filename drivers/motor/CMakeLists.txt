zephyr_library()
zephyr_library_sources(can_bus.cpp motor.cpp group.cpp dji/dji_protocol.cpp dm/dm_protocol.cpp)

add_subdirectory_ifdef(CONFIG_SKYWALKER_MOTOR_DJI dji)
add_subdirectory_ifdef(CONFIG_SKYWALKER_MOTOR_DM dm)
# add_subdirectory_ifdef(CONFIG_SKYWALKER_MOTOR_LK lk)
