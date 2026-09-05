zephyr_library()

zephyr_library_sources(
    dji_protocol.cpp
    dji_profiles.cpp
    dji_motor.cpp
    dji_bus.cpp
    dji_m3508_instance.cpp
    dji_m2006_instance.cpp
    dji_gm6020_instance.cpp
)

# zephyr_library_compile_options(
#   -O3
#   -fno-strict-overflow
#   -fno-common
#   -ffunction-sections
#   -fdata-sections
#   -ffreestanding
#   -fno-builtin)