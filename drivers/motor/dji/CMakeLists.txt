zephyr_library()

zephyr_library_sources(
    dji_tx_group.cpp
    dji_protocol.cpp
    dji_m3508.cpp
)

# zephyr_library_compile_options(
#   -O3
#   -fno-strict-overflow
#   -fno-common
#   -ffunction-sections
#   -fdata-sections
#   -ffreestanding
#   -fno-builtin)