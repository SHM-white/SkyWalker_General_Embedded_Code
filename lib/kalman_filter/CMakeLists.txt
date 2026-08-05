zephyr_library()

file(GLOB KALMAN_FILTER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
zephyr_library_sources(${KALMAN_FILTER_SOURCES})

zephyr_library_compile_options(
  -O3
  -fno-strict-overflow
  -fno-common
  -ffunction-sections
  -fdata-sections
  -ffreestanding
  -fno-builtin)
