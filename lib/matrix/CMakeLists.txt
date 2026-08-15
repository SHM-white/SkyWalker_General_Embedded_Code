zephyr_library()

if(CONFIG_SKYWALKER_LIB_MATRIX_STORAGE)
  zephyr_library_sources(matrix.c)
endif()

zephyr_library_compile_options(
  -O3
  -fno-strict-overflow
  -fno-common
  -ffunction-sections
  -fdata-sections
  -ffreestanding
  -fno-builtin)