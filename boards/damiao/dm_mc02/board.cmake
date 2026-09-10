# SPDX-License-Identifier: Apache-2.0

set(SKYWALKER_OPENOCD_PROBE "cmsis-dap" CACHE STRING
    "OpenOCD probe: cmsis-dap, stlink, or stlink-hla")
set_property(CACHE SKYWALKER_OPENOCD_PROBE PROPERTY STRINGS
             "cmsis-dap" "stlink" "stlink-hla")

if(SKYWALKER_OPENOCD_PROBE STREQUAL "cmsis-dap")
  set(SKYWALKER_OPENOCD_CONFIG "openocd.cfg")
elseif(SKYWALKER_OPENOCD_PROBE STREQUAL "stlink")
  set(SKYWALKER_OPENOCD_CONFIG "openocd_stlink.cfg")
elseif(SKYWALKER_OPENOCD_PROBE STREQUAL "stlink-hla")
  set(SKYWALKER_OPENOCD_CONFIG "openocd_stlink_hla.cfg")
else()
  message(FATAL_ERROR
          "Unsupported SKYWALKER_OPENOCD_PROBE='${SKYWALKER_OPENOCD_PROBE}'. "
          "Choose cmsis-dap, stlink, or stlink-hla.")
endif()

# keep first — default runner
board_runner_args(openocd
  "--config=${BOARD_DIR}/support/${SKYWALKER_OPENOCD_CONFIG}")
board_runner_args(pyocd "--target=stm32h723vgtx")
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=sw")

board_runner_args(stlink_gdbserver)
board_runner_args(jlink "--device=STM32H723VG" "--speed=12000")

# keep first
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stlink_gdbserver.board.cmake)

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
