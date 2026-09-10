# SPDX-License-Identifier: Apache-2.0

# RM Type-C 板无板载调试器，默认经 OpenOCD 使用外接调试器烧录。
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

board_runner_args(openocd
  "--config=${BOARD_DIR}/support/${SKYWALKER_OPENOCD_CONFIG}")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
