# SPDX-License-Identifier: Apache-2.0

# RM Type-C 板无板载调试器，默认经 OpenOCD 使用外接调试器（ST-Link/CMSIS-DAP）烧录。
# 对应接口配置见 support/openocd.cfg。
board_runner_args(openocd)

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
