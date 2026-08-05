# SPDX-License-Identifier: Apache-2.0

# keep first — default runner
board_runner_args(pyocd "--target=stm32h723vgtx")
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=sw")
board_runner_args(openocd)

board_runner_args(stlink_gdbserver)
board_runner_args(jlink "--device=STM32H723VG" "--speed=12000")

# keep first
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stlink_gdbserver.board.cmake)

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
