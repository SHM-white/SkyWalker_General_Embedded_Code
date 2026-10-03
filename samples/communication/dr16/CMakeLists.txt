cmake_minimum_required(VERSION 3.20.0)
# This USB telemetry sample requires its alias override even when an IDE build
# has cached a different (or empty) DTC_OVERLAY_FILE selection.
list(APPEND EXTRA_DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/app.overlay")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dr16_bench)
target_sources(app PRIVATE src/main.cpp)
