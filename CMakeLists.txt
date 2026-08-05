cmake_minimum_required(VERSION 3.20.0)

# ── Top-level mode (CLion West) ──────────────────────────────────────
# CLion's West panel uses skywalker_code/ as the source directory.
# In this mode, forward to the hello sample so find_package(Zephyr)
# runs in the correct context.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  get_filename_component(ZEPHYR_BASE "${CMAKE_CURRENT_SOURCE_DIR}/../zephyr" ABSOLUTE)
  set(ZEPHYR_BASE "${ZEPHYR_BASE}" CACHE PATH "Zephyr base")
  set(ENV{ZEPHYR_BASE} "${ZEPHYR_BASE}")
  set(BOARD dm_mc02 CACHE STRING "Board to build for")
  add_subdirectory(samples/hello hello_app)
  return()
endif()

# ── Module mode (included by Zephyr) ─────────────────────────────────
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS ON)

zephyr_include_directories(include)

add_subdirectory(drivers)
add_subdirectory(lib)
