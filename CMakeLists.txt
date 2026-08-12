cmake_minimum_required(VERSION 3.20.0)

# ── Module mode (included by Zephyr) ─────────────────────────────────
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS ON)

zephyr_include_directories(include)

add_subdirectory(drivers)
add_subdirectory(lib)
