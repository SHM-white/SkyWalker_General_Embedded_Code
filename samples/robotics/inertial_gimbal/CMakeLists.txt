cmake_minimum_required(VERSION 3.20.0)
set(CONF_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/vehicle/installed/base.conf"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/vehicle/installed/gimbal.conf"
    "${CMAKE_CURRENT_SOURCE_DIR}/prj.conf")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(inertial_gimbal)
target_sources(app PRIVATE src/main.cpp)
