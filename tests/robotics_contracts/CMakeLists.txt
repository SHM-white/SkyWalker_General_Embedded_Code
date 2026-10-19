cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(robotics_contracts LANGUAGES C CXX)
target_sources(app PRIVATE src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/robotics/yaw_centering.cpp)
