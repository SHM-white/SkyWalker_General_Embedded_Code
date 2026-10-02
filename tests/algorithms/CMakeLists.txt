cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(algorithm_units LANGUAGES C CXX)

target_sources(app PRIVATE
  src/attitude_ekf.cpp
  src/core_attitude.cpp
  src/control.cpp
  src/kalman_matrix.cpp
  src/robotics.cpp
)
include(${CMAKE_CURRENT_SOURCE_DIR}/../common/benchmark.cmake)
