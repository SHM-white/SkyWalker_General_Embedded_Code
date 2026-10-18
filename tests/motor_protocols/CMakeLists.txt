cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor_protocol_contracts LANGUAGES C CXX)
# The pure codecs are compiled directly: no live CAN controller or motor worker.
target_sources(app PRIVATE
  src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/motor/dji/dji_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/motor/dm/dm_protocol.cpp)
