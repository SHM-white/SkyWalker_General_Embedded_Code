cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(m3508_can1_speed_control)

target_sources(app PRIVATE src/main.cpp)
