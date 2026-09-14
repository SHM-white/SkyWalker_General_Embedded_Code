cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(dm_mit_position_control)

target_sources(app PRIVATE
    src/main.cpp
    src/dm_sample_support.cpp
)
target_include_directories(app PRIVATE include)
