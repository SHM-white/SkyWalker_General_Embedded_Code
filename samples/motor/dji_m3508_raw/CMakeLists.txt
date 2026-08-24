cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dji_m3508_raw)

target_sources(app PRIVATE src/main.cpp 
    src/dji_m3508_protocol.cpp
)