cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(protocol_contracts LANGUAGES C CXX)
target_sources(app PRIVATE src/remote.cpp src/referee.cpp src/interboard.cpp src/wire.cpp)
