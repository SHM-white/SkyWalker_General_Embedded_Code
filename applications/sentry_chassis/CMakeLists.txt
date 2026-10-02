cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sentry_chassis)
# The common runtime owns topology and one execution writer per board.
# Legacy private executors are retained as historical files, not compiled.
target_sources(app PRIVATE src/main.cpp)
