cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(swerve_bench)
# Temporary dual-speed diagnostic; select src/main.cpp to restore swerve control.
target_sources(app PRIVATE src/dual_speed.cpp)
