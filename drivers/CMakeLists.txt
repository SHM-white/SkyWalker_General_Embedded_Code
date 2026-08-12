# SPDX-License-Identifier: Apache-2.0

# Skywalker drivers — add subdirectories as drivers are created
add_subdirectory_ifdef(SKYWALKER_DRIVER_PID pid)
add_subdirectory_ifdef(SKYWALKER_DRIVER_KALMAN_FILTER kalman_filter)
add_subdirectory_ifdef(SKYWALKER_DRIVER_IMU imu)