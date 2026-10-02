#pragma once

#include "../../command_gimbal/src/board_config.hpp"
#include <drivers/imu/dm_imu_rs485.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <robotics/gimbal/inertial_gimbal.hpp>
#include <robotics/vehicle/calibration.hpp>

namespace inertial_bench {
inline const device *head_uart = DEVICE_DT_GET(DT_ALIAS(rs485_2));
inline constexpr bool mounting_configured = skywalker::robotics::vehicle::imu_mounting_confirmed;
inline constexpr skywalker::imu::ImuReceiver::Config head_receiver{.poll_interval_us = 1000, .priority = 6};

inline skywalker::imu::DmImuRs485Source::Config head_sensor() {
    skywalker::imu::DmImuRs485Source::Config c{};
    c.protocol = {1, 20000};
    c.reference = skywalker::robotics::vehicle::head_reference;
    c.sensor_to_body = skywalker::robotics::vehicle::head_sensor_to_body;
    // TODO(IMU): confirm module quaternion direction and scales with dual_imu.
    c.device_quaternion_is_world_to_sensor = false;
    c.acceleration_scale = c.angular_velocity_scale = 1;
    return c;
}

inline skywalker::robotics::InertialGimbalAdapter::Config controller() {
    skywalker::robotics::InertialGimbalAdapter::Config c{};
    c.yaw = board_config::yaw;
    c.pitch = board_config::pitch;
    c.yaw_direction = skywalker::robotics::vehicle::small_yaw.direction;
    c.pitch_direction = skywalker::robotics::vehicle::pitch.direction;
    c.pitch_locked = IS_ENABLED(CONFIG_INERTIAL_GIMBAL_LOCK_PITCH);
    // DM packets expose vendor attitude without estimator quality metadata.
    // TODO(IMU): establish allowed module quality and drop-out behavior on board.
    c.allow_unknown_quality = true;
    // TODO(control): tune carrier compensation, error thresholds and limits.
    c.yaw_kp = c.pitch_kp = 3;
    return c;
}
} // namespace inertial_bench
