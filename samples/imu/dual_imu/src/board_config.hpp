#pragma once
#include <drivers/imu/bmi088_imu.hpp>
#include <drivers/imu/dm_imu_rs485.hpp>
#include <drivers/imu/imu_heater.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <robotics/vehicle/calibration.hpp>
namespace bench {
// Frame IDs distinguish installations, epochs distinguish reference sessions.
// TODO(IMU): measure the three mounting transforms in the shared calibration.
// Acquisition remains available while imu_mounting_confirmed is false, so this
// observation bench can produce the calibration used by execution samples.
namespace calibration = skywalker::robotics::vehicle;
inline constexpr bool chassis_board = IS_ENABLED(CONFIG_DUAL_IMU_CHASSIS_BOARD);
inline constexpr bool use_external = !chassis_board;
inline constexpr const char *onboard_mount = chassis_board ? "chassis" : "large_yaw_carrier";
inline constexpr const char *external_mount = "pitching_head";
inline constexpr skywalker::imu::ImuReceiver::Config onboard_receiver{.poll_interval_us = 500, .priority = 5};
inline constexpr skywalker::imu::ImuReceiver::Config external_receiver{.poll_interval_us = 1000, .priority = 6};
inline const device *accel = DEVICE_DT_GET(DT_ALIAS(accel0));
inline const device *gyro = DEVICE_DT_GET(DT_ALIAS(gyro0));
inline const device *external_uart = DEVICE_DT_GET(DT_ALIAS(rs485_2));
inline const device *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
inline constexpr skywalker::control::QuaternionEkf::Config estimator{};
inline constexpr skywalker::imu::Bmi088Imu::Config onboard{.reference = chassis_board ? calibration::chassis_reference
                                                                                                   : calibration::carrier_reference,
                                                           .sensor_to_body = chassis_board ? calibration::chassis_sensor_to_body
                                                                                                        : calibration::carrier_sensor_to_body,
                                                           .freshness = {20000, 20000, 20000, 200000}};
inline constexpr skywalker::imu::DmImuRs485Source::Config external{.protocol = {1, 20000},
                                                                   .reference = calibration::head_reference,
                                                                   .sensor_to_body = calibration::head_sensor_to_body,
                                                                   .device_quaternion_is_world_to_sensor = false,
                                                                   .acceleration_scale = 1,
                                                                   .angular_velocity_scale = 1};
inline const skywalker::imu::ImuHeater::Config heater{.pwm = PWM_DT_SPEC_GET(DT_ALIAS(imu_heater)),
                                                      .target_c = 50,
                                                      .maximum_c = 65};
}
