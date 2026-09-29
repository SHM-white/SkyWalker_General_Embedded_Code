#pragma once
#include <drivers/imu/bmi088_imu.hpp>
#include <drivers/imu/dm_imu_rs485.hpp>
#include <drivers/imu/imu_heater.hpp>
#include <drivers/imu/imu_receiver.hpp>
namespace bench {
inline constexpr skywalker::imu::ImuReceiver::Config onboard_receiver{.poll_interval_us = 500, .priority = 5};
inline constexpr skywalker::imu::ImuReceiver::Config external_receiver{.poll_interval_us = 1000, .priority = 6};
inline const device *accel = DEVICE_DT_GET(DT_NODELABEL(bmi08x_accel));
inline const device *gyro = DEVICE_DT_GET(DT_NODELABEL(bmi08x_gyro));
inline const device *external_uart = DEVICE_DT_GET(DT_ALIAS(rs485_2));
inline const device *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
inline constexpr skywalker::control::QuaternionEkf::Config estimator{};
inline constexpr skywalker::imu::Bmi088Imu::Config onboard{.reference = {1, 1},
                                                           .sensor_to_body = {},
                                                           .freshness = {20000, 20000, 20000, 200000}};
inline constexpr skywalker::imu::DmImuRs485Source::Config external{.protocol = {1, 20000},
                                                                   .reference = {2, 1},
                                                                   .sensor_to_body = {},
                                                                   .device_quaternion_is_world_to_sensor = false,
                                                                   .acceleration_scale = 1,
                                                                   .angular_velocity_scale = 1};
inline const skywalker::imu::ImuHeater::Config heater{.pwm = PWM_DT_SPEC_GET(DT_NODELABEL(imu_heater)),
                                                      .target_c = 50,
                                                      .maximum_c = 65};
}
