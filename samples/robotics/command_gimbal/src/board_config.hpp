#pragma once
#include <cstdint>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <control/position_motor.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/dm_motor.hpp>
#include <robotics/gimbal/gimbal_axis.hpp>
#include <robotics/gimbal/gimbal_executor.hpp>
#include <robotics/command/command_manager.hpp>
#include <robotics/vehicle/calibration.hpp>
#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
#include <drivers/imu/dm_imu_rs485.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <robotics/gimbal/inertial_gimbal.hpp>
#endif

namespace board_config {
// Set true only after checking BOTH motors, units, direction, zero and limits.
namespace calibration = skywalker::robotics::vehicle;
inline constexpr bool connections_configured = calibration::connections_confirmed;
inline constexpr std::uint32_t command_timeout_ms = 100;
inline constexpr std::uint16_t yaw_encoder_zero_ticks = calibration::small_yaw.encoder_zero_ticks;
inline constexpr std::uint16_t yaw_travel_low_ticks = calibration::yaw_low_ticks;
inline constexpr std::uint16_t yaw_travel_high_ticks = calibration::yaw_high_ticks;
inline constexpr float dji_encoder_ticks_per_turn = 8192.0f;
inline constexpr float two_pi = 6.2831853071795864769f;
inline constexpr float yaw_min_angle_rad = calibration::yaw_min_rad;
inline constexpr float yaw_max_angle_rad = calibration::yaw_max_rad;
// Independently calibrated joint center for the future big-yaw follower.
// This initial value is the safe travel midpoint, NOT the driver encoder zero.
inline constexpr float yaw_center_rad = calibration::yaw_center_rad;
static_assert(yaw_travel_low_ticks < yaw_travel_high_ticks && yaw_travel_high_ticks < 8192 &&
              yaw_encoder_zero_ticks < 8192);
// These native-shaft driver profiles do not implement an external gearbox.
// TODO(hardware): add an explicit output-shaft transform if either joint has one.
static_assert(calibration::small_yaw.gear_ratio == 1 && calibration::pitch.gear_ratio == 1);
#if DT_NODE_HAS_STATUS(DT_ALIAS(remote_uart), okay)
inline const device *remote_uart = DEVICE_DT_GET(DT_ALIAS(remote_uart));
#else
inline const device *remote_uart = nullptr;
#endif
// Attach BOTH motors before starting either bus. One execution-thread owner
// stages all mechanisms and commits each physical bus once per cycle.
inline const device *yaw_can = DEVICE_DT_GET(DT_NODELABEL(can1));
inline const device *pitch_can = DEVICE_DT_GET(DT_NODELABEL(can2));

// Confirm these settings against the installed motors before enabling them.
inline skywalker::motor::dji::Config yawHardware() {
    return skywalker::motor::dji::gm6020({.id = calibration::small_yaw.id,
                                          .current_limit_a = calibration::small_yaw.effort_limit,
                                          .encoder_zero_ticks = yaw_encoder_zero_ticks,
                                          .current_mode_confirmed = connections_configured,
                                          .timing = {.feedback_timeout_ms = 20,
                                                     .command_timeout_ms = 20,
                                                     .enable_timeout_ms = 100,
                                                     .retry_interval_ms = 100}});
}
inline skywalker::motor::dm::Config pitchHardware() {
    return skywalker::motor::dm::j4310Mit({.id = calibration::pitch.id,
                                           .master_id = calibration::pitch.master_id,
                                           .position_max_rad = 12.5f,
                                           .velocity_max_rad_s = 30.0f,
                                           .torque_max_nm = 10.0f,
                                           .torque_limit_nm = calibration::pitch.effort_limit,
                                           .timing = {.feedback_timeout_ms = 50,
                                                      .command_timeout_ms = 20,
                                                      .enable_timeout_ms = 3000,
                                                      .retry_interval_ms = 100}});
}

// ManualCommandMapper already maps right_x to negative yaw and right_y to
// positive pitch, matching the source gimbal_control calibration. These signs
// map the resulting mechanical command into the calibrated driver coordinates.
inline constexpr float yaw_command_sign = calibration::small_yaw.direction,
                       pitch_command_sign = calibration::pitch.direction;
// Limited axes use calibrated driver coordinates, not a startup-relative zero.
// Yaw and pitch use the coordinates calibrated in gimbal_control.
inline constexpr skywalker::robotics::GimbalAxisConfig yaw{skywalker::robotics::AxisTopology::Limited,
                                                           yaw_min_angle_rad,
                                                           yaw_max_angle_rad,
                                                           calibration::small_yaw.velocity_limit_rad_s,
                                                           true,
                                                           skywalker::robotics::AxisReferenceInit::CalibratedFeedback};
// Pitch range is synchronized with the current gimbal_control bench configuration.
inline constexpr skywalker::robotics::GimbalAxisConfig
    pitch{skywalker::robotics::AxisTopology::Limited,
          calibration::pitch_min_rad,
          calibration::pitch_max_rad,
          calibration::pitch.velocity_limit_rad_s,
          true,
          skywalker::robotics::AxisReferenceInit::CalibratedFeedback};

inline skywalker::control::PositionMotor::Config yawMotorConfig() {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = skywalker::control::EffortUnit::Ampere;
    c.reference = skywalker::control::PositionReference::DriverContinuous;
    c.loop.position = {.kp = 20.0f,
                       .ki = 0.5f,
                       .kd = 1.48f,
                       .derivative_tau_s = 0.0f,
                       .integral_min = -0.3f,
                       .integral_max = 0.3f,
                       .output_min = -4.0f,
                       .output_max = 4.0f,
                       .deadband = 0.012f,
                       .dt_min_s = 0.001f,
                       .dt_max_s = 0.020f};
    c.loop.velocity.regulator.feedback = {.kp = 0.33f,
                                          .ki = 0.55f,
                                          .kd = 0.00005f,
                                          .derivative_tau_s = 0.0f,
                                          .integral_min = -1.2f,
                                          .integral_max = 1.2f,
                                          .output_min = -1.2f,
                                          .output_max = 1.2f,
                                          .deadband = 0.0f,
                                          .dt_min_s = 0.001f,
                                          .dt_max_s = 0.020f};
    c.loop.velocity.reference_slew = {20.0f, 20.0f};
    c.loop.velocity.measurement_filter_tau_s = 0.0f;
    c.loop.velocity.soft_deadband_rad_s = 0.0f;
    c.loop.velocity.requested_velocity_abs_max_rad_s = 4.0f;
    c.loop.velocity.effort_abs_max = 1.2f;
    return c;
}
inline skywalker::control::PositionMotor::Config pitchMotorConfig() {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = skywalker::control::EffortUnit::NewtonMeter;
    c.reference = skywalker::control::PositionReference::DriverContinuous;
    c.loop.position = {.kp = 10.0f,
                       .ki = 0.1f,
                       .kd = 0.0f,
                       .derivative_tau_s = 0.0f,
                       .integral_min = -0.3f,
                       .integral_max = 0.3f,
                       .output_min = -5.0f,
                       .output_max = 5.0f,
                       .deadband = 0.01f,
                       .dt_min_s = 0.001f,
                       .dt_max_s = 0.020f};
    c.loop.velocity.regulator.feedback = {.kp = 0.25f,
                                          .ki = 0.20f,
                                          .kd = 0.0f,
                                          .derivative_tau_s = 0.0f,
                                          .integral_min = -0.7f,
                                          .integral_max = 0.7f,
                                          .output_min = -1.8f,
                                          .output_max = 1.8f,
                                          .deadband = 0.0f,
                                          .dt_min_s = 0.001f,
                                          .dt_max_s = 0.020f};
    c.loop.velocity.reference_slew = {8.0f, 8.0f};
    c.loop.velocity.measurement_filter_tau_s = 0.02f;
    c.loop.velocity.soft_deadband_rad_s = 0.02f;
    c.loop.velocity.requested_velocity_abs_max_rad_s = 5.0f;
    c.loop.velocity.effort_abs_max = 0.8f;
    return c;
}

inline const skywalker::robotics::CommandManager::Config command_policy = [] {
    skywalker::robotics::CommandManager::Config c{};
    c.mapper.input_profile = skywalker::robotics::RemoteInputProfile::PhysicalRemote;
    c.max_gimbal_yaw_rate_rad_s = 1.0f;
    c.max_gimbal_pitch_rate_rad_s = 0.8f;
    c.input_timeout_ms = command_timeout_ms;
    c.require_referee_for_motion = IS_ENABLED(CONFIG_COMMAND_GIMBAL_REFEREE);
    c.allow_auto = IS_ENABLED(CONFIG_COMMAND_GIMBAL_VISION_EXECUTE);
    // TODO(reference): AB carries no epoch metadata; negotiate a new reference
    // session with the vision producer after IMU reset before enabling Auto.
    c.expected_vision_reference = calibration::head_reference;
    return c;
}();
inline const skywalker::robotics::GimbalExecutor::Config execution_policy{
    .command_timeout_ms = command_timeout_ms,
    .source_timeout_us = command_timeout_ms * 1000ULL,
    .permission_timeout_ms = 300,
    .max_cycle_us = 20000,
};

#ifdef CONFIG_COMMAND_GIMBAL_VISION_EXECUTE
inline const device *head_uart = DEVICE_DT_GET(DT_ALIAS(rs485_2));
inline constexpr skywalker::imu::ImuReceiver::Config head_receiver{.poll_interval_us = 1000, .priority = 6};
inline skywalker::imu::DmImuRs485Source::Config headSensor() {
    skywalker::imu::DmImuRs485Source::Config c{};
    c.protocol = {1, 20000};
    c.reference = calibration::head_reference;
    c.sensor_to_body = calibration::head_sensor_to_body;
    // TODO(IMU): confirm quaternion direction/scales and vendor quality policy.
    return c;
}
inline skywalker::robotics::InertialGimbalAdapter::Config inertialPolicy() {
    skywalker::robotics::InertialGimbalAdapter::Config c{};
    c.yaw = yaw;
    c.pitch = pitch;
    c.yaw_direction = yaw_command_sign;
    c.pitch_direction = pitch_command_sign;
    c.pitch_locked = false;
    c.allow_unknown_quality = true;
    // TODO(control): validate inertial_gimbal before opening vision execution.
    return c;
}
#endif

// TODO(hardware): connect physical estop/reset. RC disable is recoverable.
inline bool emergencyStopRequested() {
    return false;
}
inline bool takeEmergencyResetRequest() {
    return false;
}
} // namespace board_config
