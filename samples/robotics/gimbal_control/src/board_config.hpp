#pragma once
#include <cstdint>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <control/position_motor.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/dm_motor.hpp>
#include <robotics/gimbal/gimbal_axis.hpp>

namespace board_config {
// Set true only after checking BOTH motors, units, direction, zero and limits.
inline constexpr bool connections_configured = false;
inline constexpr std::uint32_t command_timeout_ms = 100;
inline constexpr std::uint16_t yaw_encoder_zero_ticks = 5670;
inline constexpr std::uint16_t yaw_encoder_min_ticks = 5670;
inline constexpr std::uint16_t yaw_encoder_max_ticks = 7440;
inline constexpr std::uint16_t yaw_travel_margin_ticks = 100;
inline constexpr std::uint16_t yaw_travel_low_ticks = yaw_encoder_min_ticks + yaw_travel_margin_ticks;
inline constexpr std::uint16_t yaw_travel_high_ticks = yaw_encoder_max_ticks - yaw_travel_margin_ticks;
inline constexpr float dji_encoder_ticks_per_turn = 8192.0f;
inline constexpr float two_pi = 6.2831853071795864769f;
inline constexpr float yaw_min_angle_rad = (yaw_travel_low_ticks - yaw_encoder_zero_ticks) * two_pi /
                                           dji_encoder_ticks_per_turn;
inline constexpr float yaw_max_angle_rad = (yaw_travel_high_ticks - yaw_encoder_zero_ticks) * two_pi /
                                           dji_encoder_ticks_per_turn;
static_assert(yaw_encoder_min_ticks <= yaw_encoder_zero_ticks && yaw_encoder_zero_ticks < yaw_encoder_max_ticks &&
              yaw_encoder_max_ticks < 8192 &&
              2 * yaw_travel_margin_ticks < yaw_encoder_max_ticks - yaw_encoder_min_ticks);
#if DT_NODE_HAS_STATUS(DT_ALIAS(remote_uart), okay)
inline const device *remote_uart = DEVICE_DT_GET(DT_ALIAS(remote_uart));
#else
inline const device *remote_uart = nullptr;
#endif
// Change pitch_can to can2 for the cross-CAN Group example. Both buses must be
// attached before either is started. CAN transceiver power follows can_start.
inline const device *yaw_can = DEVICE_DT_GET(DT_NODELABEL(can1));
inline const device *pitch_can = DEVICE_DT_GET(DT_NODELABEL(can1));

// Confirm these settings against the installed motors before enabling them.
inline skywalker::motor::dji::Config yawHardware() {
    return skywalker::motor::dji::gm6020({.id = 7,
                                          .current_limit_a = 1.5f,
                                          .encoder_zero_ticks = yaw_encoder_zero_ticks,
                                          .current_mode_confirmed = true,
                                          .timing = {20, 20, 30, 100}});
}
inline skywalker::motor::dm::Config pitchHardware() {
    return skywalker::motor::dm::j4310Mit({.id = 1,
                                           .master_id = 0x11,
                                           .position_max_rad = 12.5f,
                                           .velocity_max_rad_s = 30.0f,
                                           .torque_max_nm = 10.0f,
                                           .torque_limit_nm = 1.0f,
                                           .timing = {50, 20, 50, 3000}});
}

inline constexpr float yaw_direction = -1.0f, pitch_direction = 1.0f;
// Limited axes use calibrated driver coordinates, not a startup-relative zero.
// Yaw uses the calibrated encoder range; replace the pitch placeholder with measured limits.
inline constexpr skywalker::robotics::GimbalAxisConfig yaw{skywalker::robotics::AxisTopology::Limited,
                                                           yaw_min_angle_rad,
                                                           yaw_max_angle_rad,
                                                           4.0f,
                                                           true,
                                                           skywalker::robotics::AxisReferenceInit::CalibratedFeedback};
// Pitch mechanical limits are still placeholders until measured on the installed gimbal.
inline constexpr skywalker::robotics::GimbalAxisConfig
    pitch{skywalker::robotics::AxisTopology::Limited,
          -0.5f,
          0.5f,
          5.0f,
          true,
          skywalker::robotics::AxisReferenceInit::CalibratedFeedback};

inline skywalker::control::PositionMotor::Config yawMotorConfig() {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = skywalker::control::EffortUnit::Ampere;
    c.safety = {4.0f, 70.0f};
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
    c.loop.velocity.regulator.feedback = {.kp = 0.43f,
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
    c.safety = {10.0f, 60.0f};
    c.reference = skywalker::control::PositionReference::DriverContinuous;
    c.loop.position = {.kp = 0.8f,
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
    c.loop.velocity.regulator.feedback = {.kp = 0.03f,
                                          .ki = 0.1f,
                                          .kd = 0.0f,
                                          .derivative_tau_s = 0.0f,
                                          .integral_min = -0.3f,
                                          .integral_max = 0.3f,
                                          .output_min = -0.5f,
                                          .output_max = 0.5f,
                                          .deadband = 0.0f,
                                          .dt_min_s = 0.001f,
                                          .dt_max_s = 0.020f};
    c.loop.velocity.reference_slew = {2.0f, 2.0f};
    c.loop.velocity.measurement_filter_tau_s = 0.02f;
    c.loop.velocity.soft_deadband_rad_s = 0.02f;
    c.loop.velocity.requested_velocity_abs_max_rad_s = 5.0f;
    c.loop.velocity.effort_abs_max = 0.5f;
    return c;
}

// Connect physical estop/reset inputs here. RC disable is recoverable, not estop.
inline bool emergencyStopRequested() {
    return false;
}
inline bool takeEmergencyResetRequest() {
    return false;
}
} // namespace board_config
