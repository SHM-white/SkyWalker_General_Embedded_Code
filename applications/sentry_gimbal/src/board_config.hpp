#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <control/position_motor.hpp>
#include <robotics/gimbal/yaw_gimbal.hpp>
#include <communication/referee/referee_protocol.hpp>
namespace board_config {
// Edit this application only; samples and the chassis app have their own configuration.
inline constexpr bool connections_configured = false;
inline constexpr bool require_referee_for_motion = true;
inline constexpr auto referee_version = skywalker::communication::RefereeVersion::Unspecified;
inline constexpr std::uint32_t permission_timeout_ms = 300, command_timeout_ms = 100,
                               chassis_heartbeat_timeout_ms = 100, chassis_feedback_timeout_ms = 100;
#if DT_NODE_HAS_STATUS(DT_ALIAS(interboard_uart), okay)
inline const device *interboard_uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart));
#else
inline const device *interboard_uart = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(remote_uart), okay)
inline const device *remote_uart = DEVICE_DT_GET(DT_ALIAS(remote_uart));
#else
inline const device *remote_uart = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(referee_uart), okay)
inline const device *referee_uart = DEVICE_DT_GET(DT_ALIAS(referee_uart));
#else
inline const device *referee_uart = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(yaw_motor), okay)
inline const device *yaw_motor = DEVICE_DT_GET(DT_ALIAS(yaw_motor));
#else
inline const device *yaw_motor = nullptr;
#endif
// Backend choice, effort units and position capability must match the real motor.
inline constexpr bool yaw_is_dm = false;
inline constexpr skywalker::robotics::YawGimbalConfig yaw{skywalker::robotics::YawTopology::Continuous, -3.14159265f,
                                                          3.14159265f, 1.0f, true};
inline skywalker::control::PositionMotor::Config motorConfig() {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = yaw_is_dm ? skywalker::control::EffortUnit::NewtonMeter : skywalker::control::EffortUnit::Ampere;
    c.safety = {12, 0, 30, 100};
    c.reference = yaw.topology == skywalker::robotics::YawTopology::Continuous
                      ? skywalker::control::PositionReference::AbsoluteNearest
                      : skywalker::control::PositionReference::DriverContinuous;
    c.loop.position = {3, 0, 0, 0, -3, 3, -6, 6, 0.01f, 0.001f, 0.02f};
    c.loop.velocity.regulator.feedback = {0.03f, 0.1f, 0, 0, -0.3f, 0.3f, -0.3f, 0.3f, 0, 0.001f, 0.02f};
    c.loop.velocity.reference_slew = {10, 10};
    c.loop.velocity.requested_velocity_abs_max_rad_s = 6;
    c.loop.velocity.effort_abs_max = 0.3f;
    return c;
}
// Wire a physical estop/reset input here. Safe on the RC is a recoverable disable.
inline bool emergencyStopRequested() {
    return false;
}
inline bool takeEmergencyResetRequest() {
    return false;
}
}
