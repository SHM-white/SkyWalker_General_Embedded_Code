#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <control/position_motor.hpp>
#include <robotics/gimbal/yaw_gimbal.hpp>

namespace board_config {
// Set true only after checking BOTH motors, units, direction, zero and limits.
inline constexpr bool connections_configured = false;
inline constexpr std::uint32_t command_timeout_ms = 100;
#if DT_NODE_HAS_STATUS(DT_ALIAS(remote_uart), okay)
inline const device *remote_uart = DEVICE_DT_GET(DT_ALIAS(remote_uart));
#else
inline const device *remote_uart = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(yaw_motor), okay)
inline const device *yaw_motor = DEVICE_DT_GET(DT_ALIAS(yaw_motor));
#else
inline const device *yaw_motor = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(pitch_motor), okay)
inline const device *pitch_motor = DEVICE_DT_GET(DT_ALIAS(pitch_motor));
#else
inline const device *pitch_motor = nullptr;
#endif

// Templates only. DM requires a matching MIT-mode device node and torque limits.
// Two independent DJI backends MUST use different CAN controllers.
inline constexpr bool yaw_is_dm = false, pitch_is_dm = false;
inline constexpr float yaw_direction = -1.0f, pitch_direction = 1.0f;
// Limited axes use calibrated driver coordinates, not a startup-relative zero.
// Replace these example ranges with the actual mechanical limits in radians.
inline constexpr skywalker::robotics::YawGimbalConfig yaw{
    skywalker::robotics::YawTopology::Limited, -1.0f, 1.0f, 0.3f, true};
inline constexpr skywalker::robotics::YawGimbalConfig pitch{
    skywalker::robotics::YawTopology::Limited, -0.5f, 0.5f, 0.3f, true};

inline skywalker::control::PositionMotor::Config motorConfig(
    bool is_dm, const skywalker::robotics::YawGimbalConfig &axis) {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = is_dm ? skywalker::control::EffortUnit::NewtonMeter : skywalker::control::EffortUnit::Ampere;
    c.safety = {12, 0, 30, 100};
    c.reference = axis.topology == skywalker::robotics::YawTopology::Continuous
                      ? skywalker::control::PositionReference::AbsoluteNearest
                      : skywalker::control::PositionReference::DriverContinuous;
    c.loop.position = {3, 0, 0, 0, -3, 3, -6, 6, 0.01f, 0.001f, 0.02f};
    c.loop.velocity.regulator.feedback = {0.03f, 0.1f, 0, 0, -0.3f, 0.3f, -0.3f, 0.3f, 0, 0.001f, 0.02f};
    c.loop.velocity.reference_slew = {10, 10};
    c.loop.velocity.requested_velocity_abs_max_rad_s = 6;
    c.loop.velocity.effort_abs_max = 0.3f;
    return c;
}
inline skywalker::control::PositionMotor::Config yawMotorConfig() {
    return motorConfig(yaw_is_dm, yaw);
}
inline skywalker::control::PositionMotor::Config pitchMotorConfig() {
    // Tune pitch independently here; these defaults are not a loaded-axis tune.
    return motorConfig(pitch_is_dm, pitch);
}

// Connect physical estop/reset inputs here. RC disable is recoverable, not estop.
inline bool emergencyStopRequested() {
    return false;
}
inline bool takeEmergencyResetRequest() {
    return false;
}
} // namespace board_config
