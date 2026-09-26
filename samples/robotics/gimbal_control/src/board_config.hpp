#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <control/position_motor.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/dm_motor.hpp>
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
// Change pitch_can to can2 for the cross-CAN Group example. Both buses must be
// attached before either is started. CAN transceiver power follows can_start.
inline const device *yaw_can = DEVICE_DT_GET(DT_NODELABEL(can1));
inline const device *pitch_can = DEVICE_DT_GET(DT_NODELABEL(can1));

// Disabled hardware template: match IDs, drive mode, zero and protocol limits
// to the installed motors before setting connections_configured to true.
inline skywalker::motor::dji::Config yawHardware() {
    return skywalker::motor::dji::gm6020({.id = 7,
                                          .current_limit_a = 1.5f,
                                          .encoder_zero_ticks = 0,
                                          .current_mode_confirmed = true,
                                          .timing = {20, 20, 30, 100}});
}
inline skywalker::motor::dm::Config pitchHardware() {
    return skywalker::motor::dm::j4310Mit({.id = 1,
                                           .master_id = 0x00,
                                           .position_max_rad = 12.5f,
                                           .velocity_max_rad_s = 30.0f,
                                           .torque_max_nm = 10.0f,
                                           .torque_limit_nm = 1.0f,
                                           .timing = {50, 20, 50, 3000}});
}

inline constexpr float yaw_direction = -1.0f, pitch_direction = 1.0f;
// Limited axes use calibrated driver coordinates, not a startup-relative zero.
// Replace these example ranges with the actual mechanical limits in radians.
inline constexpr skywalker::robotics::YawGimbalConfig yaw{skywalker::robotics::YawTopology::Limited, -1.0f, 1.0f, 0.3f,
                                                          true};
inline constexpr skywalker::robotics::YawGimbalConfig pitch{skywalker::robotics::YawTopology::Limited, -0.5f, 0.5f,
                                                            0.3f, true};

inline skywalker::control::PositionMotor::Config motorConfig(skywalker::control::EffortUnit unit,
                                                             const skywalker::robotics::YawGimbalConfig &axis) {
    skywalker::control::PositionMotor::Config c{};
    c.effort_unit = unit;
    c.safety = {12, 0};
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
    return motorConfig(skywalker::control::EffortUnit::Ampere, yaw);
}
inline skywalker::control::PositionMotor::Config pitchMotorConfig() {
    // Tune pitch independently here; these defaults are not a loaded-axis tune.
    return motorConfig(skywalker::control::EffortUnit::NewtonMeter, pitch);
}

// Connect physical estop/reset inputs here. RC disable is recoverable, not estop.
inline bool emergencyStopRequested() {
    return false;
}
inline bool takeEmergencyResetRequest() {
    return false;
}
} // namespace board_config
