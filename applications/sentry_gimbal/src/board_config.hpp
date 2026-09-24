#pragma once
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <control/position_motor.hpp>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/dm_motor.hpp>
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
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
inline const device *yaw_can = DEVICE_DT_GET(DT_NODELABEL(can1));
#else
inline const device *yaw_can = nullptr;
#endif
// This one factory is the active yaw configuration. Confirm the physical CAN ID,
// zero and current control mode before setting connections_configured to true.
inline auto yawMotorConfig() {
    return skywalker::motor::dji::gm6020({
        .id = 1,
        .current_limit_a = 1.5f,
        .encoder_zero_ticks = 0,
        .current_mode_confirmed = false,
        .timing = {20, 20, 30, 100},
    });
    // For a DM J4310 MIT yaw, replace the return above with:
    // return skywalker::motor::dm::j4310Mit({.id = <motor ID>,
    //     .master_id = <feedback CAN ID>, .position_max_rad = <drive range>,
    //     .velocity_max_rad_s = <drive range>, .torque_max_nm = <drive range>,
    //     .torque_limit_nm = <application limit>, .timing = {50, 20, 50, 3000}});
    // Also select a mechanically valid Limited yaw topology below: DM currently
    // does not provide the fixed-zero absolute angle required by Continuous yaw.
    // Recheck the PID gains and effort cap in NewtonMeter before enabling motion.
}

inline constexpr skywalker::robotics::YawGimbalConfig yaw{skywalker::robotics::YawTopology::Continuous, -3.14159265f,
                                                          3.14159265f, 1.0f, true};
inline skywalker::control::PositionMotor::Config motorConfig(const skywalker::motor::MotorInfo &info) {
    skywalker::control::PositionMotor::Config c{};
    if ((info.capabilities & skywalker::motor::CommandCurrent) != 0u)
        c.effort_unit = skywalker::control::EffortUnit::Ampere;
    else if ((info.capabilities & skywalker::motor::CommandTorque) != 0u)
        c.effort_unit = skywalker::control::EffortUnit::NewtonMeter;
    c.safety = {12, 0};
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
