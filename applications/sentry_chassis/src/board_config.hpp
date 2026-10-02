// TODO(legacy): historical private implementation. The active entry now uses
// samples/robotics/common/vehicle_bench.hpp and central vehicle calibration.
#pragma once

#include <array>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <robotics/swerve/swerve_chassis.hpp>
#include <communication/interboard/configured_interboard_transport.hpp>
#include "chassis_hardware.hpp"

namespace board_config {

inline constexpr bool connections_configured = false;
inline constexpr bool require_power_budget = true;
inline constexpr std::uint32_t command_timeout_ms = 100, heartbeat_timeout_ms = 100, permission_timeout_ms = 300,
                               feedback_stable_ms = 30, recovery_retry_ms = 100;
// The estimator is disabled until calibrated with this robot's power measurements.
inline constexpr bool power_model_calibrated = false;
inline constexpr float idle_power_w = 0, power_per_abs_amp_w = 0, bench_effort_scale = 0.15f;
inline constexpr float velocity_safety_rad_s = 40, temperature_limit_c = 70;
inline constexpr std::array<float, 8> motor_direction = {1, 1, 1, 1, 1, 1, 1, 1}; // steer 4, drive 4

#if DT_NODE_HAS_STATUS(DT_ALIAS(interboard_uart), okay)
inline const device *interboard_uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart));
#else
inline const device *interboard_uart = nullptr;
#endif
// Change only kind to select the physical link; both boards must agree.
inline const skywalker::communication::ConfiguredInterBoardTransport::Config interboard_transport = [] {
    using namespace skywalker::communication;
    ConfiguredInterBoardTransport::Config c{};
    c.kind = InterBoardTransportKind::Uart;
    c.uart = interboard_uart;
#if DT_NODE_HAS_STATUS(DT_ALIAS(interboard_rs485), okay)
    c.rs485.uart = DEVICE_DT_GET(DT_ALIAS(interboard_rs485));
#endif
    c.rs485.role = Rs485InterBoardTransport::Role::Responder;
#if DT_NODE_HAS_STATUS(DT_ALIAS(interboard_can), okay)
    c.can.can = DEVICE_DT_GET(DT_ALIAS(interboard_can));
#endif
    c.can.tx_id = 0x601;
    c.can.rx_id = 0x600;
    return c;
}();
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
inline const device *can1 = DEVICE_DT_GET(DT_NODELABEL(can1));
#else
inline const device *can1 = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can2), okay)
inline const device *can2 = DEVICE_DT_GET(DT_NODELABEL(can2));
#else
inline const device *can2 = nullptr;
#endif

// Original wiring: all eight motors on CAN1. Set drive_can to can2 when the
// four M3508 are physically on CAN2. Confirm wiring before opening the gate.
inline const device *steer_can = can1;
inline const device *drive_can = can1;

inline skywalker::motor::dji::Config steerConfig(std::uint8_t id) {
    return skywalker::motor::dji::gm6020({
        .id = id,
        .current_limit_a = 1.5f,
        .encoder_zero_ticks = 0,
        .current_mode_confirmed = false,
        .timing = {20, 20, 30, 100},
    });
}

inline skywalker::motor::dji::Config driveConfig(std::uint8_t id) {
    return skywalker::motor::dji::m3508({
        .id = id,
        .current_limit_a = 1.5f,
        .gear_ratio = 3591.0f / 187.0f,
        .timing = {20, 20, 30, 100},
    });
}

// Order: steer FL, FR, RL, RR; drive FL, FR, RL, RR. All four GM6020
// current-control modes and encoder zeroes must be confirmed on the real robot.
inline const std::array<ChassisMotorConnection, 8> motors{{
    {steer_can, steerConfig(1)},
    {steer_can, steerConfig(2)},
    {steer_can, steerConfig(3)},
    {steer_can, steerConfig(4)},
    {drive_can, driveConfig(1)},
    {drive_can, driveConfig(2)},
    {drive_can, driveConfig(3)},
    {drive_can, driveConfig(4)},
}};

inline skywalker::robotics::SwerveChassis::Config chassisConfig() {
    using namespace skywalker::robotics;
    SwerveChassis::Config c{};
    c.kinematics.locations = {ModuleLocation{0.2f, 0.2f}, {0.2f, -0.2f}, {-0.2f, 0.2f}, {-0.2f, -0.2f}};
    c.kinematics.max_wheel_velocity_m_s = 0.5f;
    for (auto &m : c.modules) {
        m.wheel_radius_m = 0.05f;
        m.drive.regulator.feedback = {0.03f, 0.1f, 0, 0, -0.3f, 0.3f, -0.3f, 0.3f, 0, 0.001f, 0.02f};
        m.drive.reference_slew = {10, 10};
        m.drive.requested_velocity_abs_max_rad_s = 10;
        m.drive.effort_abs_max = 0.3f;
        m.steer.velocity = m.drive;
        m.steer.position = {3, 0, 0, 0, -10, 10, -10, 10, 0.01f, 0.001f, 0.02f};
    }
    return c;
}

} // namespace board_config
