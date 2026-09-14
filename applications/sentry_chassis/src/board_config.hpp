#pragma once
#include <array>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <robotics/swerve/swerve_chassis.hpp>
namespace board_config {
inline constexpr bool connections_configured = false;
inline constexpr bool require_referee_for_motion = true;
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
#if DT_NODE_HAS_STATUS(DT_ALIAS(steer_fl), okay)
inline const device *steer_fl = DEVICE_DT_GET(DT_ALIAS(steer_fl));
#else
inline const device *steer_fl = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(steer_fr), okay)
inline const device *steer_fr = DEVICE_DT_GET(DT_ALIAS(steer_fr));
#else
inline const device *steer_fr = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(steer_rl), okay)
inline const device *steer_rl = DEVICE_DT_GET(DT_ALIAS(steer_rl));
#else
inline const device *steer_rl = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(steer_rr), okay)
inline const device *steer_rr = DEVICE_DT_GET(DT_ALIAS(steer_rr));
#else
inline const device *steer_rr = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(drive_fl), okay)
inline const device *drive_fl = DEVICE_DT_GET(DT_ALIAS(drive_fl));
#else
inline const device *drive_fl = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(drive_fr), okay)
inline const device *drive_fr = DEVICE_DT_GET(DT_ALIAS(drive_fr));
#else
inline const device *drive_fr = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(drive_rl), okay)
inline const device *drive_rl = DEVICE_DT_GET(DT_ALIAS(drive_rl));
#else
inline const device *drive_rl = nullptr;
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(drive_rr), okay)
inline const device *drive_rr = DEVICE_DT_GET(DT_ALIAS(drive_rr));
#else
inline const device *drive_rr = nullptr;
#endif
inline const std::array<const device *, 8> motors = {steer_fl, steer_fr, steer_rl, steer_rr,
                                                     drive_fl, drive_fr, drive_rl, drive_rr};
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
inline bool emergencyStopRequested() {
    return false;
}
inline bool takeEmergencyResetRequest() {
    return false;
}
}
