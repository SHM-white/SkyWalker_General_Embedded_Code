#pragma once

#include <algorithm>
#include <robotics/vehicle/chassis_profile.hpp>

namespace skywalker::robotics::vehicle::bringup {
// Stage limits only. Installed IDs, geometry, encoder zeros and signs remain
// in the installed profiles; no stage overrides physical calibration.
inline constexpr float chassis_speed_m_s = 5.0f;
inline constexpr float chassis_yaw_rate_rad_s = 5.0f;
inline constexpr float steer_current_a = 3.0f, drive_current_a = 5.0f;
inline constexpr std::uint32_t command_timeout_ms = 100;

inline SwerveChassis::Config chassisConfig() {
    auto config = swerveConfig();
    config.kinematics.max_wheel_velocity_m_s = std::min(config.kinematics.max_wheel_velocity_m_s, chassis_speed_m_s);
    for (auto &module : config.modules) {
        auto &steer = module.steer.velocity;
        steer.effort_abs_max = std::min(steer.effort_abs_max, steer_current_a);
        steer.regulator.feedback.output_min = std::max(steer.regulator.feedback.output_min, -steer_current_a);
        steer.regulator.feedback.output_max = std::min(steer.regulator.feedback.output_max, steer_current_a);
        auto &drive = module.drive;
        drive.effort_abs_max = std::min(drive.effort_abs_max, drive_current_a);
        drive.regulator.feedback.output_min = std::max(drive.regulator.feedback.output_min, -drive_current_a);
        drive.regulator.feedback.output_max = std::min(drive.regulator.feedback.output_max, drive_current_a);
    }
    return config;
}

inline ChassisExecutor::Config executionConfig() {
    return chassisExecutionConfig();
}

inline ChassisCommand boundedCommand(const ChassisCommand &input) {
    auto output = input;
    output.vx_m_s = std::clamp(output.vx_m_s, -chassis_speed_m_s, chassis_speed_m_s);
    output.vy_m_s = std::clamp(output.vy_m_s, -chassis_speed_m_s, chassis_speed_m_s);
    output.wz_rad_s = std::clamp(output.wz_rad_s, -chassis_yaw_rate_rad_s, chassis_yaw_rate_rad_s);
    return output;
}
} // namespace skywalker::robotics::vehicle::bringup
