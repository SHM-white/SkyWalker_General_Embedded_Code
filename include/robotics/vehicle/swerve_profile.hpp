#pragma once
#include <algorithm>
#include <robotics/vehicle/calibration.hpp>
#include <robotics/swerve/swerve_chassis.hpp>
namespace skywalker::robotics::vehicle {
// Pure profile: shared by the single module, four wheels and vehicle runtime.
inline constexpr SwerveChassis::Config swerveConfig() {
    SwerveChassis::Config c{};
    const float x = wheelbase_m * 0.5f, y = track_m * 0.5f;
    c.kinematics.locations = {ModuleLocation{x, y}, {x, -y}, {-x, y}, {-x, -y}};
    // One effective linear-speed ceiling: derive it from the wheel radius and the
    // minimum configured wheel-shaft velocity limit. No second 3/10 m/s entry.
    float wheel_shaft_limit = wheel[0].velocity_limit_rad_s;
    for (const auto &w : wheel)
        wheel_shaft_limit = std::min(wheel_shaft_limit, w.velocity_limit_rad_s);
    c.kinematics.max_wheel_velocity_m_s = wheel_radius_m * wheel_shaft_limit;
    c.kinematics.stationary_epsilon_m_s = chassis_zero_speed_epsilon_m_s;
    c.kinematics.resume_velocity_m_s = chassis_resume_speed_m_s;
    for (std::size_t i = 0; i < 4; ++i) {
        auto &m = c.modules[i];
        m.wheel_radius_m = wheel_radius_m;
        // effort_limit is the single per-axis current/torque source.
        m.drive.effort_abs_max = wheel[i].effort_limit;
        m.drive.requested_velocity_abs_max_rad_s = wheel[i].velocity_limit_rad_s;
        const float drive_limit = m.drive.effort_abs_max;
        m.drive.regulator.feedback =
            {drive_rotor_velocity_kp * wheel[i].gear_ratio, 0, 0, 0, 0, 0, -drive_limit, drive_limit, 0, .001f, .02f};
        m.drive.reference_slew = {chassis_drive_slew_rad_s2, chassis_drive_slew_rad_s2};
        m.steer.velocity.effort_abs_max = steer[i].effort_limit;
        m.steer.velocity.requested_velocity_abs_max_rad_s = steer[i].velocity_limit_rad_s;
        const float steer_limit = m.steer.velocity.effort_abs_max;
        const float steer_speed = std::min(chassis_steer_speed_rad_s, steer[i].velocity_limit_rad_s);
        m.steer.velocity.regulator.feedback = {steer_velocity_kp, 0,           0, 0,     0,   0,
                                               -steer_limit,      steer_limit, 0, .001f, .02f};
        m.steer.velocity.reference_slew = {chassis_steer_slew_rad_s2, chassis_steer_slew_rad_s2};
        m.steer.position = {steer_position_kp, 0, 0, 0, 0, 0, -steer_speed, steer_speed, 0, .001f, .02f};
        m.steer_target_rate_rad_s = steer_speed;
        m.idle_behavior = SwerveModule::IdleBehavior::Hold;
    }
    return c;
}
static_assert(swerveConfig().kinematics.max_wheel_velocity_m_s > 0);
}
