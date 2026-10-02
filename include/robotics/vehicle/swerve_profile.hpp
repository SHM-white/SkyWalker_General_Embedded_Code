#pragma once
#include <algorithm>
#include <robotics/vehicle/calibration.hpp>
#include <robotics/swerve/swerve_chassis.hpp>
namespace skywalker::robotics::vehicle {
// Pure profile: shared by the single module, four wheels and vehicle runtime.
inline SwerveChassis::Config swerveConfig() {
    SwerveChassis::Config c{};
    const float x = wheelbase_m * 0.5f, y = track_m * 0.5f;
    c.kinematics.locations = {ModuleLocation{x, y}, {x, -y}, {-x, y}, {-x, -y}};
    c.kinematics.max_wheel_velocity_m_s = 0.3f;
    c.kinematics.stationary_epsilon_m_s = 0.01f;
    c.kinematics.resume_velocity_m_s = 0.02f;
    c.require_all_modules_aligned = true;
    for (std::size_t i = 0; i < 4; ++i) {
        auto &m = c.modules[i];
        m.wheel_radius_m = wheel_radius_m;
        m.drive.effort_abs_max = std::min(drive_bench_current_a, wheel[i].effort_limit);
        m.drive.requested_velocity_abs_max_rad_s = wheel[i].velocity_limit_rad_s;
        c.kinematics.max_wheel_velocity_m_s = std::min(c.kinematics.max_wheel_velocity_m_s,
            wheel_radius_m * m.drive.requested_velocity_abs_max_rad_s);
        const float drive_limit = m.drive.effort_abs_max;
        m.drive.regulator.feedback = {drive_rotor_velocity_kp * wheel[i].gear_ratio, 0, 0, 0,
            0, 0, -drive_limit, drive_limit, 0, .001f, .02f};
        m.drive.reference_slew = {10, 10};
        m.steer.velocity.effort_abs_max = std::min(steer_bench_current_a, steer[i].effort_limit);
        m.steer.velocity.requested_velocity_abs_max_rad_s = steer[i].velocity_limit_rad_s;
        const float steer_limit = m.steer.velocity.effort_abs_max;
        const float steer_speed = std::min(4.0f, steer[i].velocity_limit_rad_s);
        m.steer.velocity.regulator.feedback = {steer_velocity_kp, 0, 0, 0,
            0, 0, -steer_limit, steer_limit, 0, .001f, .02f};
        m.steer.velocity.reference_slew = {100, 100};
        m.steer.position = {steer_position_kp, 0, 0, 0, 0, 0, -steer_speed, steer_speed, 0, .001f, .02f};
        m.steer_target_rate_rad_s = steer_speed;
        m.idle_behavior = SwerveModule::IdleBehavior::Coast;
    }
    return c;
}
}
