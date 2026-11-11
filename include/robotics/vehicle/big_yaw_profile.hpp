#pragma once
#include <robotics/execution/big_yaw_executor.hpp>
#include <robotics/execution/big_yaw_heading_follower.hpp>
#include <robotics/vehicle/calibration.hpp>
#include <robotics/vehicle/motion_profile.hpp>
namespace skywalker::robotics::vehicle {
inline motor::dm::Config bigYawHardware() {
    // TODO(hardware): confirm MIT mode and PMAX/VMAX/TMAX stored in the drive.
    const auto &cal = big_yaw_native_velocity_profile_confirmed ? reference_big_yaw : big_yaw;
    const motor::dm::J4310Options options{.id = cal.id,
                                          .master_id = cal.master_id,
                                          .position_max_rad = 12.5f,
                                          .velocity_max_rad_s = 30,
                                          .torque_max_nm = 10,
                                          .torque_limit_nm = cal.effort_limit,
                                          .timing = {.feedback_timeout_ms = 50,
                                                     .command_timeout_ms = 20,
                                                     .enable_timeout_ms = 3000,
                                                     .retry_interval_ms = 100}};
    return big_yaw_native_velocity_profile_confirmed ? motor::dm::j4310Velocity(options) : motor::dm::j4310Mit(options);
}
inline control::VelocityMotor::Config bigYawMotorConfig() {
    control::VelocityMotor::Config c{};
    c.effort_unit = control::EffortUnit::NewtonMeter;
    // TODO(tuning): identify the loaded big-Yaw inertia and tune the velocity loop.
    c.loop.regulator.feedback = {.kp = 0.02f,
                                 .ki = 0.15f,
                                 .kd = 0,
                                 .derivative_tau_s = 0,
                                 .integral_min = -0.3f,
                                 .integral_max = 0.3f,
                                 .output_min = -big_yaw.effort_limit,
                                 .output_max = big_yaw.effort_limit,
                                 .deadband = 0,
                                 .dt_min_s = 0.001f,
                                 .dt_max_s = 0.020f};
    c.loop.reference_slew = {0.5f, 0.5f};
    c.loop.measurement_filter_tau_s = 0.02f;
    c.loop.soft_deadband_rad_s = 0.02f;
    c.loop.requested_velocity_abs_max_rad_s = big_yaw.velocity_limit_rad_s * big_yaw.gear_ratio;
    c.loop.effort_abs_max = big_yaw.effort_limit;
    return c;
}
inline constexpr BigYawExecutor::Config bigYawExecutionConfig() {
    BigYawExecutor::Config c{};
    c.heading_lock_enabled = big_yaw_heading_lock_enabled;
    c.native_velocity_mode_confirmed = big_yaw_native_velocity_profile_confirmed;
    c.native_velocity_acceleration_rad_s2 = big_yaw_native_acceleration_rad_s2;
    c.mit_mode_confirmed = big_yaw_mit_mode_confirmed;
    c.heading_reference_confirmed = startup_heading_alignment_confirmed;
    c.heading.torque_max_nm = big_yaw.effort_limit;
    const auto &cal = big_yaw_native_velocity_profile_confirmed ? reference_big_yaw : big_yaw;
    c.rate_abs_max_rad_s = cal.velocity_limit_rad_s;
    c.motor_to_joint_ratio = cal.gear_ratio;
    c.direction = cal.direction;
    return c;
}
// World-heading follower for the FollowCenter request. Output is a physical
// joint rate; BigYawExecutor applies direction and ratio exactly once.
inline BigYawWorldHeadingFollower::Config bigYawHeadingFollowerConfig() {
    const auto &cal = big_yaw_native_velocity_profile_confirmed ? reference_big_yaw : big_yaw;
    BigYawWorldHeadingFollower::Config c{};
    c.heading_kp = big_yaw_heading_kp;
    c.heading_rate_max_rad_s = big_yaw_heading_rate_max_rad_s;
    c.heading_error_max_rad = 1.5707963f;
    c.rate_kp = big_yaw_heading_rate_kp;
    c.rate_ki = big_yaw_heading_rate_ki;
    c.rate_integral_max_rad_s = big_yaw_heading_integral_max_rad_s;
    c.output_max_rad_s = cal.velocity_limit_rad_s;
    c.release_epsilon_rad_s = big_yaw_heading_release_epsilon_rad_s;
    // The world envelope and the joint capability both come from the installed
    // calibration; the executor still applies sign, ratio and the native slew.
    c.world_rate_max_rad_s = cal.velocity_limit_rad_s;
    c.shaping_acceleration_rad_s2 = big_yaw_world_acceleration_rad_s2;
    c.shaping_jerk_rad_s3 = big_yaw_world_jerk_rad_s3;
    c.joint_rate_max_rad_s = cal.velocity_limit_rad_s;
    c.stop_capture_dwell_s = big_yaw_stop_capture_dwell_s;
    c.stop_capture_enter_rad_s = big_yaw_stop_capture_enter_rad_s;
    c.stop_capture_exit_rad_s = big_yaw_stop_capture_exit_rad_s;
    return c;
}
} // namespace skywalker::robotics::vehicle
