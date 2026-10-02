#pragma once
#include <robotics/execution/big_yaw_executor.hpp>
#include <robotics/vehicle/calibration.hpp>
namespace skywalker::robotics::vehicle {
inline motor::dm::Config bigYawHardware() {
    // TODO(hardware): confirm MIT mode and PMAX/VMAX/TMAX stored in the drive.
    return motor::dm::j4310Mit({.id = big_yaw.id, .master_id = big_yaw.master_id,
        .position_max_rad = 12.5f, .velocity_max_rad_s = 30, .torque_max_nm = 10,
        .torque_limit_nm = big_yaw.effort_limit, .timing = {50, 20, 50, 3000}});
}
inline control::VelocityMotor::Config bigYawMotorConfig() {
    control::VelocityMotor::Config c{};
    c.effort_unit = control::EffortUnit::NewtonMeter;
    c.safety = {10, 60};
    // TODO(tuning): identify the loaded big-Yaw inertia and tune the velocity loop.
    c.loop.regulator.feedback = {.kp = 0.02f, .ki = 0.15f, .kd = 0,
        .derivative_tau_s = 0, .integral_min = -0.3f, .integral_max = 0.3f,
        .output_min = -big_yaw.effort_limit, .output_max = big_yaw.effort_limit,
        .deadband = 0, .dt_min_s = 0.001f, .dt_max_s = 0.020f};
    c.loop.reference_slew = {0.5f, 0.5f};
    c.loop.measurement_filter_tau_s = 0.02f;
    c.loop.soft_deadband_rad_s = 0.02f;
    c.loop.requested_velocity_abs_max_rad_s = big_yaw.velocity_limit_rad_s * big_yaw.gear_ratio;
    c.loop.effort_abs_max = big_yaw.effort_limit;
    return c;
}
inline BigYawExecutor::Config bigYawExecutionConfig() {
    BigYawExecutor::Config c{};
    c.rate_abs_max_rad_s = big_yaw.velocity_limit_rad_s;
    c.motor_to_joint_ratio = big_yaw.gear_ratio; c.direction = big_yaw.direction;
    return c;
}
} // namespace skywalker::robotics::vehicle
