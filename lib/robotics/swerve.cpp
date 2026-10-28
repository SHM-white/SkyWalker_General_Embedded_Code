#include <algorithm>
#include <cmath>
#include <cerrno>
#include <control/angle.h>
#include <robotics/swerve/swerve_chassis.hpp>
namespace skywalker::robotics {
namespace {
constexpr float pi = 3.14159265358979323846f;
}
int SwerveKinematics::validate() const {
    if (!std::isfinite(config_.max_wheel_velocity_m_s) || config_.max_wheel_velocity_m_s <= 0 ||
        !std::isfinite(config_.stationary_epsilon_m_s) || config_.stationary_epsilon_m_s < 0 ||
        !std::isfinite(config_.resume_velocity_m_s) || config_.resume_velocity_m_s < config_.stationary_epsilon_m_s)
        return -EINVAL;
    for (const auto &p : config_.locations)
        if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m))
            return -EINVAL;
    return 0;
}
int SwerveKinematics::reset(const ModuleTargets &current) {
    int ret = validate();
    if (ret < 0)
        return ret;
    for (const auto &t : current)
        if (!std::isfinite(t.angle_rad))
            return -EINVAL;
    for (unsigned i = 0; i < 4; ++i)
        last_angle_rad_[i] = current[i].angle_rad;
    moving_ = {};
    initialized_ = true;
    return 0;
}
int SwerveKinematics::solve(const ChassisCommand &c, ModuleTargets &out) {
    if (!initialized_)
        return -EACCES;
    if (c.mode > ChassisMode::Spin || !std::isfinite(c.vx_m_s) || !std::isfinite(c.vy_m_s) ||
        !std::isfinite(c.wz_rad_s))
        return -EINVAL;
    ModuleTargets next{};
    auto moving = moving_;
    float maximum = 0;
    for (unsigned i = 0; i < 4; ++i) {
        const auto &p = config_.locations[i];
        const float vx = c.mode == ChassisMode::Disabled
                             ? 0
                             : (c.mode == ChassisMode::Spin ? 0 : c.vx_m_s) - c.wz_rad_s * p.y_m;
        const float vy = c.mode == ChassisMode::Disabled
                             ? 0
                             : (c.mode == ChassisMode::Spin ? 0 : c.vy_m_s) + c.wz_rad_s * p.x_m;
        const float speed = std::hypot(vx, vy);
        if (!std::isfinite(speed))
            return -ERANGE;
        moving[i] = speed > (moving_[i] ? config_.stationary_epsilon_m_s : config_.resume_velocity_m_s);
        next[i] = {moving[i] ? std::atan2(vy, vx) : last_angle_rad_[i], moving[i] ? speed : 0};
        maximum = std::max(maximum, next[i].wheel_velocity_m_s);
    }
    const float scale = maximum > config_.max_wheel_velocity_m_s ? config_.max_wheel_velocity_m_s / maximum : 1;
    for (unsigned i = 0; i < 4; ++i) {
        // The multiply after normalization can round just above the ceiling.
        next[i].wheel_velocity_m_s = std::min(next[i].wheel_velocity_m_s * scale, config_.max_wheel_velocity_m_s);
        last_angle_rad_[i] = next[i].angle_rad;
    }
    moving_ = moving;
    out = next;
    return 0;
}
int SwerveModule::validate() const {
    if (!std::isfinite(config_.wheel_radius_m) || config_.wheel_radius_m <= 0 ||
        !std::isfinite(config_.steer_target_rate_rad_s) || config_.steer_target_rate_rad_s <= 0 ||
        !std::isfinite(config_.flip_enter_error_rad) || !std::isfinite(config_.flip_exit_error_rad) ||
        config_.flip_exit_error_rad <= 0 || config_.flip_exit_error_rad >= pi / 2 ||
        config_.flip_enter_error_rad <= pi / 2 || config_.flip_enter_error_rad >= pi ||
        (config_.idle_behavior != IdleBehavior::Hold && config_.idle_behavior != IdleBehavior::Coast))
        return -EINVAL;
    int ret = control_motor_position_validate(&config_.steer);
    if (ret < 0)
        return ret;
    ret = control_motor_velocity_validate(&config_.drive);
    if (ret < 0)
        return ret;
    const auto &a = config_.steer.position, &b = config_.steer.velocity.regulator.feedback;
    return std::max(a.dt_min_s, b.dt_min_s) <= std::min(a.dt_max_s, b.dt_max_s) ? 0 : -ERANGE;
}
int SwerveModule::reset(const ModuleFeedback &) {
    steer_history_valid_ = drive_history_valid_ = false;
    steer_ = {};
    drive_ = {};
    return 0;
}
int SwerveModule::steer(const ModuleFeedback &f, float dt, ModuleOutput &out) {
    out.steer_enable_generation = f.steer_enable_generation;
    const auto &a = config_.steer.position, &b = config_.steer.velocity.regulator.feedback;
    if (!f.steer_valid || !std::isfinite(f.steer_absolute_rad) || !std::isfinite(f.steer_velocity_rad_s) ||
        dt < std::max(a.dt_min_s, b.dt_min_s) || dt > std::min(a.dt_max_s, b.dt_max_s) || out.coasting) {
        steer_history_valid_ = false;
        out.steer_output_valid = out.coasting && f.steer_valid;
        return 0;
    }
    const bool initialize = !steer_history_valid_ || f.steer_enable_generation != observed_steer_enable_generation_ ||
                            f.steer_reference_generation != observed_steer_reference_generation_;
    out.steer_reinitialized = initialize;
    if (initialize) {
        steer_local_position_rad_ = steer_origin_rad_ = f.steer_absolute_rad;
        previous_absolute_rad_ = f.steer_absolute_rad;
        previous_steer_stamp_ms_ = f.steer_feedback_stamp_ms;
        steer_reference_rad_ = f.steer_absolute_rad;
        const int ret = control_motor_position_reset(&steer_, &config_.steer, 0, f.steer_velocity_rad_s);
        if (ret < 0)
            return ret;
        observed_steer_enable_generation_ = f.steer_enable_generation;
        observed_steer_reference_generation_ = f.steer_reference_generation;
        steer_history_valid_ = true;
    }
    else if (f.steer_feedback_stamp_ms > previous_steer_stamp_ms_) {
        float delta = 0;
        const int ret = control_shortest_angle_error(f.steer_absolute_rad, previous_absolute_rad_, &delta);
        if (ret < 0) {
            steer_history_valid_ = false;
            return ret;
        }
        steer_local_position_rad_ += static_cast<double>(delta);
        previous_absolute_rad_ = f.steer_absolute_rad;
        previous_steer_stamp_ms_ = f.steer_feedback_stamp_ms;
    }
    int ret = control_shortest_angle_error(out.optimized_angle_rad, f.steer_absolute_rad, &out.alignment_error_rad);
    if (ret < 0) {
        steer_history_valid_ = false;
        return ret;
    }
    out.steer_continuous_target_rad = static_cast<float>(steer_local_position_rad_ + static_cast<double>(out.alignment_error_rad));
    if (!std::isfinite(out.steer_continuous_target_rad)) {
        steer_history_valid_ = false;
        return -ERANGE;
    }
    out.steer_reference_rad = steer_reference_rad_;
    out.steer_output_valid = true;
    if (initialize)
        return 0;
    const float delta = out.steer_continuous_target_rad - steer_reference_rad_;
    const float reference = steer_reference_rad_ + std::clamp(delta, -config_.steer_target_rate_rad_s * dt,
                                                              config_.steer_target_rate_rad_s * dt);
    auto next = steer_;
    double origin = steer_origin_rad_;
    if (std::fabs(steer_local_position_rad_ - origin) > 128) {
        next.position.previous_measurement -= static_cast<float>(steer_local_position_rad_ - origin);
        origin = steer_local_position_rad_;
    }
    const control_motor_position_input input{static_cast<float>(static_cast<double>(reference) - origin),
                                             static_cast<float>(steer_local_position_rad_ - origin),
                                             f.steer_velocity_rad_s,
                                             dt,
                                             reference,
                                             true};
    control_motor_position_output output{};
    ret = control_motor_position_step(&next, &config_.steer, &input, &output);
    if (ret < 0) {
        steer_history_valid_ = false;
        out.steer_output_valid = false;
        return ret;
    }
    steer_ = next;
    steer_origin_rad_ = origin;
    steer_reference_rad_ = out.steer_reference_rad = reference;
    out.steer_effort = output.effort_command;
    return 0;
}
int SwerveModule::drive(const ModuleFeedback &f, float dt, ModuleOutput &out) {
    out.drive_enable_generation = f.drive_enable_generation;
    const auto &pid = config_.drive.regulator.feedback;
    if (!f.drive_valid || !std::isfinite(f.drive_velocity_rad_s) || dt < pid.dt_min_s || dt > pid.dt_max_s ||
        out.coasting) {
        drive_history_valid_ = false;
        out.drive_output_valid = out.coasting && f.drive_valid;
        return 0;
    }
    const bool initialize = !drive_history_valid_ || f.drive_enable_generation != observed_drive_enable_generation_;
    if (initialize) {
        const int ret = control_motor_velocity_reset(&drive_, f.drive_velocity_rad_s, 0);
        if (ret < 0)
            return ret;
        observed_drive_enable_generation_ = f.drive_enable_generation;
        drive_history_valid_ = true;
        out.drive_output_valid = true;
        return 0;
    }
    auto next = drive_;
    const control_motor_velocity_input input{out.drive_target_rad_s, f.drive_velocity_rad_s, 0, dt, false};
    control_motor_velocity_output output{};
    const int ret = control_motor_velocity_step(&next, &config_.drive, &input, &output);
    if (ret < 0) {
        drive_history_valid_ = false;
        return ret;
    }
    drive_ = next;
    out.drive_effort = output.effort_command;
    out.drive_output_valid = true;
    return 0;
}
int SwerveModule::finishDrive(const ModuleFeedback &f, float dt, bool permitted, ModuleOutput &out) {
    out.drive_enable_generation = f.drive_enable_generation;
    out.drive_permitted = permitted && out.steer_output_valid && out.cosine_scale > 0;
    if (!out.drive_permitted) {
        // No hidden integral or slewed reference survives an alignment/fault stop.
        drive_history_valid_ = false;
        drive_ = {};
        out.drive_target_rad_s = out.drive_effort = 0;
        const auto &pid = config_.drive.regulator.feedback;
        out.drive_output_valid = f.drive_valid && std::isfinite(f.drive_velocity_rad_s) &&
                                 std::isfinite(dt) && dt >= pid.dt_min_s && dt <= pid.dt_max_s;
        return 0;
    }
    return drive(f, dt, out);
}
int SwerveModule::step(const ModuleTarget &target, const ModuleFeedback &f, float dt, ModuleOutput &out) {
    const int ret = prepare(target, f, dt, out);
    if (ret < 0) {
        (void)reset(f);
        return ret;
    }
    return finishDrive(f, dt, true, out);
}
int SwerveModule::prepare(const ModuleTarget &target, const ModuleFeedback &f, float dt, ModuleOutput &out) {
    if (!std::isfinite(target.angle_rad) || !std::isfinite(target.wheel_velocity_m_s) || !std::isfinite(dt) || dt < 0) {
        out = {};
        return -EINVAL;
    }
    if (std::fabs(target.wheel_velocity_m_s / config_.wheel_radius_m) >
        config_.drive.requested_velocity_abs_max_rad_s) {
        out = {};
        return -ERANGE;
    }
    latest_target_ = target;
    ModuleOutput next{};
    const bool stationary = target.wheel_velocity_m_s == 0;
    next.coasting = stationary && config_.idle_behavior == IdleBehavior::Coast;
    if (f.steer_valid && std::isfinite(f.steer_absolute_rad) && !stationary) {
        float error = 0;
        const int ret = control_shortest_angle_error(target.angle_rad, f.steer_absolute_rad, &error);
        if (ret < 0)
            return ret;
        if (flipped_) {
            if (std::fabs(error) < config_.flip_exit_error_rad)
                flipped_ = false;
        }
        else if (std::fabs(error) > config_.flip_enter_error_rad)
            flipped_ = true;
    }
    next.flipped = flipped_;
    next.optimized_angle_rad = std::remainder(target.angle_rad + (flipped_ ? pi : 0), 2 * pi);
    next.optimized_wheel_velocity_m_s = flipped_ ? -target.wheel_velocity_m_s : target.wheel_velocity_m_s;
    next.drive_target_rad_s = next.optimized_wheel_velocity_m_s / config_.wheel_radius_m;
    const int steer_error = steer(f, dt, next);
    if (next.steer_output_valid) {
        const float cosine = std::max(std::cos(next.alignment_error_rad), 0.0f);
        next.cosine_scale = config_.cosine_compensation ? (cosine > 1e-6f ? cosine * cosine * cosine : 0) : 1;
    }
    next.drive_target_rad_s *= next.cosine_scale;
    out = next;
    return steer_error;
}
int SwerveChassis::validate() const {
    if (!std::isfinite(config_.align_enter_rad) || !std::isfinite(config_.align_exit_rad) ||
        config_.align_enter_rad <= 0 || config_.align_enter_rad >= config_.align_exit_rad ||
        config_.align_exit_rad >= pi / 2 || !std::isfinite(config_.rotation_epsilon_rad_s) ||
        config_.rotation_epsilon_rad_s < 0)
        return -EINVAL;
    int ret = kinematics_.validate();
    if (ret < 0)
        return ret;
    for (const auto &module : modules_) {
        ret = module.validate();
        if (ret < 0)
            return ret;
    }
    return 0;
}
int SwerveChassis::reset(const ChassisFeedback &f) {
    drive_permitted_ = rotating_ = false;
    drive_generation_ = {};
    ModuleTargets angles{};
    for (unsigned i = 0; i < 4; ++i) {
        (void)modules_[i].reset(f.module[i]);
        if (f.module[i].steer_valid && std::isfinite(f.module[i].steer_absolute_rad))
            angles[i].angle_rad = f.module[i].steer_absolute_rad;
    }
    const int ret = kinematics_.reset(angles);
    initialized_ = ret == 0;
    return ret;
}
int SwerveChassis::step(const ChassisCommand &command, const ChassisFeedback &f, float dt, ChassisOutput &out) {
    if (!initialized_) {
        const int ret = reset(f);
        if (ret < 0)
            return ret;
    }
    ChassisOutput next{};
    const int solve_error = kinematics_.solve(command, next.target);
    if (solve_error < 0) {
        (void)reset(f);
        out = {};
        return solve_error;
    }
    int first_error = 0;
    for (unsigned i = 0; i < 4; ++i) {
        if (command.mode == ChassisMode::Disabled) {
            (void)modules_[i].reset(f.module[i]);
            auto &m = next.module[i];
            m.coasting = true;
            m.optimized_angle_rad = next.target[i].angle_rad;
            m.steer_output_valid = f.module[i].steer_valid;
            m.drive_output_valid = f.module[i].drive_valid;
            m.steer_enable_generation = f.module[i].steer_enable_generation;
            m.drive_enable_generation = f.module[i].drive_enable_generation;
        }
        else {
            const int ret = modules_[i].prepare(next.target[i], f.module[i], dt, next.module[i]);
            if (first_error == 0 && ret < 0)
                first_error = ret;
        }
    }
    bool healthy = first_error == 0, moving = false;
    float max_error = 0;
    for (unsigned i = 0; i < 4; ++i) {
        const auto &m = next.module[i];
        const auto &fb = f.module[i];
        healthy = healthy && m.steer_output_valid && !m.steer_reinitialized && fb.drive_valid &&
                  std::isfinite(fb.drive_velocity_rad_s) && drive_generation_[i] == fb.drive_enable_generation;
        drive_generation_[i] = fb.drive_enable_generation;
        max_error = std::max(max_error, std::fabs(m.alignment_error_rad));
        moving = moving || next.target[i].wheel_velocity_m_s != 0;
    }
    const bool rotating = command.mode != ChassisMode::Disabled &&
                          std::fabs(command.wz_rad_s) > config_.rotation_epsilon_rad_s;
    if (!healthy || !moving || command.mode == ChassisMode::Disabled || rotating != rotating_)
        drive_permitted_ = false;
    // Rotation starts aligned, then tracks with individual cos^3 compensation.
    // Ordinary motion uses 0.10/0.40 rad hysteresis against FINAL targets.
    if (healthy && moving && command.mode != ChassisMode::Disabled) {
        if (!drive_permitted_ && max_error < config_.align_enter_rad)
            drive_permitted_ = true;
        else if (drive_permitted_ && !rotating && max_error > config_.align_exit_rad)
            drive_permitted_ = false;
    }
    rotating_ = rotating;
    next.drive_permitted = drive_permitted_;
    if (command.mode != ChassisMode::Disabled) {
        for (unsigned i = 0; i < 4; ++i) {
            const int ret = modules_[i].finishDrive(f.module[i], dt, drive_permitted_, next.module[i]);
            if (!first_error && ret < 0)
                first_error = ret;
        }
        if (first_error < 0) {
            drive_permitted_ = next.drive_permitted = false;
            for (unsigned i = 0; i < 4; ++i)
                (void)modules_[i].finishDrive(f.module[i], dt, false, next.module[i]);
        }
    }
    out = next;
    return first_error;
}
}
