#include <algorithm>
#include <cmath>
#include <cerrno>
#include <control/angle.h>
#include <robotics/swerve/swerve_chassis.hpp>
namespace skywalker::robotics {
namespace {
constexpr float pi = 3.14159265358979323846f;
bool valid(const ModuleFeedback &f) {
    return std::isfinite(f.steer_absolute_rad) && std::isfinite(f.steer_continuous_rad) &&
           std::isfinite(f.steer_velocity_rad_s) && std::isfinite(f.drive_velocity_rad_s);
}
}
int SwerveKinematics::validate() const {
    if (!std::isfinite(config_.max_wheel_velocity_m_s) || config_.max_wheel_velocity_m_s <= 0 ||
        !std::isfinite(config_.stationary_epsilon_m_s) || config_.stationary_epsilon_m_s < 0 ||
        !std::isfinite(config_.resume_velocity_m_s) || config_.resume_velocity_m_s < config_.stationary_epsilon_m_s)
        return -EINVAL;
    for (const auto &p : config_.locations)
        if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m)) return -EINVAL;
    return 0;
}
int SwerveKinematics::reset(const ModuleTargets &current) {
    int ret = validate();
    if (ret < 0) return ret;
    for (const auto &t : current)
        if (!std::isfinite(t.angle_rad)) return -EINVAL;
    for (unsigned i = 0; i < 4; ++i) last_angle_rad_[i] = current[i].angle_rad;
    moving_ = {};
    initialized_ = true;
    return 0;
}
int SwerveKinematics::solve(const ChassisCommand &c, ModuleTargets &out) {
    if (!initialized_) return -EACCES;
    if (c.mode > ChassisMode::Spin || !std::isfinite(c.vx_m_s) || !std::isfinite(c.vy_m_s) ||
        !std::isfinite(c.wz_rad_s)) return -EINVAL;
    ModuleTargets next{};
    auto moving = moving_;
    float maximum = 0;
    for (unsigned i = 0; i < 4; ++i) {
        const auto &p = config_.locations[i];
        const float vx = c.mode == ChassisMode::Disabled ? 0 :
            (c.mode == ChassisMode::Spin ? 0 : c.vx_m_s) - c.wz_rad_s * p.y_m;
        const float vy = c.mode == ChassisMode::Disabled ? 0 :
            (c.mode == ChassisMode::Spin ? 0 : c.vy_m_s) + c.wz_rad_s * p.x_m;
        const float speed = std::hypot(vx, vy);
        if (!std::isfinite(speed)) return -ERANGE;
        moving[i] = speed > (moving_[i] ? config_.stationary_epsilon_m_s : config_.resume_velocity_m_s);
        next[i] = {moving[i] ? std::atan2(vy, vx) : last_angle_rad_[i], moving[i] ? speed : 0};
        maximum = std::max(maximum, next[i].wheel_velocity_m_s);
    }
    const float scale = maximum > config_.max_wheel_velocity_m_s ? config_.max_wheel_velocity_m_s / maximum : 1;
    for (unsigned i = 0; i < 4; ++i) {
        next[i].wheel_velocity_m_s *= scale;
        last_angle_rad_[i] = next[i].angle_rad;
    }
    moving_ = moving;
    out = next;
    return 0;
}
int SwerveModule::validate() const {
    if (!std::isfinite(config_.wheel_radius_m) || config_.wheel_radius_m <= 0 ||
        !std::isfinite(config_.steer_target_rate_rad_s) || config_.steer_target_rate_rad_s <= 0 ||
        !std::isfinite(config_.drive_enable_error_rad) || config_.drive_enable_error_rad <= 0 ||
        !std::isfinite(config_.drive_disable_error_rad) || config_.drive_disable_error_rad <= config_.drive_enable_error_rad ||
        config_.drive_disable_error_rad >= pi / 2 || !std::isfinite(config_.flip_enter_error_rad) ||
        !std::isfinite(config_.flip_exit_error_rad) || config_.flip_exit_error_rad <= 0 ||
        config_.flip_exit_error_rad >= pi / 2 || config_.flip_enter_error_rad <= pi / 2 ||
        config_.flip_enter_error_rad >= pi ||
        (config_.idle_behavior != IdleBehavior::Hold && config_.idle_behavior != IdleBehavior::Coast)) return -EINVAL;
    int ret = control_motor_position_validate(&config_.steer);
    if (ret < 0) return ret;
    ret = control_motor_velocity_validate(&config_.drive);
    if (ret < 0) return ret;
    const auto &a = config_.steer.position, &b = config_.steer.velocity.regulator.feedback,
               &c = config_.drive.regulator.feedback;
    return std::max({a.dt_min_s, b.dt_min_s, c.dt_min_s}) <= std::min({a.dt_max_s, b.dt_max_s, c.dt_max_s}) ? 0 : -ERANGE;
}
int SwerveModule::reset(const ModuleFeedback &f) {
    int ret = validate();
    if (ret < 0) return ret;
    if (!valid(f)) return -EINVAL;
    auto s = steer_;
    auto d = drive_;
    ret = control_motor_position_reset(&s, &config_.steer, 0, f.steer_velocity_rad_s);
    if (ret < 0) return ret;
    ret = control_motor_velocity_reset(&d, f.drive_velocity_rad_s, 0);
    if (ret < 0) return ret;
    steer_ = s;
    drive_ = d;
    origin_rad_ = steer_reference_rad_ = f.steer_continuous_rad;
    flipped_ = drive_ready_ = false;
    initialized_ = true;
    return 0;
}
int SwerveModule::plan(const ModuleTarget &t, const ModuleFeedback &f, float dt_s, ModuleOutput &out) {
    if (!initialized_) return -EACCES;
    if (!valid(f) || !std::isfinite(t.angle_rad) || !std::isfinite(t.wheel_velocity_m_s) || !std::isfinite(dt_s))
        return -EINVAL;
    const auto &a = config_.steer.position, &b = config_.steer.velocity.regulator.feedback,
               &c = config_.drive.regulator.feedback;
    if (dt_s <= 0 || dt_s < std::max({a.dt_min_s, b.dt_min_s, c.dt_min_s}) ||
        dt_s > std::min({a.dt_max_s, b.dt_max_s, c.dt_max_s})) return -ERANGE;
    if (std::fabs(t.wheel_velocity_m_s / config_.wheel_radius_m) > config_.drive.requested_velocity_abs_max_rad_s)
        return -ERANGE;
    ModuleOutput n{};
    const bool stationary = t.wheel_velocity_m_s == 0;
    if (stationary && config_.idle_behavior == IdleBehavior::Coast) {
        const int ret = reset(f);
        if (ret < 0) return ret;
        n.optimized_angle_rad = f.steer_absolute_rad;
        n.steer_continuous_target_rad = n.steer_reference_rad = f.steer_continuous_rad;
        n.coasting = true;
        out = n;
        return 0;
    }
    float error = 0;
    int ret = control_shortest_angle_error(t.angle_rad, f.steer_absolute_rad, &error);
    if (ret < 0) return ret;
    // Do not switch the drive polarity while holding a stopped module.
    if (!stationary) {
        if (flipped_) { if (std::fabs(error) < config_.flip_exit_error_rad) flipped_ = false; }
        else if (std::fabs(error) > config_.flip_enter_error_rad) flipped_ = true;
    }
    n.flipped = flipped_;
    n.optimized_angle_rad = std::remainder(t.angle_rad + (flipped_ ? pi : 0), 2 * pi);
    n.optimized_wheel_velocity_m_s = flipped_ ? -t.wheel_velocity_m_s : t.wheel_velocity_m_s;
    ret = control_shortest_angle_error(n.optimized_angle_rad, f.steer_absolute_rad, &n.alignment_error_rad);
    if (ret < 0) return ret;
    const float alignment_error = std::fabs(n.alignment_error_rad);
    drive_ready_ = drive_ready_ ? alignment_error < config_.drive_disable_error_rad :
                                alignment_error < config_.drive_enable_error_rad;
    n.drive_ready = drive_ready_;
    ret = control_angle_nearest_continuous_target(n.optimized_angle_rad, f.steer_absolute_rad, f.steer_continuous_rad,
                                                  &n.steer_continuous_target_rad);
    if (ret < 0) return ret;
    const float delta = n.steer_continuous_target_rad - steer_reference_rad_;
    const float maximum_step = config_.steer_target_rate_rad_s * dt_s;
    if (!std::isfinite(delta) || !std::isfinite(maximum_step)) return -ERANGE;
    steer_reference_rad_ += std::clamp(delta, -maximum_step, maximum_step);
    if (!std::isfinite(steer_reference_rad_)) return -ERANGE;
    n.steer_reference_rad = steer_reference_rad_;
    if (std::fabs(f.steer_continuous_rad - origin_rad_) > 128) {
        steer_.position.previous_measurement -= f.steer_continuous_rad - origin_rad_;
        origin_rad_ = f.steer_continuous_rad;
    }
    control_motor_position_input si{steer_reference_rad_ - origin_rad_, f.steer_continuous_rad - origin_rad_,
                                    f.steer_velocity_rad_s, dt_s, steer_reference_rad_, true};
    control_motor_position_output so{};
    ret = control_motor_position_step(&steer_, &config_.steer, &si, &so);
    if (ret < 0) return ret;
    n.steer_effort = so.effort_command;
    out = n;
    return 0;
}
int SwerveModule::drive(const ModuleFeedback &f, float dt_s, bool permitted, ModuleOutput &out) {
    out.drive_enabled = permitted && out.drive_ready && !out.coasting;
    if (!out.drive_enabled) {
        // Zero current, not a zero-speed braking loop. Clear both I and slew
        // state before reopening the alignment gate, including a peer's veto.
        out.drive_target_rad_s = out.drive_effort = 0;
        return control_motor_velocity_reset(&drive_, f.drive_velocity_rad_s, 0);
    }
    const float alignment = std::clamp(std::cos(out.alignment_error_rad), 0.0f, 1.0f);
    out.drive_target_rad_s = out.optimized_wheel_velocity_m_s * alignment / config_.wheel_radius_m;
    control_motor_velocity_input di{out.drive_target_rad_s, f.drive_velocity_rad_s, 0, dt_s, false};
    control_motor_velocity_output dout{};
    const int ret = control_motor_velocity_step(&drive_, &config_.drive, &di, &dout);
    if (ret < 0) return ret;
    out.drive_effort = dout.effort_command;
    return 0;
}
int SwerveModule::step(const ModuleTarget &t, const ModuleFeedback &f, float dt_s, ModuleOutput &out) {
    auto next = *this;
    ModuleOutput n{};
    int ret = next.plan(t, f, dt_s, n);
    if (ret == 0) ret = next.drive(f, dt_s, true, n);
    if (ret < 0) return ret;
    *this = next;
    out = n;
    return 0;
}
int SwerveChassis::validate() const {
    int ret = kinematics_.validate();
    if (ret < 0) return ret;
    for (const auto &m : modules_) {
        ret = m.validate();
        if (ret < 0) return ret;
    }
    return 0;
}
int SwerveChassis::reset(const ChassisFeedback &f) {
    auto next = *this;
    ModuleTargets angles{};
    for (unsigned i = 0; i < 4; ++i) {
        int ret = next.modules_[i].reset(f.module[i]);
        if (ret < 0) return ret;
        angles[i].angle_rad = f.module[i].steer_absolute_rad;
    }
    int ret = next.kinematics_.reset(angles);
    if (ret < 0) return ret;
    *this = next;
    return 0;
}
int SwerveChassis::step(const ChassisCommand &c, const ChassisFeedback &f, float dt_s, ChassisOutput &out) {
    auto next = *this;
    ChassisOutput n{};
    int ret = next.kinematics_.solve(c, n.target);
    if (ret < 0) return ret;
    if (c.mode == ChassisMode::Disabled) {
        ret = next.reset(f);
        if (ret < 0) return ret;
        for (unsigned i = 0; i < 4; ++i) {
            n.target[i].angle_rad = n.module[i].optimized_angle_rad = f.module[i].steer_absolute_rad;
            n.module[i].steer_reference_rad = n.module[i].steer_continuous_target_rad = f.module[i].steer_continuous_rad;
            n.module[i].coasting = true;
        }
    } else {
        bool aligned = true;
        for (unsigned i = 0; i < 4; ++i) {
            ret = next.modules_[i].plan(n.target[i], f.module[i], dt_s, n.module[i]);
            if (ret < 0) return ret;
            // A zero-vector/coasting module has no desired rolling direction.
            if (n.target[i].wheel_velocity_m_s != 0) aligned = aligned && n.module[i].drive_ready;
        }
        for (unsigned i = 0; i < 4; ++i) {
            ret = next.modules_[i].drive(f.module[i], dt_s, !require_all_modules_aligned_ || aligned, n.module[i]);
            if (ret < 0) return ret;
        }
    }
    *this = next;
    out = n;
    return 0;
}
}
