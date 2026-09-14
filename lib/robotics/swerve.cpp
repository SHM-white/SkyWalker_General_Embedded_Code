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
        !std::isfinite(config_.stationary_epsilon_m_s) || config_.stationary_epsilon_m_s < 0)
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
    float maximum = 0;
    for (unsigned i = 0; i < 4; ++i) {
        const auto &p = config_.locations[i];
        const float vx = c.mode == ChassisMode::Disabled
                             ? 0
                             : (c.mode == ChassisMode::Spin ? 0 : c.vx_m_s) - c.wz_rad_s * p.y_m;
        const float vy = c.mode == ChassisMode::Disabled
                             ? 0
                             : (c.mode == ChassisMode::Spin ? 0 : c.vy_m_s) + c.wz_rad_s * p.x_m;
        float speed = std::hypot(vx, vy);
        if (!std::isfinite(speed))
            return -ERANGE;
        const bool stationary = speed <= config_.stationary_epsilon_m_s;
        next[i] = {stationary ? last_angle_rad_[i] : std::atan2(vy, vx), stationary ? 0 : speed};
        maximum = std::max(maximum, next[i].wheel_velocity_m_s);
    }
    const float scale = maximum > config_.max_wheel_velocity_m_s ? config_.max_wheel_velocity_m_s / maximum : 1;
    for (unsigned i = 0; i < 4; ++i) {
        next[i].wheel_velocity_m_s *= scale;
        last_angle_rad_[i] = next[i].angle_rad;
    }
    out = next;
    return 0;
}
int SwerveModule::validate() const {
    if (!std::isfinite(config_.wheel_radius_m) || config_.wheel_radius_m <= 0)
        return -EINVAL;
    int ret = control_motor_position_validate(&config_.steer);
    if (ret < 0)
        return ret;
    ret = control_motor_velocity_validate(&config_.drive);
    if (ret < 0)
        return ret;
    const auto &a = config_.steer.position, &b = config_.steer.velocity.regulator.feedback,
               &c = config_.drive.regulator.feedback;
    return std::max({a.dt_min_s, b.dt_min_s, c.dt_min_s}) <= std::min({a.dt_max_s, b.dt_max_s, c.dt_max_s}) ? 0
                                                                                                            : -ERANGE;
}
int SwerveModule::reset(const ModuleFeedback &f) {
    int ret = validate();
    if (ret < 0)
        return ret;
    if (!valid(f))
        return -EINVAL;
    auto s = steer_;
    auto d = drive_;
    ret = control_motor_position_reset(&s, &config_.steer, 0, f.steer_velocity_rad_s);
    if (ret < 0)
        return ret;
    ret = control_motor_velocity_reset(&d, f.drive_velocity_rad_s, 0);
    if (ret < 0)
        return ret;
    steer_ = s;
    drive_ = d;
    origin_rad_ = f.steer_continuous_rad;
    initialized_ = true;
    return 0;
}
int SwerveModule::step(const ModuleTarget &t, const ModuleFeedback &f, float dt_s, ModuleOutput &out) {
    if (!initialized_)
        return -EACCES;
    if (!valid(f) || !std::isfinite(t.angle_rad) || !std::isfinite(t.wheel_velocity_m_s))
        return -EINVAL;
    auto s = steer_;
    auto d = drive_;
    float origin = origin_rad_;
    ModuleOutput n{};
    float error = 0;
    int ret = control_shortest_angle_error(t.angle_rad, f.steer_absolute_rad, &error);
    if (ret < 0)
        return ret;
    const bool flip = std::fabs(error) > pi / 2;
    n.optimized_angle_rad = std::remainder(t.angle_rad + (flip ? pi : 0), 2 * pi);
    n.optimized_wheel_velocity_m_s = flip ? -t.wheel_velocity_m_s : t.wheel_velocity_m_s;
    ret = control_angle_nearest_continuous_target(n.optimized_angle_rad, f.steer_absolute_rad, f.steer_continuous_rad,
                                                  &n.steer_continuous_target_rad);
    if (ret < 0)
        return ret;
    if (std::fabs(f.steer_continuous_rad - origin) > 128) {
        s.position.previous_measurement -= f.steer_continuous_rad - origin;
        origin = f.steer_continuous_rad;
    }
    control_motor_position_input si{n.steer_continuous_target_rad - origin,
                                    f.steer_continuous_rad - origin,
                                    f.steer_velocity_rad_s,
                                    dt_s,
                                    n.steer_continuous_target_rad,
                                    true};
    control_motor_position_output so{};
    ret = control_motor_position_step(&s, &config_.steer, &si, &so);
    if (ret < 0)
        return ret;
    n.drive_target_rad_s = n.optimized_wheel_velocity_m_s / config_.wheel_radius_m;
    control_motor_velocity_input di{n.drive_target_rad_s, f.drive_velocity_rad_s, 0, dt_s, false};
    control_motor_velocity_output dout{};
    ret = control_motor_velocity_step(&d, &config_.drive, &di, &dout);
    if (ret < 0)
        return ret;
    n.steer_effort = so.effort_command;
    n.drive_effort = dout.effort_command;
    steer_ = s;
    drive_ = d;
    origin_rad_ = origin;
    out = n;
    return 0;
}
int SwerveChassis::validate() const {
    int ret = kinematics_.validate();
    if (ret < 0)
        return ret;
    for (const auto &m : modules_) {
        ret = m.validate();
        if (ret < 0)
            return ret;
    }
    return 0;
}
int SwerveChassis::reset(const ChassisFeedback &f) {
    auto next = *this;
    ModuleTargets angles{};
    for (unsigned i = 0; i < 4; ++i) {
        int ret = next.modules_[i].reset(f.module[i]);
        if (ret < 0)
            return ret;
        angles[i].angle_rad = f.module[i].steer_absolute_rad;
    }
    int ret = next.kinematics_.reset(angles);
    if (ret < 0)
        return ret;
    *this = next;
    return 0;
}
int SwerveChassis::step(const ChassisCommand &c, const ChassisFeedback &f, float dt_s, ChassisOutput &out) {
    auto next = *this;
    ChassisOutput n{};
    int ret = next.kinematics_.solve(c, n.target);
    if (ret < 0)
        return ret;
    for (unsigned i = 0; i < 4; ++i) {
        ret = next.modules_[i].step(n.target[i], f.module[i], dt_s, n.module[i]);
        if (ret < 0)
            return ret;
    }
    *this = next;
    out = n;
    return 0;
}
}
