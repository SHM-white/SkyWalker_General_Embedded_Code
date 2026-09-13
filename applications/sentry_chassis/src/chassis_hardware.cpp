#include <cmath>
#include <cerrno>
#include <zephyr/kernel.h>
#include <control/can_recovery.hpp>
#include "board_config.hpp"
#include "chassis_hardware.hpp"
using namespace skywalker;
int DjiChassisHardware::init() {
    if (initialized_)
        return -EALREADY;
    for (unsigned i = 0; i < 8; ++i) {
        if (!motors_[i] || !device_is_ready(motors_[i]))
            return -ENODEV;
        if (board_config::motor_direction[i] != 1 && board_config::motor_direction[i] != -1)
            return -EINVAL;
        int ret = motor::dji::describe(motors_[i], descriptors_[i]);
        if (ret < 0)
            return ret;
        if (i && descriptors_[i].can != descriptors_[0].can)
            return -ENOTSUP; // V1: one physical shared CAN.
        auto required = motor::CommandCurrent | motor::FeedbackVelocity | motor::FeedbackPosition |
                        motor::FeedbackCurrent;
        if (i < 4)
            required |= motor::FeedbackAbsolutePosition;
        if ((motor::capabilities(motors_[i]) & required) != required)
            return -ENOTSUP;
        for (unsigned j = 0; j < i; ++j)
            if (motors_[i] == motors_[j] || descriptors_[i].feedback_id == descriptors_[j].feedback_id ||
                (descriptors_[i].command_id == descriptors_[j].command_id &&
                 descriptors_[i].command_slot == descriptors_[j].command_slot))
                return -EINVAL;
    }
    const auto config = board_config::chassisConfig();
    for (unsigned i = 0; i < 4; ++i)
        if (config.modules[i].steer.velocity.effort_abs_max > descriptors_[i].configured_current_limit_a ||
            config.modules[i].drive.effort_abs_max > descriptors_[i + 4].configured_current_limit_a)
            return -ERANGE;
    int ret = bus_.init(descriptors_[0].can);
    if (ret < 0)
        return ret;
    for (const auto *dev : motors_) {
        ret = bus_.attach(dev);
        if (ret < 0)
            return ret;
    }
    initialized_ = true;
    return 0;
}
int DjiChassisHardware::read(robotics::ChassisFeedback &out) {
    if (!initialized_)
        return -EACCES;
    robotics::ChassisFeedback next{};
    auto stamps = stamps_;
    float power = board_config::idle_power_w;
    const auto now = static_cast<std::uint64_t>(k_uptime_get());
    for (unsigned i = 0; i < 8; ++i) {
        motor::Feedback f{};
        int ret = motor::readFeedback(motors_[i], f);
        if (ret < 0)
            return ret;
        if (!f.timestamp_ms || now < f.timestamp_ms || now - f.timestamp_ms > CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS)
            return -ESTALE;
        if (motor::getState(motors_[i]) != motor::State::Ready)
            return -EHOSTDOWN;
        auto required = motor::FeedbackPosition | motor::FeedbackVelocity | motor::FeedbackCurrent;
        if (i < 4)
            required |= motor::FeedbackAbsolutePosition;
        if ((f.valid & required) != required || !std::isfinite(f.position_rad) || !std::isfinite(f.velocity_rad_s) ||
            !std::isfinite(f.current_a) || (i < 4 && !std::isfinite(f.absolute_position_rad)))
            return -ENODATA;
        if (std::fabs(f.velocity_rad_s) > board_config::velocity_safety_rad_s)
            return -ERANGE;
        if ((f.valid & motor::FeedbackTemperature) &&
            (!std::isfinite(f.temperature_c) || f.temperature_c >= board_config::temperature_limit_c))
            return -ERANGE;
        const float sign = board_config::motor_direction[i];
        if (i < 4) {
            next.module[i].steer_absolute_rad = sign * f.absolute_position_rad;
            next.module[i].steer_continuous_rad = sign * f.position_rad;
            next.module[i].steer_velocity_rad_s = sign * f.velocity_rad_s;
        }
        else
            next.module[i - 4].drive_velocity_rad_s = sign * f.velocity_rad_s;
        stamps[i] = f.timestamp_ms;
        power += std::fabs(f.current_a) * board_config::power_per_abs_amp_w;
    }
    stamps_ = stamps;
    estimated_power_w_ = power;
    out = next;
    return 0;
}
int DjiChassisHardware::suspend() {
    ready_ = armed_ = stable_ = false;
    next_retry_ms_ = 0;
    if (!initialized_ || stopped_)
        return 0;
    stopped_ = true;
    return bus_.stop(report_); // Withdraw all software commands even if zero transmission fails.
}
int DjiChassisHardware::pollRecovery(std::uint64_t now) {
    if (!initialized_ || armed_)
        return -EACCES;
    if (now < next_retry_ms_)
        return -EAGAIN;
    int ret = control::pollCanRecovery(descriptors_[0].can);
    if (ret == 0 && bus_.state() == motor::dji::BusState::Fault)
        ret = bus_.recover(report_);
    robotics::ChassisFeedback f{};
    if (ret == 0)
        ret = read(f);
    if (ret < 0) {
        ready_ = stable_ = false;
        next_retry_ms_ = now + board_config::recovery_retry_ms;
        return ret;
    }
    if (ready_)
        return 0;
    if (!stable_) {
        for (const auto *dev : motors_) {
            ret = motor::dji::resetMeasurementReference(dev);
            if (ret < 0)
                return ret;
        }
        stable_ = true;
        stable_since_ms_ = now;
        first_stamps_ = stamps_;
        return -EAGAIN;
    }
    for (unsigned i = 0; i < 8; ++i)
        if (stamps_[i] == first_stamps_[i])
            return -EAGAIN;
    if (now < stable_since_ms_ || now - stable_since_ms_ < board_config::feedback_stable_ms)
        return -EAGAIN;
    ready_ = true;
    return 0;
}
int DjiChassisHardware::arm() {
    if (!ready_ || armed_)
        return -EAGAIN;
    robotics::ChassisFeedback f{};
    int ret = read(f);
    if (ret < 0) {
        suspend();
        return ret;
    }
    stopped_ = false;
    ret = bus_.arm(report_);
    if (ret < 0 || !report_.zero_sent) {
        suspend();
        return ret < 0 ? ret : -EIO;
    }
    armed_ = true;
    return 0;
}
int DjiChassisHardware::apply(const robotics::ChassisOutput &out, float scale) {
    if (!armed_)
        return -EACCES;
    if (!std::isfinite(scale) || scale < 0 || scale > 1) {
        suspend();
        return -EINVAL;
    }
    std::array<float, 8> current{};
    for (unsigned i = 0; i < 4; ++i) {
        current[i] = out.module[i].steer_effort * scale * board_config::motor_direction[i];
        current[i + 4] = out.module[i].drive_effort * scale * board_config::motor_direction[i + 4];
    }
    for (unsigned i = 0; i < 8; ++i)
        if (!std::isfinite(current[i]) || std::fabs(current[i]) > descriptors_[i].configured_current_limit_a) {
            suspend();
            return -ERANGE;
        }
    for (unsigned i = 0; i < 8; ++i) {
        int ret = motor::setCurrent(motors_[i], current[i]);
        if (ret < 0) {
            suspend();
            return ret;
        }
    }
    const int ret = bus_.flush(report_);
    if (ret < 0)
        suspend();
    return ret;
}
