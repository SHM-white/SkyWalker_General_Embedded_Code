#include <cmath>
#include <cerrno>
#include <zephyr/kernel.h>
#include <control/can_recovery.hpp>
#include "board_config.hpp"
#include "chassis_hardware.hpp"
using namespace skywalker;
int DjiChassisHardware::findOrCreateBus(const device *can) {
    if (!can)
        return -ENODEV;
    for (std::size_t i = 0; i < bus_count_; ++i)
        if (can_devices_[i] == can)
            return static_cast<int>(i);
    if (bus_count_ >= kMaxBuses)
        return -ENOTSUP;
    const std::size_t index = bus_count_;
    const int ret = buses_[index].init(can);
    if (ret < 0)
        return ret;
    can_devices_[index] = can;
    ++bus_count_;
    return static_cast<int>(index);
}
int DjiChassisHardware::stopAllBuses() {
    int first_error = 0;
    for (std::size_t i = 0; i < bus_count_; ++i) {
        const int ret = buses_[i].stop(reports_[i]);
        if (ret < 0 && first_error == 0)
            first_error = ret;
    }
    return first_error;
}
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
        if (!descriptors_[i].can)
            return -ENODEV;
        auto required = motor::CommandCurrent | motor::FeedbackVelocity | motor::FeedbackPosition |
                        motor::FeedbackCurrent;
        if (i < 4)
            required |= motor::FeedbackAbsolutePosition;
        if ((motor::capabilities(motors_[i]) & required) != required)
            return -ENOTSUP;
        for (unsigned j = 0; j < i; ++j) {
            if (motors_[i] == motors_[j])
                return -EINVAL;
            if (descriptors_[i].can != descriptors_[j].can)
                continue;
            if (descriptors_[i].feedback_id == descriptors_[j].feedback_id ||
                (descriptors_[i].command_id == descriptors_[j].command_id &&
                 descriptors_[i].command_slot == descriptors_[j].command_slot))
                return -EINVAL;
        }
    }
    const auto config = board_config::chassisConfig();
    for (unsigned i = 0; i < 4; ++i)
        if (config.modules[i].steer.velocity.effort_abs_max > descriptors_[i].configured_current_limit_a ||
            config.modules[i].drive.effort_abs_max > descriptors_[i + 4].configured_current_limit_a)
            return -ERANGE;
    for (unsigned i = 0; i < 8; ++i) {
        const int bus_index = findOrCreateBus(descriptors_[i].can);
        if (bus_index < 0) {
            stopAllBuses();
            return bus_index;
        }
        motor_bus_index_[i] = static_cast<std::uint8_t>(bus_index);
        const int ret = buses_[motor_bus_index_[i]].attach(motors_[i]);
        if (ret < 0) {
            stopAllBuses();
            return ret;
        }
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
    return stopAllBuses(); // Withdraw every bus's commands even if a zero transmission fails.
}
int DjiChassisHardware::pollRecovery(std::uint64_t now) {
    if (!initialized_ || armed_)
        return -EACCES;
    if (now < next_retry_ms_)
        return -EAGAIN;
    auto fail_recovery = [&](int error) {
        ready_ = armed_ = stable_ = false;
        stopped_ = true;
        stopAllBuses();
        next_retry_ms_ = now + board_config::recovery_retry_ms;
        return error;
    };
    int ret = 0;
    for (std::size_t i = 0; i < bus_count_; ++i) {
        int bus_ret = control::pollCanRecovery(can_devices_[i]);
        if (bus_ret == 0 && buses_[i].state() == motor::dji::BusState::Fault)
            bus_ret = buses_[i].recover(reports_[i]);
        if (bus_ret < 0 && ret == 0)
            ret = bus_ret;
    }
    robotics::ChassisFeedback f{};
    if (ret == 0)
        ret = read(f);
    if (ret < 0)
        return fail_recovery(ret);
    if (ready_)
        return 0;
    if (!stable_) {
        for (const auto *dev : motors_) {
            ret = motor::dji::resetMeasurementReference(dev);
            if (ret < 0)
                return fail_recovery(ret);
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
    for (std::size_t i = 0; i < bus_count_; ++i) {
        ret = buses_[i].arm(reports_[i]);
        if (ret < 0 || !reports_[i].zero_sent) {
            suspend();
            return ret < 0 ? ret : -EIO;
        }
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
    int first_error = 0;
    for (std::size_t i = 0; i < bus_count_; ++i) {
        const int ret = buses_[i].flush(reports_[i]);
        if (ret < 0 && first_error == 0)
            first_error = ret;
    }
    if (first_error < 0)
        suspend();
    return first_error;
}
