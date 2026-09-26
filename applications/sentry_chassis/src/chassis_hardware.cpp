#include "chassis_hardware.hpp"

#include <cerrno>
#include <cmath>

#include <zephyr/kernel.h>

#include "board_config.hpp"

using namespace skywalker;

const device *DjiChassisHardware::secondaryCan(const std::array<ChassisMotorConnection, 8> &connections) {
    for (const auto &connection : connections)
        if (connection.can != connections[0].can)
            return connection.can;
    return nullptr;
}

DjiChassisHardware::DjiChassisHardware(const std::array<ChassisMotorConnection, 8> &connections)
    : connections_(connections), motors_{{motor::Motor(connections[0].config), motor::Motor(connections[1].config),
                                          motor::Motor(connections[2].config), motor::Motor(connections[3].config),
                                          motor::Motor(connections[4].config), motor::Motor(connections[5].config),
                                          motor::Motor(connections[6].config), motor::Motor(connections[7].config)}},
      group_(motors_[0], motors_[1], motors_[2], motors_[3], motors_[4], motors_[5], motors_[6], motors_[7]),
      first_bus_(connections[0].can), second_bus_(secondaryCan(connections)), buses_{{&first_bus_, &second_bus_}} {
    can_devices_[0] = connections[0].can;
    can_devices_[1] = secondaryCan(connections);
}

int DjiChassisHardware::init() {
    if (initialized_)
        return -EALREADY;
    if (!board_config::connections_configured)
        return -ENODEV;
    if (can_devices_[0] == nullptr || !device_is_ready(can_devices_[0]))
        return -ENODEV;

    bus_count_ = can_devices_[1] == nullptr ? 1 : 2;
    if (bus_count_ == 2 && !device_is_ready(can_devices_[1]))
        return -ENODEV;

    for (std::size_t i = 0; i < motors_.size(); ++i) {
        if (connections_[i].can == nullptr ||
            (connections_[i].can != can_devices_[0] && connections_[i].can != can_devices_[1]))
            return -ENOTSUP;
        if (board_config::motor_direction[i] != 1.0f && board_config::motor_direction[i] != -1.0f)
            return -EINVAL;
        const int described = motor::dji::describe(connections_[i].config, descriptors_[i]);
        if (described < 0)
            return described;
        auto required = motor::CommandCurrent | motor::FeedbackVelocity | motor::FeedbackPosition |
                        motor::FeedbackCurrent;
        if (i < 4)
            required |= motor::FeedbackAbsolutePosition;
        if ((motors_[i].info().capabilities & required) != required)
            return -ENOTSUP;
        for (std::size_t j = 0; j < i; ++j) {
            if (connections_[i].can != connections_[j].can)
                continue;
            if (descriptors_[i].feedback_id == descriptors_[j].feedback_id ||
                (descriptors_[i].command_id == descriptors_[j].command_id &&
                 descriptors_[i].command_slot == descriptors_[j].command_slot))
                return -EINVAL;
        }
        motor_bus_index_[i] = connections_[i].can == can_devices_[0] ? 0 : 1;
    }

    const auto config = board_config::chassisConfig();
    for (std::size_t i = 0; i < 4; ++i)
        if (config.modules[i].steer.velocity.effort_abs_max > descriptors_[i].configured_current_limit_a ||
            config.modules[i].drive.effort_abs_max > descriptors_[i + 4].configured_current_limit_a)
            return -ERANGE;

    // Group topology is checked when the first bus starts, so attach every
    // member on both buses before starting either CAN controller.
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        const int ret = buses_[motor_bus_index_[i]]->attach(motors_[i]);
        if (ret < 0)
            return ret;
    }
    for (std::size_t i = 0; i < bus_count_; ++i) {
        const int ret = buses_[i]->start();
        if (ret < 0) {
            group_.disable();
            return ret;
        }
    }
    initialized_ = true;
    return 0;
}

int DjiChassisHardware::validateFeedback(std::size_t index, const motor::MotorSnapshot &snapshot,
                                         bool require_reference) const {
    const motor::Feedback &f = snapshot.feedback;
    if (!snapshot.feedback_fresh)
        return -ESTALE;
    if (snapshot.state == motor::MotorState::Offline || snapshot.state == motor::MotorState::Fault)
        return -EHOSTDOWN;
    auto required = motor::FeedbackVelocity | motor::FeedbackCurrent;
    if (index < 4)
        required |= motor::FeedbackAbsolutePosition;
    if (require_reference)
        required |= motor::FeedbackPosition;
    if ((f.valid & required) != required || !std::isfinite(f.velocity_rad_s) || !std::isfinite(f.current_a) ||
        (index < 4 && !std::isfinite(f.absolute_position_rad)) ||
        (require_reference && (!snapshot.position_reference_valid || !std::isfinite(f.position_rad))))
        return -ENODATA;
    if (require_reference && stable_ && snapshot.reference_generation != references_[index])
        return -ESTALE;
    if (std::fabs(f.velocity_rad_s) > board_config::velocity_safety_rad_s)
        return -ERANGE;
    if ((f.valid & motor::FeedbackTemperature) &&
        (!std::isfinite(f.temperature_c) || f.temperature_c >= board_config::temperature_limit_c))
        return -ERANGE;
    return 0;
}

int DjiChassisHardware::read(robotics::ChassisFeedback &out) {
    if (!initialized_)
        return -EACCES;
    robotics::ChassisFeedback next{};
    auto stamps = stamps_;
    float power = board_config::idle_power_w;
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        const auto snapshot = motors_[i].snapshot();
        const int ret = validateFeedback(i, snapshot, true);
        if (ret < 0)
            return ret;
        const motor::Feedback &f = snapshot.feedback;
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
    ready_ = stable_ = false;
    next_retry_ms_ = 0;
    if (!initialized_ || stopped_)
        return 0;
    stopped_ = true;
    group_.disable();
    return 0;
}

int DjiChassisHardware::pollRecovery(std::uint64_t now) {
    if (!initialized_ || armed())
        return -EACCES;
    if (now < next_retry_ms_)
        return -EAGAIN;
    const auto fail = [&](int error) {
        suspend();
        next_retry_ms_ = now + board_config::recovery_retry_ms;
        return error;
    };

    const auto group_status = group_.status();
    if (group_status.enable_pending)
        return -EAGAIN;
    if (!stopped_) {
        // A requested run that is neither active nor enabling has lost its
        // permit. Withdraw every target before rebuilding the control state.
        suspend();
        return -EAGAIN;
    }
    for (std::size_t i = 0; i < bus_count_; ++i) {
        const auto bus_status = buses_[i]->status();
        if (bus_status.state == motor::BusState::Recovering) {
            ready_ = stable_ = false;
            return -EAGAIN;
        }
        if (bus_status.state != motor::BusState::Running)
            return fail(bus_status.last_error < 0 ? bus_status.last_error : -EHOSTDOWN);
    }
    if (!group_.ready()) {
        ready_ = stable_ = false;
        for (const auto &motor : motors_) {
            const auto snapshot = motor.snapshot();
            if (snapshot.state == motor::MotorState::Fault)
                return snapshot.last_fault.error < 0 ? snapshot.last_fault.error : -EIO;
        }
        return -EAGAIN;
    }

    if (!stable_) {
        for (std::size_t i = 0; i < motors_.size(); ++i) {
            const auto snapshot = motors_[i].snapshot();
            const int checked = validateFeedback(i, snapshot, false);
            if (checked < 0)
                return fail(checked);
            const double seed = i < 4 ? static_cast<double>(snapshot.feedback.absolute_position_rad) : 0.0;
            const int ret = motors_[i].reseedPosition(seed);
            if (ret < 0)
                return fail(ret);
            first_stamps_[i] = snapshot.feedback.timestamp_ms;
            references_[i] = motors_[i].snapshot().reference_generation;
        }
        stable_ = true;
        stable_since_ms_ = now;
        return -EAGAIN;
    }

    robotics::ChassisFeedback feedback{};
    const int read_result = read(feedback);
    if (read_result < 0)
        return fail(read_result);
    for (std::size_t i = 0; i < motors_.size(); ++i)
        if (stamps_[i] == first_stamps_[i])
            return -EAGAIN;
    if (now < stable_since_ms_ || now - stable_since_ms_ < board_config::feedback_stable_ms)
        return -EAGAIN;
    if (!group_.ready())
        return fail(-EAGAIN);
    ready_ = true;
    return 0;
}

int DjiChassisHardware::arm() {
    if (!initialized_ || !ready_)
        return -EAGAIN;
    if (armed() || group_.status().enable_pending)
        return 0;
    robotics::ChassisFeedback feedback{};
    const int checked = read(feedback);
    if (checked < 0) {
        suspend();
        return checked;
    }
    const int ret = group_.enable();
    if (ret < 0) {
        suspend();
        return ret;
    }
    stopped_ = false;
    return 0;
}

int DjiChassisHardware::apply(const robotics::ChassisOutput &out, float scale) {
    if (!armed()) {
        suspend();
        return -EACCES;
    }
    if (!std::isfinite(scale) || scale < 0 || scale > 1) {
        suspend();
        return -EINVAL;
    }
    std::array<float, 8> current{};
    for (std::size_t i = 0; i < 4; ++i) {
        current[i] = out.module[i].steer_effort * scale * board_config::motor_direction[i];
        current[i + 4] = out.module[i].drive_effort * scale * board_config::motor_direction[i + 4];
    }
    for (std::size_t i = 0; i < current.size(); ++i) {
        if (!std::isfinite(current[i]) || std::fabs(current[i]) > descriptors_[i].configured_current_limit_a) {
            suspend();
            return -ERANGE;
        }
    }
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        const int ret = motors_[i].setCurrent(current[i]);
        if (ret < 0) {
            suspend();
            return ret;
        }
    }
    int first_error = 0;
    for (std::size_t i = 0; i < bus_count_; ++i) {
        const auto result = buses_[i]->commit();
        if (result.error < 0 && first_error == 0)
            first_error = result.error;
    }
    if (first_error < 0)
        suspend();
    return first_error;
}

int DjiChassisHardware::clearFault() {
    if (!initialized_)
        return -EACCES;
    return group_.clearFault();
}
