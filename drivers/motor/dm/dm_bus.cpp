#include <cstdint>
#include <errno.h>
#include <limits>

#include <zephyr/kernel.h>

#include <drivers/motor/dm_bus.hpp>

#include "dm_internal.hpp"

namespace skywalker::motor::dm {

int Bus::init(const struct device *can) {
    if (can == nullptr) {
        return -EINVAL;
    }
    if (state_ != BusState::Uninitialized) {
        return -EALREADY;
    }
    if (!device_is_ready(can)) {
        return -ENODEV;
    }

    const int ret = can_start(can);
    if (ret < 0 && ret != -EALREADY) {
        return ret;
    }

    can_ = can;
    state_ = BusState::Safe;
    return 0;
}

int Bus::routeIndex(std::uint16_t master_id) const {
    for (std::size_t i = 0; i < route_count_; ++i) {
        if (routes_[i].master_id == master_id) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int Bus::attach(const struct device *motor) {
    if (state_ != BusState::Safe) {
        return -EACCES;
    }
    if (!internal::isDmMotor(motor)) {
        return -ENOTSUP;
    }
    if (motor_count_ >= CONFIG_SKYWALKER_DM_MAX_MOTORS_PER_BUS) {
        return -ENOSPC;
    }

    Descriptor descriptor{};
    int ret = describe(motor, descriptor);
    if (ret < 0) {
        return ret;
    }
    if (descriptor.can != can_) {
        return -EXDEV;
    }

    for (std::size_t i = 0; i < motor_count_; ++i) {
        if (motors_[i] == motor) {
            return -EALREADY;
        }
        if (descriptors_[i].master_id == descriptor.master_id && descriptors_[i].motor_id == descriptor.motor_id) {
            return -EADDRINUSE;
        }
        if (descriptors_[i].control_id == descriptor.control_id) {
            return -EADDRINUSE;
        }
    }

    const bool needs_route = routeIndex(descriptor.master_id) < 0;
    if (needs_route && route_count_ >= CONFIG_SKYWALKER_DM_MAX_MASTER_IDS_PER_BUS) {
        return -ENOSPC;
    }

    ret = internal::claimMotor(motor, this);
    if (ret < 0) {
        return ret;
    }

    int filter_id = -1;
    if (needs_route) {
        struct can_filter filter{};
        filter.id = descriptor.master_id;
        filter.mask = CAN_STD_ID_MASK;
        filter.flags = 0u;
        filter_id = can_add_rx_filter(can_, Bus::rxCallback, this, &filter);
        if (filter_id < 0) {
            internal::releaseMotor(motor, this);
            return filter_id;
        }
    }

    const k_spinlock_key_t key = k_spin_lock(&route_lock_);
    if (needs_route) {
        routes_[route_count_].master_id = descriptor.master_id;
        routes_[route_count_].filter_id = filter_id;
        ++route_count_;
    }
    motors_[motor_count_] = motor;
    descriptors_[motor_count_] = descriptor;
    ++motor_count_;
    k_spin_unlock(&route_lock_, key);
    return 0;
}

void Bus::rxCallback(const struct device *, struct can_frame *frame, void *user_data) {
    Bus *bus = static_cast<Bus *>(user_data);
    if (bus == nullptr || frame == nullptr) {
        return;
    }
    bus->dispatchFeedback(*frame);
}

void Bus::dispatchFeedback(const struct can_frame &frame) {
    if (frame.dlc != 8u || (frame.flags & (CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF)) != 0u) {
        return;
    }

    const std::uint8_t motor_id = frame.data[0] & 0x0Fu;
    const struct device *target = nullptr;
    const k_spinlock_key_t key = k_spin_lock(&route_lock_);
    for (std::size_t i = 0; i < motor_count_; ++i) {
        if (descriptors_[i].master_id == frame.id && descriptors_[i].motor_id == motor_id) {
            target = motors_[i];
            break;
        }
    }
    k_spin_unlock(&route_lock_, key);

    if (target != nullptr) {
        const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
        if (now_ms != 0u) {
            (void)internal::acceptFeedback(target, frame, now_ms);
        }
    }
}

int Bus::sendFrame(const struct can_frame &frame, std::uint16_t motor_id, TxReport &report) {
    const int ret = can_send(can_, &frame, K_MSEC(2), nullptr, nullptr);
    if (ret < 0) {
        if (report.tx_error == 0) {
            report.tx_error = ret;
            report.failed_motor_id = motor_id;
        }
        return ret;
    }
    if (report.frames_sent < std::numeric_limits<std::uint8_t>::max()) {
        ++report.frames_sent;
    }
    return 0;
}

int Bus::disableAll(TxReport &report, bool latch_fault) {
    for (std::size_t i = 0; i < motor_count_; ++i) {
        internal::prepareMotorStop(motors_[i], latch_fault);
    }

    const std::size_t expected = static_cast<std::size_t>(report.frames_expected) + motor_count_;
    report.frames_expected = static_cast<std::uint8_t>(expected > std::numeric_limits<std::uint8_t>::max() ? std::numeric_limits<std::uint8_t>::max() : expected);

    int first_error = 0;
    for (std::size_t i = 0; i < motor_count_; ++i) {
        struct can_frame frame{};
        int ret = buildSpecialFrame(descriptors_[i].mode, descriptors_[i].motor_id, SpecialCommand::Disable, frame);
        if (ret == 0) {
            ret = sendFrame(frame, descriptors_[i].motor_id, report);
        }
        if (ret < 0 && first_error == 0) {
            first_error = ret;
            if (report.preparation_error == 0 && report.tx_error == 0) {
                report.preparation_error = ret;
                report.failed_motor_id = descriptors_[i].motor_id;
            }
        }
    }
    return first_error;
}

int Bus::enterFaultAndDisable(TxReport &report) {
    const int original_error = report.tx_error < 0 ? report.tx_error : report.preparation_error;
    (void)disableAll(report, true);
    recovery_started_ms_ = 0u;
    state_ = BusState::Fault;
    if (report.tx_error < 0) {
        return report.tx_error;
    }
    if (original_error < 0) {
        return original_error;
    }
    return -EIO;
}

int Bus::arm(TxReport &report) {
    report = {};
    if (state_ != BusState::Safe) {
        report.preparation_error = -EACCES;
        return report.preparation_error;
    }
    if (motor_count_ == 0u) {
        report.preparation_error = -ENODEV;
        return report.preparation_error;
    }
    if (lifecycle_epoch_ == std::numeric_limits<std::uint64_t>::max()) {
        report.preparation_error = -EOVERFLOW;
        return report.preparation_error;
    }

    struct can_frame neutral_frames[CONFIG_SKYWALKER_DM_MAX_MOTORS_PER_BUS]{};
    bool enable_required[CONFIG_SKYWALKER_DM_MAX_MOTORS_PER_BUS]{};
    std::size_t frames_expected = motor_count_;
    for (std::size_t i = 0; i < motor_count_; ++i) {
        int ret = internal::preflightArm(motors_[i], enable_required[i]);
        if (ret == 0) {
            ret = internal::buildNeutralFrame(motors_[i], neutral_frames[i]);
        }
        if (ret < 0) {
            report.preparation_error = ret;
            report.failed_motor_id = descriptors_[i].motor_id;
            return ret;
        }
        if (enable_required[i]) {
            ++frames_expected;
        }
    }

    ++lifecycle_epoch_;
    if (lifecycle_epoch_ == 0u) {
        ++lifecycle_epoch_;
    }
    report.frames_expected = static_cast<std::uint8_t>(frames_expected);

    for (std::size_t i = 0; i < motor_count_; ++i) {
        int ret = 0;
        if (enable_required[i]) {
            struct can_frame enable_frame{};
            ret = buildSpecialFrame(descriptors_[i].mode, descriptors_[i].motor_id, SpecialCommand::Enable, enable_frame);
            if (ret == 0) {
                ret = sendFrame(enable_frame, descriptors_[i].motor_id, report);
            }
            if (ret == 0) {
                // CAN transmission completion is not an Enable acknowledgement.
                // Wait before sending the first control frame; some drives need
                // time to leave Disabled after processing Enable.
                const std::int64_t deadline_ms = k_uptime_get() + 100;
                for (;;) {
                    RawFeedback feedback{};
                    const int feedback_ret = readRawFeedback(motors_[i], feedback);
                    const State motor_state = skywalker::motor::getState(motors_[i]);
                    if (feedback_ret == 0 && feedback.status == DriveStatus::Enabled &&
                        motor_state == State::Ready) {
                        break;
                    }
                    if (motor_state == State::Fault) {
                        ret = -EHOSTDOWN;
                        break;
                    }
                    if (k_uptime_get() >= deadline_ms) {
                        ret = -ETIMEDOUT;
                        break;
                    }
                    k_sleep(K_MSEC(1));
                }
            }
        }
        if (ret == 0) {
            ret = sendFrame(neutral_frames[i], descriptors_[i].motor_id, report);
        }
        if (ret < 0) {
            if (report.preparation_error == 0 && report.tx_error == 0) {
                report.preparation_error = ret;
                report.failed_motor_id = descriptors_[i].motor_id;
            }
            return enterFaultAndDisable(report);
        }
    }

    const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
    for (std::size_t i = 0; i < motor_count_; ++i) {
        const int ret = internal::armMotor(motors_[i], lifecycle_epoch_, now_ms == 0u ? 1u : now_ms, neutral_frames[i]);
        if (ret < 0) {
            report.preparation_error = ret;
            report.failed_motor_id = descriptors_[i].motor_id;
            return enterFaultAndDisable(report);
        }
    }

    recovery_started_ms_ = 0u;
    state_ = BusState::Armed;
    return 0;
}

int Bus::flush(TxReport &report) {
    report = {};
    if (state_ != BusState::Armed) {
        report.preparation_error = -EACCES;
        if (state_ == BusState::Uninitialized) {
            return report.preparation_error;
        }
        return enterFaultAndDisable(report);
    }

    internal::CommandSnapshot snapshots[CONFIG_SKYWALKER_DM_MAX_MOTORS_PER_BUS]{};
    const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
    for (std::size_t i = 0; i < motor_count_; ++i) {
        const int ret = internal::snapshotCommand(motors_[i], lifecycle_epoch_, now_ms, snapshots[i]);
        if (ret < 0) {
            report.preparation_error = ret;
            report.failed_motor_id = descriptors_[i].motor_id;
            return enterFaultAndDisable(report);
        }
    }

    report.frames_expected = static_cast<std::uint8_t>(motor_count_);
    for (std::size_t i = 0; i < motor_count_; ++i) {
        const int ret = sendFrame(snapshots[i].frame, descriptors_[i].motor_id, report);
        if (ret < 0) {
            return enterFaultAndDisable(report);
        }
    }
    return 0;
}

int Bus::stop(TxReport &report) {
    report = {};
    if (state_ == BusState::Uninitialized) {
        report.preparation_error = -EACCES;
        return report.preparation_error;
    }

    const bool preserve_fault = state_ == BusState::Fault;
    const int ret = disableAll(report, preserve_fault);
    recovery_started_ms_ = 0u;
    if (ret < 0) {
        for (std::size_t i = 0; i < motor_count_; ++i) {
            internal::prepareMotorStop(motors_[i], true);
        }
        state_ = BusState::Fault;
        return ret;
    }

    state_ = preserve_fault ? BusState::Fault : BusState::Safe;
    return 0;
}

int Bus::recover(TxReport &report) {
    report = {};
    if (state_ != BusState::Fault) {
        report.preparation_error = -EACCES;
        return report.preparation_error;
    }

    if (recovery_started_ms_ == 0u) {
        for (std::size_t i = 0; i < motor_count_; ++i) {
            internal::prepareMotorStop(motors_[i], true);
        }

        report.frames_expected = static_cast<std::uint8_t>(motor_count_ * 2u);
        std::uint64_t recovery_stamp = static_cast<std::uint64_t>(k_uptime_get());
        if (recovery_stamp == 0u) {
            recovery_stamp = 1u;
        }

        for (std::size_t i = 0; i < motor_count_; ++i) {
            struct can_frame clear_frame{};
            struct can_frame disable_frame{};
            int ret = buildSpecialFrame(descriptors_[i].mode, descriptors_[i].motor_id, SpecialCommand::ClearError, clear_frame);
            if (ret == 0) {
                ret = buildSpecialFrame(descriptors_[i].mode, descriptors_[i].motor_id, SpecialCommand::Disable, disable_frame);
            }
            if (ret == 0) {
                ret = sendFrame(clear_frame, descriptors_[i].motor_id, report);
            }
            if (ret == 0) {
                ret = sendFrame(disable_frame, descriptors_[i].motor_id, report);
            }
            if (ret < 0) {
                if (report.preparation_error == 0 && report.tx_error == 0) {
                    report.preparation_error = ret;
                    report.failed_motor_id = descriptors_[i].motor_id;
                }
                recovery_started_ms_ = 0u;
                return ret;
            }
        }

        recovery_started_ms_ = recovery_stamp;
        return -EINPROGRESS;
    }

    for (std::size_t i = 0; i < motor_count_; ++i) {
        if (!internal::recoveryReady(motors_[i], recovery_started_ms_)) {
            report.preparation_error = -EAGAIN;
            report.failed_motor_id = descriptors_[i].motor_id;
            return report.preparation_error;
        }
    }

    for (std::size_t i = 0; i < motor_count_; ++i) {
        internal::clearMotorFault(motors_[i]);
    }
    recovery_started_ms_ = 0u;
    state_ = BusState::Safe;
    return 0;
}

int Bus::savePositionZero(const struct device *motor, TxReport &report) {
    report = {};
    if (state_ != BusState::Safe) {
        report.preparation_error = -EACCES;
        return report.preparation_error;
    }
    if (motor == nullptr) {
        report.preparation_error = -EINVAL;
        return report.preparation_error;
    }

    std::size_t motor_index = motor_count_;
    for (std::size_t i = 0; i < motor_count_; ++i) {
        if (motors_[i] == motor) {
            motor_index = i;
            break;
        }
    }
    if (motor_index == motor_count_) {
        report.preparation_error = -ENOENT;
        return report.preparation_error;
    }

    const int preflight_ret = internal::preflightDisabled(motor);
    if (preflight_ret < 0) {
        report.preparation_error = preflight_ret;
        report.failed_motor_id = descriptors_[motor_index].motor_id;
        return preflight_ret;
    }

    struct can_frame frame{};
    const int build_ret = buildSpecialFrame(descriptors_[motor_index].mode, descriptors_[motor_index].motor_id, SpecialCommand::SaveZero, frame);
    if (build_ret < 0) {
        report.preparation_error = build_ret;
        report.failed_motor_id = descriptors_[motor_index].motor_id;
        return build_ret;
    }

    report.frames_expected = 1u;
    return sendFrame(frame, descriptors_[motor_index].motor_id, report);
}

BusState Bus::state() const {
    return state_;
}

} // namespace skywalker::motor::dm
