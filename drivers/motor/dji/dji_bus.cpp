#include <errno.h>
#include <limits>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include <drivers/motor/dji_bus.hpp>

#include "dji_internal.hpp"

namespace skywalker::motor::dji
{

namespace
{

    struct OwnerEntry
    {
        const struct device *can = nullptr;
        Bus *owner = nullptr;
    };

    OwnerEntry owner_registry[CONFIG_SKYWALKER_DJI_MAX_BUSES]{};
    struct k_spinlock owner_registry_lock{};

    int claimCan(const struct device *can, Bus *owner)
    {
        const k_spinlock_key_t key = k_spin_lock(&owner_registry_lock);
        int empty_index = -1;

        for (std::size_t i = 0; i < CONFIG_SKYWALKER_DJI_MAX_BUSES; ++i)
        {
            if (owner_registry[i].can == can)
            {
                const int ret = owner_registry[i].owner == owner ? -EALREADY : -EBUSY;
                k_spin_unlock(&owner_registry_lock, key);
                return ret;
            }
            if (owner_registry[i].can == nullptr && empty_index < 0)
            {
                empty_index = static_cast<int>(i);
            }
        }

        if (empty_index < 0)
        {
            k_spin_unlock(&owner_registry_lock, key);
            return -ENOSPC;
        }

        owner_registry[empty_index].can = can;
        owner_registry[empty_index].owner = owner;
        k_spin_unlock(&owner_registry_lock, key);
        return 0;
    }

    void releaseCan(const struct device *can, Bus *owner)
    {
        const k_spinlock_key_t key = k_spin_lock(&owner_registry_lock);
        for (std::size_t i = 0; i < CONFIG_SKYWALKER_DJI_MAX_BUSES; ++i)
        {
            if (owner_registry[i].can == can && owner_registry[i].owner == owner)
            {
                owner_registry[i] = {};
                break;
            }
        }
        k_spin_unlock(&owner_registry_lock, key);
    }

    int reportError(const FlushReport &report)
    {
        if (report.zero_tx_error < 0)
        {
            return report.zero_tx_error;
        }
        if (report.command_tx_error < 0)
        {
            return report.command_tx_error;
        }
        return report.preparation_error;
    }

} // namespace

int Bus::init(const struct device *can)
{
    if (can == nullptr)
        return -EINVAL;
    if (state_ != BusState::Uninitialized)
        return -EALREADY;
    if (!device_is_ready(can))
        return -ENODEV;

    int ret = claimCan(can, this);
    if (ret < 0)
        return ret;

    ret = can_start(can);
    if (ret < 0 && ret != -EALREADY)
    {
        releaseCan(can, this);
        return ret;
    }

    can_ = can;
    state_ = BusState::Safe;
    return 0;
}

int Bus::groupIndex(std::uint16_t command_id) const
{
    for (std::size_t i = 0; i < group_count_; ++i)
    {
        if (group_ids_[i] == command_id)
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int Bus::attach(const struct device *motor)
{
    if (state_ != BusState::Safe)
        return -EACCES;
    if (!internal::isDjiMotor(motor))
        return -ENOTSUP;
    if (motor_count_ >=
        CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS)
    {
        return -ENOSPC;
    }

    Descriptor descriptor{};
    int ret = describe(motor, descriptor);
    if (ret < 0)
        return ret;
    if (descriptor.can != can_)
        return -EXDEV;

    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        if (motors_[i] == motor)
            return -EALREADY;
        if (descriptors_[i].feedback_id == descriptor.feedback_id)
        {
            return -EADDRINUSE;
        }
        if (descriptors_[i].command_id == descriptor.command_id && descriptors_[i].command_slot == descriptor.command_slot)
        {
            return -EADDRINUSE;
        }
    }

    if (groupIndex(descriptor.command_id) < 0)
    {
        if (group_count_ >= CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS)
        {
            return -ENOSPC;
        }
        group_ids_[group_count_++] = descriptor.command_id;
    }

    motors_[motor_count_] = motor;
    descriptors_[motor_count_] = descriptor;
    ++motor_count_;
    return 0;
}

int Bus::sendZeros(FlushReport &report)
{
    report.zero_tx_error = 0;
    report.zero_failed_command_id = 0;
    report.zero_groups_expected = static_cast<std::uint8_t>(group_count_);
    report.zero_groups_attempted = 0;
    report.zero_groups_succeeded = 0;
    report.zero_sent = false;

    const std::int16_t zeros[4] = {0, 0, 0, 0};
    for (std::size_t i = 0; i < group_count_; ++i)
    {
        struct can_frame frame{};
        int ret = buildCommandFrame(frame, group_ids_[i], zeros);
        if (ret == 0)
        {
            ++report.zero_groups_attempted;
            ret = can_send(can_, &frame, K_MSEC(2), nullptr, nullptr);
        }

        if (ret < 0)
        {
            if (report.zero_tx_error == 0)
            {
                report.zero_tx_error = ret;
                report.zero_failed_command_id = group_ids_[i];
            }
        }
        else
        {
            ++report.zero_groups_succeeded;
        }
    }

    report.zero_sent = report.zero_groups_expected > 0u && report.zero_groups_attempted == report.zero_groups_expected && report.zero_groups_succeeded == report.zero_groups_expected;
    return report.zero_tx_error;
}

int Bus::enterFaultAndZero(FlushReport &report)
{
    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        internal::prepareMotorStop(motors_[i], true);
    }
    state_ = BusState::Fault;
    sendZeros(report);
    return reportError(report);
}

int Bus::arm(FlushReport &report)
{
    report = {};
    if (state_ != BusState::Safe)
    {
        report.preparation_error = -EACCES;
        return report.preparation_error;
    }
    if (motor_count_ == 0u)
    {
        report.preparation_error = -ENODEV;
        return report.preparation_error;
    }

    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        if (!internal::feedbackReady(motors_[i]))
        {
            report.preparation_error = -EHOSTDOWN;
            report.preparation_failed_slot = descriptors_[i].command_slot;
            return enterFaultAndZero(report);
        }
    }

    if (sendZeros(report) < 0 || !report.zero_sent)
    {
        for (std::size_t i = 0; i < motor_count_; ++i)
        {
            internal::prepareMotorStop(motors_[i], true);
        }
        state_ = BusState::Fault;
        return reportError(report);
    }

    if (lifecycle_epoch_ ==
        std::numeric_limits<std::uint64_t>::max())
    {
        report.preparation_error = -EOVERFLOW;
        return enterFaultAndZero(report);
    }
    ++lifecycle_epoch_;
    if (lifecycle_epoch_ == 0u)
        ++lifecycle_epoch_;

    const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        const int ret = internal::armMotor(motors_[i], lifecycle_epoch_, now_ms);
        if (ret < 0)
        {
            report.preparation_error = ret;
            report.preparation_failed_slot = descriptors_[i].command_slot;
            return enterFaultAndZero(report);
        }
    }

    state_ = BusState::Armed;
    return 0;
}

int Bus::flush(FlushReport &report)
{
    report = {};
    if (state_ != BusState::Armed)
    {
        report.preparation_error = -EACCES;
        if (state_ == BusState::Uninitialized)
        {
            return report.preparation_error;
        }
        return enterFaultAndZero(report);
    }

    std::int16_t commands[CONFIG_SKYWALKER_DJI_MAX_MOTORS_PER_BUS][4]{};
    const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());

    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        internal::CommandSnapshot snapshot{};
        const int ret = internal::snapshotCommand(motors_[i], lifecycle_epoch_, now_ms, snapshot);
        if (ret < 0)
        {
            report.preparation_error = ret;
            report.preparation_failed_slot = descriptors_[i].command_slot;
            return enterFaultAndZero(report);
        }

        const int group = groupIndex(snapshot.command_id);
        if (group < 0 || snapshot.command_slot >= 4u)
        {
            report.preparation_error = -EFAULT;
            report.preparation_failed_slot = snapshot.command_slot;
            return enterFaultAndZero(report);
        }
        commands[group][snapshot.command_slot] = snapshot.command_raw;
    }

    for (std::size_t i = 0; i < group_count_; ++i)
    {
        struct can_frame frame{};
        int ret =
            buildCommandFrame(frame, group_ids_[i], commands[i]);
        if (ret == 0)
        {
            ret = can_send(can_, &frame, K_MSEC(2), nullptr, nullptr);
        }
        if (ret < 0)
        {
            report.command_tx_error = ret;
            report.target_failed_command_id = group_ids_[i];
            return enterFaultAndZero(report);
        }
    }
    return 0;
}

int Bus::stop(FlushReport &report)
{
    report = {};
    if (state_ == BusState::Uninitialized)
    {
        report.preparation_error = -EACCES;
        return report.preparation_error;
    }

    const bool was_fault = state_ == BusState::Fault;
    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        internal::prepareMotorStop(motors_[i], was_fault);
    }

    if (sendZeros(report) < 0 || !report.zero_sent)
    {
        for (std::size_t i = 0; i < motor_count_; ++i)
        {
            internal::prepareMotorStop(motors_[i], true);
        }
        state_ = BusState::Fault;
        return reportError(report);
    }

    if (was_fault)
    {
        state_ = BusState::Fault;
    }
    else
    {
        for (std::size_t i = 0; i < motor_count_; ++i)
        {
            internal::clearMotorFault(motors_[i]);
        }
        state_ = BusState::Safe;
    }
    return 0;
}

int Bus::recover(FlushReport &report)
{
    report = {};
    if (state_ != BusState::Fault)
    {
        report.preparation_error = -EACCES;
        return report.preparation_error;
    }

    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        internal::prepareMotorStop(motors_[i], true);
    }
    if (sendZeros(report) < 0 || !report.zero_sent)
    {
        state_ = BusState::Fault;
        return reportError(report);
    }

    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        internal::clearMotorFault(motors_[i]);
    }
    state_ = BusState::Safe;
    return 0;
}

BusState Bus::state() const
{
    return state_;
}

} // namespace skywalker::motor::dji
