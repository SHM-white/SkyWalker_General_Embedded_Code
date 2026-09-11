#include <cmath>
#include <cstdint>
#include <errno.h>

#include <zephyr/kernel.h>

#include "dm_internal.hpp"

namespace skywalker::motor::dm {
namespace internal {
namespace {

DmData *dataOf(const struct device *dev) {
    return dev == nullptr ? nullptr : static_cast<DmData *>(dev->data);
}

const DmConfig *configOf(const struct device *dev) {
    return dev == nullptr ? nullptr : static_cast<const DmConfig *>(dev->config);
}

bool feedbackFreshLocked(const DmData &data, std::uint64_t now_ms) {
    return data.last_rx_ms != 0u && now_ms >= data.last_rx_ms && now_ms - data.last_rx_ms <= CONFIG_SKYWALKER_DM_FEEDBACK_TIMEOUT_MS;
}

std::uint32_t getCapabilities(const struct device *dev) {
    const DmConfig *cfg = configOf(dev);
    if (cfg == nullptr) {
        return 0u;
    }

    std::uint32_t capabilities = FeedbackPosition | FeedbackVelocity | FeedbackTorque | FeedbackTemperature;
    if (cfg->mode == ControlMode::Mit) {
        capabilities |= CommandTorque;
    }
    return capabilities;
}

State getStateImpl(const struct device *dev) {
    DmData *data = dataOf(dev);
    if (data == nullptr) {
        return State::Offline;
    }

    const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    const bool fault = data->fault_latched || isFaultStatus(data->drive_status);
    const bool fresh = feedbackFreshLocked(*data, now_ms);
    const bool has_feedback = data->last_rx_ms != 0u;
    k_spin_unlock(&data->lock, key);

    if (!has_feedback) {
        return State::Offline;
    }
    if (fault) {
        return State::Fault;
    }
    return fresh ? State::Ready : State::Offline;
}

int readFeedbackImpl(const struct device *dev, Feedback *out) {
    DmData *data = dataOf(dev);
    if (data == nullptr || out == nullptr) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    *out = data->feedback;
    k_spin_unlock(&data->lock, key);
    return 0;
}

int stageFrame(const struct device *dev, ControlMode required_mode, const struct can_frame &frame) {
    DmData *data = dataOf(dev);
    const DmConfig *cfg = configOf(dev);
    if (data == nullptr || cfg == nullptr) {
        return -EINVAL;
    }
    if (cfg->mode != required_mode) {
        return -ENOTSUP;
    }

    const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    int ret = 0;
    if (!data->armed || data->active_epoch == 0u) {
        ret = -EACCES;
    } else if (data->fault_latched || isFaultStatus(data->drive_status)) {
        ret = -EHOSTDOWN;
    } else if (!feedbackFreshLocked(*data, now_ms)) {
        ret = -EHOSTDOWN;
    } else {
        data->command_frame = frame;
        data->command_stamp_ms = now_ms;
        data->command_epoch = data->active_epoch;
        data->command_valid = true;
    }
    k_spin_unlock(&data->lock, key);
    return ret;
}

int setTorqueImpl(const struct device *dev, float torque_nm) {
    MitCommand command{};
    command.torque_ff_nm = torque_nm;
    return setMitCommand(dev, command);
}

} // namespace

const skywalker::motor::Api dm_motor_api = {
    getCapabilities, nullptr, setTorqueImpl, readFeedbackImpl, getStateImpl,
};

int dmMotorInit(const struct device *dev) {
    DmData *data = dataOf(dev);
    const DmConfig *cfg = configOf(dev);
    if (dev == nullptr || data == nullptr || cfg == nullptr || cfg->can == nullptr) {
        return -EINVAL;
    }
    if (!device_is_ready(cfg->can)) {
        return -ENODEV;
    }
    if (cfg->motor_id == 0u || cfg->motor_id > 15u || cfg->master_id > CAN_STD_ID_MASK) {
        return -ERANGE;
    }
    if (cfg->mode != ControlMode::Mit && cfg->mode != ControlMode::PositionVelocity && cfg->mode != ControlMode::Velocity) {
        return -EINVAL;
    }

    Limits limits{};
    limits.position_max_rad = static_cast<float>(cfg->p_max_millirad) / 1000.0f;
    limits.velocity_max_rad_s = static_cast<float>(cfg->v_max_millirad_s) / 1000.0f;
    limits.torque_max_nm = static_cast<float>(cfg->t_max_millinewton_meter) / 1000.0f;
    const float torque_limit = static_cast<float>(cfg->torque_limit_millinewton_meter) / 1000.0f;
    if (!std::isfinite(limits.position_max_rad) || limits.position_max_rad <= 0.0f || !std::isfinite(limits.velocity_max_rad_s) || limits.velocity_max_rad_s <= 0.0f ||
        !std::isfinite(limits.torque_max_nm) || limits.torque_max_nm <= 0.0f || !std::isfinite(torque_limit) || torque_limit <= 0.0f || torque_limit > limits.torque_max_nm) {
        return -ERANGE;
    }

    std::uint16_t command_id = 0u;
    const int id_ret = controlFrameId(cfg->mode, static_cast<std::uint16_t>(cfg->motor_id), command_id);
    if (id_ret < 0) {
        return id_ret;
    }

    data->feedback = {};
    data->raw_feedback = {};
    data->drive_status = DriveStatus::Unknown;
    data->limits = limits;
    data->torque_limit_nm = torque_limit;
    data->command_frame = {};
    data->last_rx_ms = 0u;
    data->command_stamp_ms = 0u;
    data->command_epoch = 0u;
    data->active_epoch = 0u;
    data->armed_at_ms = 0u;
    data->command_valid = false;
    data->armed = false;
    data->fault_latched = false;
    data->owner = nullptr;
    return 0;
}

bool isDmMotor(const struct device *dev) {
    return dev != nullptr && dev->api == &dm_motor_api;
}

int claimMotor(const struct device *dev, Bus *owner) {
    DmData *data = dataOf(dev);
    if (data == nullptr || owner == nullptr) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    int ret = 0;
    if (data->owner == owner) {
        ret = -EALREADY;
    } else if (data->owner != nullptr) {
        ret = -EBUSY;
    } else {
        data->owner = owner;
    }
    k_spin_unlock(&data->lock, key);
    return ret;
}

void releaseMotor(const struct device *dev, Bus *owner) {
    DmData *data = dataOf(dev);
    if (data == nullptr || owner == nullptr) {
        return;
    }

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    if (data->owner == owner) {
        data->owner = nullptr;
    }
    k_spin_unlock(&data->lock, key);
}

int acceptFeedback(const struct device *dev, const struct can_frame &frame, std::uint64_t now_ms) {
    DmData *data = dataOf(dev);
    const DmConfig *cfg = configOf(dev);
    if (data == nullptr || cfg == nullptr || now_ms == 0u) {
        return -EINVAL;
    }

    DecodedFeedback decoded{};
    const int ret = decodeFeedback(frame, static_cast<std::uint8_t>(cfg->motor_id), data->limits, decoded);
    if (ret < 0) {
        return ret;
    }
    decoded.raw.timestamp_ms = now_ms;

    Feedback feedback{};
    feedback.position_rad = decoded.position_rad;
    feedback.velocity_rad_s = decoded.velocity_rad_s;
    feedback.torque_nm = decoded.torque_nm;
    feedback.temperature_c = static_cast<float>(decoded.raw.rotor_temperature_c);
    feedback.valid = FeedbackPosition | FeedbackVelocity | FeedbackTorque | FeedbackTemperature;
    feedback.timestamp_ms = now_ms;

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->raw_feedback = decoded.raw;
    data->feedback = feedback;
    data->drive_status = decoded.raw.status;
    data->last_rx_ms = now_ms;
    if (isFaultStatus(decoded.raw.status)) {
        data->fault_latched = true;
        data->armed = false;
        data->active_epoch = 0u;
        data->command_valid = false;
    }
    k_spin_unlock(&data->lock, key);
    return 0;
}

int preflightArm(const struct device *dev) {
    DmData *data = dataOf(dev);
    if (data == nullptr) {
        return -EINVAL;
    }

    const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    int ret = 0;
    if (data->fault_latched || isFaultStatus(data->drive_status)) {
        ret = -EHOSTDOWN;
    } else if (!feedbackFreshLocked(*data, now_ms)) {
        ret = -EHOSTDOWN;
    } else if (data->drive_status != DriveStatus::Disabled) {
        ret = -EBUSY;
    }
    k_spin_unlock(&data->lock, key);
    return ret;
}

int buildNeutralFrame(const struct device *dev, struct can_frame &out) {
    DmData *data = dataOf(dev);
    const DmConfig *cfg = configOf(dev);
    if (data == nullptr || cfg == nullptr) {
        return -EINVAL;
    }

    float current_position = 0.0f;
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    current_position = data->feedback.position_rad;
    k_spin_unlock(&data->lock, key);

    switch (cfg->mode) {
    case ControlMode::Mit:
        return buildMitFrame(static_cast<std::uint16_t>(cfg->motor_id), data->limits, MitCommand{}, out);
    case ControlMode::PositionVelocity:
        return buildPositionVelocityFrame(static_cast<std::uint16_t>(cfg->motor_id), current_position, 0.0f, out);
    case ControlMode::Velocity:
        return buildVelocityFrame(static_cast<std::uint16_t>(cfg->motor_id), 0.0f, out);
    }
    return -EINVAL;
}

int armMotor(const struct device *dev, std::uint64_t epoch, std::uint64_t now_ms, const struct can_frame &neutral_frame) {
    DmData *data = dataOf(dev);
    if (data == nullptr || epoch == 0u || now_ms == 0u) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->command_frame = neutral_frame;
    data->command_stamp_ms = now_ms;
    data->command_epoch = epoch;
    data->active_epoch = epoch;
    data->armed_at_ms = now_ms;
    data->command_valid = true;
    data->armed = true;
    k_spin_unlock(&data->lock, key);
    return 0;
}

void prepareMotorStop(const struct device *dev, bool latch_fault) {
    DmData *data = dataOf(dev);
    if (data == nullptr) {
        return;
    }

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->armed = false;
    data->active_epoch = 0u;
    data->command_epoch = 0u;
    data->command_stamp_ms = 0u;
    data->command_valid = false;
    if (latch_fault) {
        data->fault_latched = true;
    }
    k_spin_unlock(&data->lock, key);
}

void clearMotorFault(const struct device *dev) {
    DmData *data = dataOf(dev);
    if (data == nullptr) {
        return;
    }

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->fault_latched = false;
    data->armed = false;
    data->active_epoch = 0u;
    k_spin_unlock(&data->lock, key);
}

bool recoveryReady(const struct device *dev, std::uint64_t recovery_started_ms) {
    DmData *data = dataOf(dev);
    if (data == nullptr || recovery_started_ms == 0u) {
        return false;
    }

    const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    const bool ready = feedbackFreshLocked(*data, now_ms) && data->last_rx_ms > recovery_started_ms && data->drive_status == DriveStatus::Disabled;
    k_spin_unlock(&data->lock, key);
    return ready;
}

int snapshotCommand(const struct device *dev, std::uint64_t expected_epoch, std::uint64_t now_ms, CommandSnapshot &out) {
    DmData *data = dataOf(dev);
    if (data == nullptr || expected_epoch == 0u) {
        return -EINVAL;
    }

    CommandSnapshot next{};
    int ret = 0;
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    if (!data->armed || data->fault_latched || isFaultStatus(data->drive_status)) {
        ret = -EACCES;
    } else if (!data->command_valid || data->active_epoch != expected_epoch || data->command_epoch != expected_epoch) {
        ret = -ESTALE;
    } else if (!feedbackFreshLocked(*data, now_ms)) {
        ret = -EHOSTDOWN;
    } else if (data->command_stamp_ms == 0u || now_ms < data->command_stamp_ms || now_ms - data->command_stamp_ms > CONFIG_SKYWALKER_DM_COMMAND_TIMEOUT_MS) {
        ret = -ESTALE;
    } else {
        next.frame = data->command_frame;
    }
    k_spin_unlock(&data->lock, key);

    if (ret < 0) {
        return ret;
    }
    out = next;
    return 0;
}

} // namespace internal

int describe(const struct device *dev, Descriptor &out) {
    if (!internal::isDmMotor(dev)) {
        return -ENOTSUP;
    }
    const internal::DmConfig *cfg = static_cast<const internal::DmConfig *>(dev->config);
    internal::DmData *data = static_cast<internal::DmData *>(dev->data);
    if (cfg == nullptr || data == nullptr) {
        return -EINVAL;
    }

    Descriptor next{};
    next.model = cfg->model;
    next.can = cfg->can;
    next.mode = cfg->mode;
    next.motor_id = static_cast<std::uint16_t>(cfg->motor_id);
    next.master_id = static_cast<std::uint16_t>(cfg->master_id);
    const int ret = controlFrameId(next.mode, next.motor_id, next.control_id);
    if (ret < 0) {
        return ret;
    }
    next.limits = data->limits;
    next.torque_limit_nm = data->torque_limit_nm;
    out = next;
    return 0;
}

int setMitCommand(const struct device *dev, const MitCommand &command) {
    if (!internal::isDmMotor(dev)) {
        return -ENOTSUP;
    }
    const internal::DmConfig *cfg = static_cast<const internal::DmConfig *>(dev->config);
    internal::DmData *data = static_cast<internal::DmData *>(dev->data);
    if (cfg == nullptr || data == nullptr) {
        return -EINVAL;
    }
    if (!std::isfinite(command.torque_ff_nm) || std::fabs(command.torque_ff_nm) > data->torque_limit_nm) {
        return -ERANGE;
    }

    struct can_frame frame{};
    const int ret = buildMitFrame(static_cast<std::uint16_t>(cfg->motor_id), data->limits, command, frame);
    if (ret < 0) {
        return ret;
    }
    return internal::stageFrame(dev, ControlMode::Mit, frame);
}

int setPositionVelocity(const struct device *dev, float position_rad, float velocity_limit_rad_s) {
    if (!internal::isDmMotor(dev)) {
        return -ENOTSUP;
    }
    const internal::DmConfig *cfg = static_cast<const internal::DmConfig *>(dev->config);
    internal::DmData *data = static_cast<internal::DmData *>(dev->data);
    if (cfg == nullptr || data == nullptr) {
        return -EINVAL;
    }
    if (!std::isfinite(position_rad) || !std::isfinite(velocity_limit_rad_s)) {
        return -EINVAL;
    }
    if (std::fabs(position_rad) > data->limits.position_max_rad || velocity_limit_rad_s < 0.0f || velocity_limit_rad_s > data->limits.velocity_max_rad_s) {
        return -ERANGE;
    }

    struct can_frame frame{};
    const int ret = buildPositionVelocityFrame(static_cast<std::uint16_t>(cfg->motor_id), position_rad, velocity_limit_rad_s, frame);
    if (ret < 0) {
        return ret;
    }
    return internal::stageFrame(dev, ControlMode::PositionVelocity, frame);
}

int setVelocity(const struct device *dev, float velocity_rad_s) {
    if (!internal::isDmMotor(dev)) {
        return -ENOTSUP;
    }
    const internal::DmConfig *cfg = static_cast<const internal::DmConfig *>(dev->config);
    internal::DmData *data = static_cast<internal::DmData *>(dev->data);
    if (cfg == nullptr || data == nullptr) {
        return -EINVAL;
    }
    if (!std::isfinite(velocity_rad_s)) {
        return -EINVAL;
    }
    if (std::fabs(velocity_rad_s) > data->limits.velocity_max_rad_s) {
        return -ERANGE;
    }

    struct can_frame frame{};
    const int ret = buildVelocityFrame(static_cast<std::uint16_t>(cfg->motor_id), velocity_rad_s, frame);
    if (ret < 0) {
        return ret;
    }
    return internal::stageFrame(dev, ControlMode::Velocity, frame);
}

int readRawFeedback(const struct device *dev, RawFeedback &out) {
    if (!internal::isDmMotor(dev)) {
        return -ENOTSUP;
    }
    internal::DmData *data = static_cast<internal::DmData *>(dev->data);
    if (data == nullptr) {
        return -EINVAL;
    }

    RawFeedback next{};
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    next = data->raw_feedback;
    k_spin_unlock(&data->lock, key);
    if (next.timestamp_ms == 0u) {
        return -ENODATA;
    }
    out = next;
    return 0;
}

int getDriveStatus(const struct device *dev, DriveStatus &out) {
    if (!internal::isDmMotor(dev)) {
        return -ENOTSUP;
    }
    internal::DmData *data = static_cast<internal::DmData *>(dev->data);
    if (data == nullptr) {
        return -EINVAL;
    }

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    const DriveStatus status = data->drive_status;
    const std::uint64_t timestamp_ms = data->last_rx_ms;
    k_spin_unlock(&data->lock, key);
    if (timestamp_ms == 0u) {
        return -ENODATA;
    }
    out = status;
    return 0;
}

} // namespace skywalker::motor::dm
