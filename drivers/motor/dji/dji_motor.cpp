#include <cstdint>
#include <cmath>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>

#include "dji_internal.hpp"

namespace skywalker::motor::dji {
namespace internal {

namespace {

constexpr float kTwoPi = 6.28318530717958647692f;
DjiData *dataOf(const struct device *dev) {
    return dev == nullptr ? nullptr : static_cast<DjiData *>(dev->data);
}

const DjiConfig *configOf(const struct device *dev) {
    return dev == nullptr ? nullptr : static_cast<const DjiConfig *>(dev->config);
}

std::uint32_t getCapabilities(const struct device *dev) {
    const DjiConfig *cfg = configOf(dev);
    if (cfg == nullptr || cfg->profile == nullptr)
        return 0u;

    std::uint32_t caps = CommandCurrent | FeedbackPosition | FeedbackVelocity | FeedbackCurrent;
    if (cfg->profile->temperature_valid) {
        caps |= FeedbackTemperature;
    }
    /* A rotor single-turn angle maps uniquely to the output only at 1:1. */
    if (cfg->profile->position_sensor == PositionSensorType::FixedZeroSingleTurn &&
        cfg->gear_ratio_num == cfg->gear_ratio_den) {
        caps |= FeedbackAbsolutePosition;
    }
    return caps;
}

State getStateImpl(const struct device *dev) {
    DjiData *data = dataOf(dev);
    if (data == nullptr)
        return State::Offline;

    bool fault = false;
    std::uint64_t last_rx_ms = 0;
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    fault = data->fault_latched;
    last_rx_ms = data->last_rx_ms;
    k_spin_unlock(&data->lock, key);

    if (fault)
        return State::Fault;
    if (last_rx_ms == 0u)
        return State::Offline;

    const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
    if (now_ms < last_rx_ms || now_ms - last_rx_ms > CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS) {
        return State::Offline;
    }
    return State::Ready;
}

int setCurrentImpl(const struct device *dev, float current_a) {
    DjiData *data = dataOf(dev);
    const DjiConfig *cfg = configOf(dev);
    if (data == nullptr || cfg == nullptr || cfg->profile == nullptr) {
        return -EINVAL;
    }
    if (!std::isfinite(current_a))
        return -EINVAL;
    if (std::fabs(current_a) > data->current_limit_a) {
        return -ERANGE;
    }
    if (getStateImpl(dev) != State::Ready) {
        return -EHOSTDOWN;
    }

    std::int16_t raw = 0;
    const int convert_ret = currentToRaw(*cfg->profile, current_a, raw);
    if (convert_ret < 0)
        return convert_ret;

    const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    if (!data->armed || data->fault_latched || data->active_epoch == 0u) {
        k_spin_unlock(&data->lock, key);
        return -EACCES;
    }

    data->command_raw = raw;
    data->command_stamp_ms = now_ms;
    ++data->command_generation;
    data->command_epoch = data->active_epoch;
    k_spin_unlock(&data->lock, key);
    return 0;
}

int readFeedbackImpl(const struct device *dev, Feedback *out) {
    DjiData *data = dataOf(dev);
    if (data == nullptr || out == nullptr)
        return -EINVAL;

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    *out = data->feedback;
    k_spin_unlock(&data->lock, key);
    return 0;
}

void rxCallback(const struct device *, struct can_frame *frame, void *user_data) {
    const struct device *dev = static_cast<const struct device *>(user_data);
    DjiData *data = dataOf(dev);
    const DjiConfig *cfg = configOf(dev);
    if (dev == nullptr || data == nullptr || cfg == nullptr || cfg->profile == nullptr || frame == nullptr) {
        return;
    }

    RawFeedback raw{};
    if (!decodeFeedback(*frame, raw))
        return;
    const std::int32_t ticks_per_turn = static_cast<std::int32_t>(cfg->profile->encoder_ticks_per_turn);
    if (ticks_per_turn <= 0 || raw.encoder >= ticks_per_turn)
        return;

    const std::uint64_t now_ms = static_cast<std::uint64_t>(k_uptime_get());
    raw.timestamp_ms = now_ms;

    const k_spinlock_key_t key = k_spin_lock(&data->lock);

    if (!data->has_encoder) {
        data->last_encoder = raw.encoder;
        data->total_encoder_ticks = 0;
        data->has_encoder = true;
    }
    else {
        std::int32_t delta = static_cast<std::int32_t>(raw.encoder) - static_cast<std::int32_t>(data->last_encoder);
        const std::int32_t half_turn = ticks_per_turn / 2;
        if (delta > half_turn)
            delta -= ticks_per_turn;
        if (delta < -half_turn)
            delta += ticks_per_turn;
        data->total_encoder_ticks += delta;
        data->last_encoder = raw.encoder;
    }

    Feedback next{};
    next.position_rad = static_cast<float>(data->total_encoder_ticks) * (kTwoPi / static_cast<float>(ticks_per_turn)) /
                        data->gear_ratio;
    next.valid |= FeedbackPosition;

    if (cfg->profile->position_sensor == PositionSensorType::FixedZeroSingleTurn &&
        cfg->gear_ratio_num == cfg->gear_ratio_den) {
        std::int32_t relative_ticks = static_cast<std::int32_t>(raw.encoder) -
                                      static_cast<std::int32_t>(cfg->encoder_zero_ticks);
        const std::int32_t half_turn = ticks_per_turn / 2;
        if (relative_ticks >= half_turn)
            relative_ticks -= ticks_per_turn;
        if (relative_ticks < -half_turn)
            relative_ticks += ticks_per_turn;

        next.absolute_position_rad = static_cast<float>(relative_ticks) * (kTwoPi / static_cast<float>(ticks_per_turn));
        next.valid |= FeedbackAbsolutePosition;
    }

    next.velocity_rad_s = static_cast<float>(raw.speed_rpm) * (kTwoPi / 60.0f) / data->gear_ratio;
    next.valid |= FeedbackVelocity;

    float feedback_current_a = 0.0f;
    if (rawToCurrent(*cfg->profile, raw.current_raw, feedback_current_a) == 0) {
        next.current_a = feedback_current_a;
        next.valid |= FeedbackCurrent;
    }

    if (cfg->profile->temperature_valid) {
        next.temperature_c = static_cast<float>(raw.temperature_raw);
        next.valid |= FeedbackTemperature;
    }

    next.timestamp_ms = now_ms;
    data->raw_feedback = raw;
    data->feedback = next;
    data->last_rx_ms = now_ms;

    k_spin_unlock(&data->lock, key);
}

} // namespace

const skywalker::motor::Api dji_motor_api = {
    getCapabilities, setCurrentImpl, nullptr, readFeedbackImpl, getStateImpl,
};

int djiMotorInit(const struct device *dev) {
    DjiData *data = dataOf(dev);
    const DjiConfig *cfg = configOf(dev);
    if (dev == nullptr || data == nullptr || cfg == nullptr || cfg->can == nullptr || cfg->profile == nullptr) {
        return -EINVAL;
    }
    if (!device_is_ready(cfg->can))
        return -ENODEV;
    if (cfg->motor_id == 0u || cfg->motor_id > cfg->profile->max_motor_id || cfg->motor_id > 255u) {
        return -ERANGE;
    }
    if (cfg->gear_ratio_num == 0u || cfg->gear_ratio_den == 0u) {
        return -EINVAL;
    }
    if (cfg->profile->encoder_ticks_per_turn == 0u || (cfg->profile->encoder_ticks_per_turn % 2u) != 0u) {
        return -EINVAL;
    }
    if (cfg->profile->position_sensor == PositionSensorType::FixedZeroSingleTurn &&
        cfg->encoder_zero_ticks >= cfg->profile->encoder_ticks_per_turn) {
        return -ERANGE;
    }

    const float current_limit_a = static_cast<float>(cfg->current_limit_ma) / 1000.0f;
    const float gear_ratio = static_cast<float>(cfg->gear_ratio_num) / static_cast<float>(cfg->gear_ratio_den);
    if (!std::isfinite(current_limit_a) || current_limit_a < 0.0f ||
        current_limit_a > cfg->profile->protocol_current_max_a) {
        return -ERANGE;
    }
    if (!std::isfinite(gear_ratio) || gear_ratio <= 0.0f) {
        return -ERANGE;
    }

    Endpoint endpoint{};
    const int endpoint_ret = resolveEndpoint(*cfg->profile, static_cast<std::uint8_t>(cfg->motor_id), endpoint);
    if (endpoint_ret < 0)
        return endpoint_ret;

    data->feedback = {};
    data->raw_feedback = {};
    data->endpoint = endpoint;
    data->current_limit_a = current_limit_a;
    data->gear_ratio = gear_ratio;
    data->command_raw = 0;
    data->command_stamp_ms = 0;
    data->command_generation = 0;
    data->command_epoch = 0;
    data->active_epoch = 0;
    data->armed = false;
    data->fault_latched = false;
    data->last_encoder = 0;
    data->total_encoder_ticks = 0;
    data->has_encoder = false;
    data->last_rx_ms = 0;
    data->rx_filter_id = -1;

    struct can_filter filter{};
    filter.id = endpoint.feedback_id;
    filter.mask = CAN_STD_ID_MASK;
    filter.flags = 0u;

    const int filter_id = can_add_rx_filter(cfg->can, rxCallback, const_cast<struct device *>(dev), &filter);
    if (filter_id < 0)
        return filter_id;
    data->rx_filter_id = filter_id;
    return 0;
}

bool isDjiMotor(const struct device *dev) {
    return dev != nullptr && dev->api == &dji_motor_api;
}

bool feedbackReady(const struct device *dev) {
    return getStateImpl(dev) == State::Ready;
}

int armMotor(const struct device *dev, std::uint64_t epoch, std::uint64_t now_ms) {
    DjiData *data = dataOf(dev);
    if (data == nullptr || epoch == 0u)
        return -EINVAL;

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->fault_latched = false;
    data->command_raw = 0;
    data->command_stamp_ms = now_ms;
    ++data->command_generation;
    data->command_epoch = epoch;
    data->active_epoch = epoch;
    data->armed = true;
    k_spin_unlock(&data->lock, key);
    return 0;
}

void prepareMotorStop(const struct device *dev, bool latch_fault) {
    DjiData *data = dataOf(dev);
    if (data == nullptr)
        return;

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->armed = false;
    data->active_epoch = 0;
    data->command_epoch = 0;
    data->command_raw = 0;
    data->command_stamp_ms = 0;
    ++data->command_generation;
    if (latch_fault)
        data->fault_latched = true;
    k_spin_unlock(&data->lock, key);
}

void clearMotorFault(const struct device *dev) {
    DjiData *data = dataOf(dev);
    if (data == nullptr)
        return;

    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->fault_latched = false;
    data->armed = false;
    k_spin_unlock(&data->lock, key);
}

int snapshotCommand(const struct device *dev, std::uint64_t expected_epoch, std::uint64_t now_ms,
                    CommandSnapshot &out) {
    DjiData *data = dataOf(dev);
    if (data == nullptr || expected_epoch == 0u) {
        return -EINVAL;
    }

    CommandSnapshot next{};
    int ret = 0;
    const k_spinlock_key_t key = k_spin_lock(&data->lock);

    if (!data->armed || data->fault_latched) {
        ret = -EACCES;
    }
    else if (data->active_epoch != expected_epoch || data->command_epoch != expected_epoch) {
        ret = -ESTALE;
    }
    else if (data->last_rx_ms == 0u || now_ms < data->last_rx_ms ||
             now_ms - data->last_rx_ms > CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS) {
        ret = -EHOSTDOWN;
    }
    else if (data->command_stamp_ms == 0u || now_ms < data->command_stamp_ms ||
             now_ms - data->command_stamp_ms > CONFIG_SKYWALKER_DJI_COMMAND_TIMEOUT_MS) {
        ret = -ESTALE;
    }
    else {
        next.command_id = data->endpoint.command_id;
        next.command_slot = data->endpoint.command_slot;
        next.command_raw = data->command_raw;
    }

    k_spin_unlock(&data->lock, key);
    if (ret < 0)
        return ret;
    out = next;
    return 0;
}

} // namespace internal

int describe(const struct device *dev, Descriptor &out) {
    if (!internal::isDjiMotor(dev))
        return -ENOTSUP;
    const internal::DjiConfig *cfg = static_cast<const internal::DjiConfig *>(dev->config);
    internal::DjiData *data = static_cast<internal::DjiData *>(dev->data);
    if (cfg == nullptr || cfg->profile == nullptr || data == nullptr) {
        return -EINVAL;
    }

    Descriptor next{};
    next.model = cfg->profile->model;
    next.can = cfg->can;
    next.motor_id = static_cast<std::uint8_t>(cfg->motor_id);
    next.feedback_id = data->endpoint.feedback_id;
    next.command_id = data->endpoint.command_id;
    next.command_slot = data->endpoint.command_slot;
    next.protocol_current_max_a = cfg->profile->protocol_current_max_a;
    next.configured_current_limit_a = data->current_limit_a;
    next.gear_ratio = data->gear_ratio;
    next.temperature_valid = cfg->profile->temperature_valid;
    out = next;
    return 0;
}

int resetMeasurementReference(const struct device *dev) {
    if (!internal::isDjiMotor(dev))
        return -ENOTSUP;
    auto *data = static_cast<internal::DjiData *>(dev->data);
    const auto *cfg = static_cast<const internal::DjiConfig *>(dev->config);
    const auto now = static_cast<std::uint64_t>(k_uptime_get());
    const auto key = k_spin_lock(&data->lock);
    int ret = 0;
    if (data->armed)
        ret = -EACCES;
    else if (!data->last_rx_ms || now < data->last_rx_ms ||
             now - data->last_rx_ms > CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS)
        ret = -EAGAIN;
    else {
        const auto ticks = static_cast<std::int32_t>(cfg->profile->encoder_ticks_per_turn);
        std::int32_t seed = 0;
        if (data->feedback.valid & FeedbackAbsolutePosition) {
            seed = static_cast<std::int32_t>(data->raw_feedback.encoder) -
                   static_cast<std::int32_t>(cfg->encoder_zero_ticks);
            if (seed >= ticks / 2)
                seed -= ticks;
            if (seed < -ticks / 2)
                seed += ticks;
        }
        data->last_encoder = data->raw_feedback.encoder;
        data->has_encoder = true;
        data->total_encoder_ticks = seed;
        data->feedback.position_rad = static_cast<float>(seed) * (6.283185307179586f / ticks) / data->gear_ratio;
    }
    k_spin_unlock(&data->lock, key);
    return ret;
}

int readRawFeedback(const struct device *dev, RawFeedback &out) {
    if (!internal::isDjiMotor(dev))
        return -ENOTSUP;
    internal::DjiData *data = static_cast<internal::DjiData *>(dev->data);
    if (data == nullptr)
        return -EINVAL;

    RawFeedback next{};
    const k_spinlock_key_t key = k_spin_lock(&data->lock);
    next = data->raw_feedback;
    k_spin_unlock(&data->lock, key);

    if (next.timestamp_ms == 0u)
        return -ENODATA;
    out = next;
    return 0;
}

} // namespace skywalker::motor::dji
