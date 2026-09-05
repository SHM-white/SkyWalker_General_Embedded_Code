#pragma once

#include <cstdint>
#include <errno.h>
#include <zephyr/device.h>

namespace skywalker::motor {

enum class State : std::uint8_t {
    Offline = 0,
    Ready,
    Fault,
};

enum Capability : std::uint32_t {
    CommandCurrent      = 1u << 0,
    CommandTorque       = 1u << 1,
    FeedbackPosition    = 1u << 8,
    FeedbackVelocity    = 1u << 9,
    FeedbackCurrent     = 1u << 10,
    FeedbackTorque      = 1u << 11,
    FeedbackTemperature = 1u << 12,
};

struct Feedback {
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float current_a = 0.0f;
    float torque_nm = 0.0f;
    float temperature_c = 0.0f;
    std::uint32_t valid = 0;
    std::uint64_t timestamp_ms = 0;
};

struct Api {
    std::uint32_t (*get_capabilities)(const struct device *dev);
    int (*set_current)(const struct device *dev, float current_a);
    int (*set_torque)(const struct device *dev, float torque_nm);
    int (*read_feedback)(const struct device *dev, Feedback *out);
    State (*get_state)(const struct device *dev);
};

inline std::uint32_t capabilities(const struct device *dev)
{
    if (dev == nullptr || dev->api == nullptr) return 0u;
    const Api *api = static_cast<const Api *>(dev->api);
    return api->get_capabilities == nullptr
        ? 0u
        : api->get_capabilities(dev);
}

inline int setCurrent(const struct device *dev, float current_a)
{
    if (dev == nullptr || dev->api == nullptr) return -EINVAL;
    const Api *api = static_cast<const Api *>(dev->api);
    if ((capabilities(dev) & CommandCurrent) == 0u ||
        api->set_current == nullptr) {
        return -ENOTSUP;
    }
    return api->set_current(dev, current_a);
}

inline int setTorque(const struct device *dev, float torque_nm)
{
    if (dev == nullptr || dev->api == nullptr) return -EINVAL;
    const Api *api = static_cast<const Api *>(dev->api);
    if ((capabilities(dev) & CommandTorque) == 0u ||
        api->set_torque == nullptr) {
        return -ENOTSUP;
    }
    return api->set_torque(dev, torque_nm);
}

inline int readFeedback(const struct device *dev, Feedback &out)
{
    if (dev == nullptr || dev->api == nullptr) return -EINVAL;
    const Api *api = static_cast<const Api *>(dev->api);
    if (api->read_feedback == nullptr) return -ENOTSUP;
    return api->read_feedback(dev, &out);
}

inline State getState(const struct device *dev)
{
    if (dev == nullptr || dev->api == nullptr) return State::Offline;
    const Api *api = static_cast<const Api *>(dev->api);
    return api->get_state == nullptr
        ? State::Offline
        : api->get_state(dev);
}

} // namespace skywalker::motor
