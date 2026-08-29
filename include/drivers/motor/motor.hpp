#pragma once

#include <cstdint>
#include <zephyr/device.h>

namespace skywalker::motor
{
    enum class State : std::uint8_t
    {
        Offline = 0,
        Ready,
        Fault,
    };
    enum FeedbackValid : std::uint32_t
    {
        FeedbackPosition = 1u << 0,
        FeedbackVelocity = 1u << 1,
        FeedbackTorque = 1u << 2,
        FeedbackTemperature = 1u << 3,
    };

    struct Feedback
    {
        float position_rad = 0.0f;
        float velocity_rad_s = 0.0f;
        float torque_nm = 0.0f;
        float temperature_c = 0.0f;

        std::uint32_t valid = 0;
        std::uint64_t timestamp_ms = 0;
    };

    struct Api {
        int (*enable)(const struct device *dev);
        int (*disable)(const struct device *dev);
        int (*set_torque)(const struct device *dev, float torque_nm);
        int (*read_feedback)(const struct device *dev, Feedback *out);
        State (*get_state)(const struct device *dev);
    };

    inline int setTorque(const struct device *dev, float torque_nm){
        if (dev == nullptr || dev->api == nullptr) return -EINVAL;

        const auto *api = static_cast<const Api *>(dev->api);
        if (api->set_torque == nullptr){
            return -ENOSYS;
        }
        return api->set_torque(dev, torque_nm);
    }
    inline int enable(const struct device *dev){
        if (dev == nullptr || dev->api == nullptr) return -EINVAL;

        const auto *api = static_cast<const Api *>(dev->api);
        if (api->enable == nullptr){
            return -ENOSYS;
        }
        return api->enable(dev);
    }
    inline int disable(const struct device *dev){
        if (dev == nullptr || dev->api == nullptr) return -EINVAL;

        const auto *api = static_cast<const Api *>(dev->api);
        if (api->disable == nullptr){
            return -ENOSYS;
        }
        return api->disable(dev);
    }
    inline int readFeedback(const struct device *dev, Feedback &out){
        if (dev == nullptr || dev->api == nullptr) return -EINVAL;

        const auto *api = static_cast<const Api *>(dev->api);
        if (api->read_feedback == nullptr){
            return -ENOSYS;
        }
        return api->read_feedback(dev, &out);
    }
    inline State getState(const struct device *dev){
        if (dev == nullptr || dev->api == nullptr) return State::Offline;

        const auto *api = static_cast<const Api *>(dev->api);
        if (api->get_state == nullptr){
            return State::Offline;
        }
        return api->get_state(dev);
    }

} // namespace skywalker::motor
