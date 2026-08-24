#include <cstdint>
#include <errno.h>

#include <zephyr/spinlock.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <drivers/motor/motor.hpp>
#include <drivers/motor/dji_protocol.hpp>
#include <drivers/motor/dji_m3508.hpp>

#define DT_DRV_COMPAT dji_m3508

namespace
{

    constexpr float TWO_PI = 6.28318530717958647692f;

    struct M3508Config
    {
        const struct device *can;
        std::uint16_t feedback_id;
        std::uint16_t command_id;
        std::uint8_t command_slot;
        float gear_ratio;
    };

    struct M3508Data
    {
        skywalker::motor::Feedback feedback{};
        skywalker::motor::dji::M3508Feedback raw_feedback{};
        std::int16_t command_raw = 0;
        std::uint64_t last_rx_ms = 0;

        struct k_spinlock lock{};

        int rx_filter_id = -1;
    };

    static void m3508RxCallback(const struct device *, struct can_frame *frame, void *user_data)
    {
        auto *dev = static_cast<const struct device *>(user_data);
        if (dev == nullptr || frame == nullptr)
            return;
        auto *data = static_cast<M3508Data *>(dev->data);
        const auto *cfg = static_cast<const M3508Config *>(dev->config);

        skywalker::motor::dji::M3508Feedback raw{};
        if (!skywalker::motor::dji::decodeM3508Feedback(*frame, raw))
        {
            return;
        }
        skywalker::motor::Feedback feedback{};
        if (cfg->gear_ratio > 0.0f)
        {
            feedback.velocity_rad_s = static_cast<float>(raw.rpm) * (TWO_PI / 60.0f) / cfg->gear_ratio;
            feedback.valid |= skywalker::motor::FeedbackVelocity;
        }
        feedback.temperature_c = static_cast<float>(raw.temperature);
        feedback.valid |= skywalker::motor::FeedbackTemperature;
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        feedback.timestamp_ms = now;
        auto key = k_spin_lock(&data->lock);
        data->raw_feedback = raw;
        data->feedback = feedback;
        data->last_rx_ms = now;
        k_spin_unlock(&data->lock, key);
    }

    static int m3508Init(const struct device *dev)
    {
        if (dev == nullptr)
        {
            return -EINVAL;
        }
        auto *data = static_cast<M3508Data *>(dev->data);
        const auto *cfg = static_cast<const M3508Config *>(dev->config);
        if (!device_is_ready(cfg->can))
            return -ENODEV;

        int ret = can_start(cfg->can);

        if (ret < 0 && ret != -EALREADY)
        {
            return ret;
        }

        struct can_filter filter{};
        filter.id = cfg->feedback_id;
        filter.mask = CAN_STD_ID_MASK;
        filter.flags = 0;

        ret = can_add_rx_filter(cfg->can, m3508RxCallback, const_cast<struct device *>(dev), &filter);
        if (ret < 0)
            return ret;

        data->rx_filter_id = ret;
        data->command_raw = 0;
        data->last_rx_ms = 0;
        data->feedback = {};
        data->raw_feedback = {};
        return 0;
    }
    static int m3508Enable(const struct device *dev)
    {
        if (dev == nullptr)
            return -EINVAL;
        return 0;
    }
    static int m3508Disable(const struct device *dev)
    {
        if (dev == nullptr)
            return -EINVAL;
        auto *data = static_cast<M3508Data *>(dev->data);
        k_spinlock_key_t key = k_spin_lock(&data->lock);
        data->command_raw = 0;
        k_spin_unlock(&data->lock, key);
        return 0;
    }
    static int m3508SetTorque(const struct device *dev, float torque_nm)
    {
        ARG_UNUSED(dev);
        ARG_UNUSED(torque_nm);

        return -ENOSYS;
    }

    static int m3508ReadFeedback(const struct device *dev, skywalker::motor::Feedback *out)
    {
        if (out == nullptr)
        {
            return -EINVAL;
        }

        auto *data = static_cast<M3508Data *>(dev->data);
        k_spinlock_key_t key = k_spin_lock(&data->lock);
        *out = data->feedback;
        k_spin_unlock(&data->lock, key);
        return 0;
    }

    static skywalker::motor::State m3508GetState(const struct device *dev)
    {
        if (dev == nullptr)
            return skywalker::motor::State::Offline;

        auto *data = static_cast<M3508Data *>(dev->data);
        std::uint64_t last_rx_ms;
        k_spinlock_key_t key = k_spin_lock(&data->lock);
        last_rx_ms = data->last_rx_ms;
        k_spin_unlock(&data->lock, key);

        if (last_rx_ms == 0)
        {
            return skywalker::motor::State::Offline;
        }

        const std::uint64_t now = static_cast<std::uint64_t>(k_uptime_get());

        if ((now - last_rx_ms) > CONFIG_SKYWALKER_MOTOR_HEARTBEAT_TIMEOUT_MS)
        {
            return skywalker::motor::State::Offline;
        }
        return skywalker::motor::State::Ready;
    };

    static const skywalker::motor::Api m3508_api = {
        m3508Enable,
        m3508Disable,
        m3508SetTorque,
        m3508ReadFeedback,
        m3508GetState,
    };

} // namespace

namespace skywalker::motor::dji::m3508
{
    int setCurrentRaw(const device *dev, std::int16_t current_raw)
    {
        if (dev == nullptr)
            return -EINVAL;
        if (current_raw < -16384 || current_raw > 16384)
        {
            return -ERANGE;
        }
        auto *data = static_cast<M3508Data *>(dev->data);
        auto key = k_spin_lock(&data->lock);
        data->command_raw = current_raw;
        k_spin_unlock(&data->lock, key);
        return 0;
    }

    int getCurrentRaw(const device *dev, std::int16_t &out)
    {
        if (dev == nullptr)
            return -EINVAL;
        auto *data = static_cast<M3508Data *>(dev->data);
        auto key = k_spin_lock(&data->lock);
        out = data->command_raw;
        k_spin_unlock(&data->lock, key);
        return 0;
    }

    int readRawFeedback(const device *dev, M3508Feedback &out)
    {
        if (dev == nullptr)
            return -EINVAL;
        auto *data = static_cast<M3508Data *>(dev->data);
        auto key = k_spin_lock(&data->lock);
        out = data->raw_feedback;
        k_spin_unlock(&data->lock, key);
        return 0;
    }

    int getCommandInfo(const device *dev, std::uint16_t &command_id, std::uint8_t &command_slot)
    {
        if (dev == nullptr)
            return -EINVAL;
        const auto *cfg = static_cast<const M3508Config *>(dev->config);
        command_id = cfg->command_id;
        command_slot = cfg->command_slot;
        return 0;
    }

} // namespace skywalker::motor::dji::m3508

#define M3508_DEFINE(inst)                                                   \
    static M3508Data m3508_data_##inst;                                      \
    static const M3508Config m3508_config_##inst = {                         \
        DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_bus)),                       \
        static_cast<std::uint16_t>(DT_INST_PROP(inst, feedback_id)),         \
        static_cast<std::uint16_t>(DT_INST_PROP(inst, command_id)),          \
        static_cast<std::uint8_t>(DT_INST_PROP(inst, command_slot)),         \
        static_cast<float>(DT_INST_PROP(inst, gear_ratio)),                  \
    };                                                                       \
    DEVICE_DT_INST_DEFINE(inst, m3508Init,                                   \
                          nullptr, &m3508_data_##inst, &m3508_config_##inst, \
                          POST_KERNEL, CONFIG_SKYWALKER_MOTOR_INIT_PRIORITY, \
                          &m3508_api);

DT_INST_FOREACH_STATUS_OKAY(M3508_DEFINE)