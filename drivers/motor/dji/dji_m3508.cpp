#include <cstdint>
#include <zephyr/spinlock.h>
#include <zephyr/kernel.h>
#include "drivers/can.h"
#include "motor.hpp"

#define DT_DRV_COMPAT dji_m3508

#define M3508_DEFINE(inst) \
    DEVICE_DT_INST_DEFINE(...)

DT_INST_FOREACH_STATUS_OKAY(M3508_DEFINE)

namespace skywalker::motor::dji::m3508
{

struct M3508Config {
    const struct device *can;

    std::uint16_t feedback_id;
    std::uint16_t command_id;

    std::uint8_t command_slot;

    float gear_ratio;
};

struct M3508Data {
    skywalker::motor::Feedback feedback{};

    std::int16_t command_raw = 0;
    std::uint64_t last_rx_ms = 0;

    struct k_spinlock lock{};

    int rx_filter_id = -1;
};

static void m3508RxCallback(const struct device*, struct can_frame* frame, void* user_data){
    auto *dev = static_cast<const struct device *>(user_data);
    if (dev == nullptr || frame == nullptr) return;
    auto *data = static_cast<M3508Data *>(dev->data);
    const auto *cfg = static_cast<const M3508Config *>(dev->config);
    int ret = m3508ReadFeedback(dev, &data->feedback);
    
    data->last_rx_ms = k_uptime_get();

}

static int m3508Init(const struct device *dev){
    if (dev == nullptr) {
        return -EINVAL;
    }
    auto *data = static_cast<M3508Data *>(dev->data);
    const auto *cfg = static_cast<const M3508Config *>(dev->config);
    if (!device_is_ready(cfg->can)) return -ENODEV;

    int ret = can_start(cfg->can);

    if (ret < 0 && ret != -EALREADY) {
        return ret;
    }

    data->command_raw = 0;
    data->last_rx_ms = 0;
    data->feedback = {};
    return 0;
}
static int m3508Enable(const struct device *dev) {
    if (dev == nullptr) return -EINVAL;
    return 0;
}
static int m3508Disable(const struct device *dev) {
    if (dev == nullptr) return -EINVAL;
    auto *data = static_cast<M3508Data *>(dev->data);
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->command_raw = 0;
    k_spin_unlock(&data->lock, key);
    return 0;
}
static int m3508SetTorque(const struct device *dev, float torque_nm){
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

static skywalker::motor::State m3508GetState(const struct device *dev) {
    if (dev == nullptr) return skywalker::motor::State::Offline;

    auto *data = static_cast<M3508Data *>(dev->data);
    std::uint64_t last_rx_ms;
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    last_rx_ms = data->last_rx_ms;
    k_spin_unlock(&data->lock, key);

    if (last_rx_ms == 0){
        return skywalker::motor::State::Offline;
    }

    const std::uint64_t now = static_cast<std::uint64_t>(k_uptime_get());

    if ((now - last_rx_ms) > CONFIG_MOTOR_HEARTBEAT_TIMEOUT_MS) {
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