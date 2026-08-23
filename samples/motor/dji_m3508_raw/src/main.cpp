#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include "dji_m3508_protocol.hpp"

LOG_MODULE_REGISTER(dji_m3508_raw, LOG_LEVEL_INF);

struct MotorRuntime {
    skywalker::motor::dji::M3508Feedback feedback{};
    std::int64_t last_rx_ms = 0;
};

static MotorRuntime motor;


static void rxCallback(const struct device* dev, struct can_frame* frame, void* user_data){
    auto *runtime = static_cast<MotorRuntime *>(user_data);
    if (runtime == nullptr || frame == nullptr) { return; }
    if (skywalker::motor::dji::decodeM3508Feedback(*frame, runtime->feedback)){
        runtime->last_rx_ms = k_uptime_get();
    }
}

int main()
{
    const struct device *can = DEVICE_DT_GET(DT_NODELABEL(can1));

    if (!device_is_ready(can)) {
        LOG_ERR("CAN device not ready");
        return -ENODEV;
    }

    int ret = can_start(can);

    if (ret < 0 && ret != -EALREADY) {
        LOG_ERR("can_start failed: %d", ret);
        return ret;
    }

    LOG_INF("CAN ready");

    struct can_filter filter = {
        .id = 0,
        .mask = 0,
        .flags = 0,
    };

    int filter_id = can_add_rx_filter(can, rxCallback, nullptr, &filter);
    if (filter_id < 0) {
        LOG_ERR("Filter register failed");
    }

    std::int16_t current[4] = {300, 0, 0, 0};
    can_frame frame;

    while (true) {
        k_sleep(K_SECONDS(1));
        skywalker::motor::dji::buildGroupCurrentFrame(frame, 0x200, current);
        
    }

    return 0;
}