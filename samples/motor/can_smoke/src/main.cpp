#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(can_smoke, LOG_LEVEL_INF);

static void rxCallback(const struct device* dev, struct can_frame* frame, void* user_data){
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    printk("RX id=0x%03x dlc=%d\n", frame->id, frame->dlc);
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

    while (true) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}