#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/motor.hpp>

LOG_MODULE_REGISTER(m3508_driver_test, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

int main()
{
    const struct device *motor =
        DEVICE_DT_GET(MOTOR0_NODE);

    if (!device_is_ready(motor)) {
        LOG_ERR("M3508 device not ready");
        return -ENODEV;
    }

    LOG_INF("M3508 device ready");

    while (true)
    {
        skywalker::motor::Feedback feedback{};

        int ret =
            skywalker::motor::readFeedback(motor, feedback);

        if (ret == 0)
        {
            const auto state =
                skywalker::motor::getState(motor);

            const int velocity_mrad_s =
                static_cast<int>(
                    feedback.velocity_rad_s * 1000.0f);

            const int temp_c =
                static_cast<int>(
                    feedback.temperature_c);

            printk(
                "state=%d vel=%d mrad/s temp=%d C ts=%llu\n",
                static_cast<int>(state),
                velocity_mrad_s,
                temp_c,
                static_cast<unsigned long long>(
                    feedback.timestamp_ms));
        }

        k_sleep(K_MSEC(500));
    }

    return 0;
}