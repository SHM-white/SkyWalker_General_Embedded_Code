#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/motor.hpp>
#include <drivers/motor/dji_m3508.hpp>
#include <drivers/motor/dji_tx_group.hpp>

LOG_MODULE_REGISTER(m3508_driver_test, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

static skywalker::motor::dji::TxGroup tx_group;

int main()
{
    const struct device *motor =
        DEVICE_DT_GET(MOTOR0_NODE);

    const struct device *can =
        DEVICE_DT_GET(DT_NODELABEL(can1));

    if (!device_is_ready(motor)) {
        LOG_ERR("M3508 device not ready");
        return -ENODEV;
    }

    if (!device_is_ready(can)) {
        LOG_ERR("CAN device not ready");
        return -ENODEV;
    }

    int ret = tx_group.init(can, 0x200);

    if (ret < 0) {
        LOG_ERR("TxGroup init failed: %d", ret);
        return ret;
    }

    ret = tx_group.bindMotor(motor);

    if (ret < 0) {
        LOG_ERR("TxGroup bind failed: %d", ret);
        return ret;
    }

    ret =
        skywalker::motor::dji::m3508::
            setCurrentRaw(motor, 300);

    if (ret < 0) {
        LOG_ERR("setCurrentRaw failed: %d", ret);
        return ret;
    }

    LOG_INF("M3508 driver ready");

    int print_counter = 0;

    while (true)
    {
        ret = tx_group.send();

        if (ret < 0) {
            LOG_ERR("CAN send failed: %d", ret);
        }

        ++print_counter;

        if (print_counter >= 25)
        {
            print_counter = 0;

            skywalker::motor::dji::M3508Feedback raw{};

            skywalker::motor::dji::m3508::
                readRawFeedback(motor, raw);

            const auto state =
                skywalker::motor::getState(motor);

            printk(
                "state=%d encoder=%u rpm=%d current=%d temp=%u\n",
                static_cast<int>(state),
                raw.encoder,
                raw.rpm,
                raw.current_raw,
                raw.temperature);
        }

        k_sleep(K_MSEC(20));
    }

    return 0;
}


// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/logging/log.h>

// #include <drivers/motor/motor.hpp>

// LOG_MODULE_REGISTER(m3508_driver_test, LOG_LEVEL_INF);

// #define MOTOR0_NODE DT_ALIAS(motor0)

// int main()
// {
//     const struct device *motor =
//         DEVICE_DT_GET(MOTOR0_NODE);

//     if (!device_is_ready(motor)) {
//         LOG_ERR("M3508 device not ready");
//         return -ENODEV;
//     }

//     LOG_INF("M3508 device ready");

//     while (true)
//     {
//         skywalker::motor::Feedback feedback{};

//         int ret =
//             skywalker::motor::readFeedback(motor, feedback);

//         if (ret == 0)
//         {
//             const auto state =
//                 skywalker::motor::getState(motor);

//             const int velocity_mrad_s =
//                 static_cast<int>(
//                     feedback.velocity_rad_s * 1000.0f);

//             const int temp_c =
//                 static_cast<int>(
//                     feedback.temperature_c);

//             printk(
//                 "state=%d vel=%d mrad/s temp=%d C ts=%llu\n",
//                 static_cast<int>(state),
//                 velocity_mrad_s,
//                 temp_c,
//                 static_cast<unsigned long long>(
//                     feedback.timestamp_ms));
//         }

//         k_sleep(K_MSEC(500));
//     }

//     return 0;
// }