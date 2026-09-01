#include <errno.h>
#include <cstdint>

#include <zephyr/console/console.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/motor/dji_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>

LOG_MODULE_REGISTER(dji_unified, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

static skywalker::motor::dji::Bus dji_bus;

static int waitForFreshFeedback(const struct device *motor)
{
    const std::int64_t deadline = k_uptime_get() + 2000;
    while (skywalker::motor::getState(motor) !=
           skywalker::motor::State::Ready) {
        if (k_uptime_get() >= deadline) return -ETIMEDOUT;
        k_sleep(K_MSEC(5));
    }
    return 0;
}

static int waitForManualArmToken()
{
    const int init_ret = console_init();
    if (init_ret < 0) return init_ret;

    printk("保持电机架空且 current-limit-ma=0；输入 a 后只发送 0A：\n");
    for (;;) {
        const int ch = console_getchar();
        if (ch < 0) return ch;
        if (ch == 'a' || ch == 'A') return 0;
    }
}

int main()
{
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    if (!device_is_ready(motor)) {
        LOG_ERR("motor device not ready");
        return -ENODEV;
    }

    skywalker::motor::dji::Descriptor descriptor{};
    int ret =
        skywalker::motor::dji::describe(motor, descriptor);
    if (ret < 0 || descriptor.can == nullptr ||
        !device_is_ready(descriptor.can)) {
        LOG_ERR("describe/CAN failed: %d", ret);
        return ret < 0 ? ret : -ENODEV;
    }

    ret = dji_bus.init(descriptor.can);
    if (ret < 0) {
        LOG_ERR("Bus init failed: %d", ret);
        return ret;
    }

    ret = dji_bus.attach(motor);
    if (ret < 0) {
        LOG_ERR("Bus attach failed: %d", ret);
        return ret;
    }

    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("No fresh feedback: %d", ret);
        return ret;
    }

    ret = waitForManualArmToken();
    if (ret < 0) {
        LOG_ERR("Manual arm input failed: %d", ret);
        return ret;
    }

    skywalker::motor::dji::FlushReport arm_report{};
    ret = dji_bus.arm(arm_report);
    if (ret < 0 || !arm_report.zero_sent) {
        LOG_ERR("Arm/zero failed: ret=%d zero=%d",
                ret,
                arm_report.zero_sent ? 1 : 0);
        return ret < 0 ? ret : -EIO;
    }

    std::uint32_t print_divider = 0;
    for (;;) {
        ret = skywalker::motor::setCurrent(motor, 0.0f);
        if (ret < 0) {
            skywalker::motor::dji::FlushReport stop_report{};
            dji_bus.stop(stop_report);
            LOG_ERR("setCurrent(0A) failed: %d", ret);
            return ret;
        }

        skywalker::motor::dji::FlushReport report{};
        ret = dji_bus.flush(report);
        if (ret < 0) {
            LOG_ERR("flush failed: ret=%d zero=%d zero_err=%d",
                    ret,
                    report.zero_sent ? 1 : 0,
                    report.zero_tx_error);
            return ret;
        }

        if (++print_divider >= 100u) {
            print_divider = 0;
            skywalker::motor::Feedback feedback{};
            skywalker::motor::dji::RawFeedback raw{};
            const int feedback_ret =
                skywalker::motor::readFeedback(motor, feedback);
            const int raw_ret =
                skywalker::motor::dji::readRawFeedback(motor, raw);

            printk("state=%d feedback_ret=%d raw_ret=%d "
                   "encoder=%u rpm=%d current_raw=%d ts=%llu\n",
                   static_cast<int>(
                       skywalker::motor::getState(motor)),
                   feedback_ret,
                   raw_ret,
                   raw.encoder,
                   raw.speed_rpm,
                   raw.current_raw,
                   static_cast<unsigned long long>(
                       feedback.timestamp_ms));
        }

        k_sleep(K_MSEC(5));
    }
}
