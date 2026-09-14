#include <cerrno>
#include <zephyr/device.h>
#if defined(CONFIG_BOARD_DM_MC02)
#include <zephyr/drivers/regulator.h>
#endif
#include <dm_sample_support.hpp>
namespace skywalker::samples::dm {
int enableMotorPower() {
#if defined(CONFIG_BOARD_DM_MC02)
    const device *power = DEVICE_DT_GET(DT_NODELABEL(power1));
    if (!device_is_ready(power))
        return -ENODEV;
    return regulator_enable(power); // Startup feedback is polled, never sleep here.
#else
    return 0;
#endif
}
}
