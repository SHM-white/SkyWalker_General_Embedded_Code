#include <cerrno>
#include <dm_sample_support.hpp>
#if defined(CONFIG_BOARD_DM_MC02)
#include <zephyr/drivers/regulator.h>
#endif
namespace skywalker::samples::dm {
int prepare(Session &session) {
    if (session.can == nullptr || !device_is_ready(session.can)) return -ENODEV;
    int ret = motor::dm::describe(session.config, session.descriptor);
    if (ret == 0) ret = session.bus.attach(session.motor);
    if (ret == 0) ret = session.bus.start();
#if defined(CONFIG_BOARD_DM_MC02)
    if (ret == 0) {
        const device *power = DEVICE_DT_GET(DT_NODELABEL(power1));
        ret = device_is_ready(power) ? regulator_enable(power) : -ENODEV;
    }
#endif
    return ret;
}
int arm(Session &session) { return session.motor.enable(); }
int flush(Session &session) { return session.bus.commit().error; }
int stop(Session &session) { return session.motor.disable(); }
}
