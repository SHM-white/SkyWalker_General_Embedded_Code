#include "../../common/shooter_bench.hpp"
int main() {
#if defined(CONFIG_SHOOTER_INTERBOARD_BIG_YAW)
    const int link_error = skywalker::samples::manual_big_yaw::endpoint.start();
    if (link_error < 0) {
        printk("bigYaw start=%d\n", link_error);
        return link_error;
    }
    printk("bigYaw: UART1 460800; RC wheel +/-0.1 rad/s in RC and keyboard/mouse modes; no IMU\n");
#endif
    return skywalker::samples::shooter::run(true, IS_ENABLED(CONFIG_SHOOTER_UNLOADED_FEED));
}
