#include "../../common/shooter_bench.hpp"
#if defined(CONFIG_SHOOTER_INTERBOARD_BIG_YAW)
K_THREAD_DEFINE(shared_can_big_yaw_link, 4096, skywalker::samples::manual_big_yaw::communicationTask,
                nullptr, nullptr, nullptr, 6, K_FP_REGS, 0);
#endif
int main() {
#if defined(CONFIG_SHOOTER_INTERBOARD_BIG_YAW)
    printk("bigYaw: UART1 460800; RC wheel +/-0.1 rad/s in RC and keyboard/mouse modes; no IMU\n");
#endif
    return skywalker::samples::shooter::run(true, IS_ENABLED(CONFIG_SHOOTER_UNLOADED_FEED));
}
