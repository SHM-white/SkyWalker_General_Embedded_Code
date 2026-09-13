#pragma once
#include <cerrno>
#include <zephyr/drivers/can.h>
namespace skywalker::control {
// Driver auto-recovery is left enabled unless the application selected manual mode.
inline int pollCanRecovery(const device *can) {
    can_state state;
    int ret = can_get_state(can, &state, nullptr);
    if (ret < 0)
        return ret;
    if (state == CAN_STATE_STOPPED) {
        ret = can_start(can);
        return ret < 0 && ret != -EALREADY ? ret : -EAGAIN;
    }
    if (state != CAN_STATE_BUS_OFF)
        return 0;
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
    if (can_get_mode(can) & CAN_MODE_MANUAL_RECOVERY) {
        ret = can_recover(can, K_NO_WAIT);
        if (ret < 0 && ret != -EALREADY && ret != -EAGAIN)
            return ret;
    }
#endif
    return -EAGAIN;
}
}
