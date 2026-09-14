#pragma once

#include <cerrno>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

namespace skywalker::motor {

// One exclusive CAN owner, one outstanding transmission. The callback context
// lives with its Bus, never on the stack of a potentially timed-out send().
// A controller stop cancels pending frames before any subsequent transmission.
class BoundedCanTx {
public:
    BoundedCanTx() {
        k_sem_init(&done_, 0, 1);
    }
    BoundedCanTx(const BoundedCanTx &) = delete;
    BoundedCanTx &operator=(const BoundedCanTx &) = delete;

    int send(const device *can, const can_frame &frame) {
        if (cancel_required_) {
            const int ret = cancel(can);
            if (ret < 0)
                return ret;
        }
        if (restart_required_) {
            const int ret = can_start(can);
            if (ret < 0 && ret != -EALREADY)
                return ret;
            restart_required_ = false;
        }
        k_sem_reset(&done_);
        const int ret = can_send(can, &frame, K_NO_WAIT, completed, this);
        if (ret < 0)
            return ret;
        if (k_sem_take(&done_, K_MSEC(2)) == 0)
            return result_;

        // Even if completion races the deadline, pause this cycle. Never reuse
        // the callback context or enqueue new targets until cancellation succeeds.
        cancel_required_ = true;
        (void)cancel(can);
        return -ETIMEDOUT;
    }

private:
    static void completed(const device *, int error, void *context) {
        auto &self = *static_cast<BoundedCanTx *>(context);
        self.result_ = error;
        k_sem_give(&self.done_);
    }
    int cancel(const device *can) {
        // Zephyr can_stop aborts pending TX. MC02's M_CAN implementation invokes
        // their callbacks before returning, retaining RX filters across restart.
        const int ret = can_stop(can);
        if (ret < 0 && ret != -EALREADY)
            return ret;
        cancel_required_ = false;
        restart_required_ = true;
        return 0;
    }
    k_sem done_{};
    int result_ = 0;
    bool cancel_required_ = false;
    bool restart_required_ = false;
};
} // namespace skywalker::motor
