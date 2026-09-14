#pragma once

#include <cstddef>
#include <drivers/motor/bounded_can_tx.hpp>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/spinlock.h>

#include <drivers/motor/dm_motor.hpp>

namespace skywalker::motor::dm {

enum class BusState : std::uint8_t {
    Uninitialized = 0,
    Safe,
    Armed,
    Fault,
};

struct TxReport {
    int preparation_error = 0;
    int tx_error = 0;
    std::uint16_t failed_motor_id = 0;
    std::uint8_t frames_expected = 0;
    std::uint8_t frames_sent = 0;
};

class Bus {
public:
    Bus() = default;
    Bus(const Bus &) = delete;
    Bus &operator=(const Bus &) = delete;
    Bus(Bus &&) = delete;
    Bus &operator=(Bus &&) = delete;

    int init(const struct device *can);
    int attach(const struct device *motor);
    int arm(TxReport &report);
    int flush(TxReport &report);
    int stop(TxReport &report);
    int recover(TxReport &report);
    int savePositionZero(const struct device *motor, TxReport &report);
    BusState state() const;

private:
    struct FeedbackRoute {
        std::uint16_t master_id = 0;
        int filter_id = -1;
    };

    static void rxCallback(const struct device *can, struct can_frame *frame, void *user_data);
    void dispatchFeedback(const struct can_frame &frame);
    int routeIndex(std::uint16_t master_id) const;
    int sendFrame(const struct can_frame &frame, std::uint16_t motor_id, TxReport &report);
    int disableAll(TxReport &report, bool latch_fault);
    int enterFaultAndDisable(TxReport &report);

    BoundedCanTx tx_{};
    const struct device *can_ = nullptr;
    const struct device *motors_[CONFIG_SKYWALKER_DM_MAX_MOTORS_PER_BUS]{};
    Descriptor descriptors_[CONFIG_SKYWALKER_DM_MAX_MOTORS_PER_BUS]{};
    FeedbackRoute routes_[CONFIG_SKYWALKER_DM_MAX_MASTER_IDS_PER_BUS]{};
    std::size_t motor_count_ = 0;
    std::size_t route_count_ = 0;
    std::uint64_t lifecycle_epoch_ = 0;
    std::uint64_t recovery_started_ms_ = 0;
    BusState state_ = BusState::Uninitialized;
    mutable struct k_spinlock route_lock_{};
};

} // namespace skywalker::motor::dm
