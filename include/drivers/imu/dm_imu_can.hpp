#pragma once
#include <cstddef>
#include <drivers/imu/imu.hpp>
#include <drivers/imu/imu_state.hpp>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/atomic.h>

namespace skywalker::imu {
// Dedicated classic CAN controller. Do not share with motor::CanBus or another
// start/stop owner. Source and device must outlive the receiver and callbacks.
// Configure/save the module with the vendor tool; this driver only reads data.
class DmImuCanSource final : public ImuSource {
public:
    enum class Mode : std::uint8_t { ActiveReport, RequestReply };
    enum class Attitude : std::uint8_t { Euler = 3, Quaternion = 4 };
    struct Config {
        const device *can = nullptr;
        std::uint16_t feedback_id = 0x11, slave_id = 1;
        Mode mode = Mode::ActiveReport;
        Attitude attitude = Attitude::Quaternion;
        core::OrientationReference reference{2, 1};
        core::Quaternion sensor_to_body{};
        bool device_quaternion_is_world_to_sensor = false;
        Freshness freshness{};
        core::TimeUs request_interval_us = 1000, response_timeout_us = 50000;
        core::TimeUs recovery_retry_us = 100000;
        std::uint8_t max_retries = 2;
    };
    explicit DmImuCanSource(const Config &c) : config_(c), state_(c.freshness) {
    }
    int init() override;
    int service() override;
    Snapshot snapshot() const override {
        return state_.snapshot();
    }
    // Owner thread only, after known remote zero/calibration/reset. No device write.
    int resetReference();

private:
    static constexpr std::size_t rx_depth = 64;
    struct RxEvent {
        can_frame frame{};
        core::TimeUs received_us = 0;
    };
    static void onRx(const device *, can_frame *, void *);
    static void onTx(const device *, int, void *);
    int startController();
    int fail(int error, core::TimeUs now);
    int recover(core::TimeUs now);
    int consume(const RxEvent &);
    void clearRx();
    void finishRequest(bool retry, core::TimeUs now);
    void logStatistics(core::TimeUs now);
    Config config_;
    ImuState state_;
    k_spinlock rx_lock_{};
    RxEvent rx_[rx_depth]{};
    std::size_t rx_head_ = 0, rx_count_ = 0;
    bool rx_gap_ = false;
    atomic_t accept_rx_ = 0, tx_done_ = 0, tx_error_ = 0;
    can_frame tx_frame_{};
    core::TimeUs tx_finished_us_ = 0;
    core::TimeUs request_us_ = 0, deadline_us_ = 0, next_request_us_ = 0, retry_us_ = 0, next_log_us_ = 0;
    core::TimeUs accept_after_us_ = 0;
    std::uint8_t rid_ = 4, retries_ = 0;
    std::uint32_t frames_ = 0, updates_ = 0, requests_ = 0, replies_ = 0, timeouts_ = 0, invalid_ = 0;
    int filter_ = -1;
    bool initialized_ = false, owns_controller_ = false, recovering_ = false, stopped_ = false;
    bool in_flight_ = false, awaiting_reply_ = false;
};
}
