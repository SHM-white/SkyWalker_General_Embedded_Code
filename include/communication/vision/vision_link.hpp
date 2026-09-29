#pragma once
#include <communication/vision/vision_protocol.hpp>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
namespace skywalker::communication::vision {
// Protocol processing/encoding has one owner. Other threads may setFeedback/snapshot.
// The protocol dependency must outlive this object.
class VisionLink final : private CommandSink {
public:
    struct Config {
        core::TimeUs aim_timeout_us = 100000, orientation_timeout_us = 20000, gyro_timeout_us = 20000;
        core::TimeUs bullet_speed_timeout_us = 1000000, bullet_count_timeout_us = 1000000;
        core::TimeUs max_feedback_skew_us = 20000;
    };
    VisionLink(VisionProtocol &p, const Config &c) : protocol_(p), config_(c) {
    }
    VisionLink(const VisionLink &) = delete;
    VisionLink &operator=(const VisionLink &) = delete;
    int init();
    int processRxBytes(const std::uint8_t *, std::size_t, core::TimeUs);
    void discardPartial();
    int setFeedback(const Feedback &);
    int encodeFeedback(std::uint8_t *, std::size_t);
    Snapshot snapshot() const;

private:
    int accept(const AimCommand &, core::TimeUs) override;
    VisionProtocol &protocol_;
    const Config config_;
    mutable k_spinlock lock_{};
    Snapshot value_{};
    Feedback feedback_{};
    atomic_t initialized_ = 0;
};
}
