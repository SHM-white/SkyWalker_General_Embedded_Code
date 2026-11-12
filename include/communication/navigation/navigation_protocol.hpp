#pragma once

#include <cstddef>
#include <cstdint>
#include <core/measurement.hpp>

namespace skywalker::communication::navigation {
// +X forward, +Y left, +wz counterclockwise. Translation uses the installed
// interboard link's gimbal frame. Wire int16 values are little endian / 1000.
struct VelocityCommand {
    float vx_m_s = 0, vy_m_s = 0, wz_rad_s = 0;
};
using Command = core::Measurement<VelocityCommand>;

// One receive worker owns the parser. No allocation and no packed-struct casts.
class NavigationProtocol {
public:
    static constexpr std::size_t frame_size = 9;
    static constexpr core::TimeUs partial_timeout_us = 20000;
    struct Snapshot {
        Command command{};
        std::uint32_t accepted = 0, bad_length = 0, bad_checksum = 0, partial_timeouts = 0;
    };
    void process(const std::uint8_t *bytes, std::size_t size, core::TimeUs rx_us);
    // Transport loss invalidates the last command as well as partial bytes.
    void invalidate();
    const Snapshot &snapshot() const {
        return value_;
    }

private:
    void discardFirst();
    std::uint8_t frame_[frame_size]{};
    std::size_t used_ = 0;
    core::TimeUs last_rx_us_ = 0;
    Snapshot value_{};
};
} // namespace skywalker::communication::navigation
