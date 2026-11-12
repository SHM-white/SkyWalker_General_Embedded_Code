#include <communication/navigation/navigation_protocol.hpp>
#include <cstring>

namespace skywalker::communication::navigation {
namespace {
float velocity(const std::uint8_t *p) {
    const std::uint16_t raw = std::uint16_t(p[0]) | (std::uint16_t(p[1]) << 8);
    // Explicit sign extension avoids implementation-defined unsigned-to-int16 conversion.
    const std::int32_t signed_value = raw & 0x8000u ? std::int32_t(raw) - 65536 : std::int32_t(raw);
    return static_cast<float>(signed_value) * 0.001f;
}
}
void NavigationProtocol::discardFirst() {
    if (used_) {
        --used_;
        std::memmove(frame_, frame_ + 1, used_);
    }
}
void NavigationProtocol::invalidate() {
    used_ = 0;
    value_.command.stamp.valid = false;
}
void NavigationProtocol::process(const std::uint8_t *bytes, std::size_t size, core::TimeUs rx_us) {
    if (used_ && (rx_us < last_rx_us_ || rx_us - last_rx_us_ > partial_timeout_us)) {
        used_ = 0;
        ++value_.partial_timeouts;
    }
    if (!bytes)
        return;
    for (std::size_t i = 0; i < size; ++i) {
        frame_[used_++] = bytes[i];
        last_rx_us_ = rx_us;
        // Sliding recovery retains a possible frame header inside corrupt data.
        while (used_) {
            if (frame_[0] != 0xAA) {
                discardFirst();
                continue;
            }
            if (used_ < 2)
                break;
            if (frame_[1] != 0x06) {
                ++value_.bad_length;
                discardFirst();
                continue;
            }
            if (used_ < frame_size)
                break;
            std::uint8_t checksum = 0;
            for (std::size_t j = 0; j < frame_size - 1; ++j)
                checksum ^= frame_[j];
            if (checksum != frame_[frame_size - 1]) {
                ++value_.bad_checksum;
                discardFirst();
                continue;
            }
            value_.command.value = {velocity(frame_ + 2), velocity(frame_ + 4), velocity(frame_ + 6)};
            value_.command.stamp = {rx_us, value_.command.stamp.sequence + 1, true};
            ++value_.accepted;
            used_ = 0;
        }
    }
}
} // namespace skywalker::communication::navigation
