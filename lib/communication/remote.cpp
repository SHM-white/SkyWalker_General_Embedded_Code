#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <communication/wire.hpp>
#include <communication/remote/remote_service.hpp>
namespace skywalker::communication {
using namespace robotics;
int Dr16Decoder::reset() {
    valid_frames_ = invalid_frames_ = sequence_ = 0;
    return 0;
}
int Dr16Decoder::decodeFrame(const std::uint8_t *p, std::size_t size, std::uint64_t now, RemoteState &out) {
    auto bad = [&](int ret) {
        ++invalid_frames_;
        return ret;
    };
    if (!p || size != kFrameSize || config_.channel_min < 0 || config_.channel_max > 2047 ||
        config_.channel_min >= config_.channel_center || config_.channel_center >= config_.channel_max ||
        config_.center_deadband < 0)
        return bad(-EINVAL);
    const int channels[] = {(p[0] | (p[1] << 8)) & 0x7ff, ((p[1] >> 3) | (p[2] << 5)) & 0x7ff,
                            ((p[2] >> 6) | (p[3] << 2) | (p[4] << 10)) & 0x7ff, ((p[4] >> 1) | (p[5] << 7)) & 0x7ff,
                            wire::loadLe16(p + 16)};
    for (int c : channels)
        if (c < config_.channel_min || c > config_.channel_max)
            return bad(-EBADMSG);
    const auto left = (p[5] >> 6) & 3, right = (p[5] >> 4) & 3;
    if (!left || !right || p[12] > 1 || p[13] > 1)
        return bad(-EBADMSG);
    auto channel = [&](int i) {
        const int x = channels[i] - config_.channel_center;
        return static_cast<std::int16_t>(std::abs(x) <= config_.center_deadband ? 0 : x);
    };
    auto sw = [](unsigned x) { return x == 1 ? RcSwitch::Up : x == 2 ? RcSwitch::Down : RcSwitch::Middle; };
    auto signed16 = [](const std::uint8_t *v) {
        const auto u = wire::loadLe16(v);
        return static_cast<std::int16_t>(u < 0x8000 ? int(u) : int(u) - 65536);
    };
    RemoteState n{};
    n.analog = {channel(0), channel(1), channel(2), channel(3), channel(4)};
    n.left_switch = sw(left);
    n.right_switch = sw(right);
    n.mouse = {signed16(p + 6), signed16(p + 8), signed16(p + 10), p[12] != 0, p[13] != 0};
    n.keyboard.bits = wire::loadLe16(p + 14);
    n.stamp = {now, ++sequence_, true};
    n.online = true;
    out = n;
    ++valid_frames_;
    return 0;
}
int RemoteService::processBytes(const std::uint8_t *p, std::size_t n, std::uint64_t now) {
    if (!p && n)
        return -EINVAL;
    if (used_ && (now < last_bytes_ms_ || now - last_bytes_ms_ > config_.assembly_gap_ms))
        used_ = 0;
    if (n)
        last_bytes_ms_ = now;
    for (std::size_t i = 0; i < n; ++i) {
        buffer_[used_++] = p[i];
        if (used_ == buffer_.size()) {
            if (decoder_.decodeFrame(buffer_.data(), used_, now, latest_) == 0)
                used_ = 0;
            else {
                std::memmove(buffer_.data(), buffer_.data() + 1, --used_);
            }
        }
    }
    return 0;
}
bool RemoteService::online(std::uint64_t now) const {
    return isFresh(latest_.stamp, now, config_.offline_timeout_ms);
}
int RemoteService::snapshot(std::uint64_t now, RemoteState &out) const {
    if (!latest_.stamp.valid)
        return -EAGAIN;
    out = latest_;
    out.online = online(now);
    return out.online ? 0 : -ESTALE;
}
}
