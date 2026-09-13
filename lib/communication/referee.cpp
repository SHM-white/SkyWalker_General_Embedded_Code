#include <cerrno>
#include <cstring>
#include <communication/wire.hpp>
#include <communication/referee/referee_parser.hpp>
namespace skywalker::communication {
std::uint8_t refereeCrc8(const std::uint8_t *p, std::size_t n) {
    std::uint8_t c = 0xff;
    for (std::size_t i = 0; i < n; ++i) {
        c ^= p[i];
        for (unsigned b = 0; b < 8; ++b)
            c = (c & 1) ? (c >> 1) ^ 0x8c : c >> 1;
    }
    return c;
}
std::uint16_t refereeCrc16(const std::uint8_t *p, std::size_t n) {
    std::uint16_t c = 0xffff;
    for (std::size_t i = 0; i < n; ++i) {
        c ^= p[i];
        for (unsigned b = 0; b < 8; ++b)
            c = (c & 1) ? (c >> 1) ^ 0x8408 : c >> 1;
    }
    return c;
}
int RefereeParser::reset() {
    used_ = 0;
    started_ms_ = 0;
    sequence_ = 0;
    stats_ = {};
    state_ = {};
    return 0;
}
void RefereeParser::discard(std::size_t n) {
    used_ -= n;
    std::memmove(buffer_.data(), buffer_.data() + n, used_);
    std::memmove(receive_ms_.data(), receive_ms_.data() + n, used_ * sizeof(receive_ms_[0]));
    if (used_)
        started_ms_ = receive_ms_[0];
}
void RefereeParser::decode(std::uint16_t id, const std::uint8_t *p, std::size_t n, std::uint64_t now) {
    using namespace robotics;
    // RM2026 communication protocol V1.3.0, tables 1-12 and 1-13.
    if (version_ != RefereeVersion::Rm2026V1_3 || (id != 0x0201 && id != 0x0202)) {
        ++stats_.unknown_commands;
        return;
    }
    if ((id == 0x0201 && n != 13) || (id == 0x0202 && n != 14)) {
        ++stats_.length_errors;
        return;
    }
    auto next = state_;
    const MessageStamp stamp{now, ++sequence_, true};
    if (id == 0x0201) {
        next.robot.robot_id = p[0];
        next.robot.gimbal_output = {true, (p[12] & 1) != 0, stamp};
        next.robot.chassis_output = {true, (p[12] & 2) != 0, stamp};
        next.robot.shooter_output = {true, (p[12] & 4) != 0, stamp};
        next.power.chassis_power_limit_w = wire::loadLe16(p + 10);
        next.power.limit_stamp = stamp;
    }
    else {
        // Bytes 0..7 are RESERVED in this version: never publish fake measured power.
        next.power.buffer_energy_j = wire::loadLe16(p + 8);
        next.power.stamp = stamp;
    }
    next.stamp = stamp;
    next.online = true;
    state_ = next;
}
void RefereeParser::scan(std::uint64_t now) {
    while (used_) {
        const auto *p = buffer_.data();
        if (p[0] != 0xa5) {
            discard(1);
            continue;
        }
        if (used_ < 5)
            break;
        if (refereeCrc8(p, 4) != p[4]) {
            ++stats_.crc8_errors;
            discard(1);
            continue;
        }
        const std::size_t n = wire::loadLe16(p + 1), total = n + 9;
        if (total > buffer_.size()) {
            ++stats_.length_errors;
            discard(1);
            continue;
        }
        if (used_ < total)
            break;
        if (refereeCrc16(p, total - 2) != wire::loadLe16(p + total - 2)) {
            ++stats_.crc16_errors;
            discard(1);
            continue;
        }
        ++stats_.valid_frames;
        decode(wire::loadLe16(p + 5), p + 7, n, receive_ms_[total - 1]);
        discard(total);
        started_ms_ = now;
    }
}
int RefereeParser::consume(const std::uint8_t *p, std::size_t n, std::uint64_t now) {
    if (!p && n)
        return -EINVAL;
    if (used_ && (now < started_ms_ || now - started_ms_ > 100)) {
        ++stats_.assembly_timeouts;
        discard(1);
        scan(now);
        started_ms_ = now;
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (!used_)
            started_ms_ = now;
        receive_ms_[used_] = now;
        buffer_[used_++] = p[i];
        scan(now);
    }
    return 0;
}
}
