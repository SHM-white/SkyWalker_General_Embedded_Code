#include <communication/vision/ab_protocol.hpp>
#include <core/attitude.hpp>
#include <core/byte_codec.hpp>
#include <cerrno>
#include <cstring>
namespace skywalker::communication::vision {
int AbProtocol::validateConfig() const {
    return config_.assembly_timeout_us && config_.command_reference.frame_id && config_.command_reference.epoch
               ? 0
               : -EINVAL;
}
void AbProtocol::reset() {
    used_ = 0;
    stats_ = {};
}
void AbProtocol::drop() {
    --used_;
    std::memmove(bytes_, bytes_ + 1, used_);
    std::memmove(times_, times_ + 1, used_ * sizeof(times_[0]));
}
std::uint16_t AbProtocol::crc16(const std::uint8_t *p, std::size_t n) {
    std::uint16_t crc = 0xffff;
    for (std::size_t i = 0; i < n; ++i) {
        crc ^= p[i];
        for (unsigned b = 0; b < 8; ++b)
            crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
    }
    return crc;
}
int AbProtocol::consume(const std::uint8_t *p, std::size_t n, core::TimeUs rx, CommandSink &sink) {
    if (!p && n)
        return -EINVAL;
    if (used_ && (rx < times_[used_ - 1] || rx - times_[0] > config_.assembly_timeout_us)) {
        used_ = 0;
        ++stats_.assembly_timeouts;
    }
    int accepted = 0;
    for (std::size_t i = 0; i < n; ++i) {
        bytes_[used_] = p[i];
        times_[used_] = rx;
        ++used_;
        while (used_) {
            if (bytes_[0] != 'A') {
                drop();
                continue;
            }
            if (used_ < 2)
                break;
            if (bytes_[1] != 'B') {
                drop();
                continue;
            }
            if (used_ < 3)
                break;
            if (bytes_[2] > 2) {
                ++stats_.invalid_frames;
                drop();
                continue;
            }
            if (used_ < CommandSize)
                break;
            if (crc16(bytes_, CommandSize - 2) != core::wire::u16(bytes_ + CommandSize - 2)) {
                ++stats_.crc_errors;
                drop();
                continue;
            }
            AimCommand command{};
            command.control_requested = bytes_[2] != 0;
            command.fire_requested = bytes_[2] == 2;
            command.reference = config_.command_reference;
            command.yaw = {core::wire::f32(bytes_ + 3), core::wire::f32(bytes_ + 7), core::wire::f32(bytes_ + 11)};
            command.pitch = {core::wire::f32(bytes_ + 15), core::wire::f32(bytes_ + 19), core::wire::f32(bytes_ + 23)};
            if (sink.accept(command, times_[CommandSize - 1]) < 0)
                ++stats_.invalid_frames;
            else {
                ++stats_.frames;
                ++accepted;
            }
            used_ = 0;
        }
    }
    return accepted;
}
int AbProtocol::encodeFeedback(const Feedback &f, core::TimeUs, std::uint8_t *dst, std::size_t cap) {
    if (!dst)
        return -EINVAL;
    if (cap < FeedbackSize)
        return -EMSGSIZE;
    // AB has no validity bits: refuse incomplete feedback instead of inventing measurements.
    if (!f.orientation.stamp.valid || !f.gyro_rad_s.stamp.valid || !f.bullet_speed_m_s.stamp.valid ||
        !f.bullet_count.stamp.valid)
        return -ENODATA;
    if (f.reference != config_.command_reference)
        return -ESTALE; // AB cannot convey a changed reference/session on the wire.
    auto q = f.orientation.value;
    if (!core::normalize(q) || !core::finite(f.gyro_rad_s.value) || !std::isfinite(f.bullet_speed_m_s.value) ||
        f.bullet_speed_m_s.value < 0 || static_cast<unsigned>(f.mode) > 3)
        return -EINVAL;
    const auto e = core::euler(q);
    const float cp = std::cos(e.pitch);
    if (std::fabs(cp) < 0.01f)
        return -ERANGE; // ZYX Euler rate singularity.
    const auto g = f.gyro_rad_s.value;
    const float yaw_rate = (std::sin(e.roll) * g.y + std::cos(e.roll) * g.z) / cp;
    const float pitch_rate = std::cos(e.roll) * g.y - std::sin(e.roll) * g.z;
    if (!std::isfinite(yaw_rate) || !std::isfinite(pitch_rate))
        return -ERANGE;
    dst[0] = 'A';
    dst[1] = 'B';
    dst[2] = static_cast<std::uint8_t>(f.mode);
    const float values[] = {q.w, q.x, q.y, q.z, e.yaw, yaw_rate, e.pitch, pitch_rate, f.bullet_speed_m_s.value};
    for (unsigned i = 0; i < 9; ++i)
        core::wire::putFloat(dst + 3 + 4 * i, values[i]);
    core::wire::put16(dst + 39, f.bullet_count.value);
    core::wire::put16(dst + 41, crc16(dst, 41));
    return FeedbackSize;
}
}
