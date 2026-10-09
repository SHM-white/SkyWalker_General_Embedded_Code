#include <drivers/imu/dm_imu_protocol.hpp>
#include <core/attitude.hpp>
#include <core/byte_codec.hpp>
#include <array>
#include <cerrno>
#include <cstring>
namespace skywalker::imu {
namespace {
constexpr auto makeTable() {
    std::array<std::uint16_t, 256> t{};
    for (unsigned i = 0; i < 256; ++i) {
        std::uint16_t c = i << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            c = std::uint16_t((std::uint32_t(c) << 1) ^ ((c & 0x8000) ? 0x1021 : 0));
        t[i] = c;
    }
    return t;
}
constexpr auto table = makeTable();
}
std::uint16_t dmImuCrc16(const std::uint8_t *p, std::size_t size) {
    std::uint16_t crc = 0xffff;
    for (std::size_t i = 0; i < size; ++i)
        crc = std::uint16_t((std::uint32_t(crc) << 1) ^ table[std::uint8_t((crc >> 8) ^ p[i])]);
    return crc;
}
int DmImuRs485Parser::init() {
    if (initialized_)
        return -EALREADY;
    if (!config_.assembly_timeout_us)
        return -EINVAL;
    used_ = 0;
    stats_ = {};
    initialized_ = true;
    return 0;
}
int DmImuRs485Parser::encodeRead(std::uint8_t rid, std::uint8_t *out, std::size_t capacity) const {
    if (!out || capacity < frame_size || rid > 3)
        return -EINVAL;
    std::memset(out, 0, frame_size);
    out[0] = 0xa5;
    out[1] = 0x0d;
    out[2] = config_.id;
    out[3] = rid;
    out[23] = 0x5a;
    return frame_size;
}
void DmImuRs485Parser::drop() {
    --used_;
    std::memmove(bytes_, bytes_ + 1, used_);
    std::memmove(times_, times_ + 1, used_ * sizeof(times_[0]));
}
int DmImuRs485Parser::consume(const std::uint8_t *p, std::size_t size, core::TimeUs rx,
                             DmImuRs485Sink &sink) {
    if (!initialized_)
        return -EACCES;
    if (!p && size)
        return -EINVAL;
    if (used_ && (rx < times_[used_ - 1] || rx - times_[0] > config_.assembly_timeout_us)) {
        used_ = 0;
        ++stats_.assembly_timeouts;
    }
    int accepted = 0;
    for (std::size_t i = 0; i < size; ++i) {
        bytes_[used_] = p[i];
        times_[used_++] = rx;
        while (used_) {
            if (bytes_[0] != 0xa5) {
                drop();
                continue;
            }
            if (used_ < 5)
                break;
            if (bytes_[1] != 0x0d || bytes_[2] != config_.id || bytes_[3] > 3 || bytes_[4] != 0) {
                ++stats_.invalid_frames;
                drop();
                continue;
            }
            if (used_ < frame_size)
                break;
            if (bytes_[22] != 0 || bytes_[23] != 0x5a) {
                ++stats_.invalid_frames;
                drop();
                continue;
            }
            const auto rid = bytes_[3], code = bytes_[21];
            Update u{};
            bool valid = true;
            if (code == 0) {
                const core::Stamp stamp{times_[23], 0, true};
                const float x = core::wire::f32(bytes_ + 5), y = core::wire::f32(bytes_ + 9),
                            z = core::wire::f32(bytes_ + 13);
                valid = core::finite(core::Vec3{x, y, z});
                if (rid == 0) {
                    u.updated_mask = Accel;
                    u.sample.accel_m_s2 = {{x, y, z}, stamp};
                }
                else if (rid == 1) {
                    u.updated_mask = Gyro;
                    u.sample.gyro_rad_s = {{x, y, z}, stamp};
                }
                else if (rid == 3) {
                    u.updated_mask = Orientation;
                    u.sample.orientation = {{x, y, z, core::wire::f32(bytes_ + 17)}, stamp};
                    u.sample.attitude_quality = AttitudeQuality::Unknown;
                    valid = core::normalize(u.sample.orientation.value);
                }
            }
            if (!valid) {
                ++stats_.invalid_frames;
                drop();
                continue;
            }
            ++stats_.frames;
            const int result = sink.acceptDmRs485(rid, code, u, times_[0], times_[23]);
            if (result > 0)
                accepted += result;
            else if (result < 0)
                ++stats_.invalid_frames;
            used_ = 0;
        }
    }
    return accepted;
}
int DmImuParser::init() {
    if (initialized_)
        return -EALREADY;
    if (!config_.assembly_timeout_us)
        return -EINVAL;
    used_ = 0;
    stats_ = {};
    initialized_ = true;
    return 0;
}
void DmImuParser::drop() {
    --used_;
    std::memmove(bytes_, bytes_ + 1, used_);
    std::memmove(times_, times_ + 1, used_ * sizeof(times_[0]));
}
int DmImuParser::consume(const std::uint8_t *p, std::size_t size, core::TimeUs rx, DmImuSink &sink) {
    if (!initialized_)
        return -EACCES;
    if (!p && size)
        return -EINVAL;
    if (used_ && (rx < times_[used_ - 1] || rx - times_[0] > config_.assembly_timeout_us)) {
        used_ = 0;
        ++stats_.assembly_timeouts;
    }
    int accepted = 0;
    for (std::size_t i = 0; i < size; ++i) {
        bytes_[used_] = p[i];
        times_[used_] = rx;
        ++used_;
        while (used_) {
            if (bytes_[0] != 0x55) {
                drop();
                continue;
            }
            if (used_ < 2)
                break;
            if (bytes_[1] != 0xaa) {
                drop();
                continue;
            }
            if (used_ < 4)
                break;
            const auto type = bytes_[3];
            if (bytes_[2] != config_.id || type < 1 || type > 4) {
                ++stats_.invalid_frames;
                drop();
                continue;
            }
            const std::size_t length = type == 4 ? 23 : 19;
            if (used_ < length)
                break;
            if (bytes_[length - 1] != 0x0a) {
                ++stats_.invalid_frames;
                drop();
                continue;
            }
            if (dmImuCrc16(bytes_, length - 3) != core::wire::u16(bytes_ + length - 3)) {
                ++stats_.crc_errors;
                drop();
                continue;
            }
            Update u{};
            const core::Stamp stamp{times_[length - 1], 0, true};
            const float x = core::wire::f32(bytes_ + 4), y = core::wire::f32(bytes_ + 8),
                        z = core::wire::f32(bytes_ + 12);
            bool valid = core::finite(core::Vec3{x, y, z});
            if (type == 1) {
                u.updated_mask = Accel;
                u.sample.accel_m_s2 = {{x, y, z}, stamp};
            }
            if (type == 2) {
                u.updated_mask = Gyro;
                u.sample.gyro_rad_s = {{x, y, z}, stamp};
            }
            if (type == 4) {
                u.updated_mask = Orientation;
                u.sample.orientation = {{x, y, z, core::wire::f32(bytes_ + 16)}, stamp};
                u.sample.attitude_quality = AttitudeQuality::Unknown;
                valid = core::normalize(u.sample.orientation.value);
            }
            if (!valid) {
                ++stats_.invalid_frames;
                drop();
                continue;
            }
            ++stats_.frames;
            // Euler frames are validated/consumed but never supersede quaternion orientation.
            if (u.updated_mask) {
                if (sink.acceptDm(u) == 0)
                    ++accepted;
                else
                    ++stats_.invalid_frames;
            }
            used_ -= length;
            std::memmove(bytes_, bytes_ + length, used_);
            std::memmove(times_, times_ + length, used_ * sizeof(times_[0]));
        }
    }
    return accepted;
}
}
