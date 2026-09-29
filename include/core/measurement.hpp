#pragma once
#include <cstdint>

namespace skywalker::core {
using TimeUs = std::uint64_t;
struct Vec3 {
    float x = 0, y = 0, z = 0;
};
struct Quaternion {
    float w = 1, x = 0, y = 0, z = 0;
};
struct Stamp {
    TimeUs time_us = 0;
    std::uint64_t sequence = 0;
    bool valid = false;
};
template <class T> struct Measurement {
    T value{};
    Stamp stamp{};
};
struct OrientationReference {
    std::uint32_t frame_id = 0, epoch = 0;
    bool operator==(const OrientationReference &) const = default;
};
inline bool fresh(const Stamp &s, TimeUs now, TimeUs limit) {
    return s.valid && limit && now >= s.time_us && now - s.time_us <= limit;
}
} // namespace skywalker::core
