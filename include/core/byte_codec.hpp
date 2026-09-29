#pragma once
#include <cstdint>
#include <cstring>
#include <limits>
namespace skywalker::core::wire {
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
inline std::uint16_t u16(const std::uint8_t *p) {
    return p[0] | (std::uint16_t(p[1]) << 8);
}
inline std::uint32_t u32(const std::uint8_t *p) {
    return p[0] | (std::uint32_t(p[1]) << 8) | (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
}
inline float f32(const std::uint8_t *p) {
    auto bits = u32(p);
    float v;
    std::memcpy(&v, &bits, 4);
    return v;
}
inline void put16(std::uint8_t *p, std::uint16_t v) {
    p[0] = v;
    p[1] = v >> 8;
}
inline void put32(std::uint8_t *p, std::uint32_t v) {
    for (unsigned i = 0; i < 4; ++i)
        p[i] = v >> (8 * i);
}
inline void putFloat(std::uint8_t *p, float v) {
    std::uint32_t bits;
    std::memcpy(&bits, &v, 4);
    put32(p, bits);
}
}
