#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
namespace skywalker::communication::wire {
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
inline void storeLe16(std::uint8_t *p, std::uint16_t v) {
    p[0] = v;
    p[1] = v >> 8;
}
inline void storeLe32(std::uint8_t *p, std::uint32_t v) {
    for (unsigned i = 0; i < 4; ++i)
        p[i] = v >> (8 * i);
}
inline void storeLe64(std::uint8_t *p, std::uint64_t v) {
    for (unsigned i = 0; i < 8; ++i)
        p[i] = v >> (8 * i);
}
inline std::uint16_t loadLe16(const std::uint8_t *p) {
    return std::uint16_t(p[0]) | (std::uint16_t(p[1]) << 8);
}
inline std::uint32_t loadLe32(const std::uint8_t *p) {
    std::uint32_t v = 0;
    for (unsigned i = 0; i < 4; ++i)
        v |= std::uint32_t(p[i]) << (8 * i);
    return v;
}
inline std::uint64_t loadLe64(const std::uint8_t *p) {
    std::uint64_t v = 0;
    for (unsigned i = 0; i < 8; ++i)
        v |= std::uint64_t(p[i]) << (8 * i);
    return v;
}
inline void storeFloatLe(std::uint8_t *p, float f) {
    std::uint32_t v;
    std::memcpy(&v, &f, 4);
    storeLe32(p, v);
}
inline float loadFloatLe(const std::uint8_t *p) {
    auto v = loadLe32(p);
    float f;
    std::memcpy(&f, &v, 4);
    return f;
}
}
