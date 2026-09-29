#pragma once
#include <algorithm>
#include <cmath>
#include <core/measurement.hpp>

namespace skywalker::core {
inline bool finite(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
inline bool finite(Quaternion q) {
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}
inline float norm(Vec3 v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
// Reject corrupt quaternions; normalization only removes small numerical errors.
inline bool normalize(Quaternion &q, float tolerance = 0.2f) {
    const float n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (!finite(q) || !std::isfinite(n2) || n2 < 1e-8f || std::fabs(n2 - 1.0f) > tolerance)
        return false;
    const float inv = 1.0f / std::sqrt(n2);
    q.w *= inv;
    q.x *= inv;
    q.y *= inv;
    q.z *= inv;
    return true;
}
inline Quaternion conjugate(Quaternion q) {
    return {q.w, -q.x, -q.y, -q.z};
}
inline Quaternion multiply(Quaternion a, Quaternion b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
inline Vec3 rotate(Quaternion q, Vec3 v) {
    const auto r = multiply(multiply(q, {0, v.x, v.y, v.z}), conjugate(q));
    return {r.x, r.y, r.z};
}
struct Euler {
    float roll = 0, pitch = 0, yaw = 0;
};
// Hamilton wxyz; q maps body to world; right-handed ZYX Euler in radians.
inline Euler euler(Quaternion q) {
    return {std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
            std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0f, 1.0f)),
            std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
}
inline Quaternion fromEuler(Euler e) {
    const float cr = std::cos(e.roll / 2), sr = std::sin(e.roll / 2);
    const float cp = std::cos(e.pitch / 2), sp = std::sin(e.pitch / 2);
    const float cy = std::cos(e.yaw / 2), sy = std::sin(e.yaw / 2);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy};
}
} // namespace skywalker::core
