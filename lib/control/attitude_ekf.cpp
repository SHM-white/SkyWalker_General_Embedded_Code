#include <control/attitude_ekf.hpp>
#include <core/attitude.hpp>
#include <cerrno>
#include <cstring>

namespace skywalker::control {
namespace {
bool inverse3(const float a[3][3], float out[3][3]) {
    const float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                      a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                      a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (!std::isfinite(det) || det <= 1e-12f)
        return false;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out[c][r] = (a[(r + 1) % 3][(c + 1) % 3] * a[(r + 2) % 3][(c + 2) % 3] -
                         a[(r + 1) % 3][(c + 2) % 3] * a[(r + 2) % 3][(c + 1) % 3]) /
                        det;
    return true;
}
}
int QuaternionEkf::init() {
    const auto &c = config_;
    const float values[] = {c.dt_min_s,        c.dt_max_s,      c.gravity_m_s2,
                            c.accel_gate_m_s2, c.process_noise, c.measurement_noise,
                            c.innovation_gate, c.accel_tau_s,   c.stationary_gyro_rad_s,
                            c.bias_tau_s};
    for (float v : values)
        if (!std::isfinite(v) || v <= 0)
            return -EINVAL;
    if (c.dt_min_s > c.dt_max_s || c.initialization_samples < 2)
        return -EINVAL;
    if (configured_)
        return -EALREADY;
    configured_ = true;
    reset();
    return 0;
}
void QuaternionEkf::reset() {
    q_ = {};
    bias_ = filtered_ = accel_sum_ = gyro_sum_ = {};
    std::memset(p_, 0, sizeof(p_));
    for (int i = 0; i < 4; ++i)
        p_[i][i] = 0.01f;
    initial_count_ = 0;
    previous_us_ = 0;
    initialized_ = have_time_ = false;
    quality_ = Quality::Initializing;
    if (++generation_ == 0)
        ++generation_;
}
int QuaternionEkf::update(const Input &in) {
    if (!configured_)
        return -EACCES;
    if (!core::finite(in.accel_m_s2) || !core::finite(in.gyro_rad_s))
        return -EINVAL;
    const auto &c = config_;
    if (have_time_ && in.time_us <= previous_us_)
        return -ESTALE;
    float dt = have_time_ ? float(in.time_us - previous_us_) * 1e-6f : c.dt_min_s;
    if (have_time_ && dt > c.dt_max_s) {
        reset();
        return -EAGAIN;
    }
    if (have_time_ && dt < c.dt_min_s)
        return -EAGAIN;
    previous_us_ = in.time_us;
    have_time_ = true;
    const auto a = in.accel_m_s2, g = in.gyro_rad_s;
    const float an = core::norm(a), gn = core::norm(g);
    if (!std::isfinite(an) || !std::isfinite(gn)) {
        reset();
        return -ERANGE;
    }
    const bool gravity_ok = std::fabs(an - c.gravity_m_s2) <= c.accel_gate_m_s2;
    const bool stationary = gravity_ok && gn < c.stationary_gyro_rad_s;
    if (!initialized_) {
        if (!stationary) {
            initial_count_ = 0;
            accel_sum_ = {};
            gyro_sum_ = {};
            return -EAGAIN;
        }
        accel_sum_.x += a.x;
        accel_sum_.y += a.y;
        accel_sum_.z += a.z;
        gyro_sum_.x += g.x;
        gyro_sum_.y += g.y;
        gyro_sum_.z += g.z;
        if (++initial_count_ < c.initialization_samples)
            return -EAGAIN;
        const float inv = 1.0f / initial_count_;
        filtered_ = {accel_sum_.x * inv, accel_sum_.y * inv, accel_sum_.z * inv};
        bias_ = {gyro_sum_.x * inv, gyro_sum_.y * inv, gyro_sum_.z * inv};
        const auto v = filtered_;
        q_ = core::fromEuler({std::atan2(v.y, v.z), std::atan2(-v.x, std::hypot(v.y, v.z)), 0});
        initialized_ = true;
        quality_ = Quality::Tracking;
        return 0;
    }
    if (stationary) {
        const float beta = dt / (c.bias_tau_s + dt);
        bias_.x += beta * (g.x - bias_.x);
        bias_.y += beta * (g.y - bias_.y);
        bias_.z += beta * (g.z - bias_.z);
    }
    const float hdt = 0.5f * dt;
    const float x = (g.x - bias_.x) * hdt, y = (g.y - bias_.y) * hdt, z = (g.z - bias_.z) * hdt;
    const float f[4][4] = {{1, -x, -y, -z}, {x, 1, z, -y}, {y, -z, 1, x}, {z, y, -x, 1}};
    const float oldq[4] = {q_.w, q_.x, q_.y, q_.z};
    float predicted[4]{}, fp[4][4]{}, pp[4][4]{};
    for (int r = 0; r < 4; ++r)
        for (int k = 0; k < 4; ++k) {
            predicted[r] += f[r][k] * oldq[k];
            for (int col = 0; col < 4; ++col)
                fp[r][col] += f[r][k] * p_[k][col];
        }
    for (int r = 0; r < 4; ++r)
        for (int col = 0; col < 4; ++col) {
            for (int k = 0; k < 4; ++k)
                pp[r][col] += fp[r][k] * f[col][k];
            if (r == col)
                pp[r][col] += c.process_noise * dt;
        }
    core::Quaternion next{predicted[0], predicted[1], predicted[2], predicted[3]};
    if (!core::normalize(next)) {
        reset();
        return -ERANGE;
    }
    const float alpha = dt / (c.accel_tau_s + dt);
    filtered_.x += alpha * (a.x - filtered_.x);
    filtered_.y += alpha * (a.y - filtered_.y);
    filtered_.z += alpha * (a.z - filtered_.z);
    const float fn = core::norm(filtered_);
    quality_ = Quality::Degraded;
    float result_p[4][4];
    std::memcpy(result_p, pp, sizeof(pp));
    if (gravity_ok && fn > 0.1f) {
        const float w = next.w, xq = next.x, yq = next.y, zq = next.z;
        const float h[3][4] = {{-2 * yq, 2 * zq, -2 * w, 2 * xq},
                               {2 * xq, 2 * w, 2 * zq, 2 * yq},
                               {2 * w, -2 * xq, -2 * yq, 2 * zq}};
        const float residual[3] = {filtered_.x / fn - 2 * (xq * zq - w * yq), filtered_.y / fn - 2 * (w * xq + yq * zq),
                                   filtered_.z / fn - (w * w - xq * xq - yq * yq + zq * zq)};
        float ph[4][3]{}, s[3][3]{}, si[3][3]{};
        for (int r = 0; r < 4; ++r)
            for (int col = 0; col < 3; ++col)
                for (int k = 0; k < 4; ++k)
                    ph[r][col] += pp[r][k] * h[col][k];
        for (int r = 0; r < 3; ++r)
            for (int col = 0; col < 3; ++col) {
                for (int k = 0; k < 4; ++k)
                    s[r][col] += h[r][k] * ph[k][col];
                if (r == col)
                    s[r][col] += c.measurement_noise;
            }
        if (!inverse3(s, si)) {
            reset();
            return -ERANGE;
        }
        float nis = 0;
        for (int r = 0; r < 3; ++r)
            for (int col = 0; col < 3; ++col)
                nis += residual[r] * si[r][col] * residual[col];
        if (!std::isfinite(nis)) {
            reset();
            return -ERANGE;
        }
        if (nis <= c.innovation_gate) {
            float gain[4][3]{}, correction[4]{}, aij[4][4]{}, ap[4][4]{};
            for (int r = 0; r < 4; ++r)
                for (int col = 0; col < 3; ++col)
                    for (int k = 0; k < 3; ++k)
                        gain[r][col] += ph[r][k] * si[k][col];
            for (int r = 0; r < 4; ++r) {
                for (int k = 0; k < 3; ++k)
                    correction[r] += gain[r][k] * residual[k];
                for (int col = 0; col < 4; ++col) {
                    aij[r][col] = (r == col) ? 1.0f : 0.0f;
                    for (int k = 0; k < 3; ++k)
                        aij[r][col] -= gain[r][k] * h[k][col];
                }
            }
            next.w += correction[0];
            next.x += correction[1];
            next.y += correction[2];
            next.z += correction[3];
            if (!core::normalize(next)) {
                reset();
                return -ERANGE;
            }
            // Joseph form preserves covariance symmetry/positive semidefiniteness.
            for (int r = 0; r < 4; ++r)
                for (int col = 0; col < 4; ++col)
                    for (int k = 0; k < 4; ++k)
                        ap[r][col] += aij[r][k] * pp[k][col];
            for (int r = 0; r < 4; ++r)
                for (int col = 0; col < 4; ++col) {
                    result_p[r][col] = 0;
                    for (int k = 0; k < 4; ++k)
                        result_p[r][col] += ap[r][k] * aij[col][k];
                    for (int k = 0; k < 3; ++k)
                        result_p[r][col] += gain[r][k] * c.measurement_noise * gain[col][k];
                }
            quality_ = Quality::Tracking;
        }
    }
    for (int r = 0; r < 4; ++r)
        for (int col = 0; col < 4; ++col) {
            if (!std::isfinite(result_p[r][col]) || (r == col && result_p[r][col] < 0)) {
                reset();
                return -ERANGE;
            }
        }
    q_ = next;
    std::memcpy(p_, result_p, sizeof(p_));
    return 0;
}
}
