#include <algorithm>
#include <cmath>
#include <cerrno>
#include <robotics/chassis/chassis_power_limiter.hpp>
namespace skywalker::robotics {
int ChassisPowerLimiter::reset() {
    scale_ = 0;
    return 0;
}
int ChassisPowerLimiter::step(const ChassisPowerInput &i, float dt_s, ChassisPowerDecision &out) {
    if (!std::isfinite(i.measured_power_w) || i.measured_power_w < 0 || !std::isfinite(i.power_limit_w) ||
        i.power_limit_w < 0 || !std::isfinite(i.buffer_energy_j) || i.buffer_energy_j < 0 || !std::isfinite(dt_s) ||
        dt_s <= 0 || dt_s > 0.1f || !std::isfinite(config_.recovery_per_s) || config_.recovery_per_s <= 0 ||
        !std::isfinite(config_.buffer_reserve_j) || config_.buffer_reserve_j < 0)
        return -EINVAL;
    float target = i.power_limit_w == 0 ? 0 : std::min(1.0f, i.power_limit_w / std::max(i.measured_power_w, 0.001f));
    if (config_.buffer_reserve_j > 0)
        target *= std::min(1.0f, i.buffer_energy_j / config_.buffer_reserve_j);
    scale_ = std::min(target, scale_ + config_.recovery_per_s * dt_s);
    out = {scale_};
    return 0;
}
}
