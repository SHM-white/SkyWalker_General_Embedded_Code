#include <robotics/gimbal/yaw_centering.hpp>

#include <algorithm>
#include <cmath>

namespace skywalker::robotics {
void YawCenteringController::reset() {
    rate_rad_s_ = 0;
    following_ = false;
    previous_us_ = 0;
}

YawCenteringOutput YawCenteringController::update(const YawCenteringInputs &in, core::TimeUs now) {
    YawCenteringOutput out{};
    out.stamp = {now / 1000, ++sequence_, true};
    const auto previous = previous_us_;
    previous_us_ = now;
    const auto &c = config_;
    out.center_error_rad = in.joint_yaw_rad - c.center_rad;
    const bool config = std::isfinite(c.center_rad) && std::isfinite(c.deadband_rad) &&
        std::isfinite(c.hysteresis_rad) && std::isfinite(c.kp_rad_s_per_rad) &&
        std::isfinite(c.max_rate_rad_s) && std::isfinite(c.acceleration_rad_s2) &&
        c.deadband_rad >= 0 && c.hysteresis_rad > 0 && c.kp_rad_s_per_rad > 0 &&
        c.max_rate_rad_s > 0 && c.acceleration_rad_s2 > 0 &&
        std::fabs(c.follow_direction) == 1 && c.feedback_timeout_us && c.source_timeout_us && c.max_cycle_us;
    if (!config)
        out.reason = WaitReason::Configuration;
    else if (!in.enabled)
        out.reason = WaitReason::Command;
    else if (!in.permission_valid)
        out.reason = WaitReason::Power;
    else if (!in.head_stable)
        out.reason = WaitReason::Reference;
    else if (!std::isfinite(in.joint_yaw_rad) || !core::fresh(in.joint_stamp, now, c.feedback_timeout_us))
        out.reason = WaitReason::Feedback;
    else if (!core::fresh(in.source_stamp, now, c.source_timeout_us))
        out.reason = WaitReason::Command;
    else if (!previous || now <= previous || now - previous > c.max_cycle_us)
        out.reason = WaitReason::Cycle;
    else {
        const float absolute_error = std::fabs(out.center_error_rad);
        if (following_ && absolute_error <= c.deadband_rad)
            following_ = false;
        else if (!following_ && absolute_error >= c.deadband_rad + c.hysteresis_rad)
            following_ = true;
        const float target = following_ ? std::clamp(c.follow_direction * c.kp_rad_s_per_rad *
            out.center_error_rad, -c.max_rate_rad_s, c.max_rate_rad_s) : 0;
        const float change = c.acceleration_rad_s2 * float(now - previous) * 1e-6f;
        rate_rad_s_ += std::clamp(target - rate_rad_s_, -change, change);
        out.velocity_rad_s = rate_rad_s_;
        out.enabled = true;
        out.reason = WaitReason::None;
        return out;
    }
    // A revoked prerequisite withdraws immediately; it never ramps stale motion.
    rate_rad_s_ = 0;
    following_ = false;
    return out;
}
} // namespace skywalker::robotics
