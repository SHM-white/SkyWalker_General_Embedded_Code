#include <robotics/gimbal/dual_yaw_motion_allocator.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>

namespace skywalker::robotics {
namespace {
// Continuous soft dead band: zero inside the dead band, then a linear ramp.
float softDeadband(float error, float deadband) {
    return std::copysign(std::max(std::fabs(error) - deadband, 0.0f), error);
}

// Conservative stopping distance for an outward rate `v`:
// one reaction delay with the worst-case outward acceleration, then a full
// deceleration at the measured worst-case braking capability.
float outwardBrakingDistance(float rate, const DualYawAllocatorConfig &config) {
    const float delay = config.brake_delay_s;
    const float delay_rate = rate + config.brake_outward_acceleration_rad_s2 * delay;
    return rate * delay + 0.5f * config.brake_outward_acceleration_rad_s2 * delay * delay +
           delay_rate * delay_rate / (2 * config.brake_deceleration_rad_s2);
}

// Largest outward rate whose braking distance still fits into `distance`.
float safeOutwardRate(float distance, const DualYawAllocatorConfig &config, float capability) {
    if (!(distance > 0) || !(capability > 0))
        return 0;
    if (outwardBrakingDistance(0, config) > distance)
        return 0;
    if (outwardBrakingDistance(capability, config) <= distance)
        return capability;
    float low = 0, high = capability;
    for (int i = 0; i < 24; ++i) {
        const float middle = 0.5f * (low + high);
        if (outwardBrakingDistance(middle, config) <= distance)
            low = middle;
        else
            high = middle;
    }
    return low;
}
} // namespace

int DualYawMotionAllocator::validate() const {
    const auto &c = config_;
    const bool finite = std::isfinite(c.joint_center_rad) && std::isfinite(c.joint_min_rad) &&
                        std::isfinite(c.joint_max_rad) && std::isfinite(c.joint_margin_rad) &&
                        std::isfinite(c.joint_tolerance_rad) && std::isfinite(c.joint_rate_max_rad_s) &&
                        std::isfinite(c.head_rate_max_rad_s) &&
                        std::isfinite(c.head_acceleration_max_rad_s2) && std::isfinite(c.head_jerk_max_rad_s3) &&
                        std::isfinite(c.head_steady_rate_max_rad_s) && std::isfinite(c.big_intent_max_rad_s) &&
                        std::isfinite(c.big_acceleration_max_rad_s2) && std::isfinite(c.big_jerk_max_rad_s3) &&
                        std::isfinite(c.big_track_gain_s_inv) && std::isfinite(c.slow_tau_s) &&
                        std::isfinite(c.center_deadband_rad) && std::isfinite(c.center_kp_rad_s_per_rad) &&
                        std::isfinite(c.center_rate_max_rad_s) && std::isfinite(c.brake_delay_s) &&
                        std::isfinite(c.brake_outward_acceleration_rad_s2) &&
                        std::isfinite(c.brake_deceleration_rad_s2) && std::isfinite(c.tracking_error_rad_s) &&
                        std::isfinite(c.tracking_budget_scale_min) && std::isfinite(c.tracking_budget_slew_per_s) &&
                        std::isfinite(c.still_rate_epsilon_rad_s) &&
                        std::isfinite(c.still_acceleration_epsilon_rad_s2);
    if (!finite || std::fabs(c.joint_direction) != 1 || !(c.joint_margin_rad >= 0) || !(c.joint_tolerance_rad >= 0) ||
        !(c.head_rate_max_rad_s > 0) || !(c.head_acceleration_max_rad_s2 > 0) || !(c.head_jerk_max_rad_s3 >= 0) ||
        !(c.head_steady_rate_max_rad_s > 0) || c.head_steady_rate_max_rad_s > c.head_rate_max_rad_s ||
        !(c.joint_rate_max_rad_s > 0) || !(c.big_intent_max_rad_s > 0) || !(c.big_acceleration_max_rad_s2 > 0) ||
        !(c.big_jerk_max_rad_s3 >= 0) ||
        !(c.big_track_gain_s_inv >= 0) || !(c.slow_tau_s > 0) || !(c.center_deadband_rad >= 0) ||
        !(c.center_kp_rad_s_per_rad > 0) || !(c.center_rate_max_rad_s > 0) || !(c.brake_delay_s >= 0) ||
        !(c.brake_outward_acceleration_rad_s2 >= 0) || !(c.brake_deceleration_rad_s2 > 0) ||
        !(c.tracking_error_rad_s > 0) || !(c.tracking_budget_scale_min > 0) ||
        c.tracking_budget_scale_min > 1 || !(c.tracking_budget_slew_per_s > 0) ||
        !(c.still_rate_epsilon_rad_s >= 0) || !(c.still_acceleration_epsilon_rad_s2 >= 0) ||
        !c.feedback_timeout_us || !c.source_timeout_us || !c.max_cycle_us)
        return -EINVAL;
    return c.joint_center_rad > c.joint_min_rad && c.joint_center_rad < c.joint_max_rad ? 0 : -EINVAL;
}

void DualYawMotionAllocator::dropPlans() {
    head_ = {};
    big_ = {};
    slow_rad_s_ = 0;
    center_error_rad_ = 0;
    budget_scale_ = 1;
    tracking_limited_ = false;
    still_latched_ = true;
    initialized_ = false;
}

void DualYawMotionAllocator::reset() {
    dropPlans();
    previous_us_ = 0;
}

DualYawAllocatorOutput DualYawMotionAllocator::update(const DualYawAllocatorInputs &in, core::TimeUs now) {
    DualYawAllocatorOutput out{};
    const auto previous = previous_us_;
    previous_us_ = now;
    const auto &c = config_;
    // Every rejected cycle drops the plans: a stale prerequisite must never be
    // followed by a deferred motion when the channel recovers.
    if (validate() < 0) {
        dropPlans();
        out.reason = WaitReason::Configuration;
        return out;
    }
    if (!in.enabled) {
        dropPlans();
        out.reason = WaitReason::Command;
        return out;
    }
    if (!in.permission_valid) {
        dropPlans();
        out.reason = WaitReason::Power;
        return out;
    }
    if (!std::isfinite(in.joint_rad) || !std::isfinite(in.joint_rate_rad_s) ||
        !core::fresh(in.joint_stamp, now, c.feedback_timeout_us)) {
        dropPlans();
        out.reason = WaitReason::Feedback;
        return out;
    }
    if (!std::isfinite(in.head_rate_target_rad_s) || !core::fresh(in.source_stamp, now, c.source_timeout_us)) {
        dropPlans();
        out.reason = WaitReason::Command;
        return out;
    }
    if (!previous || now <= previous || now - previous > c.max_cycle_us) {
        dropPlans();
        out.reason = WaitReason::Cycle;
        return out;
    }
    const float dt = static_cast<float>(now - previous) * 1e-6f;
    const bool carrier_valid = in.carrier_valid && std::isfinite(in.carrier_rate_rad_s);
    const float carrier = carrier_valid ? in.carrier_rate_rad_s : 0;

    // --- Small-Yaw travel envelope, each side independently -----------------
    const float usable_min = c.joint_min_rad + c.joint_margin_rad + c.joint_tolerance_rad;
    const float usable_max = c.joint_max_rad - c.joint_margin_rad - c.joint_tolerance_rad;
    const float margin_pos = std::max(usable_max - in.joint_rad, 0.0f);
    const float margin_neg = std::max(in.joint_rad - usable_min, 0.0f);
    const float capability = std::min(c.head_rate_max_rad_s, c.joint_rate_max_rad_s);
    const float budget_pos = safeOutwardRate(margin_pos, c, capability);
    const float budget_neg = safeOutwardRate(margin_neg, c, capability);

    // --- Carrier tracking supervision ---------------------------------------
    tracking_limited_ = carrier_valid && std::fabs(carrier - big_.rate_rad_s) > c.tracking_error_rad_s;
    const float budget_slew = c.tracking_budget_slew_per_s * dt;
    budget_scale_ = std::clamp(budget_scale_ + (tracking_limited_ ? -budget_slew : budget_slew),
                               c.tracking_budget_scale_min, 1.0f);
    const float steady = std::min(c.head_steady_rate_max_rad_s * budget_scale_, c.head_rate_max_rad_s);

    // --- Feasible head rate --------------------------------------------------
    float head_target = std::clamp(in.head_rate_target_rad_s, -steady, steady);
    // Relative joint rate in encoder coordinates: w_enc = direction*(w_head-carrier).
    float relative = std::clamp(c.joint_direction * (head_target - carrier), -budget_neg, budget_pos);
    bool travel_limited = relative != c.joint_direction * (head_target - carrier);
    const float measured = in.joint_rate_rad_s;
    if (measured > budget_pos) {
        // Already faster outward than the braking envelope allows: command at
        // most standstill and let the inward path stay available.
        relative = std::min(relative, 0.0f);
        travel_limited = true;
    }
    if (measured < -budget_neg) {
        relative = std::max(relative, 0.0f);
        travel_limited = true;
    }
    head_target = std::clamp(carrier + c.joint_direction * relative, -c.head_rate_max_rad_s, c.head_rate_max_rad_s);

    const BoundedRateLimits head_limits{-c.head_rate_max_rad_s, c.head_rate_max_rad_s,
                                        c.head_acceleration_max_rad_s2, c.head_jerk_max_rad_s3};
    bool head_limited = false, head_conflict = false;
    if (!stepBoundedRate(head_, (head_target - head_.rate_rad_s) / dt, dt, head_limits, head_limited, head_conflict)) {
        dropPlans();
        out.reason = WaitReason::Configuration;
        return out;
    }

    // --- Slow component and centering ---------------------------------------
    const float center_error_target = c.joint_direction * (in.joint_rad - c.joint_center_rad);
    if (!initialized_) {
        slow_rad_s_ = head_.rate_rad_s;
        center_error_rad_ = center_error_target;
        initialized_ = true;
    }
    const float alpha = 1.0f - std::exp(-dt / c.slow_tau_s);
    const float slow_previous = slow_rad_s_;
    slow_rad_s_ += alpha * (head_.rate_rad_s - slow_rad_s_);
    center_error_rad_ += alpha * (center_error_target - center_error_rad_);
    const float slow_acceleration = (slow_rad_s_ - slow_previous) / dt;
    const float center_rate = std::clamp(c.center_kp_rad_s_per_rad * softDeadband(center_error_rad_, c.center_deadband_rad),
                                         -c.center_rate_max_rad_s, c.center_rate_max_rad_s);

    // --- Big-Yaw world intent ------------------------------------------------
    if (!in.big_yaw_available) {
        // No authorized carrier: keep the small-Yaw path alone and never leave
        // a request behind for the next handshake.
        big_ = {};
    }
    else {
        const float goal = slow_rad_s_ + center_rate;
        out.big_saturated = std::fabs(goal) > c.big_intent_max_rad_s;
        const float acceleration = slow_acceleration + c.big_track_gain_s_inv * (goal - big_.rate_rad_s);
        const BoundedRateLimits big_limits{-c.big_intent_max_rad_s, c.big_intent_max_rad_s,
                                           c.big_acceleration_max_rad_s2, c.big_jerk_max_rad_s3};
        bool big_limited = false, big_conflict = false;
        (void)stepBoundedRate(big_, acceleration, dt, big_limits, big_limited, big_conflict);
    }

    // --- Standstill latch ----------------------------------------------------
    const bool still = std::fabs(head_target) <= c.still_rate_epsilon_rad_s &&
                       std::fabs(head_.rate_rad_s) <= c.still_rate_epsilon_rad_s &&
                       std::fabs(slow_rad_s_) <= c.still_rate_epsilon_rad_s &&
                       std::fabs(center_rate) <= c.still_rate_epsilon_rad_s &&
                       std::fabs(big_.rate_rad_s) <= c.still_rate_epsilon_rad_s &&
                       std::fabs(big_.acceleration_rad_s2) <= c.still_acceleration_epsilon_rad_s2 &&
                       std::fabs(head_.acceleration_rad_s2) <= c.still_acceleration_epsilon_rad_s2;
    const bool release = std::fabs(head_target) > c.still_rate_epsilon_rad_s ||
                         std::fabs(center_rate) > c.still_rate_epsilon_rad_s;
    if (still)
        still_latched_ = true;
    else if (release)
        still_latched_ = false;
    if (still_latched_) {
        // Latch an exact zero request so a sub-threshold command cannot leak a
        // non-zero world rate into the peer every 10 ms.
        head_ = {};
        big_ = {};
        slow_rad_s_ = 0;
    }

    // --- Outputs -------------------------------------------------------------
    out.head = head_;
    out.big = big_;
    out.joint_rate_world_rad_s = head_.rate_rad_s - carrier;
    out.joint_rate_rad_s = std::clamp(c.joint_direction * out.joint_rate_world_rad_s, -budget_neg, budget_pos);
    out.center_error_rad = center_error_rad_;
    out.outward_budget_pos_rad_s = budget_pos;
    out.outward_budget_neg_rad_s = budget_neg;
    out.brake_margin_pos_rad = margin_pos;
    out.brake_margin_neg_rad = margin_neg;
    out.travel_limited = travel_limited || head_limited;
    out.tracking_limited = tracking_limited_;
    out.enabled = true;
    out.reason = WaitReason::None;
    return out;
}

} // namespace skywalker::robotics
