#pragma once

#include <algorithm>
#include <cmath>
#include <core/measurement.hpp>
#include <robotics/execution/bounded_rate_step.hpp>
#include <robotics/execution/run_status.hpp>

namespace skywalker::robotics {

// Cloud-board dual-Yaw allocation.
//
// The operator (or, later, the arbitrated vision target) expresses one head
// world rate. This class splits it between the big-Yaw carrier and the small
// Yaw joint:
//
//   w_head = w_big + direction_s * qdot_small   (all at the head, CCW positive)
//
// The active head intent has its own bounded rate plan. A low-passed copy of
// that intent drives the big Yaw, with a centre term bringing the small Yaw
// back to its mechanical centre. Separately, the feasible head rate is bounded
// by the small-Yaw braking/travel envelope around the measured carrier rate.
// This protection may force the head to follow a moving carrier temporarily;
// that passive motion must never become a new active big-Yaw request.
//
// Coordinates: angles in rad, rates in rad/s, accelerations in rad/s^2, jerk in
// rad/s^3. The joint angle/rate are encoder coordinates; the world values are
// counter-clockwise positive. `joint_direction` is applied exactly once.
struct DualYawAllocatorConfig {
    // Small-Yaw joint geometry in encoder coordinates.
    float joint_center_rad = 0;
    float joint_min_rad = -0.6f, joint_max_rad = 0.6f;
    float joint_direction = 1;          // encoder -> head-relative world sign, exactly +1 or -1
    float joint_margin_rad = 0.0175f;   // kept clear of each measured mechanical end
    float joint_tolerance_rad = 0.012f; // position-loop deadband and calibration allowance
    float joint_rate_max_rad_s = 8.0f;  // executable small-Yaw joint speed, encoder coordinates
    // Head (world) rate budget.
    float head_rate_max_rad_s = 1.0f;
    float head_acceleration_max_rad_s2 = 4.0f;
    float head_jerk_max_rad_s3 = 0; // zero disables jerk shaping
    // Sustained budget; the initial bench stage limits every manual rate here.
    float head_steady_rate_max_rad_s = 0.8f;
    // Big-Yaw carrier world intent.
    float big_intent_max_rad_s = 1.0f;
    float big_acceleration_max_rad_s2 = 4.0f;
    float big_jerk_max_rad_s3 = 0;
    float big_track_gain_s_inv = 2.0f;
    // First-order low pass on the bounded active head intent, before travel protection.
    float slow_tau_s = 0.1f;
    // Vision keeps more of the carrier motion in the big Yaw, so its slow filter
    // is deliberately longer than the manual one.
    float vision_slow_tau_s = 0.24f;
    // Vision angle-error rate gain, 1/s, applied to the aligned head target.
    float vision_angle_gain_s_inv = 8.0f;
    // Small-Yaw joint acceleration capability used to tighten the head plan when
    // the carrier acceleration measurement is fresh.
    float joint_acceleration_max_pos_rad_s2 = 20.0f, joint_acceleration_max_neg_rad_s2 = 20.0f;
    // Centering of the small Yaw around its mechanical centre.
    float center_deadband_rad = 0.0087f;
    float center_kp_rad_s_per_rad = 6.0f;
    float center_rate_max_rad_s = 1.0f;
    // Outward braking envelope of the small-Yaw travel.
    float brake_delay_s = 0.02f;
    float brake_outward_acceleration_rad_s2 = 4.0f;
    float brake_deceleration_rad_s2 = 10.0f;
    // Carrier tracking supervision.
    float tracking_error_rad_s = 0.35f;
    float tracking_budget_scale_min = 0.4f;
    float tracking_budget_slew_per_s = 0.5f;
    // Standstill latch thresholds.
    float still_rate_epsilon_rad_s = 0.01f;
    float still_acceleration_epsilon_rad_s2 = 0.05f;
    core::TimeUs feedback_timeout_us = 50000, source_timeout_us = 100000, max_cycle_us = 20000;
};

struct DualYawAllocatorInputs {
    // Raw head world-rate target before any carrier compensation.
    float head_rate_target_rad_s = 0;
    // Small-Yaw joint measurement, encoder coordinates.
    float joint_rad = 0, joint_rate_rad_s = 0;
    // Measured big-Yaw carrier world rate.
    float carrier_rate_rad_s = 0;
    core::Stamp joint_stamp{}, source_stamp{}, carrier_stamp{};
    bool carrier_valid = false;
    // Big-Yaw execution channel authorized and its feedback fresh.
    bool big_yaw_available = false;
    bool enabled = false, permission_valid = false;
    // Arbitrated vision trajectory, already aligned to the head world frame.
    // When `vision_active` the head rate target is generated from the angle
    // error instead of `head_rate_target_rad_s`.
    bool vision_active = false;
    float vision_angle_rad = 0, vision_rate_rad_s = 0, vision_acceleration_rad_s2 = 0;
    std::uint32_t vision_epoch = 0;
    // Measured head world angle, used to reseed the planned angle whenever the
    // vision reference or session changes.
    float head_angle_measured_rad = 0;
    bool carrier_acceleration_valid = false;
    float carrier_acceleration_rad_s2 = 0;
};

struct DualYawAllocatorOutput {
    bool enabled = false;
    BoundedRatePlan head{}; // feasible head world rate that the small Yaw can still realize
    BoundedRatePlan big{};  // big-Yaw world intent
    // Small-Yaw joint rate in encoder coordinates, direction applied once.
    float joint_rate_rad_s = 0;
    // True whenever the joint rate above already carries the travel protection,
    // including the degraded local path. The caller may use it directly.
    bool joint_rate_valid = false;
    // The accumulated joint lead must be dropped and reseeded from the measured
    // position: an over-envelope excursion, a projected target or a vision
    // hand-over invalidated the previous plan.
    bool joint_reseed_requested = false;
    // Planned joint position/acceleration that produced the rate, encoder
    // coordinates. The position is the one the trajectory interface consumes.
    float joint_position_rad = 0, joint_acceleration_rad_s2 = 0;
    // Head world rate left to the small Yaw (head plan minus measured carrier).
    float joint_rate_world_rad_s = 0;
    float center_error_rad = 0;
    float outward_budget_pos_rad_s = 0, outward_budget_neg_rad_s = 0;
    float brake_margin_pos_rad = 0, brake_margin_neg_rad = 0;
    bool big_saturated = false, travel_limited = false, tracking_limited = false;
    bool vision_mode = false;
    WaitReason reason = WaitReason::Command;
};

// Differentiates the carrier gyro rate only between two genuinely new samples.
// Repeated reads of the same stamp keep the filtered value and its original
// timestamp; a gap, a time regression or a session change drops the history and
// reports `valid() == false`, so the acceleration feedforward can be disabled
// independently of the velocity compensation.
class CarrierAccelerationEstimator {
public:
    void reset() {
        initialized_ = false;
        valid_ = false;
        last_stamp_ = {};
        previous_rate_rad_s_ = 0;
        filtered_rad_s2_ = 0;
    }
    void update(float rate_rad_s, const core::Stamp &stamp, float filter_tau_s, float limit_rad_s2) {
        if (!stamp.valid || !std::isfinite(rate_rad_s) || !(limit_rad_s2 > 0)) {
            reset();
            return;
        }
        const bool same_sample =
            initialized_ && stamp.time_us == last_stamp_.time_us && stamp.sequence == last_stamp_.sequence;
        if (same_sample)
            return;
        const bool epoch = !initialized_ || stamp.time_us <= last_stamp_.time_us ||
                           stamp.time_us - last_stamp_.time_us > kMaxGapUs;
        if (epoch) {
            initialized_ = true;
            last_stamp_ = stamp;
            previous_rate_rad_s_ = rate_rad_s;
            filtered_rad_s2_ = 0;
            valid_ = false;
            return;
        }
        const float dt = static_cast<float>(stamp.time_us - last_stamp_.time_us) * 1e-6f;
        const float slope = (rate_rad_s - previous_rate_rad_s_) / dt;
        const float alpha = filter_tau_s > 0 ? 1.0f - std::exp(-dt / filter_tau_s) : 1.0f;
        filtered_rad_s2_ += alpha * (slope - filtered_rad_s2_);
        filtered_rad_s2_ = std::clamp(filtered_rad_s2_, -limit_rad_s2, limit_rad_s2);
        previous_rate_rad_s_ = rate_rad_s;
        last_stamp_ = stamp;
        valid_ = true;
    }
    bool valid() const {
        return valid_;
    }
    float value_rad_s2() const {
        return filtered_rad_s2_;
    }

private:
    static constexpr core::TimeUs kMaxGapUs = 50000;
    bool initialized_ = false, valid_ = false;
    core::Stamp last_stamp_{};
    float previous_rate_rad_s_ = 0, filtered_rad_s2_ = 0;
};

// Single execution-thread owner. Fixed-size state, no allocation, no clocks:
// the caller supplies the monotonic time and its own calibrated geometry.
class DualYawMotionAllocator {
public:
    explicit DualYawMotionAllocator(const DualYawAllocatorConfig &config) : config_(config) {
    }
    int validate() const;
    // Drop every plan, filter state and the previous cycle time.
    void reset();
    DualYawAllocatorOutput update(const DualYawAllocatorInputs &, core::TimeUs now_us);
    const DualYawAllocatorConfig &config() const {
        return config_;
    }

private:
    void dropPlans();
    DualYawAllocatorConfig config_{};
    // Active intent stays independent of carrier-induced travel protection.
    BoundedRatePlan head_intent_{}, head_{}, big_{};
    core::TimeUs previous_us_ = 0;
    float slow_rad_s_ = 0, center_error_rad_ = 0;
    float head_angle_plan_rad_ = 0, joint_position_plan_rad_ = 0;
    std::uint32_t vision_epoch_ = 0;
    bool vision_seeded_ = false;
    float budget_scale_ = 1;
    bool tracking_limited_ = false, still_latched_ = true, initialized_ = false;
};

} // namespace skywalker::robotics
