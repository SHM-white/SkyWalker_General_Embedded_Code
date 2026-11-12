#include <robotics/execution/big_yaw_heading_follower.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
namespace skywalker::robotics {
namespace {
constexpr float pi = 3.14159265358979323846f;
float wrap(float a) {
    return std::remainder(a, 2 * pi);
}
} // namespace
int BigYawWorldHeadingFollower::validate() const {
    for (float x : {config_.heading_kp, config_.heading_rate_max_rad_s, config_.rate_kp, config_.rate_ki,
                    config_.rate_integral_max_rad_s, config_.output_max_rad_s, config_.release_epsilon_rad_s,
                    config_.max_sample_age_s, config_.max_dt_s, config_.world_rate_max_rad_s,
                    config_.shaping_acceleration_rad_s2, config_.shaping_jerk_rad_s3, config_.joint_rate_max_rad_s,
                    config_.stop_capture_dwell_s, config_.stop_capture_enter_rad_s, config_.stop_capture_exit_rad_s})
        if (!std::isfinite(x) || x < 0)
            return -EINVAL;
    return config_.heading_error_max_rad > 0 && config_.heading_error_max_rad <= pi && config_.output_max_rad_s > 0 &&
                   config_.rate_integral_max_rad_s > 0 && config_.max_dt_s > 0 && config_.max_sample_age_s >= 0 &&
                   config_.world_rate_max_rad_s > 0 && config_.joint_rate_max_rad_s > 0 &&
                   config_.stop_capture_exit_rad_s >= config_.stop_capture_enter_rad_s &&
                   config_.stop_capture_min_sequences >= 1
               ? 0
               : -EINVAL;
}
void BigYawWorldHeadingFollower::reset() {
    diagnostic_clamping_ = false;
    initialized_ = false;
    target_heading_rad_ = previous_requested_rate_rad_s_ = 0;
    rate_integral_rad_s_ = heading_error_rad_ = world_rate_rad_s_ = 0;
    last_peer_boot_id_ = 0;
    last_zero_generation_ = 0;
    world_plan_ = {};
    limit_flags_ = 0;
    stop_capture_flag_ = false;
    stop_capture_latched_ = false;
    stop_capture_start_ms_ = stop_capture_last_ms_ = 0;
    stop_capture_sequences_ = last_stop_sequence_ = 0;
    world_applied_rad_s_ = 0;
}
int BigYawWorldHeadingFollower::step(const BigYawWorldHeadingInputs &in, float &joint_rate_rad_s) {
    joint_rate_rad_s = 0;
    limit_flags_ = 0;
    if (validate() < 0)
        return -EINVAL;
    const auto &h = in.head;
    // The transport already clears `valid` for a stale session; repeat the
    // identity/age checks here so a direct caller cannot outlive them.
    if (!h.valid || !h.sender_boot_id || !h.zero_generation || !h.receiver_boot_id ||
        !forwardedFresh(h.stamp, h.source_age_ms, in.now_ms, 50) || !std::isfinite(h.yaw_rad) ||
        !std::isfinite(h.yaw_rate_rad_s) || !std::isfinite(in.body_rate_rad_s) ||
        !std::isfinite(in.desired_world_rate_rad_s) || !std::isfinite(in.dt_s) || in.dt_s < 0) {
        reset();
        return -ESTALE;
    }
    const auto age_ms = in.now_ms > h.stamp.timestamp_ms ? in.now_ms - h.stamp.timestamp_ms : 0;
    const float age = std::min(static_cast<float>(age_ms + h.source_age_ms) * .001f, config_.max_sample_age_s);
    // Predict only by the real sample age; never integrate an old heading twice.
    const float heading = wrap(h.yaw_rad + h.yaw_rate_rad_s * age);
    if (!initialized_ || h.sender_boot_id != last_peer_boot_id_ || h.zero_generation != last_zero_generation_) {
        // Enable, gimbal-boot and software-zero changes recapture the measured
        // heading instead of rotating towards a value from the previous run.
        reset();
        initialized_ = true;
        last_peer_boot_id_ = h.sender_boot_id;
        last_zero_generation_ = h.zero_generation;
        target_heading_rad_ = heading;
        world_applied_rad_s_ = 0;
        ++diagnostics_.capture_count;
        diagnostics_.capture_reason = 1;
        diagnostics_.capture_ms = in.now_ms;
        return 0;
    }
    const float requested = in.desired_world_rate_rad_s;
    // A released stick captures the current heading; the operator must not
    // watch the joint chase a target that lagged behind the measurement.
    if (config_.stop_capture_dwell_s > 0) {
        // Dwell form: only a sustained low-speed intent carried by new request
        // frames counts, and the capture happens once per stop. The latch holds
        // until the request leaves the zero band or the session/reference changes.
        const float magnitude = std::fabs(requested);
        if (magnitude > config_.stop_capture_exit_rad_s) {
            stop_capture_flag_ = false;
            stop_capture_latched_ = false;
        }
        else if (stop_capture_latched_)
            limit_flags_ |= 16;
        else if (magnitude <= config_.stop_capture_enter_rad_s) {
            if (!stop_capture_flag_) {
                stop_capture_flag_ = true;
                stop_capture_start_ms_ = in.now_ms;
                stop_capture_sequences_ = 0;
                last_stop_sequence_ = 0;
            }
            stop_capture_last_ms_ = in.now_ms;
            if (in.request_sequence && in.request_sequence != last_stop_sequence_) {
                last_stop_sequence_ = in.request_sequence;
                ++stop_capture_sequences_;
            }
            const auto dwell_ms = static_cast<std::uint64_t>(config_.stop_capture_dwell_s * 1000.0f);
            if (in.now_ms >= stop_capture_start_ms_ + dwell_ms &&
                stop_capture_sequences_ >= config_.stop_capture_min_sequences) {
                target_heading_rad_ = heading;
                rate_integral_rad_s_ = 0;
                world_applied_rad_s_ = 0;
                ++diagnostics_.capture_count;
                diagnostics_.capture_reason = 3;
                diagnostics_.capture_ms = in.now_ms;
                stop_capture_flag_ = false;
                stop_capture_latched_ = true;
            }
            else
                limit_flags_ |= 16;
        }
    }
    else if (std::fabs(requested) <= config_.release_epsilon_rad_s &&
             std::fabs(previous_requested_rate_rad_s_) > config_.release_epsilon_rad_s) {
        target_heading_rad_ = heading;
        rate_integral_rad_s_ = 0;
        world_applied_rad_s_ = 0;
        ++diagnostics_.capture_count;
        diagnostics_.capture_reason = 2;
        diagnostics_.capture_ms = in.now_ms;
    }
    previous_requested_rate_rad_s_ = requested;
    const float dt = std::min(in.dt_s, config_.max_dt_s);
    // Advance the heading target by the world rate the actuator actually got:
    // the shuttle, the joint capability and the native slew are already inside
    // that value, so a saturated axis cannot accumulate a target debt.
    const float applied_world_rate = in.achieved_rate_valid && std::isfinite(in.achieved_joint_rate_rad_s)
                                         ? in.achieved_joint_rate_rad_s + in.body_rate_rad_s
                                         : world_applied_rad_s_;
    target_heading_rad_ = wrap(target_heading_rad_ + applied_world_rate * dt);
    float error = wrap(target_heading_rad_ - heading);
    const bool clamping = std::fabs(error) > config_.heading_error_max_rad;
    if (clamping && !diagnostic_clamping_)
        ++diagnostics_.clamp_count;
    diagnostic_clamping_ = clamping;
    if (clamping && std::fabs(error) > std::fabs(heading_error_rad_)) {
        // Saturated and the new advance deepens the error: drop that advance and
        // keep the already clamped lead instead of accumulating a heading debt.
        error = std::clamp(heading_error_rad_, -config_.heading_error_max_rad, config_.heading_error_max_rad);
    }
    // Keep the target at most 90 degrees ahead so a following failure cannot
    // flip the correction sign after the error wraps through pi.
    error = std::clamp(error, -config_.heading_error_max_rad, config_.heading_error_max_rad);
    target_heading_rad_ = wrap(heading + error);

    // --- Total world motion: intent + heading correction ---------------------
    float world_raw = requested + std::clamp(config_.heading_kp * error, -config_.heading_rate_max_rad_s,
                                             config_.heading_rate_max_rad_s);
    const float world_bounded = std::clamp(world_raw, -config_.world_rate_max_rad_s, config_.world_rate_max_rad_s);
    if (world_bounded != world_raw) {
        world_raw = world_bounded;
        limit_flags_ |= 1;
    }
    if (config_.shaping_acceleration_rad_s2 > 0) {
        if (dt > 0) {
            const BoundedRateLimits limits{-config_.world_rate_max_rad_s, config_.world_rate_max_rad_s,
                                           -config_.shaping_acceleration_rad_s2, config_.shaping_acceleration_rad_s2,
                                           config_.shaping_jerk_rad_s3};
            const float target_acceleration = std::clamp((world_raw - world_plan_.rate_rad_s) / dt,
                                                         -config_.shaping_acceleration_rad_s2,
                                                         config_.shaping_acceleration_rad_s2);
            bool limited = false, conflict = false;
            if (stepBoundedRate(world_plan_, target_acceleration, dt, limits, limited, conflict) &&
                (limited || conflict))
                limit_flags_ |= 2;
        }
    }
    else {
        world_plan_.rate_rad_s = world_raw;
        world_plan_.acceleration_rad_s2 = 0;
    }
    const float world = world_plan_.rate_rad_s;
    const float rate_error = world - h.yaw_rate_rad_s;
    // Counter the measured chassis rotation so the joint actually holds world.
    const float feedforward = world - in.body_rate_rad_s;
    const float candidate_integral = std::clamp(rate_integral_rad_s_ + config_.rate_ki * rate_error * dt,
                                                -config_.rate_integral_max_rad_s, config_.rate_integral_max_rad_s);
    const float candidate = feedforward + config_.rate_kp * rate_error + candidate_integral;
    // Achievable world window around the measured body rate. Joint speed is the
    // hard capability; the trajectory smoothness above must not override it.
    const float joint_capacity = std::min(config_.output_max_rad_s, config_.joint_rate_max_rad_s);
    // Anti-windup includes the feedforward: a saturated base rate must not keep
    // accumulating while the error points further into saturation.
    if (std::fabs(candidate) <= joint_capacity || (candidate > joint_capacity && rate_error < 0) ||
        (candidate < -joint_capacity && rate_error > 0))
        rate_integral_rad_s_ = candidate_integral;
    const float unclamped_joint = feedforward + config_.rate_kp * rate_error + rate_integral_rad_s_;
    const float world_lo = in.body_rate_rad_s - joint_capacity, world_hi = in.body_rate_rad_s + joint_capacity;
    const float world_capped = std::clamp(unclamped_joint + in.body_rate_rad_s, world_lo, world_hi);
    if (world_capped != unclamped_joint + in.body_rate_rad_s)
        limit_flags_ |= 4;
    // The world window already contains the joint capability, so this final
    // clamp only protects against a rounding excursion.
    joint_rate_rad_s = std::clamp(world_capped - in.body_rate_rad_s, -joint_capacity, joint_capacity);
    // Remember what this cycle asked for; the next cycle integrates it unless the
    // caller supplies the executor's staged joint rate.
    world_applied_rad_s_ = world_capped;
    heading_error_rad_ = error;
    world_rate_rad_s_ = world;
    return 0;
}
} // namespace skywalker::robotics
