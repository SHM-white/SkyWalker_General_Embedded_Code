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
    for (float x :
         {config_.heading_kp, config_.heading_rate_max_rad_s, config_.rate_kp, config_.rate_ki,
          config_.rate_integral_max_rad_s, config_.output_max_rad_s, config_.release_epsilon_rad_s,
          config_.stop_capture_dwell_s, config_.hold_deadband_rad, config_.max_sample_age_s, config_.max_dt_s})
        if (!std::isfinite(x) || x < 0)
            return -EINVAL;
    return config_.heading_error_max_rad > 0 && config_.heading_error_max_rad <= pi && config_.output_max_rad_s > 0 &&
                   config_.rate_integral_max_rad_s > 0 && config_.max_dt_s > 0 && config_.max_sample_age_s >= 0 &&
                   config_.stop_capture_dwell_s <= 1.0f && config_.hold_deadband_rad <= pi
               ? 0
               : -EINVAL;
}
void BigYawWorldHeadingFollower::reset() {
    diagnostic_clamping_ = false;
    initialized_ = false;
    target_heading_rad_ = previous_requested_rate_rad_s_ = 0;
    rate_integral_rad_s_ = heading_error_rad_ = world_rate_rad_s_ = 0;
    zero_request_s_ = 0;
    stop_captured_ = true;
    last_peer_boot_id_ = 0;
    last_zero_generation_ = 0;
}
int BigYawWorldHeadingFollower::step(const BigYawWorldHeadingInputs &in, float &joint_rate_rad_s) {
    joint_rate_rad_s = 0;
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
        ++diagnostics_.capture_count;
        diagnostics_.capture_reason = 1;
        diagnostics_.capture_ms = in.now_ms;
        return 0;
    }
    const float requested = in.desired_world_rate_rad_s;
    const float dt = std::min(in.dt_s, config_.max_dt_s);
    // Releasing the stick captures the current heading; the operator must not
    // watch the joint chase a target that lagged behind the measurement. A
    // continuous trajectory crosses zero as well, so the zero must persist for
    // `stop_capture_dwell_s` before it is treated as a release.
    const bool zero_request = std::fabs(requested) <= config_.release_epsilon_rad_s;
    zero_request_s_ = zero_request ? zero_request_s_ + dt : 0;
    if (zero_request && !stop_captured_ && zero_request_s_ >= config_.stop_capture_dwell_s) {
        stop_captured_ = true;
        target_heading_rad_ = heading;
        rate_integral_rad_s_ = 0;
        ++diagnostics_.capture_count;
        diagnostics_.capture_reason = 2;
        diagnostics_.capture_ms = in.now_ms;
    }
    if (!zero_request)
        stop_captured_ = false;
    previous_requested_rate_rad_s_ = requested;
    float error = wrap(target_heading_rad_ + requested * dt - heading);
    // Stationary hold: the world-heading loop must not chase sub-degree noise,
    // otherwise the big Yaw keeps micro-correcting and the small Yaw follows.
    // Re-anchoring the target on the measurement keeps the loop transparent
    // inside the band while the body-rate feedforward still holds world.
    if (config_.hold_deadband_rad > 0 && zero_request && std::fabs(error) <= config_.hold_deadband_rad) {
        error = 0;
        target_heading_rad_ = heading;
        rate_integral_rad_s_ = 0;
    }
    const bool clamping = std::fabs(error) > config_.heading_error_max_rad;
    if (clamping && !diagnostic_clamping_)
        ++diagnostics_.clamp_count;
    diagnostic_clamping_ = clamping;
    error = std::clamp(error, -config_.heading_error_max_rad, config_.heading_error_max_rad);
    // Keep the target at most 90 degrees ahead so a following failure cannot
    // flip the correction sign after the error wraps through pi.
    target_heading_rad_ = wrap(heading + error);
    const float world_rate = requested + std::clamp(config_.heading_kp * error, -config_.heading_rate_max_rad_s,
                                                    config_.heading_rate_max_rad_s);
    const float rate_error = world_rate - h.yaw_rate_rad_s;
    // Counter the measured chassis rotation so the joint actually holds world.
    const float feedforward = world_rate - in.body_rate_rad_s;
    const float candidate_integral = std::clamp(rate_integral_rad_s_ + config_.rate_ki * rate_error * dt,
                                                -config_.rate_integral_max_rad_s, config_.rate_integral_max_rad_s);
    const float candidate = feedforward + config_.rate_kp * rate_error + candidate_integral;
    // Anti-windup includes the feedforward: a saturated base rate must not keep
    // accumulating while the error points further into saturation.
    if (std::fabs(candidate) <= config_.output_max_rad_s || (candidate > config_.output_max_rad_s && rate_error < 0) ||
        (candidate < -config_.output_max_rad_s && rate_error > 0))
        rate_integral_rad_s_ = candidate_integral;
    heading_error_rad_ = error;
    world_rate_rad_s_ = world_rate;
    joint_rate_rad_s = std::clamp(feedforward + config_.rate_kp * rate_error + rate_integral_rad_s_,
                                  -config_.output_max_rad_s, config_.output_max_rad_s);
    return 0;
}
} // namespace skywalker::robotics
