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
                    config_.max_sample_age_s, config_.max_dt_s})
        if (!std::isfinite(x) || x < 0)
            return -EINVAL;
    return config_.heading_error_max_rad > 0 && config_.heading_error_max_rad <= pi && config_.output_max_rad_s > 0 &&
                   config_.rate_integral_max_rad_s > 0 && config_.max_dt_s > 0 && config_.max_sample_age_s >= 0
               ? 0
               : -EINVAL;
}
void BigYawWorldHeadingFollower::reset() {
    initialized_ = false;
    target_heading_rad_ = previous_requested_rate_rad_s_ = 0;
    rate_integral_rad_s_ = heading_error_rad_ = world_rate_rad_s_ = 0;
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
        return 0;
    }
    const float requested = in.desired_world_rate_rad_s;
    // Releasing the stick captures the current heading; the operator must not
    // watch the joint chase a target that lagged behind the measurement.
    if (std::fabs(requested) <= config_.release_epsilon_rad_s &&
        std::fabs(previous_requested_rate_rad_s_) > config_.release_epsilon_rad_s) {
        target_heading_rad_ = heading;
        rate_integral_rad_s_ = 0;
    }
    previous_requested_rate_rad_s_ = requested;
    const float dt = std::min(in.dt_s, config_.max_dt_s);
    float error = wrap(target_heading_rad_ + requested * dt - heading);
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
