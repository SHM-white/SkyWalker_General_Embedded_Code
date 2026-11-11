#pragma once
#include <cstdint>
#include <robotics/messages/interboard.hpp>

namespace skywalker::robotics {
// Chassis-board world-heading follower for the native-velocity big-Yaw backend.
//
// The wire FollowCenter rate is the *world* yaw rate intended for the head.
// This class turns it into a physical *joint* rate using the gimbal board's
// measured heading/rate and the chassis gyro feedforward. The executor still
// applies direction and transmission ratio exactly once.
//
// Physics: +X forward, +Y left, positive rotation counter-clockwise seen from
// above; all rates are rad/s at the big-Yaw joint.
struct BigYawWorldHeadingInputs {
    // Gimbal-board heading sample (0x0404). Sender identity is bound by the
    // transport to the accepted heartbeat session.
    GimbalImuYaw head{};
    // Local monotonic receive time, used only for sample-age prediction.
    std::uint64_t now_ms = 0;
    // Chassis BMI088 yaw rate, counter-clockwise positive.
    float body_rate_rad_s = 0;
    // Wire FollowCenter world-rate intent.
    float desired_world_rate_rad_s = 0;
    // Measured control-cycle interval; zero, repeated or over-long intervals
    // never integrate.
    float dt_s = 0;
};

class BigYawWorldHeadingFollower {
public:
    struct Config {
        // Heading P: correction above the operator world-rate intent.
        float heading_kp = 2.0f, heading_rate_max_rad_s = 1.5f;
        // The captured target never leads the measurement by more than this.
        float heading_error_max_rad = 1.5707963f;
        // Rate PI around the measured head rate.
        float rate_kp = 0.25f, rate_ki = 0.5f;
        // Bounds shared with the saturation check; output_max is the joint
        // limit already enforced by BigYawExecutor.
        float rate_integral_max_rad_s = 1.5f, output_max_rad_s = 10.0f;
        // Operator-release zero detection; avoids recapturing on float noise.
        float release_epsilon_rad_s = 1e-4f;
        float max_sample_age_s = 0.05f, max_dt_s = 0.02f;
    };
    BigYawWorldHeadingFollower() = default;
    explicit BigYawWorldHeadingFollower(const Config &config) : config_(config) {
    }
    int validate() const;
    // Clear capture and integral. Call whenever the request leaves
    // FollowCenter, a required sample is lost, or the peer session changes.
    void reset();
    // Returns 0 and a joint rate in rad/s, or a negative errno with the output
    // zeroed. The caller keeps its existing suspend/disable semantics.
    int step(const BigYawWorldHeadingInputs &, float &joint_rate_rad_s);

    bool initialized() const {
        return initialized_;
    }
    float targetHeadingRad() const {
        return target_heading_rad_;
    }
    float headingErrorRad() const {
        return heading_error_rad_;
    }
    float worldRateRadS() const {
        return world_rate_rad_s_;
    }
    float rateIntegralRadS() const {
        return rate_integral_rad_s_;
    }

private:
    Config config_{};
    bool initialized_ = false;
    float target_heading_rad_ = 0, previous_requested_rate_rad_s_ = 0;
    float rate_integral_rad_s_ = 0, heading_error_rad_ = 0, world_rate_rad_s_ = 0;
    std::uint64_t last_peer_boot_id_ = 0;
    std::uint32_t last_zero_generation_ = 0;
};
} // namespace skywalker::robotics
