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
        // A continuous world-rate trajectory legitimately passes through zero at
        // a reversal, or while the sustained feedforward and the centring term
        // cancel. Capturing the heading at that instant would drop the
        // accumulated target and leave the head behind the real intent, so the
        // request must stay inside the epsilon band for this long before it
        // counts as a release. Zero keeps the legacy first-zero-cycle capture.
        float stop_capture_dwell_s = 0;
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
    struct Diagnostics {
        std::uint32_t capture_count = 0, clamp_count = 0;
        std::uint64_t capture_ms = 0;
        // 1: initialize/session capture; 2: incoming world-rate request returned to zero.
        std::uint8_t capture_reason = 0;
    };
    const Diagnostics &diagnostics() const {
        return diagnostics_;
    }

private:
    Config config_{};
    bool initialized_ = false;
    float target_heading_rad_ = 0, previous_requested_rate_rad_s_ = 0;
    float rate_integral_rad_s_ = 0, heading_error_rad_ = 0, world_rate_rad_s_ = 0;
    // Time the request has spent inside the zero band, and whether the release
    // capture for the current stop has already been taken. A true initial value
    // keeps the legacy behavior of never capturing a stop that never started.
    float zero_request_s_ = 0;
    bool stop_captured_ = true;
    std::uint64_t last_peer_boot_id_ = 0;
    std::uint32_t last_zero_generation_ = 0;
    // Retained across control resets to expose otherwise invisible recaptures.
    Diagnostics diagnostics_{};
    bool diagnostic_clamping_ = false;
};
} // namespace skywalker::robotics
