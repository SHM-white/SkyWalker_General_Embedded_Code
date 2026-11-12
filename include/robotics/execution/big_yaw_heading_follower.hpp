#pragma once
#include <cstdint>
#include <robotics/execution/bounded_rate_step.hpp>
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
    // Wire request sequence of this cycle. Only distinct sequences count as new
    // frames for the stop-capture dwell; a cached payload does not.
    std::uint32_t request_sequence = 0;
    // Previous cycle's staged physical joint rate (direction, ratio and native
    // slew already applied). When valid, the heading target advances by what the
    // actuator actually received instead of the raw request.
    bool achieved_rate_valid = false;
    float achieved_joint_rate_rad_s = 0;
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
        // World-motion shaping of the total intent (operator rate + heading
        // correction). shaping_acceleration <= 0 disables shaping.
        float world_rate_max_rad_s = 10.0f;
        float shaping_acceleration_rad_s2 = 0, shaping_jerk_rad_s3 = 0;
        // Achievable world rate around the measured body rate, i.e. the joint
        // speed capability. The executor still applies sign, ratio and native
        // slew after this stage.
        float joint_rate_max_rad_s = 10.0f;
        // Stop capture: a dwell of fresh, distinct low-speed requests captures
        // the measured heading once. Zero keeps the immediate zero-crossing
        // capture above.
        float stop_capture_dwell_s = 0;
        float stop_capture_enter_rad_s = 0.05f, stop_capture_exit_rad_s = 0.1f;
        std::uint32_t stop_capture_min_sequences = 2;
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
    float shapedWorldRateRadS() const {
        return world_plan_.rate_rad_s;
    }
    // 1 world-speed clamp, 2 world acceleration shaped, 4 joint capacity clamp,
    // 8 joint rate clamp, 16 stop-capture candidate active.
    std::uint32_t limitFlags() const {
        return limit_flags_;
    }
    float stopCaptureElapsedMs() const {
        return stop_capture_flag_ ? static_cast<float>(stop_capture_last_ms_ - stop_capture_start_ms_) : 0.0f;
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
    std::uint64_t last_peer_boot_id_ = 0;
    std::uint32_t last_zero_generation_ = 0;
    // Retained across control resets to expose otherwise invisible recaptures.
    Diagnostics diagnostics_{};
    bool diagnostic_clamping_ = false;
    BoundedRatePlan world_plan_{};
    std::uint32_t limit_flags_ = 0;
    bool stop_capture_flag_ = false, stop_capture_latched_ = false;
    std::uint64_t stop_capture_start_ms_ = 0, stop_capture_last_ms_ = 0;
    std::uint32_t stop_capture_sequences_ = 0, last_stop_sequence_ = 0;
    float world_applied_rad_s_ = 0;
};
} // namespace skywalker::robotics
