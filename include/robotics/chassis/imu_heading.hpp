#pragma once
#include <core/attitude.hpp>
#include <drivers/imu/imu_types.hpp>
#include <robotics/messages/interboard.hpp>
namespace skywalker::robotics {
enum class HeadingWaitReason : std::uint8_t {
    None,
    SourceNotRunning,
    OrientationStale,
    GyroStale,
    ReferenceInvalid,
    OrientationInvalid,
    GyroInvalid,
    Degraded,
    QualityUnavailable,
    ReferenceReset,
    TiltSingular,
    RateInvalid,
    Initializing,
    SampleGap,
    ControlStale
};
struct LocalHeading {
    float yaw_rad = 0, rate_rad_s = 0;
    core::Stamp stamp{};
    std::uint32_t generation = 0;
    bool valid = false;
    HeadingWaitReason reason = HeadingWaitReason::Initializing;
};
// Read the acquisition-thread planar yaw. Does not consult EKF attitude quality,
// orientation reference, or accelerometer freshness after calibration.
LocalHeading planarYawHeading(const imu::Snapshot &, core::TimeUs now);
// A software zero belongs to one board startup. Transient losses preserve it;
// a driver reference reset invalidates it until both boards are realigned/rebooted.
class ImuHeadingReference {
public:
    struct Config {
        // Opt-in for a source which keeps publishing fresh gyro predictions
        // while accelerometer correction is rejected. Zero rejects Degraded.
        core::TimeUs max_degraded_prediction_us = 0;
    };
    ImuHeadingReference() = default;
    explicit ImuHeadingReference(const Config &config) : config_(config) {
    }
    LocalHeading update(const imu::Snapshot &, bool disarmed, core::TimeUs now);

private:
    const Config config_{};
    core::OrientationReference reference_{};
    core::Stamp last_tracking_stamp_{};
    std::uint64_t last_sequence_ = 0;
    core::TimeUs last_sample_us_ = 0;
    float zero_rad_ = 0;
    unsigned stationary_samples_ = 0;
    bool initialized_ = false, blocked_ = false, have_reference_ = false;
    HeadingWaitReason blocked_reason_ = HeadingWaitReason::ReferenceReset;
};
// Compare boot-relative uptimes before pairing a new chassis zero with a peer.
// This rejects a chassis-only restart that has lost its previous peer binding.
bool startupSessionsMatch(const BoardHeartbeat &, std::uint64_t now_ms, std::uint32_t max_skew_ms);
class ChassisMotionAdapter {
public:
    enum class Frame { Body, World, Gimbal };
    struct Config {
        Frame frame = Frame::Body;
        bool compensation_enabled = false;
        bool startup_alignment_confirmed = false;
        std::uint32_t startup_max_skew_ms = 1000;
        float heading_prediction_s = .007f;
        float spin_forward_scale = .3f, spin_lateral_scale = .4f;
        float yaw_rate_feedback_kp = 0; // Unmeasured: leave disabled.
        float max_wz_rad_s = 1;
    };
    explicit ChassisMotionAdapter(const Config &c) : config_(c) {
    }
    int validate() const;
    ChassisCommand update(const ChassisCommand &, const LocalHeading &, const GimbalImuYaw &, std::uint64_t local_boot,
                          bool disarmed, std::uint64_t now_ms, const BoardHeartbeat &peer = {});
    bool translationReady() const {
        return translation_ready_;
    }

private:
    Config config_;
    bool paired_ = false, blocked_ = false, translation_ready_ = false;
    std::uint64_t peer_boot_ = 0, local_boot_ = 0;
    std::uint32_t head_generation_ = 0, body_generation_ = 0;
};
}
