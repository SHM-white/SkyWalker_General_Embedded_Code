#pragma once
#include <core/measurement.hpp>
#include <limits>
namespace skywalker::imu {
enum Field : std::uint32_t {
    Accel = 1u << 0,
    Gyro = 1u << 1,
    Orientation = 1u << 2,
    Temperature = 1u << 3,
    PlanarYaw = 1u << 4
};
enum class State : std::uint8_t { Uninitialized, Running, Fault };
enum class AttitudeQuality : std::uint8_t { Unavailable, Unknown, Initializing, Tracking, Degraded };
struct Freshness {
    core::TimeUs accel_us = 20000, gyro_us = 20000, orientation_us = 20000, temperature_us = 1000000;
};
enum class PlanarYawState : std::uint8_t {
    Unavailable,
    Calibrating,
    Ready,
    GyroInvalid,
    SampleGap,
    TimeReset,
    RateInvalid
};
struct PlanarYawSample {
    core::Measurement<float> yaw_rad{};
    float rate_rad_s = 0, bias_rad_s = 0;
    std::uint32_t generation = 0;
    PlanarYawState state = PlanarYawState::Unavailable;
};
struct Sample {
    core::Measurement<core::Vec3> accel_m_s2{}, gyro_rad_s{};
    core::Measurement<core::Quaternion> orientation{};
    core::Measurement<float> temperature_c{};
    core::OrientationReference reference{};
    AttitudeQuality attitude_quality = AttitudeQuality::Unavailable;
    // Independent gyro-derived reference; EKF resetOrientation must not reset it.
    PlanarYawSample planar_yaw{};
};
struct EstimatorDiagnostics {
    core::TimeUs time_us = 0, last_tracking_us = 0;
    int result = 0;
    // QuaternionEkf::Correction numeric value; 0 means not evaluated.
    std::uint8_t correction = 0;
    float gravity_error_m_s2 = std::numeric_limits<float>::quiet_NaN();
    float innovation_nis = std::numeric_limits<float>::quiet_NaN();
};
struct Diagnostics {
    int last_error = 0;
    std::uint32_t io_errors = 0, invalid_updates = 0, transport_gaps = 0;
    EstimatorDiagnostics estimator{};
};
struct Snapshot {
    Sample sample{};
    State state = State::Uninitialized;
    std::uint32_t capabilities = 0, fresh_mask = 0;
    Diagnostics diagnostics{};
};
}
