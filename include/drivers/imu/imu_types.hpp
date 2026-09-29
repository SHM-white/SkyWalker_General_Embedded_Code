#pragma once
#include <core/measurement.hpp>
namespace skywalker::imu {
enum Field : std::uint32_t { Accel = 1u << 0, Gyro = 1u << 1, Orientation = 1u << 2, Temperature = 1u << 3 };
enum class State : std::uint8_t { Uninitialized, Running, Fault };
enum class AttitudeQuality : std::uint8_t { Unavailable, Unknown, Initializing, Tracking, Degraded };
struct Freshness {
    core::TimeUs accel_us = 20000, gyro_us = 20000, orientation_us = 20000, temperature_us = 1000000;
};
struct Sample {
    core::Measurement<core::Vec3> accel_m_s2{}, gyro_rad_s{};
    core::Measurement<core::Quaternion> orientation{};
    core::Measurement<float> temperature_c{};
    core::OrientationReference reference{};
    AttitudeQuality attitude_quality = AttitudeQuality::Unavailable;
};
struct Diagnostics {
    int last_error = 0;
    std::uint32_t io_errors = 0, invalid_updates = 0, transport_gaps = 0;
};
struct Snapshot {
    Sample sample{};
    State state = State::Uninitialized;
    std::uint32_t capabilities = 0, fresh_mask = 0;
    Diagnostics diagnostics{};
};
}
