#pragma once
#include <core/measurement.hpp>
namespace skywalker::communication::vision {
struct AxisTarget {
    float angle_rad = 0, rate_rad_s = 0, acceleration_rad_s2 = 0;
};
struct AimCommand {
    bool control_requested = false, fire_requested = false;
    core::OrientationReference reference{};
    AxisTarget yaw{}, pitch{};
};
enum class Mode : std::uint8_t { Idle, AutoAim, SmallBuff, BigBuff };
struct Feedback {
    Mode mode = Mode::Idle;
    core::OrientationReference reference{};
    core::Measurement<core::Quaternion> orientation{};
    core::Measurement<core::Vec3> gyro_rad_s{};
    core::Measurement<float> bullet_speed_m_s{};
    core::Measurement<std::uint16_t> bullet_count{};
};
struct ProtocolStatistics {
    std::uint32_t frames = 0, crc_errors = 0, invalid_frames = 0, assembly_timeouts = 0;
};
struct Snapshot {
    core::Measurement<AimCommand> aim{};
    bool aim_fresh = false;
    ProtocolStatistics protocol{};
    std::uint32_t rejected_commands = 0;
};
}
