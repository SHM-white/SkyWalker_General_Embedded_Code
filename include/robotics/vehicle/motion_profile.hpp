#pragma once
#include <robotics/chassis/imu_heading.hpp>
#include <robotics/vehicle/calibration.hpp>
namespace skywalker::robotics::vehicle {
// Bench-only opt-ins. Confirm common forward startup pose and both IMU mounting transforms.
inline constexpr bool startup_heading_alignment_confirmed = false;
// One compile-time backend choice. The wire mode is always a world-rate intent;
// this enum only selects which local motor backend executes it.
enum class BigYawBackend { NativeVelocity, MitVelocity, MitHeading };
inline constexpr auto big_yaw_backend = BigYawBackend::NativeVelocity;
inline constexpr bool big_yaw_native_velocity_profile_confirmed = big_yaw_backend == BigYawBackend::NativeVelocity;
inline constexpr bool big_yaw_mit_mode_confirmed = big_yaw_backend == BigYawBackend::MitVelocity ||
                                                   big_yaw_backend == BigYawBackend::MitHeading;
inline constexpr bool big_yaw_heading_lock_enabled = big_yaw_backend == BigYawBackend::MitHeading;
inline constexpr ChassisMotionAdapter::Config chassisMotionConfig() {
    ChassisMotionAdapter::Config c{};
    c.frame = ChassisMotionAdapter::Frame::Body;
    c.compensation_enabled = false;
    c.startup_alignment_confirmed = startup_heading_alignment_confirmed;
    c.spin_forward_scale = chassis_spin_forward_scale;
    c.spin_lateral_scale = chassis_spin_lateral_scale;
    c.max_wz_rad_s = chassis_max_wz_rad_s;
    return c;
}
static_assert(chassisMotionConfig().max_wz_rad_s > 0);
}
