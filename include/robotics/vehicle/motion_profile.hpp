#pragma once
#include <robotics/chassis/imu_heading.hpp>
namespace skywalker::robotics::vehicle {
// Bench-only opt-ins. Confirm common forward startup pose and both IMU mounting transforms.
inline constexpr bool startup_heading_alignment_confirmed = false;
inline constexpr bool big_yaw_heading_lock_enabled = false;
inline constexpr bool big_yaw_mit_mode_confirmed = false;
inline ChassisMotionAdapter::Config chassisMotionConfig() {
    ChassisMotionAdapter::Config c{};
    c.frame=ChassisMotionAdapter::Frame::Body;
    c.compensation_enabled=false;
    c.startup_alignment_confirmed=startup_heading_alignment_confirmed;
    return c;
}
}
