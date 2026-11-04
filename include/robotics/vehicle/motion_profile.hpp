#pragma once
#include <robotics/chassis/imu_heading.hpp>
namespace skywalker::robotics::vehicle {
// Bench-only opt-ins. Confirm common forward startup pose and both IMU mounting transforms.
inline constexpr bool startup_heading_alignment_confirmed = false;
// 只选择本仓库本地大 Yaw 执行器（MIT 力矩后端）。它不再是线上模式开关：
// Reference v4 wire mode=1 always denotes desired world yaw rate.
// No peer-specific wire mode switch is required.
inline constexpr bool big_yaw_heading_lock_enabled = false;
inline constexpr bool big_yaw_mit_mode_confirmed = false;
// Confirm actual CTRL_MODE=3, CAN ID / feedback ID and motor direction on the
// installed hardware before enabling. This selects a DIFFERENT motor backend.
inline constexpr bool big_yaw_native_velocity_profile_confirmed = false;
static_assert(!(big_yaw_native_velocity_profile_confirmed && big_yaw_heading_lock_enabled),
              "Native velocity and MIT heading torque cannot control the same motor");
inline ChassisMotionAdapter::Config chassisMotionConfig() {
    ChassisMotionAdapter::Config c{};
    c.frame=ChassisMotionAdapter::Frame::Body;
    c.compensation_enabled=false;
    c.startup_alignment_confirmed=startup_heading_alignment_confirmed;
    return c;
}
}
