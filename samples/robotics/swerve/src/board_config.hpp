#pragma once
#include <zephyr/device.h>
#include <robotics/swerve/swerve_module.hpp>
namespace bench {
inline const device *steer=DEVICE_DT_GET(DT_ALIAS(steer_motor));
inline const device *drive=DEVICE_DT_GET(DT_ALIAS(drive_motor));
inline constexpr float steer_direction=1,drive_direction=1;
inline constexpr float test_speed_m_s=0.1f;
inline skywalker::robotics::SwerveModule::Config moduleConfig() {
    skywalker::robotics::SwerveModule::Config c{}; c.wheel_radius_m=0.05f;
    c.drive.regulator.feedback={0.03f,0.1f,0,0,-0.3f,0.3f,-0.3f,0.3f,0,.001f,.02f};
    c.drive.reference_slew={10,10}; c.drive.requested_velocity_abs_max_rad_s=10; c.drive.effort_abs_max=.3f;
    c.steer.velocity=c.drive; c.steer.position={3,0,0,0,-10,10,-10,10,0.01f,.001f,.02f}; return c;
}
}
