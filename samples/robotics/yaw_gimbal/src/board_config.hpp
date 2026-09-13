#pragma once
#include <zephyr/device.h>
#include <robotics/gimbal/yaw_gimbal.hpp>
namespace bench {
inline const device *motor=DEVICE_DT_GET(DT_ALIAS(motor0));
// Limited topology requires calibrated DriverContinuous coordinates instead.
inline constexpr skywalker::robotics::YawGimbalConfig yaw{skywalker::robotics::YawTopology::Continuous,-3.14159265f,3.14159265f,0.5f,true};
inline skywalker::control::PositionMotor::Config motorConfig() {
    skywalker::control::PositionMotor::Config c{}; c.effort_unit=skywalker::control::EffortUnit::Ampere; c.safety={12,70,30,100};
    c.reference=yaw.topology==skywalker::robotics::YawTopology::Continuous ? skywalker::control::PositionReference::AbsoluteNearest : skywalker::control::PositionReference::DriverContinuous;
    c.loop.position={3,0,0,0,-3,3,-6,6,.01f,.001f,.02f};
    c.loop.velocity.regulator.feedback={.03f,.1f,0,0,-.3f,.3f,-.3f,.3f,0,.001f,.02f};
    c.loop.velocity.reference_slew={5,5}; c.loop.velocity.requested_velocity_abs_max_rad_s=6; c.loop.velocity.effort_abs_max=.3f; return c;
}
}
