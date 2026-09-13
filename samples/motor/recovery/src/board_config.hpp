#pragma once
#include <zephyr/device.h>
#include <control/velocity_motor.hpp>
namespace bench {
inline const device *motor=DEVICE_DT_GET(DT_ALIAS(motor0));
inline constexpr bool dm=DT_NODE_HAS_COMPAT(DT_ALIAS(motor0),dm_j4310_2ec_v1_1);
inline constexpr float target_velocity_rad_s=2;
inline skywalker::control::VelocityMotor::Config motorConfig() {
    skywalker::control::VelocityMotor::Config c{};
    c.effort_unit=dm ? skywalker::control::EffortUnit::NewtonMeter : skywalker::control::EffortUnit::Ampere;
    c.safety={12,70,30,100};
    c.loop.regulator.feedback={.03f,.1f,0,0,-.3f,.3f,-.3f,.3f,0,.001f,.02f};
    c.loop.reference_slew={5,5}; c.loop.requested_velocity_abs_max_rad_s=6; c.loop.effort_abs_max=.3f;
    return c;
}
}
