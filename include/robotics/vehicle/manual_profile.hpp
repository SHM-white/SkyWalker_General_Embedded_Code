#pragma once

#include <robotics/command/command_manager.hpp>
#include <robotics/gimbal/inertial_gimbal.hpp>
#include <robotics/gimbal/yaw_centering.hpp>
#include <robotics/vehicle/gimbal_profile.hpp>

namespace skywalker::robotics::vehicle {
inline constexpr std::uint32_t manual_command_timeout_ms = gimbal_command_timeout_ms;
// Existing peer adapter convention, retained with protocol v4.
inline constexpr float big_yaw_wire_direction = -1.0f;
inline CommandManager::Config manualCommandConfig() {
    auto c = gimbalCommandConfig();
    c.require_external_run_gate = true;
    c.max_chassis_vx_m_s = c.max_chassis_vy_m_s = 0.3f;
    c.max_chassis_wz_rad_s = 0.5f;
    return c;
}
inline InertialGimbalAdapter::Config manualInertialConfig() {
    InertialGimbalAdapter::Config c{};
    c.yaw = yawAxisConfig();
    c.pitch = pitchAxisConfig();
    c.yaw_direction = small_yaw.direction;
    c.pitch_direction = pitch.direction;
    c.pitch_locked = false;
    c.allow_unknown_quality = true;
    return c;
}
inline YawCenteringController::Config manualCenteringConfig() {
    YawCenteringController::Config c{};
    c.center_rad = yaw_center_rad;
    c.follow_direction = small_yaw.direction;
    c.max_rate_rad_s = 0.6f;
    c.acceleration_rad_s2 = 0.8f;
    return c;
}
} // namespace skywalker::robotics::vehicle
