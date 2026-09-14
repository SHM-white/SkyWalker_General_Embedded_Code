#pragma once
#include <robotics/messages/common.hpp>
namespace skywalker::robotics {
enum class ChassisMode : std::uint8_t { Disabled, BodyVelocity, Spin };
enum class GimbalMode : std::uint8_t { Disabled, Hold, Rate, AbsoluteAngle };
enum class ShooterMode : std::uint8_t { Disabled, Ready, FireSingle, FireContinuous };
struct ChassisCommand {
    ChassisMode mode = ChassisMode::Disabled;
    float vx_m_s = 0, vy_m_s = 0, wz_rad_s = 0;
    ControlSource source = ControlSource::None;
    MessageStamp stamp{};
};
struct GimbalCommand {
    GimbalMode mode = GimbalMode::Disabled;
    float yaw_target_rad = 0, yaw_rate_rad_s = 0, pitch_target_rad = 0, pitch_rate_rad_s = 0;
    ControlSource source = ControlSource::None;
    MessageStamp stamp{};
};
struct ShooterCommand {
    ShooterMode mode = ShooterMode::Disabled;
    float fire_rate_hz = 0, requested_bullet_speed_m_s = 0;
    ControlSource source = ControlSource::None;
    MessageStamp stamp{};
};
struct RobotCommand {
    ChassisCommand chassis{};
    GimbalCommand gimbal{};
    ShooterCommand shooter{};
    MessageStamp stamp{};
};
}
