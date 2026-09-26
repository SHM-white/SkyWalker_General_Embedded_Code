#pragma once
#include <robotics/messages/common.hpp>
namespace skywalker::robotics {
enum class ChassisMode : std::uint8_t { Disabled, BodyVelocity, Spin };
enum class GimbalMode : std::uint8_t { Disabled, Hold, Rate, AbsoluteAngle };
enum class ShooterMode : std::uint8_t { Disabled, Ready, FireSingle, FireContinuous };
struct ChassisCommand {
    // 底盘控制模式。
    ChassisMode mode = ChassisMode::Disabled;
    // 底盘前后速度目标，单位 m/s。
    float vx_m_s = 0;
    // 底盘左右速度目标，单位 m/s。
    float vy_m_s = 0;
    // 底盘偏航角速度目标，单位 rad/s。
    float wz_rad_s = 0;
    // 命令的输入来源。
    ControlSource source = ControlSource::None;
    // 命令的时间戳与序号。
    MessageStamp stamp{};
};
struct GimbalCommand {
    // 云台控制模式。
    GimbalMode mode = GimbalMode::Disabled;
    // 云台偏航角目标，单位 rad。
    float yaw_target_rad = 0;
    // 云台偏航角速度目标，单位 rad/s。
    float yaw_rate_rad_s = 0;
    // 云台俯仰角目标，单位 rad。
    float pitch_target_rad = 0;
    // 云台俯仰角速度目标，单位 rad/s。
    float pitch_rate_rad_s = 0;
    // 命令的输入来源。
    ControlSource source = ControlSource::None;
    // 命令的时间戳与序号。
    MessageStamp stamp{};
};
struct ShooterCommand {
    // 发射机构控制模式。
    ShooterMode mode = ShooterMode::Disabled;
    // 发射频率目标，单位 Hz。
    float fire_rate_hz = 0;
    // 请求的弹丸速度，单位 m/s。
    float requested_bullet_speed_m_s = 0;
    // 命令的输入来源。
    ControlSource source = ControlSource::None;
    // 命令的时间戳与序号。
    MessageStamp stamp{};
};
struct RobotCommand {
    // 底盘命令。
    ChassisCommand chassis{};
    // 云台命令。
    GimbalCommand gimbal{};
    // 发射机构命令。
    ShooterCommand shooter{};
    // 整组机器人命令的时间戳与序号。
    MessageStamp stamp{};
};
}
