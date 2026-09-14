#pragma once
#include <robotics/messages/common.hpp>
namespace skywalker::robotics {
enum class RcSwitch : std::uint8_t { Unknown, Up, Middle, Down };
struct RemoteAnalog {
    std::int16_t right_x = 0, right_y = 0, left_x = 0, left_y = 0, wheel = 0;
};
struct RemoteMouse {
    std::int16_t x = 0, y = 0, z = 0;
    bool left = false, right = false;
};
struct RemoteKeyboard {
    std::uint16_t bits = 0;
};
struct RemoteState {
    RemoteAnalog analog{};
    RcSwitch left_switch = RcSwitch::Unknown, right_switch = RcSwitch::Unknown;
    RemoteMouse mouse{};
    RemoteKeyboard keyboard{};
    MessageStamp stamp{};
    bool online = false;
};
enum class OperatorMode : std::uint8_t { Safe, Manual, Auto };
struct OperatorIntent {
    OperatorMode mode = OperatorMode::Safe;
    ControlSource source = ControlSource::None;
    float chassis_vx_norm = 0, chassis_vy_norm = 0, chassis_wz_norm = 0;
    float gimbal_yaw_rate_norm = 0, gimbal_pitch_rate_norm = 0;
    bool friction_requested = false, fire_requested = false;
    MessageStamp stamp{};
};
}
