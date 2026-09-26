#pragma once
#include <robotics/messages/common.hpp>
namespace skywalker::robotics {
enum class RcSwitch : std::uint8_t { Unknown, Up, Middle, Down };
struct RemoteAnalog {
    // 右摇杆水平通道的原始值。
    std::int16_t right_x = 0;
    // 右摇杆垂直通道的原始值。
    std::int16_t right_y = 0;
    // 左摇杆水平通道的原始值。
    std::int16_t left_x = 0;
    // 左摇杆垂直通道的原始值。
    std::int16_t left_y = 0;
    // 遥控器拨轮通道的原始值。
    std::int16_t wheel = 0;
};
struct RemoteMouse {
    // 鼠标水平位移。
    std::int16_t x = 0;
    // 鼠标垂直位移。
    std::int16_t y = 0;
    // 鼠标滚轮位移。
    std::int16_t z = 0;
    // 鼠标左键是否按下。
    bool left = false;
    // 鼠标右键是否按下。
    bool right = false;
};
struct RemoteKeyboard {
    // 键盘按键状态位掩码。
    std::uint16_t bits = 0;
};
struct RemoteState {
    // 摇杆和拨轮输入。
    RemoteAnalog analog{};
    // 左侧三档开关位置。
    RcSwitch left_switch = RcSwitch::Unknown;
    // 右侧三档开关位置。
    RcSwitch right_switch = RcSwitch::Unknown;
    // 鼠标输入。
    RemoteMouse mouse{};
    // 键盘输入。
    RemoteKeyboard keyboard{};
    // 最近一次遥控器消息的时间戳与序号。
    MessageStamp stamp{};
    // 遥控器数据当前是否在线且未超时。
    bool online = false;
};
enum class OperatorMode : std::uint8_t { Safe, Manual, Auto };
struct OperatorIntent {
    // 操作员请求的安全、手动或自动模式。
    OperatorMode mode = OperatorMode::Safe;
    // 本次操作意图的输入来源。
    ControlSource source = ControlSource::None;
    // 底盘前后速度的归一化请求，范围为 [-1, 1]。
    float chassis_vx_norm = 0;
    // 底盘左右速度的归一化请求，范围为 [-1, 1]。
    float chassis_vy_norm = 0;
    // 底盘偏航角速度的归一化请求，范围为 [-1, 1]。
    float chassis_wz_norm = 0;
    // 云台偏航角速度的归一化请求，范围为 [-1, 1]。
    float gimbal_yaw_rate_norm = 0;
    // 云台俯仰角速度的归一化请求，范围为 [-1, 1]。
    float gimbal_pitch_rate_norm = 0;
    // 是否请求启动摩擦轮。
    bool friction_requested = false;
    // 是否请求发射。
    bool fire_requested = false;
    // 本次操作意图的时间戳与序号。
    MessageStamp stamp{};
};
}
