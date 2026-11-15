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
    // DR16 透传水平原始量；默认按速度映射，未确认是鼠标硬件 counts。
    std::int16_t x = 0;
    // DR16 透传垂直原始量；单位/PC 缩放须现场确认。
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
enum class OperatorMode : std::uint8_t { Safe, Manual, Auto, Navigation };
// 自主档（左 Middle）的子模式：右拨杆选择导航、巡航扫描或巡航+小陀螺。
// 巡航在旧样例中保持关闭，此时右拨杆不改变子模式。
enum class AutonomousMode : std::uint8_t { None, Navigation, Cruise, CruiseSpin };
inline bool isCruise(AutonomousMode mode) {
    return mode == AutonomousMode::Cruise || mode == AutonomousMode::CruiseSpin;
}
// 执行线程已经接受并授权的模式选择。valid=false 时由原始遥控映射决定模式，
// 兼容不提供外部选择的旧调用方。
struct OperatorSelection {
    bool valid = false;
    OperatorMode mode = OperatorMode::Safe;
    AutonomousMode autonomous = AutonomousMode::None;
    // 最后一帧已接受遥控输入；掉线保持期间保持旧戳，不伪造在线。
    MessageStamp accepted_remote_stamp{};
};
// 只在两拨杆已知合法时调用：左 Middle 且启用巡航时，右拨杆选择巡航子模式。
inline AutonomousMode selectAutonomousMode(const RemoteState &remote, bool cruise_enabled) {
    if (remote.left_switch != RcSwitch::Middle)
        return AutonomousMode::None;
    if (!cruise_enabled || remote.right_switch == RcSwitch::Middle)
        return AutonomousMode::Navigation;
    if (remote.right_switch == RcSwitch::Up)
        return AutonomousMode::Cruise;
    if (remote.right_switch == RcSwitch::Down)
        return AutonomousMode::CruiseSpin;
    return AutonomousMode::None;
}
struct OperatorIntent {
    // 操作员请求的安全、手动或自动模式。
    OperatorMode mode = OperatorMode::Safe;
    // 主模式为 Navigation/Auto 时进一步说明自主行为（巡航两档走本地扫描）。
    AutonomousMode autonomous = AutonomousMode::None;
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
    // 独立灵敏度计算的鼠标角速度，避免机构限速改变未饱和区灵敏度。
    float mouse_yaw_rate_rad_s = 0, mouse_pitch_rate_rad_s = 0;
    // 台架合同：置位时执行层直接用上面的 rad/s，不再乘机构限速。
    bool mouse_rate_mapping = false;
    // 是否请求启动摩擦轮。
    bool friction_requested = false;
    // 是否请求发射。
    bool fire_requested = false;
    // 本次操作意图的时间戳与序号。
    MessageStamp stamp{};
};
}
