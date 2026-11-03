#pragma once
namespace skywalker::robotics::vehicle {
// 参考底盘（JJL0612/sentry_onlychassis）的链路确认位。
// 这些常量描述“对端”的能力与实板状态，不参与本板电机使能；
// 在实板验证前必须保持 false，不能因为本板执行器可用就提前置 true。
inline constexpr bool reference_chassis_link_enabled = true;
// 对端已实现 0x0405/40 字节、0x0301 OperatorControl 接收并真正执行授权门控。
inline constexpr bool reference_chassis_imu_heading_confirmed = false;
// 对端大 Yaw 处于原生速度模式，mode=2 的目标角速度语义已逐项核对。
inline constexpr bool reference_chassis_native_velocity_confirmed = false;
// 未改动的参考底盘固件只接受它私有的 0x0404/32B 航向帧和 BigYawRequest mode=1，
// 且它的自稳后端把 mode=1 的 target_rate 当作“头部世界角速度”。
// 打开后：额外发送 0x0404；mode=1 的 target 是世界角速度意图。
// 整车手控入口仍可发送操作员转速；双 Yaw 上车入口使用过去 2.5s
// 小 Yaw 世界航向均值的误差速度。不得混用机械关节相对速度。
inline constexpr bool reference_chassis_legacy_v4_compat = true;
// 线上发送 BigYawMode::ImuHeadingHold 的唯一决策点。
// mode=1 仍只表示大 Yaw 关节相对速度；mode=2 才表示头部世界角速度。
// 兼容模式与 mode=2 互斥：对端不可能同时只认 mode=1 又支持 mode=2。
inline constexpr bool reference_chassis_imu_heading_authorized =
    reference_chassis_link_enabled && !reference_chassis_legacy_v4_compat &&
    reference_chassis_imu_heading_confirmed && reference_chassis_native_velocity_confirmed;
} // namespace skywalker::robotics::vehicle
