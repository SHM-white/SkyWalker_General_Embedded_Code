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
// 线上发送 BigYawMode::ImuHeadingHold 的唯一决策点。
// mode=1 仍只表示大 Yaw 关节相对速度；mode=2 才表示头部世界角速度。
inline constexpr bool reference_chassis_imu_heading_authorized =
    reference_chassis_link_enabled && reference_chassis_imu_heading_confirmed &&
    reference_chassis_native_velocity_confirmed;
} // namespace skywalker::robotics::vehicle
