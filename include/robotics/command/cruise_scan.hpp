#pragma once

#include <robotics/messages/command.hpp>

namespace skywalker::robotics {

// 巡航扫描命令生成器：产生头部世界偏航速率与 Pitch 关节速率目标。
//
// 契约：
//   * 调用前配置已由 CommandArbiter::validateConfig() 验证，这里不返回错误码。
//   * 不积分 360° 绝对角，不自己驱动大 Yaw，不重复实现加速度整形：这些全部
//     交给现有 DualYawMotionAllocator 与执行器。
//   * Pitch 速率由调用方的扫描状态机给出（正负决定上下方向）；为 0 表示保持
//     进入扫描时重播种的关节角。Rate 模式的目标积分与软限位由 GimbalAxis 负责。
//   * 源序号与命令时间戳由实际调用它的仲裁周期产生（LocalBehavior 生产戳）；
//     本函数无状态、无 I/O、可供任意线程上下文调用。
inline GimbalCommand makeCruiseScanCommand(float head_yaw_rate_rad_s, float pitch_joint_rate_rad_s) {
    GimbalCommand command{};
    command.mode = GimbalMode::Rate;
    command.source = ControlSource::LocalBehavior;
    command.yaw_rate_rad_s = head_yaw_rate_rad_s;
    command.pitch_rate_rad_s = pitch_joint_rate_rad_s;
    return command;
}

} // namespace skywalker::robotics
