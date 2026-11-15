#pragma once

#include <robotics/messages/command.hpp>

namespace skywalker::robotics {

// 巡航扫描命令生成器：只产生头部世界偏航速率目标。
//
// 契约（第一版刻意保持极简）：
//   * 调用前配置已由 CommandArbiter::validateConfig() 验证，这里不返回错误码。
//   * 不积分 360° 绝对角，不自己驱动大 Yaw，不重复实现加速度整形：这些全部
//     交给现有 DualYawMotionAllocator 与执行器。
//   * Pitch 速率恒为零，表示“保持进入扫描时重播种的关节角”。
//   * 源序号与命令时间戳由实际调用它的仲裁周期产生（LocalBehavior 生产戳）；
//     本函数无状态、无 I/O、可供任意线程上下文调用。
inline GimbalCommand makeCruiseScanCommand(float head_yaw_rate_rad_s) {
    GimbalCommand command{};
    command.mode = GimbalMode::Rate;
    command.source = ControlSource::LocalBehavior;
    command.yaw_rate_rad_s = head_yaw_rate_rad_s;
    command.pitch_rate_rad_s = 0.0f;
    return command;
}

} // namespace skywalker::robotics
