#pragma once

#include <core/measurement.hpp>
#include <robotics/execution/run_status.hpp>

namespace skywalker::robotics {

// 【已作废 / deprecated】旧的大 Yaw 回中链路。
//
// 它只用小 Yaw 相对中心的偏角产生一个速度请求：偏差出现之前大 Yaw 不动，
// 偏差越大转得越快，且不包含持续运动前馈、载体实际跟踪、速度/加速度预算和
// 制动行程预测。正式云台（applications/sentry_gimbal）已改由
// robotics::DualYawMotionAllocator 主动分配，本类不再参与正式控制链路。
//
// 保留原因：历史 bringup 样例与 robotics_contracts 契约测试仍引用它，
// 便于对照旧行为。新代码不得再把它和分配器同时接到大 Yaw 上累加输出。
struct YawCenteringInputs {
    float joint_yaw_rad = 0;
    core::Stamp joint_stamp{}, source_stamp{};
    bool enabled = false, head_stable = false, permission_valid = false;
};

struct YawCenteringOutput {
    bool enabled = false;
    float velocity_rad_s = 0, center_error_rad = 0;
    MessageStamp stamp{};
    WaitReason reason = WaitReason::Command;
};

// Cloud/gimbal-board outer loop only. The independent chassis-board BigYaw
// execution context must still authorize boot/recovery generation and expiry.
class YawCenteringController {
public:
    struct Config {
        float center_rad = 0; // Explicit calibrated center, separate from encoder zero.
        float deadband_rad = 0.05f, hysteresis_rad = 0.025f;
        float kp_rad_s_per_rad = 0.5f, max_rate_rad_s = 0.3f;
        float acceleration_rad_s2 = 0.4f, follow_direction = 1;
        core::TimeUs feedback_timeout_us = 50000, source_timeout_us = 100000;
        core::TimeUs max_cycle_us = 20000;
    };
    explicit YawCenteringController(const Config &config) : config_(config) {
    }
    YawCenteringOutput update(const YawCenteringInputs &, core::TimeUs now_us);
    void reset();

private:
    Config config_;
    core::TimeUs previous_us_ = 0;
    float rate_rad_s_ = 0;
    std::uint32_t sequence_ = 0;
    bool following_ = false;
};

} // namespace skywalker::robotics
