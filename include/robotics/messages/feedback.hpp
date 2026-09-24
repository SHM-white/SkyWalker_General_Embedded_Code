#pragma once
#include <robotics/messages/common.hpp>
namespace skywalker::robotics {
struct ChassisFeedbackSummary {
    // 底盘执行状态。
    ExecutionState execution_state = ExecutionState::Waiting;
    // 底盘安全状态。
    SafetyState safety_state = SafetyState::Waiting;
    // 底盘是否已满足进入运行状态的条件。
    bool ready = false;
    // 底盘是否已经使能。
    bool armed = false;
    // 当前生效的 SafetyReason 位掩码。
    std::uint32_t active_reasons = 0;
    // 底盘最近处理的命令序号。
    std::uint32_t last_command_sequence = 0;
    // 反馈字段有效性位掩码。
    std::uint32_t valid_fields = 0;
    // 底盘前后速度反馈，单位 m/s。
    float vx_m_s = 0;
    // 底盘左右速度反馈，单位 m/s。
    float vy_m_s = 0;
    // 底盘偏航角速度反馈，单位 rad/s。
    float wz_rad_s = 0;
    // 底盘功率反馈，单位 W。
    float power_w = 0;
    // 本次反馈的时间戳与序号。
    MessageStamp stamp{};
};
}
