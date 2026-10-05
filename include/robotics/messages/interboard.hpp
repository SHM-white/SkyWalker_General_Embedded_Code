#pragma once
#include <robotics/messages/command.hpp>
#include <robotics/messages/feedback.hpp>
namespace skywalker::robotics {
enum class BoardRole : std::uint8_t { Unknown, GimbalController, ChassisController };
struct BoardHeartbeat {
    // 发送方在系统中的板卡角色。
    BoardRole role = BoardRole::Unknown;
    // 发送方当前的安全状态。
    SafetyState safety_state = SafetyState::Waiting;
    // 发送方当前生效的 SafetyReason 位掩码。
    std::uint32_t active_reasons = 0;
    // 发送方从启动至今的运行时间，单位 ms。
    std::uint32_t sender_uptime_ms = 0;
    // 标识发送方当前这次启动的编号。
    std::uint64_t sender_boot_id = 0;
    // 发送方是否已准备就绪。
    bool ready = false;
    // 发送方是否请求同步运行状态。
    bool sync_requested = false;
    // 心跳消息的时间戳与序号。
    MessageStamp stamp{};
};
struct RemoteChassisControl {
    // 转发给底盘板的底盘运动命令。
    ChassisCommand command{};
    // 全局安全管理器对底盘要求的动作。
    SafetyAction global_action = SafetyAction::Disable;
    // 全局安全管理器当前生效的 SafetyReason 位掩码。
    std::uint32_t active_reasons = 0;
    // 目标接收板本次启动的编号，用于拒绝旧命令。
    std::uint64_t receiver_boot_id = 0;
    // 板间底盘控制消息的时间戳与序号。
    MessageStamp stamp{};
};
struct ChassisConstraint {
    // 裁判系统给出的底盘输出许可。
    OutputPermission output{};
    // 功率上限与缓冲能量是否有效。
    bool power_valid = false;
    // 底盘功率上限，单位 W。
    float power_limit_w = 0;
    // 底盘缓冲能量，单位 J。
    float buffer_energy_j = 0;
    // 发送时输出许可已存在的时间，单位 ms。
    std::uint32_t output_age_ms = 0;
    // 发送时功率约束数据已存在的时间，单位 ms。
    std::uint32_t power_age_ms = 0;
    // 板间底盘约束消息的时间戳与序号。
    MessageStamp stamp{};
};
// One management producer owns the original ages and peer binding. Repeated
// publication/transmission must preserve the original RC and clear-event age.
struct OperatorControl {
    bool run_allowed = false;
    bool emergency_stop = false;
    std::uint32_t source_sequence = 0;
    std::uint32_t source_age_ms = UINT32_MAX;
    std::uint32_t clear_event_id = 0;
    std::uint32_t clear_event_age_ms = UINT32_MAX;
    std::uint64_t sender_boot_id = 0;   // Filled by the sending endpoint.
    std::uint64_t receiver_boot_id = 0; // Bound when produced, never on retry.
    // TX: local production time; RX: local receive time and wire sequence.
    MessageStamp stamp{};
};
enum class BigYawMode : std::uint8_t { Disabled, FollowCenter };
struct BigYawRequest {
    BigYawMode mode = BigYawMode::Disabled;
    float target_rate_rad_s = 0;
    // Original producer identity/age, never renewed by communication retries.
    std::uint32_t source_sequence = 0;
    std::uint32_t source_age_ms = 0;
    std::uint32_t command_age_ms = 0;
    std::uint64_t receiver_boot_id = 0;
    OutputPermission permission{};
    std::uint32_t permission_age_ms = 0;
    MessageStamp stamp{};
};
struct BigYawFeedback {
    ExecutionState execution_state = ExecutionState::Waiting;
    bool ready = false;
    bool armed = false;
    bool valid = false;
    float actual_rate_rad_s = 0;
    std::uint32_t active_reasons = 0;
    std::uint32_t last_command_sequence = 0;
    std::uint32_t source_sequence = 0;
    std::uint32_t production_age_ms = 0;
    // Produced by the execution owner; RX decoding stores local receive time.
    MessageStamp stamp{};
};
inline bool forwardedFresh(const MessageStamp &s, std::uint32_t age_ms, std::uint64_t now_ms,
                           std::uint32_t timeout_ms) {
    return isFresh(s, now_ms, timeout_ms) && age_ms <= timeout_ms && now_ms - s.timestamp_ms <= timeout_ms - age_ms;
}
}
