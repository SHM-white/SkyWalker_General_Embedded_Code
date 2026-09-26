#pragma once
#include <robotics/messages/common.hpp>
namespace skywalker::robotics {
struct GlobalSafetyInputs {
    // 本次全局安全判定使用的系统时间，单位 ms。
    std::uint64_t now_ms = 0;
    // 操作命令来源是否仍在有效期内。
    bool command_source_fresh = false;
    // 操作员是否允许机构运动。
    bool operator_motion_enabled = false;
    // 是否收到紧急停止请求。
    bool emergency_stop_requested = false;
    // 云台输出许可。
    OutputPermission gimbal_power{};
    // 底盘输出许可。
    OutputPermission chassis_power{};
    // 发射机构输出许可。
    OutputPermission shooter_power{};
    // 最近一次底盘板心跳的时间戳与序号。
    MessageStamp chassis_heartbeat_stamp{};
    // 最近一次底盘反馈的时间戳与序号。
    MessageStamp chassis_feedback_stamp{};
    // 底盘当前执行状态。
    ExecutionState chassis_execution_state = ExecutionState::Waiting;
    // 底盘当前生效的 SafetyReason 位掩码。
    std::uint32_t chassis_active_reasons = 0;
};
struct GlobalSafetyDecision {
    // 对云台要求的安全动作。
    SafetyAction gimbal = SafetyAction::Disable;
    // 对底盘要求的安全动作。
    SafetyAction chassis = SafetyAction::Disable;
    // 对发射机构要求的安全动作。
    SafetyAction shooter = SafetyAction::Disable;
    // 全局安全状态。
    SafetyState state = SafetyState::Waiting;
    // 本次判定生效的 SafetyReason 位掩码。
    std::uint32_t active_reasons = 0;
    // 本次安全判定的时间戳与序号。
    MessageStamp stamp{};
};
struct LocalSafetyInputs {
    // 本次本地安全判定使用的系统时间，单位 ms。
    std::uint64_t now_ms = 0;
    // 本板当前这次启动的编号。
    std::uint64_t receiver_boot_id = 0;
    // 收到的命令所指定的接收板启动编号。
    std::uint64_t command_boot_id = 0;
    // 本板当前恢复运行上下文的代次。
    std::uint32_t resume_generation = 0;
    // 收到的命令所指定的恢复代次。
    std::uint32_t command_generation = 0;
    // 对端板最近一次心跳的时间戳与序号。
    MessageStamp peer_heartbeat_stamp{};
    // 最近一次板间控制命令的时间戳与序号。
    MessageStamp command_stamp{};
    // 全局安全管理器要求本板执行的动作。
    SafetyAction global_action = SafetyAction::Disable;
    // 对应机构的电源输出是否获准开启。
    bool power_allowed = false;
    // 执行机构反馈是否仍在有效期内。
    bool feedback_fresh = false;
    // 硬件是否已完成运行准备。
    bool hardware_ready = false;
    // 执行机构是否已使能。
    bool armed = false;
    // 本地硬件与控制配置是否有效。
    bool config_valid = true;
    // 是否收到紧急停止请求。
    bool emergency_stop_requested = false;
};
struct LocalSafetyDecision {
    // 本地安全管理器要求执行的动作。
    SafetyAction action = SafetyAction::Disable;
    // 本地执行状态。
    ExecutionState state = ExecutionState::Waiting;
    // 本次判定生效的 SafetyReason 位掩码。
    std::uint32_t active_reasons = 0;
};
}
