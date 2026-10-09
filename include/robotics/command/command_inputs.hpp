#pragma once
#include <communication/vision/vision_types.hpp>
#include <robotics/messages/command.hpp>
#include <robotics/messages/referee.hpp>
#include <robotics/messages/remote.hpp>
#include <robotics/command/mouse_shooter_gesture.hpp>

namespace skywalker::robotics {

struct CommandInputs {
    core::TimeUs now_us = 0; // Arbitration time, independent of source stamps.
    RemoteState remote{};
    core::Measurement<communication::vision::AimCommand> vision{};
    RefereeState referee{};
    // 外部执行线程提供已解锁/无故障状态；未启用外部门控的应用维持 true。
    bool run_allowed = true;
};

enum ArbitrationReason : std::uint32_t {
    RcUnavailable = 1u << 0,
    SafeRequested = 1u << 1,
    VisionMissing = 1u << 2,
    VisionStale = 1u << 3,
    VisionStopped = 1u << 4,
    VisionReferenceMismatch = 1u << 5,
    WaitNewVision = 1u << 6,
    InvalidVision = 1u << 7,
    PermissionMissing = 1u << 8,
    PermissionStale = 1u << 9,
    PermissionDenied = 1u << 10,
    ValueLimited = 1u << 11,
    ShooterNotArmed = 1u << 12,
    AimNotControlling = 1u << 13,
    InvalidInputs = 1u << 14,
    InvalidManagerConfig = 1u << 15,
    ClockRegression = 1u << 16,
    ManualOverride = 1u << 17,
    OverrideQuiet = 1u << 18,
    AutoUnavailable = 1u << 19,
    OperatorGateDenied = 1u << 20,
};

struct CommandDecision {
    int error = 0;
    OperatorMode operator_mode = OperatorMode::Safe;
    bool manual_override = false;
    bool override_quiet = false;
    RobotCommand requested{}; // 源选择后的候选，尚未按裁判许可裁剪。
    RobotCommand command{};   // 本次最终命令；telemetry 主通道只使用它。
    std::uint32_t chassis_reasons = 0, gimbal_reasons = 0, shooter_reasons = 0;
    // 最终采用视觉目标时保留完整目标及原始时间；Hold/Disabled 时清空。
    core::Measurement<communication::vision::AimCommand> selected_vision{};
    MouseFireIntent mouse_fire{};
    std::uint32_t reasons() const {
        return chassis_reasons | gimbal_reasons | shooter_reasons;
    }
};
}
