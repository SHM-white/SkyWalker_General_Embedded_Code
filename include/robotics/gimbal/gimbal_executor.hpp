#pragma once

#include <core/measurement.hpp>
#include <drivers/motor/group.hpp>
#include <robotics/execution/run_status.hpp>
#include <robotics/command/command_source.hpp>
#include <robotics/execution/run_status.hpp>
#include <robotics/gimbal/gimbal_axis.hpp>

namespace skywalker::robotics {

struct GimbalExecutionInputs {
    struct DirectAxisCommand {
        bool valid = false;
        bool limit_clamped = false;
        control::PositionMotor::ExternalVelocityInput input{};
    };

    GimbalCommand command{};
    // Optional small-Yaw joint trajectory in encoder coordinates, produced by the
    // dual-Yaw allocator. When valid and fresh it owns the Yaw target for this
    // cycle; the pitch command keeps its normal path. Position is enforced by the
    // existing position loop, rate/acceleration are carried for diagnostics and
    // the stage-C velocity/effort feedforward.
    GimbalJointTrajectory yaw_trajectory{};
    DirectAxisCommand yaw_direct{};
    DirectAxisCommand pitch_direct{};
    // Source production time, preserved independently of arbitration time.
    core::Stamp source_stamp{};
    OutputPermission permission{};
    bool require_permission = false;
    bool transport_ready = false;
    bool yaw_output_valid = true, pitch_output_valid = true;
    // 视觉 Rate 桥接：只有本周期该轴确实由有效的 Active Rate 命令驱动时才置位。
    // 默认关闭；手动、Hold、Disabled 和有效 yaw_trajectory 路径保持原有行为。
    bool yaw_use_rate_reference = false;
    bool pitch_use_rate_reference = false;
    // 本周期重建该轴 Rate 会话（来源切换/恢复）：轴层丢弃旧目标并从测量播种，
    // 本周期不积分也不前馈。
    bool yaw_restart_rate_session = false;
    bool pitch_restart_rate_session = false;
    bool emergency_stop = false;
    bool clear_estop = false;
};

// One execution-thread owner. The application owns topology, attach/start and
// exactly one commit per physical bus after every mechanism stages its output.
// Both motors and their two-axis batch Group must outlive this object.
// This executor never owns, starts or commits a CAN controller.
class GimbalExecutor {
public:
    struct Config {
        std::uint32_t command_timeout_ms = 100;
        core::TimeUs source_timeout_us = 100000;
        std::uint32_t permission_timeout_ms = 300;
        core::TimeUs max_cycle_us = 20000;
    };

    GimbalExecutor(motor::Motor &yaw_drive, motor::Motor &pitch_drive, motor::Group &group,
                   const control::PositionMotor::Config &yaw_motor, const GimbalAxisConfig &yaw_axis,
                   const control::PositionMotor::Config &pitch_motor, const GimbalAxisConfig &pitch_axis,
                   const Config &config);
    // After every application-owned bus starts; configures axes without enabling.
    [[nodiscard]] int begin();
    RunStatus update(const GimbalExecutionInputs &inputs, core::TimeUs now_us);
    // Explicit input withdrawal (stop, emergency, expired input).
    RunStatus suspend(core::TimeUs now_us, WaitReason reason, int error = 0, bool blocked = false);
    double yawTargetRad() const {
        return yaw_.targetAngleRad();
    }
    double pitchTargetRad() const {
        return pitch_.targetAngleRad();
    }
    control::PositionMotor::Telemetry yawTelemetry() const {
        return yaw_.telemetry();
    }
    control::PositionMotor::Telemetry pitchTelemetry() const {
        return pitch_.telemetry();
    }
    // Drop the accumulated Yaw lead and reseed it from the current feedback.
    int reseedYawTarget() {
        return yaw_.reseedTargetFromFeedback();
    }
    // 扫描入口显式重播种 Pitch：捕获当前实际关节角作为新的保持目标。
    // 只重建控制目标，不重新定义电机编码器零点。执行线程调用；
    // 返回值沿用 GimbalAxis：-EACCES 未配置、-EAGAIN 反馈暂不可用、
    // -ENODATA 缺少参考、-EINVAL 非法测量、0 成功。
    int reseedPitchTarget() {
        return pitch_.reseedTargetFromFeedback();
    }
    const GimbalAxis::TargetStatus &yawTargetDiagnostics() const {
        return yaw_.targetStatus();
    }
    const GimbalAxis::TargetStatus &pitchTargetDiagnostics() const {
        return pitch_.targetStatus();
    } // Execution-thread only.
    const RunStatus &status() const {
        return status_;
    } // Execution-thread only.

private:
    RunStatus publish(core::TimeUs now_us, RunState state, WaitReason reason, int error = 0);
    void withdraw(WaitReason reason, int error = 0, bool blocked = false);
    motor::Motor &yaw_drive_, &pitch_drive_;
    motor::Group &group_;
    GimbalAxis yaw_, pitch_;
    const Config config_;
    RunStatus status_{};
    core::TimeUs previous_us_ = 0;
    std::uint32_t production_sequence_ = 0;
    int config_error_ = 0;
    bool configured_ = false, begin_attempted_ = false, have_time_ = false;
    // 重启事件须保留到对应 Rate 轴实际输出；提前返回和另一轴成功均不消费事件。
    bool yaw_rate_restart_pending_ = false, pitch_rate_restart_pending_ = false;
};

} // namespace skywalker::robotics
