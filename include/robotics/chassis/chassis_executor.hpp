#pragma once

#include <robotics/chassis/chassis_power_limiter.hpp>
#include <robotics/chassis/power_measurement.hpp>
#include <robotics/chassis/swerve_hardware.hpp>
#include <robotics/execution/recovery_gate.hpp>
#include <robotics/messages/referee.hpp>
#include <robotics/swerve/swerve_chassis.hpp>

namespace skywalker::robotics {

struct ChassisExecutionInputs {
    ChassisCommand command{};
    core::Stamp source_stamp{};
    OutputPermission permission{};
    RefereePowerState power_budget{};
    PowerMeasurement measured_power{};
    bool require_permission = false, transport_ready = false;
    // Local samples use a local source; cross-board applications must compare
    // the request's target boot and chassis generation before setting this.
    bool recovery_context_valid = true;
    bool emergency_stop = false, clear_fault = false;
};

class ChassisExecutor {
public:
    struct Config {
        RecoveryGate::Config recovery{};
        core::TimeUs max_cycle_us = 20000, measurement_timeout_us = 300000;
        std::uint32_t permission_timeout_ms = 300, fault_retry_ms = 100;
        bool require_power_budget = false;
        bool power_model_calibrated = false, allow_estimated_power = false;
        bool power_control_calibrated = false;
        float estimate_idle_power_w = 0, estimate_w_per_abs_amp = 0;
        // Independent bench scales; explicit ampere ceilings replace the old
        // blanket 15% rule while power_control_calibrated remains false.
        float bench_effort_scale = 0.15f, bench_steer_effort_scale = 0.15f;
        float bench_drive_current_limit_a = 0.3f, bench_steer_current_limit_a = 0.3f;
        ChassisPowerLimiter::Config limiter{};
    };
    ChassisExecutor(SwerveHardware &hardware, const SwerveChassis::Config &chassis, const Config &config)
        : hardware_(hardware), chassis_(chassis), config_(config), recovery_(config.recovery), limiter_(config.limiter) {}
    int begin();
    RunStatus update(const ChassisExecutionInputs &inputs, core::TimeUs now_us);
    RunStatus suspend(core::TimeUs now_us, WaitReason reason, int error = 0, bool blocked = false);
    const RunStatus &status() const { return status_; }
    const ChassisOutput &output() const { return output_; }
    const ChassisFeedback &feedback() const { return feedback_; }
    float effortScale() const { return effort_scale_; }
    float steerEffortScale() const { return steer_effort_scale_; }
    const PowerMeasurement &powerMeasurement() const { return selected_power_; }
private:
    void withdraw(WaitReason reason, int error = 0, bool blocked = false);
    RunStatus publish(core::TimeUs now_us, RunState state, WaitReason reason, int error = 0);
    int power(const ChassisExecutionInputs &inputs, core::TimeUs now_us, float dt_s);
    SwerveHardware &hardware_;
    SwerveChassis chassis_;
    Config config_;
    RecoveryGate recovery_;
    ChassisPowerLimiter limiter_;
    RunStatus status_{};
    ChassisOutput output_{};
    ChassisFeedback feedback_{};
    PowerMeasurement selected_power_{};
    core::TimeUs previous_us_ = 0;
    std::uint64_t retry_ms_ = 0;
    std::uint32_t production_sequence_ = 0;
    float effort_scale_ = 0, steer_effort_scale_ = 0;
    int config_error_ = 0, hard_error_ = 0;
    WaitReason hard_reason_ = WaitReason::Drive;
    bool begin_attempted_ = false, configured_ = false, have_time_ = false, emergency_latched_ = false;
    bool explicit_clear_pending_ = false;
};

} // namespace skywalker::robotics
