#include <cerrno>
#include <core/clock.hpp>
#include <robotics/command/command_manager.hpp>
namespace skywalker::robotics {
BUILD_ASSERT(CONFIG_SKYWALKER_COMMAND_PRIORITY >= 0 &&
             CONFIG_SKYWALKER_COMMAND_PRIORITY < CONFIG_NUM_PREEMPT_PRIORITIES);
BUILD_ASSERT(CONFIG_SKYWALKER_COMMAND_PERIOD_MS > 0);
namespace {
using AimMeasurement = core::Measurement<communication::vision::AimCommand>;
SourceValue emptyValue(SourceRole role) {
    if (role == SourceRole::Aim)
        return AimMeasurement{};
    return RemoteState{};
}
bool matches(SourceRole role, const SourceValue &value) {
    return role == SourceRole::Operator ? std::holds_alternative<RemoteState>(value)
                                        : std::holds_alternative<AimMeasurement>(value);
}
}
CommandManager::CommandManager(const Config &config)
    : arbiter_(config), require_permissions_(config.require_referee_for_motion),
      permission_timeout_ms_(config.permission_timeout_ms),
      require_external_gate_(config.require_external_run_gate) {
}
void CommandManager::setOperatorGate(bool allowed, std::uint32_t session) {
    if (k_is_in_isr())
        return;
    const auto now = core::monotonicTimeUs() / 1000;
    const auto key = k_spin_lock(&operator_gate_lock_);
    if (operator_gate_.allowed != allowed || operator_gate_.session != session) {
        operator_gate_.allowed = allowed;
        operator_gate_.session = session;
        operator_gate_.boundary_ms = now;
        ++operator_gate_.revision;
    }
    k_spin_unlock(&operator_gate_lock_, key);
}
int CommandManager::registerSource(ICommandSource &source) {
    if (k_is_in_isr())
        return -EWOULDBLOCK;
    if (start_attempted_)
        return -EBUSY;
    const auto role = source.role();
    if (role != SourceRole::Operator && role != SourceRole::Aim)
        return -EINVAL;
    for (std::size_t i = 0; i < source_count_; ++i)
        if (sources_[i].source == &source || sources_[i].role == role)
            return -EEXIST;
    if (source_count_ == sources_.size())
        return -ENOSPC;
    auto &slot = sources_[source_count_++];
    slot.source = &source;
    slot.role = role;
    slot.cached.value = emptyValue(role);
    return 0;
}
int CommandManager::bindPermissions(IPermissionSource &source) {
    if (k_is_in_isr())
        return -EWOULDBLOCK;
    if (start_attempted_)
        return -EBUSY;
    if (permissions_)
        return -EEXIST;
    permissions_ = &source;
    return 0;
}
int CommandManager::failStart(int error, std::uint32_t reason) {
    CommandSnapshot failed{};
    failed.decision.error = error;
    failed.decision.chassis_reasons = failed.decision.gimbal_reasons = failed.decision.shooter_reasons = reason;
    for (std::size_t i = 0; i < source_count_; ++i) {
        const auto &slot = sources_[i];
        (slot.role == SourceRole::Operator ? failed.remote : failed.vision) = slot.cached.diagnostics;
    }
    failed.permission = permission_diagnostics_;
    publish(failed); // Disabled, invalid stamps; already-started sources stay alive.
    return error;
}
int CommandManager::start() {
    if (k_is_in_isr())
        return -EWOULDBLOCK;
    if (start_attempted_)
        return -EALREADY;
    start_attempted_ = true;
    if (arbiter_.configError() < 0)
        return failStart(arbiter_.configError(), InvalidManagerConfig);
    bool have_operator = false;
    for (std::size_t i = 0; i < source_count_; ++i)
        have_operator = have_operator || sources_[i].role == SourceRole::Operator;
    if (!have_operator)
        return failStart(-EINVAL, RcUnavailable);
    if (require_permissions_ && !permissions_)
        return failStart(-ENODEV, PermissionMissing);
    for (std::size_t i = 0; i < source_count_; ++i) {
        auto &slot = sources_[i];
        const int ret = slot.source->start();
        if (ret < 0) {
            slot.cached.diagnostics.error = ret;
            return failStart(ret, InvalidInputs);
        }
    }
    if (permissions_) {
        const int ret = permissions_->start();
        if (ret < 0) {
            permission_diagnostics_.error = ret;
            return failStart(ret, PermissionMissing);
        }
    }
    k_thread_create(&thread_, stack_, K_KERNEL_STACK_SIZEOF(stack_), entry, this, nullptr, nullptr,
                    CONFIG_SKYWALKER_COMMAND_PRIORITY, 0, K_NO_WAIT);
    return 0;
}
void CommandManager::publish(const CommandSnapshot &value) {
    const auto key = k_spin_lock(&output_lock_);
    output_ = value;
    have_output_ = true;
    k_spin_unlock(&output_lock_, key);
}
int CommandManager::current(RobotCommand &out) const {
    if (k_is_in_isr())
        return -EWOULDBLOCK;
    const auto key = k_spin_lock(&output_lock_);
    const int ret = have_output_ ? 0 : -EAGAIN;
    if (ret == 0)
        out = output_.decision.command;
    k_spin_unlock(&output_lock_, key);
    return ret;
}
int CommandManager::snapshot(CommandSnapshot &out) const {
    if (k_is_in_isr())
        return -EWOULDBLOCK;
    const auto key = k_spin_lock(&output_lock_);
    const int ret = have_output_ ? 0 : -EAGAIN;
    if (ret == 0)
        out = output_;
    k_spin_unlock(&output_lock_, key);
    return ret;
}
void CommandManager::entry(void *self, void *, void *) {
    static_cast<CommandManager *>(self)->run();
}
void CommandManager::run() {
    std::uint32_t gate_revision = 0;
    bool safe_gate_latched = false;
    for (;;) {
        CommandSnapshot next{};
        for (std::size_t i = 0; i < source_count_; ++i) {
            auto &slot = sources_[i];
            SourceSample sample{};
            int ret = slot.source->sample(sample);
            if (ret == 0 && !matches(slot.role, sample.value))
                ret = -EINVAL;
            if (ret == 0) {
                slot.cached = sample;
            }
            else if (ret == -EAGAIN) {
                slot.cached.diagnostics.sample_error = ret;
            }
            else {
                slot.cached.value = emptyValue(slot.role);
                slot.cached.diagnostics = sample.diagnostics;
                slot.cached.diagnostics.sample_error = ret;
            }
            if (slot.role == SourceRole::Operator) {
                next.observed.remote = *std::get_if<RemoteState>(&slot.cached.value);
                next.remote = slot.cached.diagnostics;
            }
            else {
                next.observed.vision = *std::get_if<AimMeasurement>(&slot.cached.value);
                next.vision = slot.cached.diagnostics;
            }
        }
        if (permissions_) {
            RefereeState candidate{};
            SourceDiagnostics diagnostics{};
            const int ret = permissions_->sample(core::monotonicTimeUs() / 1000, candidate, diagnostics);
            if (ret == 0) {
                permission_cache_ = candidate;
                permission_diagnostics_ = diagnostics;
            }
            else if (ret == -EAGAIN) {
                permission_diagnostics_.sample_error = ret;
            }
            else {
                permission_cache_ = {};
                permission_diagnostics_ = diagnostics;
                permission_diagnostics_.sample_error = ret;
            }
            next.observed.referee = permission_cache_;
            next.permission = permission_diagnostics_;
        }
        next.observed.now_us = core::monotonicTimeUs();
        const auto now_ms = next.observed.now_us / 1000;
        OperatorGate gate{};
        if (require_external_gate_) {
            const auto key = k_spin_lock(&operator_gate_lock_);
            gate = operator_gate_;
            k_spin_unlock(&operator_gate_lock_, key);
            if (gate.revision != gate_revision) {
                gate_revision = gate.revision;
                safe_gate_latched = false;
                arbiter_.withdrawMouseShooter();
            }
            next.observed.run_allowed = gate.allowed && !safe_gate_latched;
        }
        const auto permission_available = [&](const OutputPermission &permission) {
            return !require_permissions_ ||
                   (permission.valid && permission.enabled &&
                    isFresh(permission.stamp, now_ms, permission_timeout_ms_));
        };
        const bool output_allowed = permission_available(next.observed.referee.robot.shooter_output) &&
                                    permission_available(next.observed.referee.robot.gimbal_output);
        for (std::size_t i = 0; i < source_count_; ++i) {
            auto &slot = sources_[i];
            if (slot.role != SourceRole::Operator)
                continue;
            // 调参：每轮消费上限与接收队列容量一致，避免失控的源占满管理线程。
            for (unsigned budget = 0; budget < 32; ++budget) {
                RemoteState frame{};
                const int ret = slot.source->nextOperatorFrame(frame);
                if (ret == -ENOTSUP || ret == -EAGAIN)
                    break;
                if (ret < 0) {
                    arbiter_.withdrawMouseShooter();
                    slot.cached.diagnostics.sample_error = ret;
                    next.remote.sample_error = ret;
                    break;
                }
                const bool after_boundary = !require_external_gate_ || frame.stamp.timestamp_ms > gate.boundary_ms;
                if (require_external_gate_ && after_boundary && frame.left_switch == RcSwitch::Down) {
                    safe_gate_latched = true;
                    next.observed.run_allowed = false;
                }
                const bool frame_allowed = next.observed.run_allowed && after_boundary && output_allowed;
                arbiter_.observeRemoteFrame(frame, frame_allowed, now_ms);
                // 接收线程可能在 snapshot 后生产新帧；仲裁不可再退回旧快照。
                if (!next.observed.remote.stamp.valid ||
                    sequenceAfter(frame.stamp.sequence, next.observed.remote.stamp.sequence)) {
                    next.observed.remote = frame;
                    slot.cached.value = frame;
                }
            }
        }
        next.decision = arbiter_.update(next.observed);
        publish(next);
        k_sleep(K_MSEC(CONFIG_SKYWALKER_COMMAND_PERIOD_MS));
    }
}
}
