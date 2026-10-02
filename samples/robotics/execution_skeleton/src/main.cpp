#include <cerrno>
#include <core/clock.hpp>
#include <robotics/command/command_manager.hpp>
#include <robotics/execution/recovery_gate.hpp>
#include <robotics/execution/snapshot_cache.hpp>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(execution_skeleton, LOG_LEVEL_INF);

namespace {
using namespace skywalker;
using namespace skywalker::robotics;

constexpr std::uint32_t execution_period_ms = 5;
constexpr std::uint32_t maximum_cycle_ms = 25;
constexpr std::uint32_t status_timeout_ms = 100;
constexpr std::uint32_t enable_handshake_ms = 20;

enum class Fault : std::uint8_t { None, InputPaused, GimbalStopped, ChassisCycle, GimbalReference };

Fault faultAt(std::uint64_t now_ms) {
    if (!IS_ENABLED(CONFIG_EXECUTION_SKELETON_AUTORUN)) return Fault::None;
    const auto phase_ms = now_ms % 20000;
    if (phase_ms >= 3000 && phase_ms < 4500) return Fault::InputPaused;
    if (phase_ms >= 7000 && phase_ms < 8500) return Fault::GimbalStopped;
    if (phase_ms >= 11000 && phase_ms < 11150) return Fault::ChassisCycle;
    if (phase_ms >= 14000 && phase_ms < 15000) return Fault::GimbalReference;
    return Fault::None;
}

// A genuine ICommandSource sampled by the real manager worker. Pausing produces
// EAGAIN, preserving the last input stamp while arbitration continues running.
class PausableOperator final : public ICommandSource {
public:
    SourceRole role() const override { return SourceRole::Operator; }
    int start() override { return 0; }
    int sample(SourceSample &out) override {
        const auto now_ms = core::monotonicTimeUs() / 1000;
        if (faultAt(now_ms) == Fault::InputPaused) return -EAGAIN;
        RemoteState remote{};
        remote.online = true;
        remote.left_switch = RcSwitch::Middle;
        remote.right_switch = RcSwitch::Middle;
        remote.analog.left_y = 180;
        remote.analog.right_x = 160;
        remote.analog.right_y = 80;
        remote.stamp = {now_ms, ++sequence_, true};
        out = {};
        out.value = remote;
        return 0;
    }
private:
    std::uint32_t sequence_ = 0;
};

const CommandManager::Config manager_config = [] {
    CommandManager::Config config{};
    config.require_referee_for_motion = false;
    config.allow_auto = false;
    config.input_timeout_ms = 100;
    config.max_chassis_vx_m_s = config.max_chassis_vy_m_s = 0.3f;
    config.max_chassis_wz_rad_s = 0.3f;
    config.max_gimbal_yaw_rate_rad_s = config.max_gimbal_pitch_rate_rad_s = 0.2f;
    return config;
}();

PausableOperator operator_source;
CommandManager manager(manager_config);

struct Observation {
    RunStatus status{};
    MessageStamp command{};
    core::Stamp source{};
    std::uint32_t cycles = 0;
    std::uint32_t cycle_overruns = 0;
    std::uint32_t last_cycle_ms = 0;
    // These are in-memory targets only; no motor driver or transport is linked.
    float target_a = 0, target_b = 0, target_c = 0;
};

// Each consumer owns its target, lifecycle, recovery generation and status.
// The only shared input is the manager's non-consuming command snapshot.
class SimulatedExecutor {
public:
    explicit SimulatedExecutor(bool is_gimbal) : is_gimbal_(is_gimbal), gate_({100, 100000}) {}

    bool paused(Fault fault) const {
        return is_gimbal_ ? fault == Fault::GimbalStopped : fault == Fault::ChassisCycle;
    }

    void update(std::uint64_t now_ms) {
        ++observation_.cycles;
        observation_.last_cycle_ms = have_cycle_ ? static_cast<std::uint32_t>(now_ms - previous_cycle_ms_) : 0;
        const bool cycle_valid = !have_cycle_ ||
            (now_ms >= previous_cycle_ms_ && now_ms - previous_cycle_ms_ <= maximum_cycle_ms);
        previous_cycle_ms_ = now_ms;
        have_cycle_ = true;
        if (!cycle_valid) {
            ++observation_.cycle_overruns;
            revoke(WaitReason::Cycle, -ETIMEDOUT);
            publish(now_ms);
            return;
        }

        CommandSnapshot frame{};
        const int ret = manager.snapshot(frame);
        if (ret < 0 || frame.decision.error < 0) {
            revoke(WaitReason::Command, ret < 0 ? ret : frame.decision.error);
            publish(now_ms);
            return;
        }

        const auto &command = frame.decision.command;
        const bool requested = is_gimbal_ ? command.gimbal.mode == GimbalMode::Rate
                                          : command.chassis.mode == ChassisMode::BodyVelocity;
        observation_.command = is_gimbal_ ? command.gimbal.stamp : command.chassis.stamp;
        observation_.source = sourceStamp(frame, is_gimbal_ ? command.gimbal.source : command.chassis.source);
        if (!requested) {
            revoke(WaitReason::Command, 0);
        } else if (is_gimbal_ && faultAt(now_ms) == Fault::GimbalReference) {
            revoke(WaitReason::Reference, -ESTALE);
        } else {
            if (gate_.stage() == RecoveryGate::Stage::WaitingPrerequisites) {
                // No-output substitute for reference preparation/history reset.
                clearTarget();
                gate_.prepared(now_ms);
            }
            if (!gate_.accept(observation_.command, observation_.source, now_ms)) {
                clearTarget();
                enabling_ = false;
            } else {
                if (gate_.stage() == RecoveryGate::Stage::Enabling) {
                    if (!enabling_) {
                        enable_started_ms_ = now_ms;
                        enabling_ = true;
                    }
                    if (now_ms - enable_started_ms_ >= enable_handshake_ms) gate_.enabled();
                }
                if (gate_.stage() == RecoveryGate::Stage::Active) {
                    observation_.target_a = is_gimbal_ ? command.gimbal.yaw_rate_rad_s : command.chassis.vx_m_s;
                    observation_.target_b = is_gimbal_ ? command.gimbal.pitch_rate_rad_s : command.chassis.vy_m_s;
                    observation_.target_c = is_gimbal_ ? 0 : command.chassis.wz_rad_s;
                }
            }
        }
        publish(now_ms);
    }

    int snapshot(Observation &out) const { return cache_.snapshot(out); }

private:
    void clearTarget() {
        observation_.target_a = observation_.target_b = observation_.target_c = 0;
    }
    void revoke(WaitReason reason, int error) {
        clearTarget();
        enabling_ = false;
        gate_.withdraw(reason, error);
    }
    void publish(std::uint64_t now_ms) {
        auto &status = observation_.status;
        const auto stage = gate_.stage();
        status.state = stage == RecoveryGate::Stage::Active ? RunState::Active
                     : stage == RecoveryGate::Stage::Blocked ? RunState::Blocked
                     : stage == RecoveryGate::Stage::WaitingPrerequisites ? RunState::Disabled
                     : RunState::Recovering;
        status.ready = stage == RecoveryGate::Stage::WaitingCommand || stage == RecoveryGate::Stage::Active;
        status.reason = gate_.reason();
        status.error = gate_.error();
        status.generation = gate_.generation();
        status.last_command_sequence = observation_.command.sequence;
        status.stamp = {now_ms, ++status_sequence_, true};
        (void)cache_.publish(observation_);
    }
    const bool is_gimbal_;
    RecoveryGate gate_;
    SnapshotCache<Observation> cache_;
    Observation observation_{};
    std::uint32_t status_sequence_ = 0;
    std::uint64_t previous_cycle_ms_ = 0, enable_started_ms_ = 0;
    bool have_cycle_ = false, enabling_ = false;
};

SimulatedExecutor chassis(false), gimbal(true);
K_THREAD_STACK_DEFINE(chassis_stack, 4096);
K_THREAD_STACK_DEFINE(gimbal_stack, 4096);
k_thread chassis_thread{}, gimbal_thread{};

void execute(void *self, void *, void *) {
    auto &executor = *static_cast<SimulatedExecutor *>(self);
    for (;;) {
        const auto now_ms = core::monotonicTimeUs() / 1000;
        if (!executor.paused(faultAt(now_ms))) executor.update(now_ms);
        k_sleep(K_MSEC(execution_period_ms));
    }
}

std::uint64_t age(std::uint64_t stamp_ms, bool valid, std::uint64_t now_ms) {
    return valid && now_ms >= stamp_ms ? now_ms - stamp_ms : UINT64_MAX;
}

void emit(const char *name, const SimulatedExecutor &executor, std::uint64_t now_ms) {
    Observation observation{};
    if (executor.snapshot(observation) < 0) return;
    const auto &status = observation.status;
    const bool status_fresh = isFresh(status.stamp, now_ms, status_timeout_ms);
    LOG_INF("%s state=%u ready=%u reason=%u err=%d generation=%u status_seq=%u status_age=%llu valid=%u "
            "command_seq=%u command_age=%llu source_seq=%llu source_age=%llu target=(%.3f,%.3f,%.3f) "
            "cycle=%u overruns=%u cycles=%u",
            name, unsigned(status.state), unsigned(status.ready && status_fresh), unsigned(status.reason),
            status.error, status.generation, status.stamp.sequence,
            static_cast<unsigned long long>(age(status.stamp.timestamp_ms, status.stamp.valid, now_ms)),
            unsigned(status_fresh), observation.command.sequence,
            static_cast<unsigned long long>(age(observation.command.timestamp_ms, observation.command.valid, now_ms)),
            static_cast<unsigned long long>(observation.source.sequence),
            static_cast<unsigned long long>(age(observation.source.time_us / 1000, observation.source.valid, now_ms)),
            static_cast<double>(observation.target_a), static_cast<double>(observation.target_b),
            static_cast<double>(observation.target_c), observation.last_cycle_ms, observation.cycle_overruns,
            observation.cycles);
}
} // namespace

int main() {
    int ret = manager.registerSource(operator_source);
    if (ret == 0) ret = manager.start();
    if (ret < 0) {
        LOG_ERR("command service start: %d", ret);
        return ret;
    }
    k_thread_create(&chassis_thread, chassis_stack, K_THREAD_STACK_SIZEOF(chassis_stack), execute,
                    &chassis, nullptr, nullptr, 4, 0, K_NO_WAIT);
    k_thread_create(&gimbal_thread, gimbal_stack, K_THREAD_STACK_SIZEOF(gimbal_stack), execute,
                    &gimbal, nullptr, nullptr, 4, 0, K_NO_WAIT);
    LOG_INF("Real command service, independent chassis/gimbal consumers, simulated targets only; autorun=%u",
            unsigned(IS_ENABLED(CONFIG_EXECUTION_SKELETON_AUTORUN)));
    Fault last_fault = Fault::None;
    for (;;) {
        const auto now_ms = core::monotonicTimeUs() / 1000;
        const auto fault = faultAt(now_ms);
        if (fault != last_fault) {
            LOG_INF("exercise=%u at=%llu ms", unsigned(fault), static_cast<unsigned long long>(now_ms));
            last_fault = fault;
        }
        emit("chassis", chassis, now_ms);
        emit("gimbal", gimbal, now_ms);
        CommandSnapshot frame{};
        if (manager.snapshot(frame) == 0) {
            LOG_INF("manager_seq=%u input_seq=%u input_sample_error=%d reasons=%x",
                    frame.decision.command.stamp.sequence, frame.observed.remote.stamp.sequence,
                    frame.remote.sample_error, frame.decision.reasons());
        }
        k_sleep(K_MSEC(250));
    }
}
