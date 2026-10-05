#include <cerrno>
#include <core/clock.hpp>
#include <robotics/command/command_manager.hpp>
#include <robotics/command/command_source.hpp>
#include <robotics/execution/run_status.hpp>
#include <robotics/execution/snapshot_cache.hpp>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(execution_skeleton, LOG_LEVEL_INF);

namespace {
using namespace skywalker;
using namespace skywalker::robotics;

constexpr std::uint32_t execution_period_ms = 5;
constexpr std::uint32_t maximum_cycle_ms = 25;
constexpr std::uint32_t status_timeout_ms = 100;

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

// Consumers retain the latest target independently of current execution ability.
class SimulatedExecutor {
public:
    explicit SimulatedExecutor(bool is_gimbal) : is_gimbal_(is_gimbal) {}

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
        if (!cycle_valid) ++observation_.cycle_overruns;

        CommandSnapshot frame{};
        const int ret = manager.snapshot(frame);
        if (ret == 0) {
            const auto &command = frame.decision.command;
            mode_requested_ = frame.decision.error == 0 &&
                (is_gimbal_ ? command.gimbal.mode == GimbalMode::Rate
                            : command.chassis.mode == ChassisMode::BodyVelocity);
            observation_.command = is_gimbal_ ? command.gimbal.stamp : command.chassis.stamp;
            observation_.source = sourceStamp(frame, is_gimbal_ ? command.gimbal.source : command.chassis.source);
            observation_.target_a = is_gimbal_ ? command.gimbal.yaw_rate_rad_s : command.chassis.vx_m_s;
            observation_.target_b = is_gimbal_ ? command.gimbal.pitch_rate_rad_s : command.chassis.vy_m_s;
            observation_.target_c = is_gimbal_ ? 0 : command.chassis.wz_rad_s;
        }
        auto &status = observation_.status;
        status.requested = mode_requested_ && isFresh(observation_.command, now_ms, 100) &&
            core::fresh(observation_.source, now_ms * 1000, 100000);
        status.member_count = is_gimbal_ ? 2 : 3;
        const bool reference_valid = !is_gimbal_ || faultAt(now_ms) != Fault::GimbalReference;
        status.active_count = status.requested && cycle_valid && reference_valid ? status.member_count : 0;
        status.waiting_count = status.requested ? status.member_count - status.active_count : 0;
        status.ready = status.active_count == status.member_count;
        status.state = !status.requested ? RunState::Disabled
                     : status.active_count ? RunState::Active : RunState::Recovering;
        status.reason = !status.requested ? WaitReason::Command
                      : !reference_valid ? WaitReason::Reference
                      : !cycle_valid ? WaitReason::Cycle : WaitReason::None;
        status.error = !status.requested ? 0 : !reference_valid ? -ESTALE : !cycle_valid ? -ETIMEDOUT : 0;
        if (!status.requested)
            observation_.target_a = observation_.target_b = observation_.target_c = 0;
        status.last_command_sequence = observation_.command.sequence;
        status.stamp = {now_ms, ++status_sequence_, true};
        (void)cache_.publish(observation_);
    }

    int snapshot(Observation &out) const { return cache_.snapshot(out); }

private:
    const bool is_gimbal_;
    SnapshotCache<Observation> cache_;
    Observation observation_{};
    std::uint32_t status_sequence_ = 0;
    std::uint64_t previous_cycle_ms_ = 0;
    bool have_cycle_ = false, mode_requested_ = false;
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
    LOG_INF("%s requested=%d state=%u active=%zu waiting=%zu reason=%u err=%d status_seq=%u status_age=%llu valid=%u "
            "command_seq=%u command_age=%llu source_seq=%llu source_age=%llu target=(%.3f,%.3f,%.3f) "
            "cycle=%u overruns=%u cycles=%u",
            name, status.requested, unsigned(status.state), status.active_count, status.waiting_count, unsigned(status.reason),
            status.error, status.stamp.sequence,
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
