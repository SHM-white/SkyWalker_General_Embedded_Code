#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/position_motor.hpp>
#include <drivers/motor/can_bus.hpp>
#include <lib/vofa/vofa.h>

LOG_MODULE_REGISTER(m2006_position_control, LOG_LEVEL_INF);

#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)
#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

namespace {
constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::uint32_t kTelemetryPeriodCycles = 5U;

constexpr float kPositionKp = 6.0f;
constexpr float kVelocityKp = 0.03f;
constexpr float kVelocityKi = 0.5f;
constexpr float kVelocityRampRateRadS2 = 40.0f;

constexpr float kVelocityAbsMaxRadS = 30.0f;
// Retain the position sample's 0.8 A loop limit for initial tuning.
constexpr float kSoftwareCurrentAbsMaxA = 0.80f;
constexpr float kPositionDeadbandRad = 0.012f;
constexpr float kMeasuredVelocitySafetyMaxRadS = 1.5f * kVelocityAbsMaxRadS;
constexpr float kPi = 3.14159265358979323846f;

// 拨弹盘参数：单位为减速器输出轴 rad；若还有外部传动，计入 kRadPerRound。
constexpr double kRadPerRound = 2.0 * kPi / 8.0; // 每发转角：示例为 8 个弹位直连
constexpr double kRoundsPerSecond = 1.0;        // 推进频率；不代表实际出弹计数
constexpr double kFeedDirection = 1.0;          // +1 或 -1
constexpr double kFeedSpeedRadS = kRadPerRound * kRoundsPerSecond;

// 卡弹：位置落后、低速和大反馈电流同时持续指定时间。
constexpr double kJamErrorRad = 0.25 * kRadPerRound;
constexpr float kJamSpeedRadS = 0.10f;
constexpr float kJamCurrentA = 0.60f;
constexpr std::int64_t kJamConfirmMs = 300;
constexpr std::int64_t kStartupGraceMs = 500;
constexpr double kMaxLeadRad = kRadPerRound;

constexpr double kReverseRad = 0.5 * kRadPerRound;
constexpr double kReverseSpeedRadS = 0.5;
constexpr double kReverseToleranceRad = 0.03;
constexpr std::int64_t kReverseSettleMs = 100;
constexpr std::int64_t kReverseTimeoutMs = 2500;
constexpr std::int64_t kTransitionTimeoutMs = 1000;
constexpr unsigned kMaxRecoveryAttempts = 3; // 整次运行累计；成功恢复不清零

static_assert(kRadPerRound > 0.0 && kRoundsPerSecond > 0.0);
static_assert(kFeedDirection == 1.0 || kFeedDirection == -1.0);
static_assert(kFeedSpeedRadS > kJamSpeedRadS && kFeedSpeedRadS <= kVelocityAbsMaxRadS);
static_assert(kJamSpeedRadS > 0.0f && kJamCurrentA > 0.0f && kJamCurrentA < kSoftwareCurrentAbsMaxA);
static_assert(kJamErrorRad > 0.0 && kMaxLeadRad > kJamErrorRad);
static_assert(kReverseSpeedRadS > kJamSpeedRadS && kReverseSpeedRadS <= kVelocityAbsMaxRadS);
static_assert(kReverseToleranceRad > 0.0 && kReverseRad > kReverseToleranceRad);
static_assert(kReverseTimeoutMs > 1000.0 * kReverseRad / kReverseSpeedRadS + kReverseSettleMs);
static_assert(kJamConfirmMs > 0 && kStartupGraceMs > 0 && kReverseSettleMs > 0);
static_assert(kTransitionTimeoutMs > 0 && kMaxRecoveryAttempts > 0);

enum class FeedState : std::uint8_t {
    Forward = 0,
    Stopping = 1,
    Enabling = 2,
    Reverse = 3,
    FaultLatched = 4,
};

struct FeedRuntime {
    FeedState state = FeedState::Forward;
    FeedState resume = FeedState::Forward;
    double target_rad = 0.0;
    unsigned recovery_attempts = 0;
    std::int64_t state_since_ms = 0;
    std::int64_t jam_since_ms = -1;
    std::int64_t settled_since_ms = -1;
    std::uint64_t last_feedback_ms = 0;
    std::uint64_t reference_generation = 0;
};

int validateFeedback(const skywalker::motor::MotorSnapshot &snapshot,
                     std::uint64_t reference_generation) {
    using namespace skywalker::motor;
    const auto &feedback = snapshot.feedback;
    constexpr auto required = FeedbackPosition | FeedbackVelocity | FeedbackCurrent;
    if (snapshot.state == MotorState::Fault)
        return snapshot.last_fault.error < 0 ? snapshot.last_fault.error : -EIO;
    if (!snapshot.feedback_fresh)
        return -ESTALE;
    if (!snapshot.position_reference_valid || (feedback.valid & required) != required)
        return -ENODATA;
    if (snapshot.reference_generation != reference_generation)
        return -ESTALE;
    if (!std::isfinite(feedback.position_rad) || !std::isfinite(feedback.velocity_rad_s) ||
        !std::isfinite(feedback.current_a))
        return -EINVAL;
    if (std::fabs(feedback.velocity_rad_s) > kMeasuredVelocitySafetyMaxRadS)
        return -ERANGE;
    return 0;
}

control_motor_position_config makePositionLoopConfig() {
    control_motor_position_config config{};

    config.position = {
        .kp = kPositionKp,
        .ki = 1.5f,
        .kd = 0.8f,
        .derivative_tau_s = 0.0f,
        .integral_min = -8.0f,
        .integral_max = 8.0f,
        .output_min = -kVelocityAbsMaxRadS,
        .output_max = kVelocityAbsMaxRadS,
        .deadband = kPositionDeadbandRad,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };

    config.velocity.regulator.feedback = {
        .kp = kVelocityKp,
        .ki = kVelocityKi,
        .kd = 0.00005f,
        .derivative_tau_s = 0.0f,
        .integral_min = -kSoftwareCurrentAbsMaxA,
        .integral_max = kSoftwareCurrentAbsMaxA,
        .output_min = -kSoftwareCurrentAbsMaxA,
        .output_max = kSoftwareCurrentAbsMaxA,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };
    config.velocity.regulator.feedforward = {
        .k_bias = 0.0f,
        .k_static = 0.0f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
    };
    config.velocity.reference_slew = {
        .rising_rate_per_s = kVelocityRampRateRadS2,
        .falling_rate_per_s = kVelocityRampRateRadS2,
    };
    /* Zero filtering and zero soft deadband preserve the old sample. */
    config.velocity.measurement_filter_tau_s = 0.0f;
    config.velocity.soft_deadband_rad_s = 0.0f;
    config.velocity.requested_velocity_abs_max_rad_s = kVelocityAbsMaxRadS;
    config.velocity.effort_abs_max = kSoftwareCurrentAbsMaxA;
    return config;
}

skywalker::control::PositionMotor::Config makeMotorConfig() {
    skywalker::control::PositionMotor::Config config{};
    config.loop = makePositionLoopConfig();
    config.effort_unit = skywalker::control::EffortUnit::Ampere;
    config.safety = {kMeasuredVelocitySafetyMaxRadS, 0.0f};
    config.reference = skywalker::control::PositionReference::StartupRelative;
    return config;
}

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    if (!device_is_ready(uart) || !device_is_ready(can))
        return -ENODEV;
    static skywalker::motor::Motor drive{skywalker::motor::dji::m2006({
        .id = 1,
        .current_limit_a = 10.0f,
        .gear_ratio = 36.0f,
        .timing = {20, 20, 20, 100},
    })};
    static skywalker::motor::CanBus bus{can};
    static skywalker::control::PositionMotor axis{drive, makeMotorConfig()};
    static Vofa vofa{};
    int ret = vofa_init(&vofa, uart);
    if (ret < 0)
        return ret;
    ret = bus.attach(drive);
    if (ret == 0)
        ret = bus.start();
    if (ret == 0)
        ret = axis.configure();
    if (ret < 0) {
        LOG_ERR("configuration blocked: %d", ret);
        return ret;
    }
    const auto ready_deadline = k_uptime_get() + 3000;
    while (!drive.ready() && k_uptime_get() < ready_deadline)
        k_sleep(K_MSEC(kControlPeriodMs));
    if (!drive.ready())
        return -ETIMEDOUT;
    // M2006 has no absolute output-shaft reference; reset anchors the startup zero.
    ret = axis.reset(); // Check speed, temperature and reference before enabling.
    if (ret == 0)
        ret = drive.enable();
    if (ret < 0) {
        (void)drive.disable();
        return ret;
    }
    const auto active_deadline = k_uptime_get() + 3000;
    while (!drive.active() && k_uptime_get() < active_deadline)
        k_sleep(K_MSEC(kControlPeriodMs));
    if (!drive.active()) {
        (void)drive.disable();
        return -ETIMEDOUT;
    }
    FeedRuntime runtime{};
    runtime.state_since_ms = k_uptime_get();
    runtime.reference_generation = drive.snapshot().reference_generation;
    auto previous_ms = runtime.state_since_ms;
    std::uint32_t telemetry_divider = 0;
    std::int64_t next_vofa_warning_ms = 0;

    const auto latch_fault = [&](int cause) {
        if (runtime.state == FeedState::FaultLatched)
            return;
        runtime.state = FeedState::FaultLatched;
        const int stop_error = drive.disable();
        LOG_ERR("Feed fault latched: %d, recovery attempts=%u; restart after clearing the obstruction",
                cause, runtime.recovery_attempts);
        if (stop_error < 0)
            LOG_ERR("Stop request failed: %d", stop_error);
    };
    const auto request_reanchor = [&](FeedState resume, std::int64_t now) {
        const int error = drive.disable();
        if (error < 0)
            return error;
        runtime.state = FeedState::Stopping;
        runtime.resume = resume;
        runtime.state_since_ms = now;
        runtime.jam_since_ms = -1;
        runtime.settled_since_ms = -1;
        LOG_INF("Stopping before %s", resume == FeedState::Reverse ? "reverse" : "forward");
        return 0;
    };
    const auto request_recovery = [&](std::int64_t now, bool confirmed_jam) {
        if (runtime.recovery_attempts >= kMaxRecoveryAttempts)
            return -EIO;
        ++runtime.recovery_attempts;
        LOG_WRN("%s; recovery %u/%u", confirmed_jam ? "Jam detected" : "Position lag limit reached",
                runtime.recovery_attempts, kMaxRecoveryAttempts);
        return request_reanchor(FeedState::Reverse, now);
    };

    LOG_INF("Continuous position feed started; VOFA: 16 channels, state 0=forward 1=stop 2=enable 3=reverse 4=fault");
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto now = k_uptime_get();
        const float dt_s = float(now - previous_ms) / 1000.0f;
        previous_ms = now;
        const auto snapshot = drive.snapshot();
        skywalker::control::PositionMotor::Telemetry data{};
        bool controlled = false;

        // Faults remain latched, while telemetry and the CAN worker keep running.
        if (runtime.state != FeedState::FaultLatched) {
            ret = validateFeedback(snapshot, runtime.reference_generation);
            if (ret < 0)
                latch_fault(ret);
        }

        switch (runtime.state) {
        case FeedState::Stopping:
            if (now - runtime.state_since_ms >= kTransitionTimeoutMs) {
                latch_fault(-ETIMEDOUT);
                break;
            }
            if (snapshot.state == skywalker::motor::MotorState::Disabled && drive.ready() &&
                std::fabs(snapshot.feedback.velocity_rad_s) <= kJamSpeedRadS) {
                // reset() is only legal while disabled. It clears both PID histories
                // and establishes a new relative zero, discarding all old target debt.
                ret = axis.reset();
                if (ret == 0) {
                    runtime.target_rad = 0.0;
                    runtime.last_feedback_ms = snapshot.feedback.timestamp_ms;
                    ret = drive.enable();
                }
                if (ret < 0) {
                    latch_fault(ret);
                    break;
                }
                runtime.state = FeedState::Enabling;
                runtime.state_since_ms = now;
            }
            break;

        case FeedState::Enabling:
            if (now - runtime.state_since_ms >= kTransitionTimeoutMs) {
                latch_fault(-ETIMEDOUT);
                break;
            }
            if (drive.active()) {
                runtime.state = runtime.resume;
                runtime.state_since_ms = now;
                runtime.last_feedback_ms = snapshot.feedback.timestamp_ms;
                LOG_INF("%s active", runtime.state == FeedState::Reverse ? "Reverse" : "Forward");
            }
            break;

        case FeedState::Forward:
        case FeedState::Reverse: {
            if (!drive.active()) {
                latch_fault(-EHOSTDOWN);
                break;
            }
            if (!std::isfinite(dt_s) || dt_s < 0.001f || dt_s > 0.020f) {
                latch_fault(-ERANGE);
                break;
            }
            const bool reversing = runtime.state == FeedState::Reverse;
            constexpr double reverse_end = -kFeedDirection * kReverseRad;
            if (reversing) {
                if (now - runtime.state_since_ms >= kReverseTimeoutMs) {
                    latch_fault(-ETIMEDOUT);
                    break;
                }
                const double step = kReverseSpeedRadS * double(dt_s);
                const double left = std::fabs(reverse_end - runtime.target_rad);
                runtime.target_rad = left <= step ? reverse_end
                                                 : runtime.target_rad - kFeedDirection * step;
            } else {
                // No modulo and no periodic reset: a constant-slope position ramp.
                runtime.target_rad += kFeedDirection * kFeedSpeedRadS * double(dt_s);
            }
            ret = axis.update(runtime.target_rad, dt_s);
            if (ret == 0) {
                data = axis.telemetry();
                ret = data.valid ? validateFeedback(data.motor, runtime.reference_generation) : -ENODATA;
            }
            if (ret == 0)
                ret = bus.commit().error;
            if (ret < 0) {
                latch_fault(ret);
                break;
            }
            controlled = true;
            const auto &fb = data.motor.feedback;
            const double lead = kFeedDirection * (runtime.target_rad - data.position_rad);
            if (!reversing && lead <= -kMaxLeadRad) {
                latch_fault(-ERANGE);
                break;
            }
            if (!reversing && lead >= kMaxLeadRad) {
                ret = request_recovery(now, false);
                if (ret < 0)
                    latch_fault(ret);
                break;
            }
            // Re-reading a cached CAN frame does not count as another jam/settle observation.
            if (fb.timestamp_ms <= runtime.last_feedback_ms)
                break;
            runtime.last_feedback_ms = fb.timestamp_ms;
            const auto feedback_ms = static_cast<std::int64_t>(fb.timestamp_ms);
            if (reversing) {
                const bool settled = runtime.target_rad == reverse_end &&
                    std::fabs(data.position_rad - reverse_end) <= kReverseToleranceRad &&
                    std::fabs(fb.velocity_rad_s) <= kJamSpeedRadS;
                if (!settled) {
                    runtime.settled_since_ms = -1;
                } else if (runtime.settled_since_ms < 0) {
                    runtime.settled_since_ms = feedback_ms;
                } else if (feedback_ms - runtime.settled_since_ms >= kReverseSettleMs) {
                    ret = request_reanchor(FeedState::Forward, now);
                    if (ret < 0)
                        latch_fault(ret);
                }
            } else {
                const bool jam = now - runtime.state_since_ms >= kStartupGraceMs &&
                    lead >= kJamErrorRad && std::fabs(fb.velocity_rad_s) <= kJamSpeedRadS &&
                    std::fabs(fb.current_a) >= kJamCurrentA;
                if (!jam) {
                    runtime.jam_since_ms = -1;
                } else if (runtime.jam_since_ms < 0) {
                    runtime.jam_since_ms = feedback_ms;
                } else if (feedback_ms - runtime.jam_since_ms >= kJamConfirmMs) {
                    ret = request_recovery(now, true);
                    if (ret < 0)
                        latch_fault(ret);
                }
            }
            break;
        }
        case FeedState::FaultLatched:
            break;
        }

        if (++telemetry_divider >= kTelemetryPeriodCycles) {
            telemetry_divider = 0;
            const auto current = drive.snapshot();
            const auto &feedback = controlled ? data.motor.feedback : current.feedback;
            const auto &output = data.output;
            const auto telemetry_ms = k_uptime_get();
            const float feedback_age_ms = telemetry_ms >= static_cast<std::int64_t>(feedback.timestamp_ms)
                ? static_cast<float>(telemetry_ms - feedback.timestamp_ms) : 0.0f;
            // Channels 0..13 retain the position sample layout, except reserved
            // channel 3 now carries measured current. Channels 14/15 are state/retries.
            const float channels[16] = {
                static_cast<float>(runtime.target_rad),
                controlled ? static_cast<float>(data.target_position_rad) : NAN,
                controlled ? static_cast<float>(data.position_rad) : NAN,
                feedback.current_a,
                output.position.error,
                output.position.output,
                output.velocity.velocity_reference_rad_s,
                feedback.velocity_rad_s,
                output.velocity.velocity_error_rad_s,
                output.velocity.regulator.feedback.p,
                output.velocity.regulator.feedback.i,
                output.effort_command,
                dt_s * 1000.0f,
                feedback_age_ms,
                static_cast<float>(runtime.state),
                static_cast<float>(runtime.recovery_attempts),
            };
            const int telemetry_error = vofa_send(&vofa, channels, 16);
            if (telemetry_error < 0 && now >= next_vofa_warning_ms) {
                LOG_WRN("vofa_send failed: %d", telemetry_error);
                next_vofa_warning_ms = now + 1000;
            }
        }
    }
}
