#include <algorithm>
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
constexpr float kPi = 3.14159265358979323846f;

// 拨弹盘参数：单位为减速器输出轴 rad；若还有外部传动，计入 kRadPerRound。
constexpr double kRadPerRound = 2.0 * kPi / 8.0; // 每发转角：示例为 8 个弹位直连
constexpr double kRoundsPerSecond = 1.0;         // 推进频率；不代表实际出弹计数
constexpr double kFeedDirection = 1.0;           // +1 或 -1
constexpr double kFeedSpeedRadS = kRadPerRound * kRoundsPerSecond;

// 卡弹：位置落后、低速和大反馈电流同时持续指定时间。
constexpr double kJamErrorRad = 0.25 * kRadPerRound;
constexpr float kJamSpeedRadS = 0.10f;
constexpr float kJamCurrentA = 0.60f;
constexpr std::int64_t kJamConfirmMs = 300;
constexpr std::int64_t kStartupGraceMs = 500;

constexpr double kReverseRad = 0.5 * kRadPerRound;
constexpr double kReverseSpeedRadS = 0.5;
constexpr double kReverseToleranceRad = 0.03;
constexpr std::int64_t kReverseSettleMs = 100;
constexpr std::int64_t kReverseTimeoutMs = 2500;

static_assert(kRadPerRound > 0.0 && kRoundsPerSecond > 0.0);
static_assert(kFeedDirection == 1.0 || kFeedDirection == -1.0);
static_assert(kFeedSpeedRadS > kJamSpeedRadS && kFeedSpeedRadS <= kVelocityAbsMaxRadS);
static_assert(kJamSpeedRadS > 0.0f && kJamCurrentA > 0.0f && kJamCurrentA < kSoftwareCurrentAbsMaxA);
static_assert(kJamErrorRad > 0.0);
static_assert(kReverseSpeedRadS > kJamSpeedRadS && kReverseSpeedRadS <= kVelocityAbsMaxRadS);
static_assert(kReverseToleranceRad > 0.0 && kReverseRad > kReverseToleranceRad);
static_assert(kReverseTimeoutMs > 1000.0 * kReverseRad / kReverseSpeedRadS + kReverseSettleMs);
static_assert(kJamConfirmMs > 0 && kStartupGraceMs > 0 && kReverseSettleMs > 0);

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

    config.reference = skywalker::control::PositionReference::StartupRelative;
    return config;
}

} // namespace

int main() {
    const device *uart = DEVICE_DT_GET(VOFA_UART_NODE);
    const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
    if (!device_is_ready(can))
        return -ENODEV;
    static skywalker::motor::Motor drive{skywalker::motor::dji::m2006({
        .id = 1,
        .current_limit_a = 10.0f,
        .gear_ratio = 36.0f,
        .timing =
            {.feedback_timeout_ms = 20, .command_timeout_ms = 20, .enable_timeout_ms = 100, .retry_interval_ms = 100},
    })};
    static skywalker::motor::CanBus bus{can};
    static skywalker::control::PositionMotor axis{drive, makeMotorConfig()};
    static Vofa vofa{};
    const int vofa_error = vofa_init(&vofa, uart);
    int ret = bus.attach(drive);
    if (ret == 0)
        ret = bus.start();
    if (ret == 0)
        ret = axis.configure();
    if (ret < 0)
        return ret;
    const auto started_ms = k_uptime_get();
    auto previous_ms = started_ms;
    double target = 0, reverse_target = 0;
    bool reversing = false;
    unsigned recovery_attempts = 0;
    std::int64_t reverse_started_ms = 0, jam_since_ms = -1, settled_since_ms = -1, next_log_ms = 0;
    std::uint64_t observed_feedback_ms = 0;
    int last_call_error = 0;
    for (;;) {
        k_sleep(K_MSEC(kControlPeriodMs));
        const auto now = k_uptime_get();
        const float dt = float(now - previous_ms) / 1000;
        previous_ms = now;
        if (std::isfinite(dt) && dt > 0 && dt <= 0.02f) {
            if (reversing) {
                const double step = kReverseSpeedRadS * dt;
                const double left = reverse_target - target;
                target += std::copysign(std::min(std::fabs(left), step), left);
            }
            else
                target += kFeedDirection * kFeedSpeedRadS * dt;
        }
        const int enable_error = drive.enable();
        const int update_error = axis.update(target, dt);
        const int commit_error = bus.commit().error;
        if (enable_error < 0)
            last_call_error = enable_error;
        if (update_error < 0)
            last_call_error = update_error;
        if (commit_error < 0)
            last_call_error = commit_error;
        const auto data = axis.telemetry();
        const auto &f = data.motor.feedback;
        if (!data.output_valid) {
            jam_since_ms = settled_since_ms = -1;
        }
        else if (f.timestamp_ms > observed_feedback_ms) {
            observed_feedback_ms = f.timestamp_ms;
            const auto feedback_ms = static_cast<std::int64_t>(f.timestamp_ms);
            if (reversing) {
                const bool settled = std::fabs(target - reverse_target) < 1e-6 &&
                                     std::fabs(data.position_rad - reverse_target) <= kReverseToleranceRad &&
                                     std::fabs(f.velocity_rad_s) <= kJamSpeedRadS;
                if (!settled)
                    settled_since_ms = -1;
                else if (settled_since_ms < 0)
                    settled_since_ms = feedback_ms;
                if ((settled_since_ms >= 0 && feedback_ms - settled_since_ms >= kReverseSettleMs) ||
                    now - reverse_started_ms >= kReverseTimeoutMs) {
                    reversing = false;
                    target = data.position_rad;
                    jam_since_ms = settled_since_ms = -1;
                }
            }
            else {
                const double lead = kFeedDirection * (target - data.position_rad);
                const bool jam = now - started_ms >= kStartupGraceMs && lead >= kJamErrorRad &&
                                 (f.valid & skywalker::motor::FeedbackCurrent) &&
                                 std::fabs(f.velocity_rad_s) <= kJamSpeedRadS && std::fabs(f.current_a) >= kJamCurrentA;
                if (!jam)
                    jam_since_ms = -1;
                else if (jam_since_ms < 0)
                    jam_since_ms = feedback_ms;
                if (jam_since_ms >= 0 && feedback_ms - jam_since_ms >= kJamConfirmMs) {
                    reversing = true;
                    ++recovery_attempts;
                    target = data.position_rad;
                    reverse_target = target - kFeedDirection * kReverseRad;
                    reverse_started_ms = now;
                    jam_since_ms = settled_since_ms = -1;
                }
            }
        }
        if (vofa_error == 0) {
            const auto &o = data.output;
            const float channels[16] = {float(target),
                                        float(data.target_position_rad),
                                        float(data.position_rad),
                                        f.current_a,
                                        o.position.error,
                                        o.position.output,
                                        o.velocity.velocity_reference_rad_s,
                                        f.velocity_rad_s,
                                        o.velocity.velocity_error_rad_s,
                                        o.velocity.regulator.feedback.p,
                                        o.velocity.regulator.feedback.i,
                                        data.output_valid ? o.effort_command : 0,
                                        dt * 1000,
                                        float(now >= f.timestamp_ms ? now - f.timestamp_ms : 0),
                                        float(reversing),
                                        float(recovery_attempts)};
            (void)vofa_send(&vofa, channels, 16);
        }
        if (now >= next_log_ms) {
            next_log_ms = now + 1000;
            LOG_INF("target=%.3f seq=%llu output=%d wait=%u reverse=%d attempts=%u call=%d", target,
                    static_cast<unsigned long long>(data.target_sequence), data.output_valid, unsigned(data.issue),
                    reversing, recovery_attempts, last_call_error);
        }
    }
}
