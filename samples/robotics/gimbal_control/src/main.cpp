#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>

#include <communication/remote/remote_receiver.hpp>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>
#include <lib/vofa/vofa.h>
#include <robotics/gimbal/gimbal_axis.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board_config.hpp"
#include "../../common/rc_controls.hpp"
#include "../../common/sample_diagnostics.hpp"

#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)

#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

static_assert(DT_NODE_HAS_COMPAT(VOFA_UART_NODE, zephyr_cdc_acm_uart),
              "gimbal_control telemetry requires USB CDC ACM; check app.overlay");

LOG_MODULE_REGISTER(gimbal_rc, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;

namespace {
static communication::AsyncUart::DmaBuffers dma_buffers __nocache;
communication::RemoteReceiver receiver(board_config::remote_uart, dma_buffers, samples::control::receiverConfig());

void gimbalTask(void *, void *, void *) {
    static motor::Motor yaw_drive(board_config::yawHardware());
    static motor::Motor pitch_drive(board_config::pitchHardware());
    static motor::Group gimbal(yaw_drive, pitch_drive);
    static motor::CanBus yaw_bus(board_config::yaw_can);
    static motor::CanBus pitch_bus(board_config::pitch_can);
    static GimbalAxis yaw(yaw_drive, board_config::yawMotorConfig(), board_config::yaw);
    static GimbalAxis pitch(pitch_drive, board_config::pitchMotorConfig(), board_config::pitch);

    if (!board_config::connections_configured) {
        LOG_ERR("hardware template disabled; check board_config.hpp");
        return;
    }
    static Vofa vofa{};
    const int vofa_ret = vofa_init(&vofa, DEVICE_DT_GET(VOFA_UART_NODE));
    if (vofa_ret < 0)
        LOG_ERR("VOFA telemetry UART initialization failed: %d", vofa_ret);
    else
        LOG_INF("Telemetry device=%s, JustFloat: 16 channels at 50 Hz", DEVICE_DT_GET(VOFA_UART_NODE)->name);

    const bool split_buses = board_config::yaw_can != board_config::pitch_can;
    int ret = split_buses ? yaw_bus.attach(yaw_drive) : yaw_bus.attach(yaw_drive, pitch_drive);
    if (ret == 0 && split_buses)
        ret = pitch_bus.attach(pitch_drive);
    if (ret == 0)
        ret = yaw_bus.start();
    if (ret == 0 && split_buses)
        ret = pitch_bus.start();
    if (ret == 0)
        ret = yaw.begin();
    if (ret == 0)
        ret = pitch.begin();
    LOG_INF("configure=%d split_buses=%d", ret, split_buses);
    if (ret < 0)
        return;

    communication::RemoteReceiver::Snapshot rc{}, operator_cache{};
    samples::control::RcControlAdapter controls;
    samples::control::SampleDiagnostics diagnostics;
    bool estop_latched = false;
    int yaw_error = 0, pitch_error = 0, enable_error = 0, yaw_commit = 0, pitch_commit = 0;
    auto previous_ms = k_uptime_get();
    std::uint64_t next_telemetry = 0;
    int last_send_error = 0;
    std::uint32_t telemetry_sequence = 0;
    std::uint64_t next_status = 0;
    for (;;) {
        (void)receiver.snapshot(operator_cache);
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const auto &operator_state = controls.update(operator_cache.remote, now);
        const auto exercise = diagnostics.update(now, operator_state.run_allowed,
                                                 !operator_state.fresh ||
                                                     operator_state.remote.left_switch == RcSwitch::Down);
        if (!exercise.input_paused)
            rc = operator_cache;
        const auto &remote = rc.remote;
        if (exercise.execution_paused && operator_state.run_allowed && !operator_state.clear_estop &&
            !board_config::emergencyStopRequested()) {
            k_sleep(K_MSEC(5));
            continue;
        }
        const float dt = (now - previous_ms) / 1000.0f;
        previous_ms = now;
        const bool fresh = remote.online && isFresh(remote.stamp, now, board_config::command_timeout_ms);
        const bool estop = board_config::emergencyStopRequested();
        if (estop && !estop_latched) {
            gimbal.disable();
            estop_latched = true;
        }
        if ((operator_state.clear_estop || board_config::takeEmergencyResetRequest()) && !estop) {
            gimbal.disable();
            estop_latched = false;
            controls.withdraw();
        }
        const bool requested = fresh && !estop_latched && operator_state.run_allowed &&
                               remote.left_switch == RcSwitch::Middle;
        const float yaw_rate = requested ? board_config::yaw_direction *
            samples::control::RcControlAdapter::normalize(remote.analog.right_x) *
            board_config::yaw.max_rate_rad_s : 0.0f;
        const float pitch_rate = requested ? board_config::pitch_direction *
            samples::control::RcControlAdapter::normalize(remote.analog.right_y) *
            board_config::pitch.max_rate_rad_s : 0.0f;
        if (!requested) {
            gimbal.disable();
            enable_error = yaw_error = pitch_error = 0;
        }
        else {
            enable_error = gimbal.enable();
            yaw_error = yaw.updateRate(yaw_rate, dt);
            pitch_error = pitch.updateRate(pitch_rate, dt);
        }
        yaw_commit = yaw_bus.commit().error;
        pitch_commit = split_buses ? pitch_bus.commit().error : 0;

        if (!exercise.status_paused && now >= next_telemetry) {
            next_telemetry = now + 20;
            const auto ys = yaw_drive.snapshot();
            const auto ps = pitch_drive.snapshot();
            const auto yt = yaw.telemetry();
            const auto pt = pitch.telemetry();
            const auto outputValid = [requested](const auto &t, const auto &snapshot) {
                return requested && t.output_valid && snapshot.feedback_fresh && snapshot.output_permitted &&
                       snapshot.state == motor::MotorState::Active &&
                       t.motor.enable_generation == snapshot.enable_generation &&
                       t.motor.reference_generation == snapshot.reference_generation;
            };
            const bool yaw_output = outputValid(yt, ys);
            const bool pitch_output = outputValid(pt, ps);
            // Keep all samples finite for plotting. Missing fields use zero; stale feedback retains
            // the last received value. The periodic console status reports freshness/output validity.
            const auto measurement = [](const auto &snapshot, auto field, float value) {
                return (snapshot.feedback.valid & field) && std::isfinite(value) ? value : 0.0f;
            };
            telemetry_sequence = (telemetry_sequence + 1) % 1000000;
            const float channels[] = {
                static_cast<float>(telemetry_sequence),
                requested ? 1.0f : 0.0f,
                static_cast<float>(yaw.targetAngleRad()),
                measurement(ys, motor::FeedbackPosition, ys.feedback.position_rad),
                static_cast<float>(pitch.targetAngleRad()),
                measurement(ps, motor::FeedbackPosition, ps.feedback.position_rad),
                yaw_rate,
                measurement(ys, motor::FeedbackVelocity, ys.feedback.velocity_rad_s),
                pitch_rate,
                measurement(ps, motor::FeedbackVelocity, ps.feedback.velocity_rad_s),
                yaw_output ? yt.effort_command : 0.0f,
                measurement(ys, motor::FeedbackCurrent, ys.feedback.current_a),
                pitch_output ? pt.effort_command : 0.0f,
                measurement(ps, motor::FeedbackTorque, ps.feedback.torque_nm),
                static_cast<float>(unsigned(ys.state)),
                static_cast<float>(unsigned(ps.state)),
            };
            constexpr auto channel_count = sizeof(channels) / sizeof(channels[0]);
            static_assert(channel_count <= VOFA_MAX_FLOATS);
            if (vofa_ret == 0) {
                const int send_ret = vofa_send(&vofa, channels, static_cast<std::uint8_t>(channel_count));
                if (send_ret < 0 && send_ret != last_send_error)
                    LOG_WRN("VOFA send failed: %d", send_ret);
                last_send_error = send_ret;
            }
            if (now >= next_status) {
                next_status = now + 1000;
                LOG_INF("VOFA seq=%u send=%d run=%d; fresh(y,p)=%d,%d ref=%d,%d output=%d,%d issue=%d,%d",
                        telemetry_sequence, vofa_ret < 0 ? vofa_ret : last_send_error, requested,
                        ys.feedback_fresh, ps.feedback_fresh, ys.position_reference_valid,
                        ps.position_reference_valid, yaw_output, pitch_output,
                        requested ? static_cast<int>(yt.issue) : -1,
                        requested ? static_cast<int>(pt.issue) : -1);
                LOG_INF("enable=%d update(y,p)=%d,%d commit=%d,%d", enable_error,
                        yaw_error, pitch_error, yaw_commit, pitch_commit);
            }
        }
        k_sleep(K_MSEC(5));
    }
}
} // namespace

K_THREAD_DEFINE(gimbal_thread, 6144, gimbalTask, nullptr, nullptr, nullptr, 4, 0, 0);
int main() {
    const int ret = receiver.start();
    if (ret < 0) {
        LOG_ERR("Remote receiver start failed: %d", ret);
        return ret;
    }
    LOG_INF("RC small yaw + pitch: configured=%d; check src/board_config.hpp", board_config::connections_configured);
    return 0;
}
