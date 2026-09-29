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

#define VOFA_UART_NODE DT_ALIAS(telemetry_uart)

#if !DT_NODE_HAS_STATUS(VOFA_UART_NODE, okay)
#error "A ready telemetry-uart alias is required for VOFA"
#endif

LOG_MODULE_REGISTER(gimbal_rc, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;

namespace {
static communication::AsyncUart::DmaBuffers dma_buffers __nocache;
communication::RemoteReceiver receiver(board_config::remote_uart, dma_buffers, {});

float normalizeStick(std::int16_t raw) {
    const float x = std::clamp(float(raw) / 660.0f, -1.0f, 1.0f);
    constexpr float deadband = 0.03f;
    return std::fabs(x) <= deadband ? 0.0f : std::copysign((std::fabs(x) - deadband) / (1.0f - deadband), x);
}

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

    communication::RemoteReceiver::Snapshot rc{};
    bool estop_latched = false, rearm_allowed = false, enable_issued = false;
    auto previous_ms = k_uptime_get();
    std::uint64_t next_telemetry = 0;
    for (;;) {
        receiver.snapshot(rc);
        const auto &remote = rc.remote;
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const float dt = (now - previous_ms) / 1000.0f;
        previous_ms = now;
        const bool fresh = remote.online && isFresh(remote.stamp, now, board_config::command_timeout_ms);
        const bool estop = board_config::emergencyStopRequested();
        if (estop && !estop_latched) {
            gimbal.disable();
            estop_latched = true;
            enable_issued = false;
            rearm_allowed = false;
        }
        if (board_config::takeEmergencyResetRequest() && !estop) {
            gimbal.disable();
            ret = gimbal.clearFault();
            if (ret < 0)
                LOG_ERR("group clear fault failed: %d", ret);
            estop_latched = false;
            enable_issued = false;
            rearm_allowed = false;
        }

        const auto yaw_status = yaw.poll(now);
        const auto pitch_status = pitch.poll(now);
        const bool yaw_ready = yaw_status.ready_for_enable;
        const bool pitch_ready = pitch_status.ready_for_enable;
        const bool feedback_ok = yaw_status.feedback_healthy && pitch_status.feedback_healthy;
        const bool timing_ok = dt > 0.0f && dt <= 0.02f;
        const bool safe_switch = remote.left_switch == RcSwitch::Up || remote.left_switch == RcSwitch::Down;
        const bool new_command = remote.stamp.timestamp_ms >
                                 std::max(yaw_status.ready_since_ms, pitch_status.ready_since_ms);
        const auto group = gimbal.status();
        if (enable_issued && !group.active && !group.enable_pending) {
            enable_issued = false;
            rearm_allowed = false; // A driver fault needs a new safe-switch cycle.
        }
        if (!fresh || !feedback_ok || !timing_ok || estop_latched || remote.left_switch == RcSwitch::Unknown)
            rearm_allowed = false;
        else if (safe_switch && yaw_ready && pitch_ready && new_command)
            rearm_allowed = true;

        const bool requested = fresh && feedback_ok && timing_ok && !estop_latched && rearm_allowed && new_command &&
                               remote.left_switch == RcSwitch::Middle;
        if (!requested) {
            if (group.active || group.enable_pending) {
                gimbal.disable();
                enable_issued = false;
            }
        }
        else if (!enable_issued && gimbal.ready()) {
            ret = yaw.reset();
            if (ret == 0)
                ret = pitch.reset();
            if (ret == 0)
                ret = gimbal.enable();
            enable_issued = ret == 0;
            if (ret < 0) {
                rearm_allowed = false;
                LOG_ERR("group enable failed: %d", ret);
            }
        }
        else if (gimbal.active()) {
            const float yaw_rate = board_config::yaw_direction * normalizeStick(remote.analog.right_x) *
                                   board_config::yaw.max_rate_rad_s;
            const float pitch_rate = board_config::pitch_direction * normalizeStick(remote.analog.right_y) *
                                     board_config::pitch.max_rate_rad_s;
            const int yr = yaw.updateRate(yaw_rate, dt);
            const int pr = yr == 0 ? pitch.updateRate(pitch_rate, dt) : 0;
            int submit = 0;
            if (yr == 0 && pr == 0)
                submit = yaw_bus.commit().error;
            if (yr == 0 && pr == 0 && submit == 0 && split_buses)
                submit = pitch_bus.commit().error;
            if (yr < 0 || pr < 0 || submit < 0) {
                gimbal.disable();
                enable_issued = false;
                rearm_allowed = false;
                LOG_ERR("axis update failed: yaw=%d pitch=%d commit=%d", yr, pr, submit);
            }
        }

        if (now >= next_telemetry) {
            next_telemetry = now + 1000;
            const auto status = gimbal.status();
            const auto ys = yaw_drive.snapshot();
            const auto ps = pitch_drive.snapshot();
            const float channels[] = {
                fresh ? 1.0f : 0.0f,
                static_cast<float>(unsigned(remote.left_switch)),
                rearm_allowed ? 1.0f : 0.0f,
                estop_latched ? 1.0f : 0.0f,
                status.active ? 1.0f : 0.0f,
                status.enable_pending ? 1.0f : 0.0f,
                static_cast<float>(unsigned(ys.state)),
                static_cast<float>(unsigned(ps.state)),
                static_cast<float>(unsigned(status.last_fault.reason)),
                static_cast<float>(yaw_bus.status().last_error),
                static_cast<float>(split_buses ? pitch_bus.status().last_error : 0),
            };
            constexpr auto channel_count = sizeof(channels) / sizeof(channels[0]);
            static_assert(channel_count <= VOFA_MAX_FLOATS);
            if (vofa_ret == 0) {
                const int send_ret = vofa_send(&vofa, channels, static_cast<std::uint8_t>(channel_count));
                if (send_ret < 0)
                    LOG_WRN("VOFA send failed: %d", send_ret);
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
