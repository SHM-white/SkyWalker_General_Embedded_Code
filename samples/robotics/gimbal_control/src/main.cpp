#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>

#include <communication/async_uart.hpp>
#include <communication/remote/remote_service.hpp>
#include <control/position_motor.hpp>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>
#include <latest.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board_config.hpp"

LOG_MODULE_REGISTER(gimbal_rc, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;

namespace {
Latest<RemoteState> remote_state;

void remoteTask(void *, void *, void *) {
    static communication::AsyncUart::DmaBuffers dma_buffers __nocache;
    static communication::AsyncUart uart(board_config::remote_uart, dma_buffers);
    communication::RemoteService service({}, {});
    int ret = uart.init();
    LOG_INF("remote UART init: %d", ret);
    if (ret < 0)
        return; // No valid snapshot is published; both axes stay disabled.
    for (;;) {
        const auto now = k_uptime_get();
        uart.service(now);
        communication::AsyncUart::RxChunk chunk{};
        for (unsigned budget = 0; budget < 8; ++budget) {
            ret = uart.read(chunk);
            if (ret == -EOVERFLOW) {
                service.discardPartial();
                continue;
            }
            if (ret < 0)
                break;
            service.processBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
        }
        RemoteState state{};
        service.snapshot(now, state);
        remote_state.put(state);
        k_sleep(K_MSEC(1));
    }
}

float normalizeStick(std::int16_t raw) {
    const float x = std::clamp(float(raw) / 660.0f, -1.0f, 1.0f);
    constexpr float deadband = 0.03f;
    return std::fabs(x) <= deadband ? 0.0f : std::copysign((std::fabs(x) - deadband) / (1.0f - deadband), x);
}

struct Axis {
    motor::Motor &drive;
    control::PositionMotor axis_motor;
    YawGimbal controller;
    const YawGimbalConfig config;
    std::uint64_t ready_ms = 0;
    bool was_ready = false;
    bool reference_seeded = false;

    Axis(motor::Motor &endpoint, const control::PositionMotor::Config &motor_config,
         const YawGimbalConfig &axis_config)
        : drive(endpoint), axis_motor(endpoint, motor_config),
          controller(endpoint, axis_motor, axis_config), config(axis_config) {}

    bool feedbackHealthy(bool require_reference = true) const {
        const auto view = drive.snapshot();
        const auto &feedback = view.feedback;
        if (!view.feedback_fresh)
            return false;
        if (config.topology == YawTopology::Limited && !require_reference) {
            return ((feedback.valid & motor::FeedbackAbsolutePosition) &&
                    std::isfinite(feedback.absolute_position_rad)) ||
                   (view.native_position_valid && std::isfinite(view.native_position_rad));
        }
        const std::uint32_t required = config.topology == YawTopology::Continuous
                                           ? motor::FeedbackAbsolutePosition : motor::FeedbackPosition;
        if ((feedback.valid & required) == 0 ||
            (config.topology == YawTopology::Limited && !view.position_reference_valid))
            return false;
        const float angle = config.topology == YawTopology::Continuous
                                ? feedback.absolute_position_rad : feedback.position_rad;
        return std::isfinite(angle) &&
               (config.topology != YawTopology::Limited ||
                (angle >= config.min_angle_rad && angle <= config.max_angle_rad));
    }

    bool ready(std::uint64_t now) {
        bool current = drive.ready() && feedbackHealthy(false);
        if (current && config.topology == YawTopology::Limited) {
            const auto view = drive.snapshot();
            if (!reference_seeded || !view.position_reference_valid) {
                const bool absolute = (view.feedback.valid & motor::FeedbackAbsolutePosition) != 0;
                const double known = absolute ? view.feedback.absolute_position_rad
                                              : view.native_position_rad;
                current = (absolute || view.native_position_valid) &&
                          drive.reseedPosition(known) == 0;
                if (current)
                    reference_seeded = true;
            }
        }
        current = current && feedbackHealthy();
        if (current && !was_ready)
            ready_ms = now;
        was_ready = current;
        return current;
    }

    int update(float rate, const MessageStamp &stamp, float dt) {
        GimbalCommand command{};
        command.mode = GimbalMode::Rate;
        command.source = ControlSource::Remote;
        command.stamp = stamp;
        // Both single-axis controllers consume yaw_rate_rad_s.
        command.yaw_rate_rad_s = rate;
        return controller.update(command, SafetyAction::Active, dt);
    }
};

void gimbalTask(void *, void *, void *) {
    static motor::Motor yaw_drive(board_config::yawHardware());
    static motor::Motor pitch_drive(board_config::pitchHardware());
    static motor::Group gimbal(yaw_drive, pitch_drive);
    static motor::CanBus yaw_bus(board_config::yaw_can);
    static motor::CanBus pitch_bus(board_config::pitch_can);
    static Axis yaw(yaw_drive, board_config::yawMotorConfig(), board_config::yaw);
    static Axis pitch(pitch_drive, board_config::pitchMotorConfig(), board_config::pitch);

    if (!board_config::connections_configured) {
        LOG_ERR("hardware template disabled; check board_config.hpp");
        return;
    }
    const bool split_buses = board_config::yaw_can != board_config::pitch_can;
    int ret = split_buses ? yaw_bus.attach(yaw_drive)
                          : yaw_bus.attach(yaw_drive, pitch_drive);
    if (ret == 0 && split_buses)
        ret = pitch_bus.attach(pitch_drive);
    if (ret == 0)
        ret = yaw_bus.start();
    if (ret == 0 && split_buses)
        ret = pitch_bus.start();
    if (ret == 0)
        ret = yaw.controller.begin();
    if (ret == 0)
        ret = pitch.controller.begin();
    LOG_INF("configure=%d split_buses=%d", ret, split_buses);
    if (ret < 0)
        return;

    RemoteState remote{};
    bool estop_latched = false, rearm_allowed = false, enable_issued = false;
    auto previous_ms = k_uptime_get();
    std::uint64_t next_log = 0;
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const float dt = (now - previous_ms) / 1000.0f;
        previous_ms = now;
        remote_state.get(remote); // Retain the snapshot and its original age on contention.
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

        const bool yaw_ready = yaw.ready(now);
        const bool pitch_ready = pitch.ready(now);
        const bool feedback_ok = yaw.feedbackHealthy() && pitch.feedbackHealthy();
        const bool timing_ok = dt > 0.0f && dt <= 0.02f;
        const bool safe_switch = remote.left_switch == RcSwitch::Up ||
                                 remote.left_switch == RcSwitch::Down;
        const bool new_command = remote.stamp.timestamp_ms > std::max(yaw.ready_ms, pitch.ready_ms);
        const auto group = gimbal.status();
        if (enable_issued && !group.active && !group.enable_pending) {
            enable_issued = false;
            rearm_allowed = false; // A driver fault needs a new safe-switch cycle.
        }
        if (!fresh || !feedback_ok || !timing_ok || estop_latched ||
            remote.left_switch == RcSwitch::Unknown)
            rearm_allowed = false;
        else if (safe_switch && yaw_ready && pitch_ready && new_command)
            rearm_allowed = true;

        const bool requested = fresh && feedback_ok && timing_ok && !estop_latched &&
                               rearm_allowed && new_command &&
                               remote.left_switch == RcSwitch::Middle;
        if (!requested) {
            if (group.active || group.enable_pending) {
                gimbal.disable();
                enable_issued = false;
            }
        }
        else if (!enable_issued && gimbal.ready()) {
            ret = yaw.controller.reset();
            if (ret == 0)
                ret = pitch.controller.reset();
            if (ret == 0)
                ret = gimbal.enable();
            enable_issued = ret == 0;
            if (ret < 0) {
                rearm_allowed = false;
                LOG_ERR("group enable failed: %d", ret);
            }
        }
        else if (gimbal.active()) {
            const float yaw_rate = board_config::yaw_direction *
                                   normalizeStick(remote.analog.right_x) *
                                   board_config::yaw.max_rate_rad_s;
            const float pitch_rate = board_config::pitch_direction *
                                     normalizeStick(remote.analog.right_y) *
                                     board_config::pitch.max_rate_rad_s;
            const int yr = yaw.update(yaw_rate, remote.stamp, dt);
            const int pr = yr == 0 ? pitch.update(pitch_rate, remote.stamp, dt) : 0;
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

        if (now >= next_log) {
            next_log = now + 1000;
            const auto status = gimbal.status();
            const auto ys = yaw_drive.snapshot();
            const auto ps = pitch_drive.snapshot();
            LOG_INF("rc=%d switch=%u rearm=%d estop=%d group=%d/%d yaw=%u pitch=%u fault=%u bus=%d/%d",
                    fresh, unsigned(remote.left_switch), rearm_allowed, estop_latched,
                    status.active, status.enable_pending, unsigned(ys.state), unsigned(ps.state),
                    unsigned(status.last_fault.reason), yaw_bus.status().last_error,
                    split_buses ? pitch_bus.status().last_error : 0);
        }
        k_sleep(K_MSEC(5));
    }
}
} // namespace

K_THREAD_DEFINE(remote_thread, 3072, remoteTask, nullptr, nullptr, nullptr, 6, 0, 0);
K_THREAD_DEFINE(gimbal_thread, 6144, gimbalTask, nullptr, nullptr, nullptr, 4, 0, 0);
int main() {
    LOG_INF("RC small yaw + pitch: configured=%d; check src/board_config.hpp",
            board_config::connections_configured);
    return 0;
}
