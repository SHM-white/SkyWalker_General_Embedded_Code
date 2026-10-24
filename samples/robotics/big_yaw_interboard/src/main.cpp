#include "../../common/operator_controls.hpp"
#include <communication/interboard/uart_interboard_transport.hpp>
#include <core/clock.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_BIG_YAW_INTERBOARD_CHASSIS_ROLE)
#include "../../common/chassis_can.hpp"
#include <cerrno>
#include <drivers/motor/can_bus.hpp>
#include <robotics/vehicle/big_yaw_profile.hpp>
#if defined(CONFIG_BOARD_DM_MC02)
#include <zephyr/drivers/regulator.h>
#endif
#else
#include "../../common/rc_controls.hpp"
#endif

LOG_MODULE_REGISTER(big_yaw_interboard, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;

namespace {
#if defined(CONFIG_BIG_YAW_INTERBOARD_CHASSIS_ROLE)
constexpr auto role = BoardRole::ChassisController;
#else
constexpr auto role = BoardRole::GimbalController;
// Output joint velocity, rad/s; retains the standalone big_yaw bench limit.
constexpr float manual_rate_max_rad_s = 0.1f;
#endif
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::UartInterBoardTransport transport(DEVICE_DT_GET(DT_ALIAS(interboard_uart)), link_dma);
communication::InterBoardEndpoint endpoint(transport, {role});

void communicationTask(void *, void *, void *) {
    for (;;) {
        endpoint.poll(k_uptime_get());
        k_sleep(K_MSEC(1));
    }
}
} // namespace

K_THREAD_DEFINE(big_yaw_link_worker, 4096, communicationTask, nullptr, nullptr, nullptr, 6, K_FP_REGS, 0);

int main() {
    LOG_INF("manual bigYaw role=%u UART1=460800; no IMU", unsigned(role));
#if defined(CONFIG_BIG_YAW_INTERBOARD_CHASSIS_ROLE)
    static motor::Motor drive(vehicle::bigYawHardware());
    static motor::CanBus bus(samples::chassis::big_yaw_can);
    static BigYawExecutor axis(drive, vehicle::bigYawMotorConfig(), vehicle::bigYawExecutionConfig());
    int setup_error = -ENODEV;
    if (vehicle::connections_confirmed) {
        setup_error = bus.attach(drive);
        if (setup_error == 0)
            setup_error = bus.start();
#if defined(CONFIG_BOARD_DM_MC02)
        if (setup_error == 0)
            setup_error = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
#endif
        if (setup_error == 0)
            setup_error = axis.begin();
    }
    const bool started = setup_error == 0;
    LOG_INF("motor bus=%s id=%u master=%u confirmed=%d setup=%d",
            samples::chassis::big_yaw_can->name, unsigned(vehicle::big_yaw.id),
            unsigned(vehicle::big_yaw.master_id), vehicle::connections_confirmed, setup_error);
    samples::control::OperatorControlConsumer operator_controls;
    std::uint64_t next_log_ms = 0;
    int commit_error = 0;
    for (;;) {
        const auto now = core::monotonicTimeUs(), ms = now / 1000;
        const auto rx = endpoint.snapshot();
        const auto controls = operator_controls.update(rx, ms);
        BigYawExecutionInputs input{};
        input.request = rx.big_yaw_request;
        if (!controls.run_allowed) {
            input.request.mode = BigYawMode::Disabled;
            input.request.target_rate_rad_s = 0;
        }
        input.local_boot_id = rx.local_boot_id;
        input.peer_online = rx.online;
        input.transport_ready = started;
        input.emergency_stop = controls.emergency_stop;
        input.clear_estop = controls.clear_estop;
        if (started) {
            axis.update(input, now);
            commit_error = bus.commit().error;
        }
        else {
            axis.suspend(now, WaitReason::Configuration, setup_error, true);
        }
        // Feedback comes only from the real execution owner and motor snapshot.
        endpoint.setBigYawFeedback(axis.feedback());
        if (ms >= next_log_ms) {
            next_log_ms = ms + 200;
            const auto state = axis.status();
            const auto feedback = axis.feedback();
            LOG_INF("online=%d operator=%d run=%d source=%u target=%f actual=%f ready=%d valid=%d state=%u reason=%u error=%d commit=%d link=%d rejected=%u",
                    rx.online, rx.operator_control_valid, controls.run_allowed,
                    input.request.source_sequence, double(input.request.target_rate_rad_s),
                    double(feedback.actual_rate_rad_s), feedback.ready, feedback.valid,
                    unsigned(state.state), unsigned(state.reason), state.error, commit_error,
                    rx.error, rx.rejected_frames);
        }
        k_sleep(K_MSEC(5));
    }
#else
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                                samples::control::receiverConfig());
    const int ret = remote.start();
    if (ret < 0) {
        LOG_ERR("remote start failed: %d", ret);
        return ret;
    }
    LOG_INF("RC both Down + centered 500ms, then left Middle; wheel +/-0.1 rad/s; left Down stops");
    samples::control::RcControlAdapter controls;
    samples::control::OperatorControlPublisher operator_controls;
    communication::RemoteReceiver::Snapshot remote_cache{};
    std::uint64_t next_log_ms = 0;
    for (;;) {
        const auto ms = static_cast<std::uint64_t>(k_uptime_get());
        (void)remote.snapshot(remote_cache);
        const auto &rc = controls.update(remote_cache.remote, ms);
        // A link loss or peer reboot withdraws the RC gate before this cycle's
        // velocity is generated. Motor recovery alone never withdraws intent.
        if (operator_controls.publish(endpoint, rc.run_allowed, false, rc.remote.stamp,
                                      rc.clear_event_id, rc.clear_stamp, ms))
            controls.withdraw();
        const auto rx = endpoint.snapshot();
        const bool run = rc.fresh && rc.run_allowed && rx.online;
        BigYawRequest request{};
        // FollowCenter is the existing v4 velocity-request wire mode; this
        // entry generates manual velocity and does not run a centering loop.
        request.mode = run ? BigYawMode::FollowCenter : BigYawMode::Disabled;
        request.target_rate_rad_s = run ? manual_rate_max_rad_s *
            samples::control::RcControlAdapter::normalize(rc.remote.analog.wheel) : 0;
        request.source_sequence = rc.remote.stamp.sequence;
        request.stamp = rc.remote.stamp;
        request.receiver_boot_id = rx.peer.sender_boot_id;
        // Explicit manual bench permission, not referee or IMU authority.
        request.permission = {rc.fresh, run, rc.remote.stamp};
        // Keep the original RC timestamp: repeated snapshots and wire retries
        // must not renew source/command/permission freshness.
        endpoint.submitBigYaw(request);
        if (ms >= next_log_ms) {
            next_log_ms = ms + 200;
            const auto &feedback = rx.big_yaw_feedback;
            LOG_INF("online=%d rc=%d run=%d source=%u target=%f peerActual=%f peerReady=%d peerValid=%d link=%d rejected=%u",
                    rx.online, rc.fresh, run, request.source_sequence,
                    double(request.target_rate_rad_s), double(feedback.actual_rate_rad_s),
                    feedback.ready, feedback.valid, rx.error, rx.rejected_frames);
        }
        k_sleep(K_MSEC(5));
    }
#endif
}
