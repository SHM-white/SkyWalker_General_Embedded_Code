#include "../../common/chassis_can.hpp"
#include "../../common/rc_controls.hpp"
#include "../../common/sample_diagnostics.hpp"
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/vehicle/big_yaw_profile.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_BOARD_DM_MC02)
#include <zephyr/drivers/regulator.h>
#endif
LOG_MODULE_REGISTER(big_yaw_bench, LOG_LEVEL_INF);
using namespace skywalker;
using namespace skywalker::robotics;
int main() {
    namespace input = skywalker::samples::control;
    static_assert(input::diagnostic_scenario == input::DiagnosticScenario::None ||
                  input::diagnostic_scenario == input::DiagnosticScenario::InputPause ||
                  input::diagnostic_scenario == input::DiagnosticScenario::ExecutionPause ||
                  input::diagnostic_scenario == input::DiagnosticScenario::StatusPause);
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                                input::receiverConfig());
    static motor::Motor drive(vehicle::bigYawHardware());
    static motor::CanBus bus(samples::chassis::big_yaw_can);
    static BigYawExecutor axis(drive, vehicle::bigYawMotorConfig(), vehicle::bigYawExecutionConfig());
    const int remote_error = remote.start();
    if (remote_error < 0)
        return remote_error;
    bool started = false;
    int ret = 0;
    if (vehicle::connections_confirmed) {
        ret = bus.attach(drive);
        if (ret == 0)
            ret = bus.start();
#if defined(CONFIG_BOARD_DM_MC02)
        if (ret == 0)
            ret = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
#endif
        if (ret == 0)
            ret = axis.begin();
        started = ret == 0;
    }
    LOG_INF("RC safe+center 0.5s then left Middle; wheel +/-0.1 rad/s; confirmed=%d setup=%d",
            vehicle::connections_confirmed, ret);
    communication::RemoteReceiver::Snapshot snapshot{};
    input::RcControlAdapter adapter;
    input::SampleDiagnostics diagnostics;
    std::uint64_t next_log = 0;
    BigYawRequest request{};
    RunStatus published{};
    int commit_error = 0;
    for (;;) {
        const auto now = core::monotonicTimeUs(), ms = now / 1000;
        (void)remote.snapshot(snapshot);
        const auto &rc = adapter.update(snapshot.remote, ms);
        const auto diagnostic = diagnostics.update(ms, rc.run_allowed,
                                                   !rc.fresh || rc.remote.left_switch == RcSwitch::Down);
        if (!diagnostic.input_paused && rc.fresh &&
            (!request.stamp.valid || sequenceAfter(rc.remote.stamp.sequence, request.stamp.sequence))) {
            request.mode = rc.run_allowed ? BigYawMode::FollowCenter : BigYawMode::Disabled;
            request.target_rate_rad_s = .1f * input::RcControlAdapter::normalize(rc.remote.analog.wheel);
            request.stamp = rc.remote.stamp;
            request.source_sequence = rc.remote.stamp.sequence;
            request.receiver_boot_id = 1;
            request.permission = {rc.fresh, vehicle::connections_confirmed && rc.run_allowed, rc.remote.stamp};
        }
        if (!rc.run_allowed) {
            request.mode = BigYawMode::Disabled;
            request.target_rate_rad_s = 0;
            request.permission.enabled = false;
        }
        if (!diagnostic.execution_paused || !rc.run_allowed || rc.clear_estop) {
            if (started) {
                BigYawExecutionInputs in{};
                in.request = request;
                in.local_boot_id = 1;
                in.peer_online = true;
                in.transport_ready = started;
                in.clear_estop = rc.clear_estop;
                axis.update(in, now);
                commit_error = bus.commit().error;
            }
            else
                axis.suspend(now, WaitReason::Configuration, ret ? ret : -ENODEV, true);
            if (!diagnostic.status_paused)
                published = axis.status();
        }
        if (ms >= next_log) {
            next_log = ms + 200;
            const auto f = axis.feedback();
            LOG_INF(
                "requested=%d run=%u wait=%u ready=%d active=%d target=%f actual=%f status_seq=%u fresh=%d error=%d commit=%d rc=%d",
                published.requested, unsigned(published.state), unsigned(published.reason), published.ready, f.armed,
                double(request.target_rate_rad_s), double(f.actual_rate_rad_s), published.stamp.sequence,
                isFresh(published.stamp, ms, 100), published.error, commit_error, rc.fresh);
        }
        k_sleep(K_MSEC(5));
    }
}
