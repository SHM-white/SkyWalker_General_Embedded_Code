#include <cstdint>
#include <drivers/motor/can_bus.hpp>
#include <robotics/gimbal/gimbal_axis.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "board_config.hpp"
#include "../../common/rc_controls.hpp"
#include "../../common/sample_diagnostics.hpp"

LOG_MODULE_REGISTER(yaw_bench, LOG_LEVEL_INF);

int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;
    namespace input = skywalker::samples::control;
    static_assert(input::diagnostic_scenario == input::DiagnosticScenario::None ||
                  input::diagnostic_scenario == input::DiagnosticScenario::InputPause ||
                  input::diagnostic_scenario == input::DiagnosticScenario::ExecutionPause);
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma, input::receiverConfig());
    static motor::Motor drive(bench::motorHardware());
    static motor::CanBus bus(bench::can);
    static GimbalAxis yaw(drive, bench::motorConfig(), bench::yaw);
    int ret = remote.start();
    if (ret == 0) ret = bus.attach(drive);
    if (ret == 0) ret = bus.start();
    if (ret == 0) ret = yaw.begin();
    if (ret < 0) { LOG_ERR("configuration blocked: %d", ret); return ret; }
    LOG_INF("RC safe+center 0.5s then left Middle; right X yaw, right switch Middle/Up absolute 0/0.5 rad");
    communication::RemoteReceiver::Snapshot snapshot{};
    input::RcControlAdapter adapter;
    input::SampleDiagnostics diagnostics;
    AxisCommand command{};
    command.mode = GimbalMode::Hold;
    MessageStamp source{};
    std::uint64_t next_log = 0;
    int control_error = 0, commit_error = 0;
    auto previous = k_uptime_get();
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const float dt = (now - previous) / 1000.0f;
        previous = now;
        (void)remote.snapshot(snapshot);
        const auto &rc = adapter.update(snapshot.remote, now);
        const auto diagnostic = diagnostics.update(now, rc.run_allowed,
            !rc.fresh || rc.remote.left_switch == RcSwitch::Down);
        if (!diagnostic.input_paused && rc.fresh &&
            (!source.valid || sequenceAfter(rc.remote.stamp.sequence, source.sequence))) {
            source = rc.remote.stamp;
            command.rate_rad_s = -.3f * input::RcControlAdapter::normalize(rc.remote.analog.right_x);
            command.mode = command.rate_rad_s == 0 ? GimbalMode::Hold : GimbalMode::Rate;
            if (rc.remote.right_switch != RcSwitch::Down) {
                command.mode = GimbalMode::AbsoluteAngle;
                command.target_rad = rc.remote.right_switch == RcSwitch::Middle ? 0 : .5f;
            }
        }
        const bool requested = rc.run_allowed && isFresh(source, now, 100);
        if (!diagnostic.execution_paused || !requested || rc.clear_estop) {
            if (requested) {
                const int enabled = drive.enable();
                const int updated = yaw.update(command, SafetyAction::Active, dt);
                control_error = enabled < 0 ? enabled : updated;
            } else {
                control_error = drive.disable();
            }
            commit_error = bus.commit().error;
        }
        if (now >= next_log) {
            next_log = now + 100;
            const auto view_now = drive.snapshot();
            const auto telemetry = yaw.telemetry();
            LOG_INF("requested=%d state=%u mode=%u activation=%llu target=%.3f absolute=%.3f velocity=%.3f effort=%.3f output=%d fault=%u control=%d commit=%d rc=%d source=%u",
                requested, unsigned(view_now.state), unsigned(command.mode),
                static_cast<unsigned long long>(view_now.enable_generation), double(yaw.targetAngleRad()),
                double(view_now.feedback.absolute_position_rad), double(view_now.feedback.velocity_rad_s),
                double(telemetry.effort_command), telemetry.output_valid, unsigned(view_now.last_fault.reason), control_error, commit_error,
                rc.fresh, source.sequence);
        }
        k_sleep(K_MSEC(5));
    }
}
