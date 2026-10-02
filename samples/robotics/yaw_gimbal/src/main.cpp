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
    bool enable_issued = false;
    AxisCommand command{};
    command.mode = GimbalMode::Hold;
    MessageStamp source{};
    std::uint64_t next_log = 0;
    auto previous = k_uptime_get();
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        const float dt = (now - previous) / 1000.0f;
        previous = now;
        (void)remote.snapshot(snapshot);
        const auto &rc = adapter.update(snapshot.remote, now);
        const auto diagnostic = diagnostics.update(now, drive.active(),
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
        bool requested = rc.run_allowed && isFresh(source, now, 100);
        if (rc.run_allowed && !requested) adapter.withdraw();
        if (rc.clear_fault && drive.snapshot().state == motor::MotorState::Fault) {
            const int clear = drive.clearFault();
            if (clear < 0) LOG_ERR("clear fault: %d", clear);
        }
        const auto axis_status = yaw.poll(now);
        const auto view = drive.snapshot();
        if (!requested) {
            if (view.state == motor::MotorState::Active || view.state == motor::MotorState::Enabling) {
                ret = drive.disable();
                if (ret < 0) LOG_ERR("disable: %d", ret);
            }
            enable_issued = false;
        } else if (enable_issued && view.state != motor::MotorState::Active &&
                   view.state != motor::MotorState::Enabling) {
            adapter.withdraw();
            requested = enable_issued = false;
        }
        if (requested && !enable_issued && axis_status.ready_for_enable) {
            ret = yaw.reset();
            if (ret == 0) ret = drive.enable();
            if (ret == 0) enable_issued = true;
            else { adapter.withdraw(); requested = false; LOG_ERR("enable: %d", ret); }
        }
        if (requested && drive.active() && !diagnostic.execution_paused) {
            ret = yaw.update(command, SafetyAction::Active, dt);
            if (ret == 0) ret = bus.commit().error;
            if (ret < 0) {
                (void)drive.disable();
                adapter.withdraw();
                enable_issued = false;
                LOG_ERR("control update: %d", ret);
            }
        }
        if (now >= next_log) {
            next_log = now + 100;
            const auto view_now = drive.snapshot();
            const auto telemetry = yaw.telemetry();
            LOG_INF("state=%u mode=%u gen=%llu target=%.3f absolute=%.3f velocity=%.3f effort=%.3f fault=%u err=%d rc=%d source=%u",
                unsigned(view_now.state), unsigned(command.mode),
                static_cast<unsigned long long>(view_now.enable_generation), double(yaw.targetAngleRad()),
                double(view_now.feedback.absolute_position_rad), double(view_now.feedback.velocity_rad_s),
                double(telemetry.effort_command), unsigned(view_now.last_fault.reason), bus.status().last_error,
                rc.fresh, source.sequence);
        }
        k_sleep(K_MSEC(5));
    }
}
