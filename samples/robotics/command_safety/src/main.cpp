#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <communication/remote/remote_receiver.hpp>
#include <robotics/command/manual_command_mapper.hpp>
#include <robotics/command/command_manager.hpp>
#include <robotics/safety/global_safety_manager.hpp>
#include "board_config.hpp"
LOG_MODULE_REGISTER(command_bench, LOG_LEVEL_INF);
int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;
    static communication::AsyncUart::DmaBuffers dma_buffers __nocache;
    static communication::RemoteReceiver receiver(
        bench::remote_uart, dma_buffers, {bench::decoder, bench::remote});
    communication::RemoteReceiver::Snapshot rc{};
    ManualCommandMapper mapper({});
    CommandManager manager({});
    GlobalSafetyManager::Config safety_config{};
    safety_config.require_referee_for_motion = false;
    safety_config.permission_timeout_ms = 300;
    safety_config.chassis_heartbeat_timeout_ms = 100;
    safety_config.chassis_feedback_timeout_ms = 100;
    GlobalSafetyManager safety(safety_config);
    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    const int ret = receiver.start();
    if (ret < 0) {
        LOG_ERR("Remote receiver start failed: %d", ret);
        return ret;
    }
    LOG_INF("RC -> intent -> safety -> command receiver start=%d. Bench mode; no motors. "
            "Console ! estop, r explicit reset", ret);
    LOG_INF("Simulated chassis Ready: h toggles heartbeat, f toggles feedback");
    std::uint64_t next_log = 0;
    bool estop = false;
    bool heartbeat_enabled = true, feedback_enabled = true;
    MessageStamp heartbeat{}, feedback{};
    for (;;) {
        receiver.snapshot(rc);
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        unsigned char key;
        if (uart_poll_in(console, &key) == 0) {
            if (key == '!')
                estop = true;
            if (key == 'r') {
                estop = false;
                safety.clearEmergencyStop(true);
            }
            if (key == 'h')
                heartbeat_enabled = !heartbeat_enabled;
            if (key == 'f')
                feedback_enabled = !feedback_enabled;
        }
        const auto &r = rc.remote;
        OperatorIntent intent{};
        mapper.map(r, intent);
        GlobalSafetyInputs input{};
        input.now_ms = now;
        input.command_source_fresh = r.online;
        input.operator_motion_enabled = r.stamp.valid && r.left_switch == RcSwitch::Middle;
        input.emergency_stop_requested = estop;
        // This motor-free bench explicitly simulates a peer; applications use actual UART snapshots.
        if (heartbeat_enabled)
            heartbeat = {now, heartbeat.sequence + 1, true};
        if (feedback_enabled)
            feedback = {now, feedback.sequence + 1, true};
        input.chassis_heartbeat_stamp = heartbeat;
        input.chassis_feedback_stamp = feedback;
        input.chassis_execution_state = ExecutionState::Ready;
        GlobalSafetyDecision decision{};
        safety.evaluate(input, decision);
        RobotCommand command{};
        manager.step(intent, decision, now, command);
        if (now >= next_log) {
            next_log = now + 100;
            LOG_INF(
                "rc=%d mode=%u safety=%u reason=%x actions=%u/%u chassis=%.2f/%.2f/%.2f yaw_rate=%.2f yaw_mode=%u seq=%u",
                r.online, unsigned(intent.mode), unsigned(decision.state), decision.active_reasons,
                unsigned(decision.gimbal), unsigned(decision.chassis), double(command.chassis.vx_m_s),
                double(command.chassis.vy_m_s), double(command.chassis.wz_rad_s), double(command.gimbal.yaw_rate_rad_s),
                unsigned(command.gimbal.mode), command.stamp.sequence);
        }
        k_sleep(K_MSEC(10));
    }
}
