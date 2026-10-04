#include <algorithm>
#include <cerrno>
#include <cmath>
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>
#include <lib/vofa/vofa.h>
#include <robotics/execution/recovery_gate.hpp>
#include <robotics/swerve/swerve_kinematics.hpp>
#include "../../common/rc_controls.hpp"
#include "../../common/sample_diagnostics.hpp"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include "board_config.hpp"
LOG_MODULE_REGISTER(swerve_bench, LOG_LEVEL_INF);

static_assert(DT_NODE_HAS_COMPAT(DT_ALIAS(telemetry_uart), zephyr_cdc_acm_uart),
    "Swerve VOFA telemetry requires a USB CDC ACM telemetry-uart");

namespace {
const char *waitReasonName(skywalker::robotics::WaitReason reason) {
    using skywalker::robotics::WaitReason;
    switch (reason) {
    case WaitReason::None: return "none";
    case WaitReason::Command: return "command";
    case WaitReason::Transport: return "transport";
    case WaitReason::Feedback: return "feedback";
    case WaitReason::Reference: return "reference";
    case WaitReason::Configuration: return "configuration";
    case WaitReason::Drive: return "drive";
    case WaitReason::Power: return "power";
    case WaitReason::Cycle: return "cycle";
    }
    return "unknown";
}

void logCanState(const char *name, const device *dev, const skywalker::motor::BusStatus &bus) {
    can_state state = CAN_STATE_STOPPED;
    can_bus_err_cnt counts{};
    const int error = can_get_state(dev, &state, &counts);
    LOG_INF("%s bus=%u hw=%u query_err=%d TEC=%u REC=%u last_err=%d tx_err=%d tx_ms=%llu bit0=%u bit1=%u stuff=%u crc=%u form=%u ack=%u",
        name, unsigned(bus.state), unsigned(state), error, unsigned(counts.tx_err_cnt),
        unsigned(counts.rx_err_cnt), bus.last_error, bus.last_tx.error,
        static_cast<unsigned long long>(bus.last_tx.completed_ms),
        can_stats_get_bit0_errors(dev), can_stats_get_bit1_errors(dev),
        can_stats_get_stuff_errors(dev), can_stats_get_crc_errors(dev),
        can_stats_get_form_errors(dev), can_stats_get_ack_errors(dev));
    const auto &fault = bus.last_recovery;
    if (fault.count != 0) {
        LOG_INF("%s pre_restart count=%llu ms=%llu reason=%u err=%d hw=%u query_err=%d TEC=%u REC=%u stats=%d bit0=%u bit1=%u stuff=%u crc=%u form=%u ack=%u",
            name, static_cast<unsigned long long>(fault.count),
            static_cast<unsigned long long>(fault.occurred_ms), unsigned(fault.reason), fault.error,
            unsigned(fault.controller_state), fault.query_error, unsigned(fault.error_counts.tx_err_cnt),
            unsigned(fault.error_counts.rx_err_cnt), fault.stats_valid, fault.bit0, fault.bit1,
            fault.stuff, fault.crc, fault.form, fault.ack);
        LOG_INF("%s pre_restart_tx valid=%d id=0x%x purpose=%u seq=%llu err=%d ms=%llu",
            name, fault.last_tx.valid, unsigned(fault.last_tx.can_id), unsigned(fault.last_tx.purpose),
            static_cast<unsigned long long>(fault.last_tx.sequence), fault.last_tx.error,
            static_cast<unsigned long long>(fault.last_tx.completed_ms));
    }
}
int configureCanTiming(const char *name, const device *dev, std::uint32_t bitrate,
                       std::uint16_t sample_point) {
    if (!device_is_ready(dev)) return -ENODEV;
    std::uint32_t clock_hz = 0;
    can_timing timing{};
    int ret = can_get_core_clock(dev, &clock_hz);
    constexpr std::uint32_t expected_clock_hz = DT_PROP(DT_NODELABEL(clk_hse), clock_frequency);
    if (ret == 0 && clock_hz != expected_clock_hz) {
        LOG_ERR("%s shared CAN clock=%u; expected HSE / 1=%u", name, clock_hz, expected_clock_hz);
        return -EINVAL;
    }
    if (ret == 0) ret = can_calc_timing(dev, &timing, bitrate, sample_point);
    if (ret >= 0) {
        // Use the full legal phase-segment correction window. At 24 MHz,
        // 1 Mbit/s and 87.5%, SJW=3 tq (125 ns); the default would be 1 tq.
        timing.sjw = std::min({timing.phase_seg1, timing.phase_seg2, can_get_timing_max(dev)->sjw});
        ret = can_set_timing(dev, &timing);
    }
    if (ret < 0) {
        LOG_ERR("%s timing configuration failed: %d", name, ret);
        return ret;
    }
    const auto quanta = 1U + timing.prop_seg + timing.phase_seg1 + timing.phase_seg2;
    LOG_INF("%s clock=%u bitrate=%u sample_permille=%u prescaler=%u seg1=%u seg2=%u sjw=%u",
        name, clock_hz, clock_hz / (timing.prescaler * quanta),
        1000U * (1U + timing.prop_seg + timing.phase_seg1) / quanta,
        unsigned(timing.prescaler), unsigned(timing.prop_seg + timing.phase_seg1),
        unsigned(timing.phase_seg2), unsigned(timing.sjw));
    return 0;
}

const char *feedbackIssue(const skywalker::motor::MotorSnapshot &v, bool absolute, bool reference) {
    using namespace skywalker::motor;
    auto required = FeedbackVelocity | FeedbackCurrent;
    if (absolute) required |= FeedbackAbsolutePosition;
    if (!v.feedback_fresh) return "stale";
    if ((v.feedback.valid & required) != required) return "missing_fields";
    if (!std::isfinite(v.feedback.velocity_rad_s) || !std::isfinite(v.feedback.current_a) ||
        (absolute && !std::isfinite(v.feedback.absolute_position_rad))) return "nonfinite";
    if (std::fabs(v.feedback.velocity_rad_s) >= 80) return "overspeed";
    if ((v.feedback.valid & FeedbackTemperature) &&
        (!std::isfinite(v.feedback.temperature_c) || v.feedback.temperature_c >= 70)) return "temperature";
    if (reference && (!v.position_reference_valid || !(v.feedback.valid & FeedbackPosition) ||
        !std::isfinite(v.feedback.position_rad))) return "reference";
    return nullptr;
}
} // namespace

int main() {
    using namespace skywalker;
    using namespace skywalker::robotics;
    static Vofa vofa{};
    const device *telemetry_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
    const int vofa_error = vofa_init(&vofa, telemetry_uart);
    LOG_INF("VOFA USB device=%s init=%d; JustFloat 16 channels, 10 ms", telemetry_uart->name, vofa_error);
    if (vofa_error < 0) LOG_WRN("VOFA unavailable; motor control continues without telemetry");
    static motor::Motor steer(bench::steerHardware()), drive(bench::driveHardware());
    static motor::Group group(steer, drive);
    static motor::CanBus steer_bus(bench::steer_can), drive_bus(bench::drive_can);
    const auto module_config = bench::moduleConfig();
    SwerveModule module(module_config);
    const float dt_min = std::max({module_config.steer.position.dt_min_s,
        module_config.steer.velocity.regulator.feedback.dt_min_s, module_config.drive.regulator.feedback.dt_min_s});
    const float dt_max = std::min({module_config.steer.position.dt_max_s,
        module_config.steer.velocity.regulator.feedback.dt_max_s, module_config.drive.regulator.feedback.dt_max_s});
    RecoveryGate recovery({100, 100000});
    int ret = bench::hardware_confirmed ? module.validate() : -ENODEV;
    if (ret == 0 && bench::steer_can == bench::drive_can) ret = -EINVAL;
    if (ret == 0) ret = steer_bus.attach(steer);
    if (ret == 0) ret = drive_bus.attach(drive);
    if (ret == 0) ret = configureCanTiming("CAN2_6020", bench::steer_can,
        DT_PROP(DT_NODELABEL(can2), bitrate), DT_PROP(DT_NODELABEL(can2), sample_point));
    if (ret == 0) ret = configureCanTiming("CAN1_3508", bench::drive_can,
        DT_PROP(DT_NODELABEL(can1), bitrate), DT_PROP(DT_NODELABEL(can1), sample_point));
    if (ret == 0) ret = steer_bus.start();
    if (ret == 0) ret = drive_bus.start();
    const int topology_error = ret;
    namespace input = skywalker::samples::control;
    static_assert(input::diagnostic_scenario == input::DiagnosticScenario::None ||
                  input::diagnostic_scenario == input::DiagnosticScenario::InputPause ||
                  input::diagnostic_scenario == input::DiagnosticScenario::ExecutionPause);
    static communication::AsyncUart::DmaBuffers remote_dma __nocache;
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma, input::receiverConfig());
    const int remote_error = remote.start();
    if (remote_error < 0) {
        LOG_ERR("remote receiver start failed: %d", remote_error);
        return remote_error;
    }
    communication::RemoteReceiver::Snapshot snapshot{};
    input::RcControlAdapter adapter;
    input::SampleDiagnostics diagnostics;
    LOG_INF("init=%d; GM6020 CAN2 ID2, M3508 CAN1 ID2; RC safe+center then left Middle, left stick xy", ret);
    LOG_INF("control_sleep_us=%u; relative sleep, measured dt, commit after active update",
        unsigned(bench::control_period_us));
    LOG_INF("startup_zero=%d current_mode=%d max_command_speed=%.2f limits=%.2f/%.2f A",
        bench::capture_startup_zero, bench::steer_current_mode, double(bench::test_speed_m_s),
        double(bench::steer_current_limit_a), double(bench::drive_current_limit_a));
    LOG_INF("fixed_zero_ticks=%u equivalent_radius=%.5f ratio=%.5f", unsigned(bench::steer_zero_ticks),
        double(bench::wheel_radius_m), double(bench::drive_gear_ratio));
    bool requested = false, seeded = false, ready = false, hard_fault = false;
    std::uint64_t stable_since = 0, first_steer = 0, first_drive = 0, next_log = 0, retry_ms = 0;
    std::uint64_t steer_reference = 0, drive_reference = 0;
    std::uint64_t next_vofa = 0, next_vofa_warning = 0;
    std::uint32_t loop_count = 0, vofa_queued = 0, vofa_dropped = 0;
    bool vofa_connected = false;
    std::uint32_t stop_count = 0;
    WaitReason last_stop_reason = WaitReason::None;
    int last_stop_error = 0;
    std::uint64_t last_stop_ms = 0;
    core::TimeUs previous_us = core::monotonicTimeUs(), cycle_us = 0;
    const char *operation = "startup";
    core::Stamp source{};
    ChassisCommand command{};
    command.source = ControlSource::Remote;
    SwerveKinematics kinematics(bench::kinematicsConfig());
    float steer_zero_rad = 0;
    bool zero_captured = !bench::capture_startup_zero;
    const auto module_feedback = [&](const motor::MotorSnapshot &sv, const motor::MotorSnapshot &dv) {
        return ModuleFeedback{bench::steer_direction * (sv.feedback.absolute_position_rad - steer_zero_rad),
            bench::steer_direction * (sv.feedback.position_rad - steer_zero_rad),
            bench::steer_direction * sv.feedback.velocity_rad_s,
            bench::drive_direction * dv.feedback.velocity_rad_s};
    };
    ModuleTargets targets{};
    ModuleFeedback feedback{};
    ModuleOutput output{};
    const auto suspend = [&](WaitReason reason, int error = 0, bool blocked = false) {
        const auto state = group.status();
        const bool had_authority = state.active || state.enable_pending ||
            recovery.stage() == RecoveryGate::Stage::Active || recovery.stage() == RecoveryGate::Stage::Enabling;
        motor::MotorSnapshot stopped_steer{}, stopped_drive{};
        const auto stopped_output = output;
        if (had_authority) {
            stopped_steer = steer.snapshot();
            stopped_drive = drive.snapshot();
            ++stop_count;
            last_stop_reason = reason;
            last_stop_error = error;
            last_stop_ms = core::monotonicTimeUs() / 1000;
        }
        if (had_authority || blocked) {
            adapter.withdraw();
            requested = false;
        }
        if (state.active || state.enable_pending) group.disable();
        seeded = ready = false;
        output = {};
        recovery.withdraw(reason, error, blocked);
        hard_fault = hard_fault || blocked;
        if (had_authority) {
            // Output is revoked before formatting the failing cycle's diagnostics.
            LOG_WRN("STOP reason=%s operation=%s err=%d blocked=%d ms=%llu dt_us=%llu group_fault=%u/%d",
                waitReasonName(reason), operation, error, blocked,
                static_cast<unsigned long long>(last_stop_ms), static_cast<unsigned long long>(cycle_us),
                unsigned(state.last_fault.reason), state.last_fault.error);
            const auto log_motor = [&](const char *name, const motor::MotorSnapshot &v, bool absolute) {
                const char *issue = feedbackIssue(v, absolute, absolute);
                LOG_WRN("STOP %s state=%u feedback=%s valid=0x%x stamp_ms=%llu fault=%u/%d speed=%.3f current=%.3f temp=%.1f",
                    name, unsigned(v.state), issue ? issue : "ok", unsigned(v.feedback.valid),
                    static_cast<unsigned long long>(v.feedback.timestamp_ms),
                    unsigned(v.last_fault.reason), v.last_fault.error, double(v.feedback.velocity_rad_s),
                    double(v.feedback.current_a), double(v.feedback.temperature_c));
            };
            log_motor("6020", stopped_steer, true);
            log_motor("3508", stopped_drive, false);
            LOG_WRN("STOP align=%.3f drive_on=%d drive_target=%.3f command_A=%.3f/%.3f",
                double(stopped_output.alignment_error_rad), stopped_output.drive_enabled,
                double(stopped_output.drive_target_rad_s), double(bench::steer_direction * stopped_output.steer_effort),
                double(bench::drive_direction * stopped_output.drive_effort));
            logCanState("STOP CAN2_6020", bench::steer_can, steer_bus.status());
            logCanState("STOP CAN1_3508", bench::drive_can, drive_bus.status());
        }
    };
    for (;;) {
        // Do not catch up to an old deadline with a sub-dt_min control step.
        k_sleep(K_USEC(bench::control_period_us));
        bool command_updated = false;
        ++loop_count;
        (void)remote.snapshot(snapshot);
        const auto now_us = core::monotonicTimeUs(), now = now_us / 1000;
        cycle_us = now_us > previous_us ? now_us - previous_us : 0;
        const float dt = float(cycle_us) / 1000000;
        const bool cycle = dt >= dt_min && dt <= dt_max;
        previous_us = now_us;
        operation = "prerequisites";
        const auto &rc = adapter.update(snapshot.remote, now);
        const auto diagnostic = diagnostics.update(now, group.active(),
            !rc.fresh || rc.remote.left_switch == RcSwitch::Down);
        const bool clear = rc.clear_fault;
        requested = rc.run_allowed;
        if (!diagnostic.input_paused && rc.fresh &&
            (!command.stamp.valid || sequenceAfter(rc.remote.stamp.sequence, command.stamp.sequence))) {
            command.mode = requested ? ChassisMode::BodyVelocity : ChassisMode::Disabled;
            command.vx_m_s = input::RcControlAdapter::normalize(rc.remote.analog.left_y) * bench::test_speed_m_s;
            command.vy_m_s = -input::RcControlAdapter::normalize(rc.remote.analog.left_x) * bench::test_speed_m_s;
            command.wz_rad_s = 0; // No chassis rotation command for a standalone module.
            command.stamp = rc.remote.stamp;
            source = {rc.remote.stamp.timestamp_ms * 1000, rc.remote.stamp.sequence, true};
        }
        if (requested && !isFresh(command.stamp, now, 100)) {
            adapter.withdraw();
            requested = false;
        }
        if (!requested) {
            command.mode = ChassisMode::Disabled;
            command.vx_m_s = command.vy_m_s = command.wz_rad_s = 0;
        }
        if (diagnostic.execution_paused && requested && !clear) {
            if (now >= next_log) {
                next_log = now + 1000;
                LOG_INF("diagnostic execution paused; RC fresh=%d", rc.fresh);
            }
            continue;
        }
        if (clear && topology_error == 0) {
            suspend(WaitReason::Reference);
            ret = group.clearFault();
            if (ret == 0 || ret == -EALREADY) { hard_fault = false; retry_ms = now + 100; }
        }
        const auto sv = steer.snapshot(), dv = drive.snapshot();
        feedback = module_feedback(sv, dv);
        const char *steer_issue = feedbackIssue(sv, true, seeded);
        const char *drive_issue = feedbackIssue(dv, false, false);
        const bool healthy = !steer_issue && !drive_issue;
        const auto steer_transport = steer_bus.status(), drive_transport = drive_bus.status();
        const bool transport = topology_error == 0 && steer_transport.state == motor::BusState::Running &&
            drive_transport.state == motor::BusState::Running;
        if (topology_error < 0) suspend(WaitReason::Configuration, topology_error, true);
        else if (hard_fault) { adapter.withdraw(); requested = false; }
        else if (!transport) {
            const auto &failed = steer_transport.state != motor::BusState::Running ? steer_transport : drive_transport;
            suspend(WaitReason::Transport, failed.last_error < 0 ? failed.last_error : -EHOSTDOWN);
        }
        else if (!cycle) suspend(WaitReason::Cycle, -ERANGE);
        else if (!healthy) suspend(WaitReason::Feedback, -ESTALE);
        else {
            bool fault = sv.state == motor::MotorState::Fault || dv.state == motor::MotorState::Fault;
            const auto recoverable = [](const motor::MotorSnapshot &v) {
                if (v.state != motor::MotorState::Fault) return true;
                switch (v.last_fault.reason) {
                case motor::FaultReason::FeedbackExpired: case motor::FaultReason::CommandExpired:
                case motor::FaultReason::UnexpectedDisabled: case motor::FaultReason::EnableTimeout:
                case motor::FaultReason::TransportError: case motor::FaultReason::RxOverflow: return true;
                default: return false;
                }
            };
            if (fault) {
                const bool transient = recoverable(sv) && recoverable(dv);
                suspend(WaitReason::Drive, -EIO, !transient);
                if (transient && now >= retry_ms) { retry_ms = now + 100; ret = group.clearFault(); }
            } else {
                auto gs = group.status();
                if (!requested && (gs.active || gs.enable_pending)) { suspend(WaitReason::Command); gs = group.status(); }
                if ((recovery.stage() == RecoveryGate::Stage::Active && !gs.active) ||
                    (recovery.stage() == RecoveryGate::Stage::Enabling && !gs.active && !gs.enable_pending))
                    suspend(WaitReason::Drive, -EAGAIN);
                if (seeded && (sv.reference_generation != steer_reference || dv.reference_generation != drive_reference))
                    suspend(WaitReason::Reference, -ESTALE);
                if (!ready && !group.active() && !group.status().enable_pending && group.ready() && now >= retry_ms) {
                    if (!seeded) {
                        ret = steer.reseedPosition(sv.feedback.absolute_position_rad);
                        if (ret == 0) ret = drive.reseedPosition(0);
                        if (ret == 0) {
                            seeded = true; stable_since = now;
                            first_steer = sv.feedback.timestamp_ms; first_drive = dv.feedback.timestamp_ms;
                            steer_reference = steer.snapshot().reference_generation;
                            drive_reference = drive.snapshot().reference_generation;
                        } else suspend(WaitReason::Reference, ret);
                    } else if (now >= stable_since + 30 && sv.feedback.timestamp_ms > first_steer &&
                        dv.feedback.timestamp_ms > first_drive) {
                        if (!zero_captured) {
                            steer_zero_rad = sv.feedback.absolute_position_rad;
                            zero_captured = true;
                            LOG_INF("startup steer zero=%.5f rad; retained until reboot", double(steer_zero_rad));
                        }
                        feedback = module_feedback(sv, dv);
                        ret = module.reset(feedback);
                        for (auto &target : targets) target.angle_rad = feedback.steer_absolute_rad;
                        if (ret == 0) ret = kinematics.reset(targets);
                        if (ret == 0) { ready = true; recovery.prepared(now); }
                        else suspend(WaitReason::Reference, ret);
                    }
                }
                if (ready && requested) {
                    const auto stage = recovery.stage();
                    if (!recovery.accept(command.stamp, source, now)) {
                        ret = -ESTALE;
                        if (stage == RecoveryGate::Stage::Active || stage == RecoveryGate::Stage::Enabling)
                            suspend(WaitReason::Command, ret);
                    } else if (stage == RecoveryGate::Stage::WaitingCommand) {
                        operation = "enable";
                        ret = group.enable();
                        if (ret < 0) suspend(WaitReason::Drive, ret);
                    } else if (group.active()) {
                        recovery.enabled();
                        feedback = module_feedback(sv, dv);
                        operation = "kinematics";
                        ret = kinematics.solve(command, targets);
                        if (ret == 0) { operation = "module_step"; ret = module.step(targets[0], feedback, dt, output); }
                        if (ret == 0) { operation = "steer_current"; ret = steer.setCurrent(bench::steer_direction * output.steer_effort); }
                        if (ret == 0) { operation = "drive_current"; ret = drive.setCurrent(bench::drive_direction * output.drive_effort); }
                        if (ret < 0) suspend(WaitReason::Drive, ret, ret == -ERANGE || ret == -EINVAL);
                        else command_updated = true;
                    }
                }
            }
        }
        // CanBus sends startup/disable/fault safety frames independently of commit().
        // Publish targets only after both active motor commands have been updated.
        if (command_updated) {
            const int steer_error = steer_bus.commit().error, drive_error = drive_bus.commit().error;
            if (steer_error < 0 || drive_error < 0) {
                operation = steer_error < 0 ? "steer_commit" : "drive_commit";
                suspend(WaitReason::Transport, steer_error < 0 ? steer_error : drive_error);
            }
        }
        const bool vofa_due = vofa_error == 0 && now >= next_vofa;
        if (vofa_due) {
            next_vofa = now + 10;
            std::uint32_t dtr = 0;
            vofa_connected = uart_line_ctrl_get(telemetry_uart, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr != 0;
        }
        if (vofa_due && vofa_connected) {
            const bool active = group.active();
            // DTR gates enqueueing; a slow connected host still cannot block control.
            // Channel order and units are documented in README.md.
            const float channels[] = {
                command.vx_m_s, command.vy_m_s,
                output.optimized_angle_rad, feedback.steer_absolute_rad,
                output.steer_reference_rad, output.alignment_error_rad,
                output.drive_target_rad_s, feedback.drive_velocity_rad_s,
                active ? bench::steer_direction * output.steer_effort : 0.0f, sv.feedback.current_a,
                active ? bench::drive_direction * output.drive_effort : 0.0f, dv.feedback.current_a,
                float(rc.fresh), float(active), float(healthy), float(active && output.drive_enabled),
            };
            constexpr auto channel_count = sizeof(channels) / sizeof(channels[0]);
            static_assert(channel_count <= VOFA_MAX_FLOATS);
            const int send_error = vofa_send(&vofa, channels, static_cast<std::uint8_t>(channel_count));
            if (send_error == 0) ++vofa_queued;
            else ++vofa_dropped;
            if (send_error < 0 && now >= next_vofa_warning) {
                next_vofa_warning = now + 1000;
                LOG_WRN("VOFA send failed: %d", send_error);
            }
        }
        if (now >= next_log) {
            next_log = now + 1000;
            const auto gs = group.status();
            const auto sb = steer_bus.status(), db = drive_bus.status();
            const char *arm_hint = !rc.fresh ? "rc_offline" : rc.run_allowed ? "unlocked" :
                (rc.remote.left_switch != RcSwitch::Down || rc.remote.right_switch != RcSwitch::Down)
                    ? "return_both_down" : !adapter.controlsCentered() ? "center_controls" :
                adapter.armReady() ? "move_left_to_middle" : "hold_down_500ms";
            LOG_INF("alive ms=%llu loops=%u vofa_connected=%d vofa_queued=%u vofa_drop=%u can_err=%d/%d",
                static_cast<unsigned long long>(now), loop_count, vofa_connected, vofa_queued, vofa_dropped,
                sb.last_error, db.last_error);
            LOG_INF("active=%d pending=%d ready=%d healthy=%d source=%u age=%llu gen=%u reason=%u err=%d can=%u/%u target=%.3f angle=%.3f wheel=%.3f",
                group.active(), group.status().enable_pending, ready, healthy, command.stamp.sequence,
                static_cast<unsigned long long>(now >= command.stamp.timestamp_ms ? now - command.stamp.timestamp_ms : 0),
                recovery.generation(), unsigned(recovery.reason()), ret, unsigned(steer_bus.status().state),
                unsigned(drive_bus.status().state), double(output.optimized_angle_rad),
                double(feedback.steer_absolute_rad), double(feedback.drive_velocity_rad_s));
            LOG_INF("rc=%d run=%d switches=%u/%u feedback=%d/%d raw_steer=%.3f vx=%.3f vy=%.3f",
                rc.fresh, rc.run_allowed, unsigned(rc.remote.left_switch), unsigned(rc.remote.right_switch),
                sv.feedback_fresh, dv.feedback_fresh, double(sv.feedback.absolute_position_rad),
                double(command.vx_m_s), double(command.vy_m_s));
            LOG_INF("arm=%s centered=%d arm_ready=%d hold_ms=%llu raw_L=%d/%d raw_R=%d/%d wheel=%d gate=%s gate_err=%d",
                arm_hint, adapter.controlsCentered(), adapter.armReady(),
                static_cast<unsigned long long>(adapter.neutralHeldMs()),
                int(rc.remote.analog.left_x), int(rc.remote.analog.left_y),
                int(rc.remote.analog.right_x), int(rc.remote.analog.right_y), int(rc.remote.analog.wheel),
                waitReasonName(recovery.reason()), recovery.error());
            const char *fault_motor = gs.last_fault.source_motor == &steer ? "6020_CAN2" :
                gs.last_fault.source_motor == &drive ? "3508_CAN1" : "none";
            LOG_INF("stops=%u last_stop=%s stop_err=%d stop_ms=%llu group_fault=%u/%d motor=%s fault_ms=%llu",
                stop_count, waitReasonName(last_stop_reason), last_stop_error,
                static_cast<unsigned long long>(last_stop_ms), unsigned(gs.last_fault.reason),
                gs.last_fault.error, fault_motor, static_cast<unsigned long long>(gs.last_fault.occurred_ms));
            logCanState("CAN2_6020", bench::steer_can, sb);
            logCanState("CAN1_3508", bench::drive_can, db);
            LOG_INF("ramp=%.3f align=%.3f ready=%d drive_on=%d flip=%d coast=%d current=%.3f/%.3f",
                double(output.steer_reference_rad), double(output.alignment_error_rad), output.drive_ready,
                output.drive_enabled, output.flipped, output.coasting,
                double(output.steer_effort),
                double(output.drive_effort));
            const char *drive_gate = !gs.active ? "inactive" : output.coasting ? "coast" :
                !output.drive_enabled ? "alignment" : "enabled";
            LOG_INF("drive_gate=%s target=%.3f measured=%.3f command_A=%.3f feedback_A=%.3f dt_us=%llu feedback_issue=%s/%s",
                drive_gate, double(output.drive_target_rad_s), double(feedback.drive_velocity_rad_s),
                double(bench::drive_direction * output.drive_effort), double(dv.feedback.current_a),
                static_cast<unsigned long long>(cycle_us), steer_issue ? steer_issue : "ok", drive_issue ? drive_issue : "ok");
        }
    }
}
