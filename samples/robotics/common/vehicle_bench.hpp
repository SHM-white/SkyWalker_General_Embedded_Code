#pragma once

#include <algorithm>
#include <cerrno>
#include <limits>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include <robotics/execution/snapshot_cache.hpp>
#include <robotics/shooter/shooter_executor.hpp>
#include <robotics/vehicle/big_yaw_profile.hpp>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#if defined(CONFIG_BOARD_DM_MC02)
#include <zephyr/drivers/regulator.h>
#endif
#include "chassis_bench.hpp"
#if !defined(CONFIG_VEHICLE_CHASSIS_ROLE)
#include "shooter_bench.hpp"
#include <drivers/imu/dm_imu_rs485.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <robotics/gimbal/inertial_gimbal.hpp>
#include <robotics/gimbal/yaw_centering.hpp>
#if defined(CONFIG_VEHICLE_VISION_OBSERVE) || defined(CONFIG_VEHICLE_VISION_EXECUTE)
#include <communication/vision/ab_protocol.hpp>
#endif
#endif

namespace skywalker::samples::vehicle {
using namespace robotics;
namespace calibration = robotics::vehicle;

// TODO(power-sensor): implement this producer with actual bus voltage/current,
// or another independent calibrated measured-power channel. V1's budget
// packet has no measured power field and cannot stand in for a measurement.
class PendingPowerSource final : public IPowerMeasurementSource {
public:
    int sample(core::TimeUs, PowerMeasurement &out) override { out = {}; return -ENODATA; }
};
class IDialHomeSource {
public:
    virtual ~IDialHomeSource() = default;
    virtual int sample(core::Measurement<double> &out) = 0;
};
// TODO(index): connect an actual home/index sensor, preserving acquisition time.
// Loaded feeding never reseeds a dial from an arbitrary post-fault position.
class PendingDialHomeSource final : public IDialHomeSource {
public:
    int sample(core::Measurement<double> &out) override { out = {}; return -ENODATA; }
};

inline constexpr std::uint32_t kCommandTimeoutMs = 100, kStatusTimeoutMs = 100;
#if defined(CONFIG_VEHICLE_CHASSIS_ROLE)
inline constexpr BoardRole kRole = BoardRole::ChassisController;
#else
inline constexpr BoardRole kRole = BoardRole::GimbalController;
#endif
inline communication::AsyncUart::DmaBuffers link_dma __nocache;
inline communication::ConfiguredInterBoardTransport::Config linkConfig() {
    communication::ConfiguredInterBoardTransport::Config c{};
    c.kind = communication::InterBoardTransportKind::Uart;
    c.uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart));
    return c;
}
inline communication::ConfiguredInterBoardTransport transport(linkConfig(), &link_dma);
inline communication::InterBoardEndpoint::Config endpointConfig() {
    communication::InterBoardEndpoint::Config c{kRole};
    c.enable_big_yaw = true;
    return c;
}
inline communication::InterBoardEndpoint endpoint(transport, endpointConfig());
inline atomic_t input_paused = 0, execution_paused = 0, status_paused = 0, head_paused = 0, measurement_paused = 0;
inline atomic_t estop = 0, clear_requested = 0, continuous_requested = 0;
inline atomic_ptr_t execution_thread = nullptr;
struct ShotEvent { std::uint32_t id = 0; MessageStamp stamp{}; };
inline SnapshotCache<ShotEvent> shot_events;
inline SnapshotCache<core::Stamp> console_fire_requests;

struct Observation {
    RunStatus mechanism{}, big_yaw{};
    BigYawFeedback big_yaw_feedback{};
    ShooterStatus shooter{};
    std::array<motor::BusStatus, 3> buses{};
    MessageStamp source_stamp{}, stamp{};
    PowerMeasurement measured_power{};
    core::Stamp head_stamp{};
    core::TimeUs duration_us = 0, peak_duration_us = 0;
    std::uint32_t overruns = 0;
    float center_error_rad = 0, centering_rate_rad_s = 0, effort_scale = 0;
    bool head_stable = false;
};
inline SnapshotCache<Observation> observations;

inline std::uint32_t ageMs(const MessageStamp &stamp, std::uint64_t now_ms) {
    return stamp.valid && now_ms >= stamp.timestamp_ms
        ? static_cast<std::uint32_t>(std::min<std::uint64_t>(now_ms - stamp.timestamp_ms, UINT32_MAX)) : UINT32_MAX;
}
inline std::uint32_t ageMs(const core::Stamp &stamp, core::TimeUs now_us) {
    return stamp.valid && now_us >= stamp.time_us
        ? static_cast<std::uint32_t>(std::min<std::uint64_t>((now_us - stamp.time_us) / 1000, UINT32_MAX)) : UINT32_MAX;
}
inline MessageStamp forwardedStamp(const MessageStamp &received, std::uint32_t age_ms) {
    if (!received.valid || age_ms == UINT32_MAX || received.timestamp_ms < age_ms) return {};
    return {received.timestamp_ms - age_ms, received.sequence, true};
}

#if !defined(CONFIG_VEHICLE_CHASSIS_ROLE)
inline communication::AsyncUart::DmaBuffers remote_dma __nocache, head_dma __nocache;
inline communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma, {});
inline RemoteSource original_remote(remote);
class PausableRemote final : public ICommandSource {
public:
    SourceRole role() const override { return original_remote.role(); }
    int start() override { return original_remote.start(); }
    int sample(SourceSample &out) override {
        return atomic_get(&input_paused) ? -EAGAIN : original_remote.sample(out);
    }
};
inline PausableRemote remote_source;
inline CommandManager::Config commandConfig() {
    auto c = board_config::command_policy;
    c.require_referee_for_motion = IS_ENABLED(CONFIG_VEHICLE_REFEREE);
    c.allow_auto = IS_ENABLED(CONFIG_VEHICLE_VISION_EXECUTE);
    c.expected_vision_reference = calibration::head_reference;
    c.max_chassis_vx_m_s = c.max_chassis_vy_m_s = 0.3f;
    c.max_chassis_wz_rad_s = 0.3f;
    c.requested_fire_rate_hz = 2;
    return c;
}
inline CommandManager commands(commandConfig());
#if defined(CONFIG_VEHICLE_REFEREE)
inline communication::AsyncUart::DmaBuffers referee_dma __nocache;
inline communication::RefereeReceiver referee(DEVICE_DT_GET(DT_ALIAS(referee_uart)), referee_dma,
    communication::RefereeVersion::Rm2026V1_3);
inline RefereePermissionSource permission_source(referee);
#endif
#if defined(CONFIG_VEHICLE_VISION_OBSERVE) || defined(CONFIG_VEHICLE_VISION_EXECUTE)
inline communication::AsyncUart::DmaBuffers vision_dma __nocache;
inline communication::vision::AbProtocol vision_protocol({.command_reference = calibration::head_reference});
inline communication::vision::VisionReceiver::Config visionConfig() {
    communication::vision::VisionReceiver::Config c{};
    // TODO(vision): wire genuine bullet-speed/count feedback before AB TX.
    // Reference epochs also require an explicit producer session agreement.
    c.feedback_period_us = 0;
    return c;
}
inline communication::vision::VisionReceiver vision(DEVICE_DT_GET(DT_ALIAS(vision_uart)), vision_dma,
    vision_protocol, visionConfig());
inline VisionSource original_vision(vision);
class PausableVision final : public ICommandSource {
public:
    SourceRole role() const override { return original_vision.role(); }
    int start() override { return original_vision.start(); }
    int sample(SourceSample &out) override {
        return atomic_get(&input_paused) ? -EAGAIN : original_vision.sample(out);
    }
};
inline PausableVision vision_source;
#endif
inline imu::DmImuRs485Source::Config headConfig() {
    imu::DmImuRs485Source::Config c{};
    c.protocol = {1, 20000};
    c.reference = calibration::head_reference;
    c.sensor_to_body = calibration::head_sensor_to_body;
    // TODO(IMU): confirm device quaternion direction, physical scale, mounting
    // and the policy for vendor packets without estimator-quality metadata.
    return c;
}
inline imu::DmImuRs485Source head_source(DEVICE_DT_GET(DT_ALIAS(rs485_2)), head_dma, headConfig());
inline imu::ImuReceiver head(head_source, {.poll_interval_us = 1000, .priority = 6});
inline InertialGimbalAdapter::Config inertialConfig() {
    InertialGimbalAdapter::Config c{};
    c.yaw = board_config::yaw; c.pitch = board_config::pitch;
    c.yaw_direction = calibration::small_yaw.direction; c.pitch_direction = calibration::pitch.direction;
    c.pitch_locked = IS_ENABLED(CONFIG_VEHICLE_LOCK_PITCH);
    c.allow_unknown_quality = true;
    // TODO(control): validate carrier compensation and settle tolerances before
    // enabling continuous big-Yaw centering or loaded firing.
    return c;
}
inline YawCenteringController::Config centeringConfig() {
    YawCenteringController::Config c{};
    c.center_rad = calibration::yaw_center_rad;
    c.follow_direction = calibration::small_yaw.direction;
    return c;
}

inline void produceShotEvent(const MessageStamp &stamp, std::uint32_t &counter) {
    if (!stamp.valid || counter == UINT32_MAX) return;
    (void)shot_events.publish({++counter, stamp});
}
#endif

// Communication and command production exchange snapshots only. Motor setters
// and controller updates belong exclusively to run()'s execution thread.
inline void communicationTask(void *, void *, void *) {
#if !defined(CONFIG_VEHICLE_CHASSIS_ROLE)
    CommandSnapshot frame{};
    std::uint64_t next_command_ms = 0, console_sequence = 0;
    std::uint32_t last_remote_sequence = 0, event_counter = 0;
    bool have_remote = false, previous_left = false;
#endif
    for (;;) {
        const auto now_ms = static_cast<std::uint64_t>(k_uptime_get());
#if !defined(CONFIG_VEHICLE_CHASSIS_ROLE)
        if (now_ms >= next_command_ms && commands.snapshot(frame) == 0) {
            next_command_ms = now_ms + 10;
            auto wheel_command = frame.decision.command.chassis;
            if (atomic_get(&estop)) wheel_command.mode = ChassisMode::Disabled;
            endpoint.submit(wheel_command);
            endpoint.setReferee(frame.observed.referee);
            const auto &operator_input = frame.observed.remote;
            if (operator_input.online && isFresh(operator_input.stamp, now_ms, kCommandTimeoutMs) &&
                (!have_remote || sequenceAfter(operator_input.stamp.sequence, last_remote_sequence))) {
                const bool press = operator_input.mouse.left && !previous_left;
                previous_left = operator_input.mouse.left;
                last_remote_sequence = operator_input.stamp.sequence; have_remote = true;
                if (press && !atomic_get(&input_paused)) produceShotEvent(operator_input.stamp, event_counter);
            }
            core::Stamp console_request{};
            if (console_fire_requests.snapshot(console_request) == 0 && console_request.valid &&
                console_request.sequence != console_sequence) {
                console_sequence = console_request.sequence;
                if (!atomic_get(&input_paused)) produceShotEvent(
                    {console_request.time_us / 1000, static_cast<std::uint32_t>(console_request.sequence), true}, event_counter);
            }
        }
#endif
        endpoint.poll(now_ms);
        k_sleep(K_MSEC(1));
    }
}

inline void consoleTask(void *, void *, void *) {
    const device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    std::uint64_t next_log_ms = 0, console_fire_sequence = 0;
    Observation observation{};
    for (;;) {
        unsigned char key = 0;
        while (uart_poll_in(console, &key) == 0) {
            if (key == 'p') atomic_set(&input_paused, !atomic_get(&input_paused));
            if (key == 'e') atomic_set(&execution_paused, !atomic_get(&execution_paused));
            if (key == 's') atomic_set(&status_paused, !atomic_get(&status_paused));
            if (key == 'i') atomic_set(&head_paused, !atomic_get(&head_paused));
            if (key == 'm') atomic_set(&measurement_paused, !atomic_get(&measurement_paused));
            if (key == '!') atomic_set(&estop, 1);
            if (key == 'r') {
                atomic_set(&estop, 0); atomic_set(&clear_requested, 1); atomic_set(&continuous_requested, 0);
            }
            if (key == 'c') atomic_set(&continuous_requested, 1);
            if (key == 'h') atomic_set(&continuous_requested, 0);
            if (key == 'f') (void)console_fire_requests.publish({core::monotonicTimeUs(), ++console_fire_sequence, true});
        }
        const auto now_ms = static_cast<std::uint64_t>(k_uptime_get());
        if (now_ms >= next_log_ms) {
            next_log_ms = now_ms + 200;
            (void)observations.snapshot(observation);
            const auto rx = endpoint.snapshot();
            const bool fresh = isFresh(observation.stamp, now_ms, kStatusTimeoutMs);
            std::size_t execution_stack_unused = 0;
#if defined(CONFIG_THREAD_STACK_INFO) && defined(CONFIG_INIT_STACKS)
            const auto owner = static_cast<k_tid_t>(atomic_ptr_get(&execution_thread));
            if (owner) (void)k_thread_stack_space_get(owner, &execution_stack_unused);
#endif
            printk("role=%u online=%d V2=%d source=%u sourceAge=%u stateAge=%u run=%u reason=%u error=%d ready=%d gen=%u bigRun=%u bigGen=%u bigRate_mrad_s=%d headStable=%d headAge=%u center_mrad=%d request_mrad_s=%d scale_milli=%d powerValid=%d frictionReady=%d feed=%u event=%u shots=%u us=%llu peakUs=%llu overruns=%u\n",
                unsigned(kRole), rx.online, rx.big_yaw_compatible, observation.source_stamp.sequence,
                ageMs(observation.source_stamp, now_ms), ageMs(observation.stamp, now_ms),
                unsigned(observation.mechanism.state), unsigned(observation.mechanism.reason), observation.mechanism.error,
                fresh && observation.mechanism.ready, observation.mechanism.generation, unsigned(observation.big_yaw.state),
                observation.big_yaw.generation, int(observation.big_yaw_feedback.actual_rate_rad_s * 1000),
                fresh && observation.head_stable, ageMs(observation.head_stamp, now_ms * 1000),
                int(observation.center_error_rad * 1000), int(observation.centering_rate_rad_s * 1000),
                int(observation.effort_scale * 1000), observation.measured_power.valid &&
                core::fresh(observation.measured_power.stamp, now_ms * 1000, 300000),
                fresh && observation.shooter.friction_ready, unsigned(observation.shooter.feed.state),
                observation.shooter.last_event_id, observation.shooter.shots,
                static_cast<unsigned long long>(observation.duration_us),
                static_cast<unsigned long long>(observation.peak_duration_us), observation.overruns);
            printk("execution_stack_unused=%u linkRejected=%u linkError=%d\n", unsigned(execution_stack_unused),
                rx.rejected_frames, rx.error);
            for (std::size_t i = 0; i < observation.buses.size(); ++i)
                printk("CAN%u state=%u error=%d rxOverflow=%llu superseded=%llu\n", unsigned(i + 1),
                    unsigned(observation.buses[i].state), observation.buses[i].last_error,
                    static_cast<unsigned long long>(observation.buses[i].rx_overflows),
                    static_cast<unsigned long long>(observation.buses[i].superseded_batches));
            // TODO(telemetry): add measured stop latency and I/O queue high-water
            // counters when the installed board's acceptance thresholds are set.
        }
        k_sleep(K_MSEC(10));
    }
}

inline int enableMotorPower() {
    if (!calibration::connections_confirmed) return -ENODEV;
#if defined(CONFIG_BOARD_DM_MC02)
    // TODO(wiring): confirm which XT30 rail supplies the installed motors.
    return regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
#else
    return 0;
#endif
}

inline int run(IPowerMeasurementSource *power_source = nullptr, IShooterHeatSource *heat_source = nullptr,
               IDialHomeSource *home_source = nullptr) {
    atomic_ptr_set(&execution_thread, k_current_get());
    printk("vehicle role=%u hardware=%d IMU mounting=%d referee=%d vision=%d shooting=%d\n", unsigned(kRole),
        calibration::connections_confirmed, calibration::imu_mounting_confirmed, IS_ENABLED(CONFIG_VEHICLE_REFEREE),
        IS_ENABLED(CONFIG_VEHICLE_VISION_EXECUTE), IS_ENABLED(CONFIG_VEHICLE_SHOOTING));
    printk("p pause source, e pause executor, s pause status, i pause head read, m pause power; ! estop, r clear; f single, c continuous, h stop feed\n");
    Observation observation{};
    core::TimeUs previous_cycle_us = 0;
    std::uint32_t observation_sequence = 0;
#if defined(CONFIG_VEHICLE_CHASSIS_ROLE)
    static samples::chassis::Hardware hardware(DEVICE_DT_GET(DT_NODELABEL(can1)), DEVICE_DT_GET(DT_NODELABEL(can2)));
    auto chassis_policy = samples::chassis::executorConfig(IS_ENABLED(CONFIG_VEHICLE_POWER_BUDGET));
    // TODO(power): only allow the separately tagged estimate after model and
    // tracking acceptance; no estimated source is enabled in the vehicle default.
    static ChassisExecutor chassis(hardware.adapter, samples::chassis::controllerConfig(), chassis_policy);
    static motor::Motor big_drive(calibration::bigYawHardware());
    static motor::CanBus big_bus(DEVICE_DT_GET(DT_NODELABEL(can3)));
    static BigYawExecutor big_axis(big_drive, calibration::bigYawMotorConfig(), calibration::bigYawExecutionConfig());
    static PendingPowerSource pending_power;
    if (!power_source) power_source = &pending_power;
    const int topology_error = hardware.start();
    const int chassis_error = chassis.begin();
    int big_error = calibration::connections_confirmed ? big_bus.attach(big_drive) : -ENODEV;
    if (big_error == 0) big_error = big_bus.start();
    if (big_error == 0) big_error = big_axis.begin();
    const int power_error = calibration::connections_confirmed ? enableMotorPower() : -ENODEV;
    printk("chassis setup=%d control=%d bigYaw=%d rail=%d\n", topology_error, chassis_error, big_error, power_error);
    std::uint64_t peer_boot = 0;
    PowerMeasurement measurement{};
    for (;;) {
        const auto now_us = core::monotonicTimeUs(), now_ms = now_us / 1000;
        const auto rx = endpoint.snapshot();
        const bool emergency = atomic_get(&estop);
        const bool clear = atomic_set(&clear_requested, 0);
        if (!atomic_get(&execution_paused) || emergency || clear) {
            if (previous_cycle_us && (now_us <= previous_cycle_us || now_us - previous_cycle_us > 20000)) ++observation.overruns;
            previous_cycle_us = now_us;
            if (rx.peer.stamp.valid && peer_boot != rx.peer.sender_boot_id) {
                peer_boot = rx.peer.sender_boot_id;
                (void)chassis.suspend(now_us, WaitReason::Reference, -ESTALE);
                (void)big_axis.suspend(now_us, WaitReason::Reference, -ESTALE);
            }
            ChassisExecutionInputs input{};
            input.command = rx.control.command;
            if (rx.control.global_action != SafetyAction::Active || atomic_get(&input_paused)) input.command.mode = ChassisMode::Disabled;
            input.source_stamp = {input.command.stamp.timestamp_ms * 1000, input.command.stamp.sequence, input.command.stamp.valid};
            input.transport_ready = topology_error == 0 && power_error == 0 && hardware.running() && rx.online;
            input.recovery_context_valid = rx.local_boot_id && rx.control.receiver_boot_id == rx.local_boot_id &&
                rx.control.resume_generation == chassis.status().generation;
            input.require_permission = IS_ENABLED(CONFIG_VEHICLE_REFEREE);
            input.permission = rx.constraint.output;
            input.permission.stamp = forwardedStamp(rx.constraint.stamp, rx.constraint.output_age_ms);
            input.permission.valid = input.permission.valid && input.permission.stamp.valid;
            input.power_budget.chassis_power_limit_w = rx.constraint.power_limit_w;
            input.power_budget.buffer_energy_j = rx.constraint.buffer_energy_j;
            if (rx.constraint.power_valid) {
                input.power_budget.stamp = forwardedStamp(rx.constraint.stamp, rx.constraint.power_age_ms);
                input.power_budget.limit_stamp = input.power_budget.stamp;
            }
            if (!atomic_get(&measurement_paused)) {
                PowerMeasurement next{};
                const int ret = power_source->sample(now_us, next);
                if (ret == 0) measurement = next;
                else if (ret != -EAGAIN) measurement = {};
            }
            input.measured_power = measurement;
            input.emergency_stop = emergency; input.clear_fault = clear;
            observation.mechanism = chassis.update(input, now_us);
            BigYawExecutionInputs big_input{};
            big_input.request = rx.big_yaw_request;
            if (atomic_get(&input_paused)) big_input.request.mode = BigYawMode::Disabled;
            big_input.local_boot_id = rx.local_boot_id;
            big_input.contract_compatible = rx.big_yaw_compatible;
            big_input.transport_ready = big_error == 0 && power_error == 0 && rx.online &&
                big_bus.status().state == motor::BusState::Running;
            big_input.emergency_stop = emergency; big_input.clear_fault = clear;
            observation.big_yaw = big_error == 0 ? big_axis.update(big_input, now_us)
                : big_axis.suspend(now_us, WaitReason::Configuration, big_error, true);
            // All three physical controllers publish once, after both
            // independent mechanism owners have staged this cycle's targets.
            const int wheel_commit = topology_error == 0 ? hardware.commit() : 0;
            const int big_commit = big_error == 0 ? big_bus.commit().error : 0;
            if (wheel_commit < 0) observation.mechanism = chassis.suspend(now_us, WaitReason::Transport, wheel_commit);
            if (big_commit < 0) observation.big_yaw = big_axis.suspend(now_us, WaitReason::Transport, big_commit);
            observation.big_yaw_feedback = big_axis.feedback();
            observation.source_stamp = input.command.stamp;
            observation.measured_power = chassis.powerMeasurement();
            observation.effort_scale = chassis.effortScale();
            observation.buses = {hardware.steer_bus.status(), hardware.drive_bus.status(), big_bus.status()};
            observation.duration_us = core::monotonicTimeUs() - now_us;
            observation.peak_duration_us = std::max(observation.peak_duration_us, observation.duration_us);
            observation.stamp = {now_ms, ++observation_sequence, true};
            if (!atomic_get(&status_paused)) {
                endpoint.setStatus(observation.mechanism);
                endpoint.setBigYawFeedback(observation.big_yaw_feedback);
                (void)observations.publish(observation);
            }
        }
        k_sleep(K_MSEC(5));
    }
#else
    static samples::shooter::Hardware hardware(DEVICE_DT_GET(DT_NODELABEL(can1)), DEVICE_DT_GET(DT_NODELABEL(can2)), false);
    static InertialGimbalAdapter adapter(inertialConfig());
    static YawCenteringController centering(centeringConfig());
    static samples::shooter::PendingHeatSource pending_heat;
    static PendingDialHomeSource pending_home;
    if (!heat_source) heat_source = &pending_heat;
    if (!home_source) home_source = &pending_home;
    int command_error = commands.registerSource(remote_source);
#if defined(CONFIG_VEHICLE_REFEREE)
    if (command_error == 0) command_error = commands.bindPermissions(permission_source);
#endif
#if defined(CONFIG_VEHICLE_VISION_OBSERVE) || defined(CONFIG_VEHICLE_VISION_EXECUTE)
    if (command_error == 0) command_error = commands.registerSource(vision_source);
#endif
    const int head_error = head.start();
    if (command_error == 0) command_error = commands.start();
    const int topology_error = hardware.start(true);
    const int power_error = calibration::connections_confirmed ? enableMotorPower() : -ENODEV;
    printk("gimbal command=%d head=%d topology=%d rail=%d loaded-home=required\n", command_error, head_error, topology_error, power_error);
    CommandSnapshot frame{};
    imu::Snapshot head_cache{};
    ShooterHeatState heat_cache{};
    core::Measurement<double> home_cache{};
    ShotEvent event{};
    for (;;) {
        const auto now_us = core::monotonicTimeUs(), now_ms = now_us / 1000;
        const bool emergency = atomic_get(&estop);
        const bool clear = atomic_set(&clear_requested, 0);
        if (!atomic_get(&execution_paused) || emergency || clear) {
            if (previous_cycle_us && (now_us <= previous_cycle_us || now_us - previous_cycle_us > 20000)) ++observation.overruns;
            previous_cycle_us = now_us;
            (void)commands.snapshot(frame);
            if (!atomic_get(&head_paused)) head_cache = head.snapshot();
            InertialGimbalInputs inertial_input{};
            inertial_input.command = frame.decision.command.gimbal;
            inertial_input.source_stamp = sourceStamp(frame, inertial_input.command.source);
            inertial_input.yaw = hardware.yaw.snapshot(); inertial_input.pitch = hardware.pitch.snapshot();
            inertial_input.head = head_cache;
            inertial_input.prerequisites_ready = topology_error == 0 && power_error == 0 && hardware.running() &&
                calibration::imu_mounting_confirmed && head_error == 0 && command_error == 0 && !emergency && !clear;
            const bool reference_matches = inertial_input.command.source != ControlSource::Vision ||
                frame.decision.selected_vision.value.reference == head_cache.sample.reference;
            auto inertial = reference_matches ? adapter.update(inertial_input, now_us)
                : adapter.suspend(now_us, WaitReason::Reference, -ESTALE);
            GimbalExecutionInputs gimbal_input{};
            gimbal_input.command = inertial.command; gimbal_input.source_stamp = inertial.source_stamp;
            gimbal_input.transport_ready = topology_error == 0 && power_error == 0 && hardware.running();
            gimbal_input.require_permission = IS_ENABLED(CONFIG_VEHICLE_REFEREE);
            gimbal_input.permission = frame.observed.referee.robot.gimbal_output;
            gimbal_input.emergency_stop = emergency; gimbal_input.clear_fault = clear;
            const auto previous = observation.mechanism;
            observation.mechanism = topology_error == 0 ? hardware.gimbal.update(gimbal_input, now_us)
                : hardware.gimbal.suspend(now_us, WaitReason::Configuration, topology_error, true);
            if (observation.mechanism.generation != previous.generation ||
                (previous.state == RunState::Active && observation.mechanism.state != RunState::Active))
                inertial = adapter.suspend(now_us, WaitReason::Reference, -ESTALE);
            const bool head_stable = inertial.stabilization_valid && observation.mechanism.state == RunState::Active;
            OutputPermission movement_permission{};
#if defined(CONFIG_VEHICLE_REFEREE)
            movement_permission = frame.observed.referee.robot.gimbal_output;
#else
            // Explicit manual suspended-bench authority comes from a fresh RC
            // operator request. This is not a fabricated referee permission.
            const auto &operator_input = frame.observed.remote;
            movement_permission = {operator_input.online && operator_input.stamp.valid,
                frame.decision.operator_mode == OperatorMode::Manual && calibration::connections_confirmed && !emergency,
                operator_input.stamp};
#endif
            const auto rx = endpoint.snapshot();
            YawCenteringInputs follow{};
            follow.joint_yaw_rad = inertial_input.yaw.feedback.position_rad;
            follow.joint_stamp = {inertial_input.yaw.feedback.timestamp_ms * 1000,
                inertial_input.yaw.feedback.timestamp_ms, inertial_input.yaw.feedback_fresh};
            follow.source_stamp = inertial_input.source_stamp;
            follow.enabled = inertial_input.command.mode != GimbalMode::Disabled && rx.big_yaw_compatible &&
                rx.big_yaw_feedback.valid && rx.big_yaw_feedback.ready && !emergency;
            follow.head_stable = head_stable;
            follow.permission_valid = movement_permission.valid && movement_permission.enabled &&
                isFresh(movement_permission.stamp, now_ms, 300);
            const auto centered = centering.update(follow, now_us);
            BigYawRequest big_request{};
            big_request.mode = centered.enabled ? BigYawMode::FollowCenter : BigYawMode::Disabled;
            big_request.target_rate_rad_s = centered.velocity_rad_s;
            big_request.source_sequence = static_cast<std::uint32_t>(inertial_input.source_stamp.sequence);
            big_request.source_age_ms = ageMs(inertial_input.source_stamp, now_us);
            big_request.command_age_ms = ageMs(inertial_input.command.stamp, now_ms);
            big_request.permission = movement_permission; big_request.stamp = centered.stamp;
            endpoint.submitBigYaw(big_request);
            ShooterExecutionInputs shooter_input{};
            shooter_input.command = frame.decision.command.shooter;
            shooter_input.source_stamp = sourceStamp(frame, shooter_input.command.source);
            shooter_input.permission = frame.observed.referee.robot.shooter_output;
            shooter_input.transport_ready = topology_error == 0 && power_error == 0 && hardware.running();
            shooter_input.emergency_stop = emergency; shooter_input.clear_fault = clear;
            shooter_input.require_permission = shooter_input.require_heat = shooter_input.require_gimbal = true;
            shooter_input.allow_feed = IS_ENABLED(CONFIG_VEHICLE_SHOOTING) && calibration::shooter_constraints_confirmed && head_stable;
            shooter_input.gimbal = observation.mechanism;
            if (!IS_ENABLED(CONFIG_VEHICLE_SHOOTING)) shooter_input.command.mode = ShooterMode::Disabled;
            else {
                ShooterHeatState next_heat{};
                const int heat_ret = heat_source->sample(next_heat);
                if (heat_ret == 0) heat_cache = next_heat;
                else if (heat_ret != -EAGAIN) heat_cache = {};
                core::Measurement<double> next_home{};
                const int home_ret = home_source->sample(next_home);
                if (home_ret == 0) home_cache = next_home;
                else if (home_ret != -EAGAIN) home_cache = {};
                (void)shot_events.snapshot(event);
                if (shooter_input.command.source != ControlSource::Vision && shooter_input.command.mode != ShooterMode::Disabled) {
                    shooter_input.command.mode = atomic_get(&continuous_requested) ? ShooterMode::FireContinuous
                        : event.stamp.valid ? ShooterMode::FireSingle : ShooterMode::Ready;
                    shooter_input.command.fire_rate_hz = 2;
                    shooter_input.command.fire_event_id = event.id; shooter_input.command.fire_event_stamp = event.stamp;
                }
            }
            shooter_input.heat = heat_cache; shooter_input.dial_home_reference = home_cache;
            observation.shooter = topology_error == 0 ? hardware.shooter.update(shooter_input, now_us)
                : hardware.shooter.suspend(now_us, WaitReason::Configuration, topology_error);
            // All DJI mechanisms stage into the same bus before a single
            // physical DJI commit; pitch's physical CAN also commits once.
            const int commit = topology_error == 0 ? hardware.commit() : 0;
            if (commit < 0) {
                observation.mechanism = hardware.gimbal.suspend(now_us, WaitReason::Transport, commit);
                observation.shooter = hardware.shooter.suspend(now_us, WaitReason::Transport, commit);
                inertial = adapter.suspend(now_us, WaitReason::Transport, commit);
            }
#if defined(CONFIG_VEHICLE_VISION_OBSERVE) || defined(CONFIG_VEHICLE_VISION_EXECUTE)
            communication::vision::Feedback vision_feedback{};
            vision_feedback.mode = frame.decision.operator_mode == OperatorMode::Auto
                ? communication::vision::Mode::AutoAim : communication::vision::Mode::Idle;
            vision_feedback.reference = head_cache.sample.reference;
            vision_feedback.orientation = head_cache.sample.orientation;
            vision_feedback.gyro_rad_s = head_cache.sample.gyro_rad_s;
            (void)vision.setFeedback(vision_feedback);
#endif
            observation.source_stamp = frame.observed.remote.stamp;
            observation.head_stamp = head_cache.sample.orientation.stamp;
            observation.head_stable = inertial.stabilization_valid && observation.mechanism.state == RunState::Active;
            observation.center_error_rad = centered.center_error_rad; observation.centering_rate_rad_s = centered.velocity_rad_s;
            observation.big_yaw_feedback = rx.big_yaw_feedback;
            observation.big_yaw.generation = rx.big_yaw_feedback.resume_generation;
            observation.big_yaw.state = rx.big_yaw_feedback.armed ? RunState::Active : RunState::Recovering;
            observation.buses[0] = hardware.dji_bus.status(); observation.buses[1] = hardware.pitch_bus.status();
            observation.duration_us = core::monotonicTimeUs() - now_us;
            observation.peak_duration_us = std::max(observation.peak_duration_us, observation.duration_us);
            observation.stamp = {now_ms, ++observation_sequence, true};
            if (!atomic_get(&status_paused)) {
                endpoint.setStatus(observation.mechanism);
                (void)observations.publish(observation);
            }
        }
        k_sleep(K_MSEC(5));
    }
#endif
}
} // namespace skywalker::samples::vehicle

K_THREAD_DEFINE(vehicle_link_worker, 8192, skywalker::samples::vehicle::communicationTask, nullptr, nullptr, nullptr, 6, K_FP_REGS, 0);
K_THREAD_DEFINE(vehicle_console_worker, 6144, skywalker::samples::vehicle::consoleTask, nullptr, nullptr, nullptr, 7, 0, 0);
