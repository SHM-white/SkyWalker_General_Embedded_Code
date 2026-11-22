#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include <core/clock.hpp>
#include <drivers/imu/bmi088_imu.hpp>
#include <drivers/imu/dm_imu_can.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <drivers/motor/can_bus.hpp>
#include <robotics/command/receiver_sources.hpp>
#include <robotics/chassis/imu_heading.hpp>
#include <core/attitude.hpp>
#include <robotics/gimbal/dual_yaw_motion_allocator.hpp>
#include <robotics/gimbal/gimbal_executor.hpp>
#include <robotics/gimbal/inertial_gimbal.hpp>
#include <robotics/vehicle/diagnostics.hpp>
#include <robotics/vehicle/led_status.hpp>
#include <drivers/led/status_led.hpp>
#include <robotics/vehicle/manual_profile.hpp>
#include <robotics/vehicle/shooter_profile.hpp>
#include <robotics/vehicle/startup_action.hpp>
#include <zephyr/drivers/regulator.h>
#if DT_HAS_ALIAS(vision_uart) && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER)
#include <communication/vision/ab_protocol.hpp>
#include <communication/vision/vision_receiver.hpp>
#endif
#include "../../../samples/robotics/common/rc_controls.hpp"

namespace {
using namespace skywalker;
using namespace robotics;
namespace installed = robotics::vehicle;
// 小陀螺 profile 的归属：手动、自主巡航、无。切换归属时重置曲线状态，
// 避免手动转速/拨轮方向带进巡航，也避免把巡航速度带回手动。
enum class SpinOwner : std::uint8_t { None, Manual, Cruise };
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::AsyncUart::DmaBuffers remote_dma __nocache;
std::uint32_t age(const MessageStamp &stamp, std::uint64_t now_ms) {
    return stamp.valid && now_ms >= stamp.timestamp_ms
               ? static_cast<std::uint32_t>(std::min<std::uint64_t>(now_ms - stamp.timestamp_ms, UINT32_MAX))
               : UINT32_MAX;
}
std::uint32_t age(const core::Stamp &stamp, core::TimeUs now_us) {
    return stamp.valid && now_us >= stamp.time_us
               ? static_cast<std::uint32_t>(std::min<std::uint64_t>((now_us - stamp.time_us) / 1000, UINT32_MAX))
               : UINT32_MAX;
}
} // namespace

int main() {
    static installed::Diagnostics diagnostics(installed::DiagnosticLayout::GimbalResponse);
    diagnostics.start("sentry_gimbal", "gimbal",
                      "dual-yaw allocation; CAN head IMU for vision; carrier BMI088 for big yaw");
    static led::StatusLed status_led;
    const int status_led_error = status_led.start();
    if constexpr (led::StatusLed::available)
        diagnostics.event("status_led", status_led_error);
    (void)status_led.setStatus(led::LedStatus::Initializing);
    static communication::ConfiguredInterBoardTransport transport({.kind = communication::InterBoardTransportKind::Uart,
                                                                   .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))},
                                                                  &link_dma);
    static communication::InterBoardEndpoint endpoint(transport, {.role = BoardRole::GimbalController});
    static communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                                samples::control::receiverConfig());
    static RemoteSource remote_source(remote);
    // Vision is only instantiated when the board assigns a UART and the build
    // enables the AB receiver; the arbitration still needs `allow_auto`.
#if DT_HAS_ALIAS(vision_uart) && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER)
    static communication::AsyncUart::DmaBuffers vision_dma __nocache;
    static communication::vision::AbProtocol vision_protocol(
        {.command_reference = installed::head_reference, .allow_unknown_ballistics = true});
    static communication::vision::VisionReceiver vision(DEVICE_DT_GET(DT_ALIAS(vision_uart)), vision_dma,
                                                        vision_protocol, {.feedback_period_us = 10000});
    static VisionSource vision_source(vision);
#endif
#if DT_HAS_ALIAS(navigation_uart) && IS_ENABLED(CONFIG_SKYWALKER_NAVIGATION_RECEIVER)
    static communication::AsyncUart::DmaBuffers navigation_dma __nocache;
    static communication::navigation::NavigationReceiver navigation(DEVICE_DT_GET(DT_ALIAS(navigation_uart)),
                                                                    navigation_dma);
    static NavigationSource navigation_source(navigation);
#endif
    // Chassis UART -> interboard reassembly -> referee decoder -> navigation TX.
    // No referee UART is opened on the gimbal board.
#if IS_ENABLED(CONFIG_SKYWALKER_REFEREE) && IS_ENABLED(CONFIG_SKYWALKER_INTERBOARD_ENDPOINT)
    static InterBoardRefereeSource referee_source(endpoint);
#if DT_HAS_ALIAS(navigation_uart) && IS_ENABLED(CONFIG_SKYWALKER_NAVIGATION_RECEIVER)
    referee_source.setFrameHandler(
        [](const std::uint8_t *bytes, std::size_t size, void *context) {
            auto &receiver = *static_cast<communication::navigation::NavigationReceiver *>(context);
            (void)receiver.forwardRefereeFrame(bytes, size);
        },
        &navigation);
#endif
#endif
    // 命令配置只在这里构造一次：仲裁器与上电启动动作判定必须使用同一份
    // 映射/巡航/键鼠语义，不能另建一份缺字段的默认 mapper。
    static const auto command_config = [] {
        auto config = installed::manualCommandConfig();
        config.mapper.input_profile = RemoteInputProfile::KeyboardMouseSelectable;
        // 解锁即合并键鼠：不再要求右拨杆上拨选择来源，摇杆离开死区时按轴优先。
        config.mapper.merge_keyboard_mouse = true;
        config.allow_navigation = DT_HAS_ALIAS(navigation_uart) && IS_ENABLED(CONFIG_SKYWALKER_NAVIGATION_RECEIVER);
        config.allow_auto = DT_HAS_ALIAS(vision_uart) && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER);
        config.mapper.auto_on_up = false;
        // 左 Middle 由右拨杆选择普通导航/巡航/巡航+小陀螺。
        config.enable_cruise = true;
        config.cruise_yaw_rate_rad_s = installed::cruise_yaw_rate_rad_s;
        config.cruise_pitch_rate_rad_s = installed::cruise_pitch_rate_rad_s;
        config.cruise_pitch_span_rad = installed::cruise_pitch_span_rad;
        config.cruise_pitch_min_rad = installed::cruise_pitch_min_rad;
        config.cruise_pitch_max_rad = installed::cruise_pitch_max_rad;
        config.requested_fire_rate_hz = installed::requested_fire_rate_hz;
        config.mapper.cruise_on_middle = true;
        config.mapper.navigation_on_middle = true;
        config.require_referee_for_chassis = true;
        config.require_referee_for_gimbal = true;
        return config;
    }();
    static CommandManager commands(command_config);
    // 板载 BMI088 测量大 Yaw 载体，继续独立发布给底盘；CAN3 IMU 随枪头运动，
    // 负责本地 Yaw/Pitch 姿态外环，不能替换板间载体航向。
    static control::QuaternionEkf onboard_estimator({.initialization_samples = 800, .adapt_gyro_bias = false});
    static imu::Bmi088Imu onboard_source(DEVICE_DT_GET(DT_ALIAS(accel0)), DEVICE_DT_GET(DT_ALIAS(gyro0)),
                                         {.reference = installed::carrier_reference,
                                          .sensor_to_body = installed::carrier_sensor_to_body,
                                          .correct_gyro_bias = false,
                                          .planar_yaw_enabled = true},
                                         &onboard_estimator);
    static imu::ImuReceiver onboard(onboard_source, {.poll_interval_us = 500, .priority = 5});
    static imu::DmImuCanSource head_source(installed::headSensorConfig());
    static imu::ImuReceiver head(head_source, {.poll_interval_us = 1000, .priority = 6});
    static const auto inertial_config = installed::sentryInertialConfig();
    static InertialGimbalAdapter head_control(inertial_config);
    // 双 Yaw 分配器是正式链路里唯一的大小 Yaw 分工点：大 Yaw 主动承担持续运动
    // 与回中，小 Yaw 补偿实测残差并守住行程。旧的 YawCenteringController
    // （robotics/gimbal/yaw_centering.hpp）已作废，不再参与本应用。
    static DualYawMotionAllocator dual_yaw(installed::dualYawAllocatorConfig());
    // Variable-speed top spin: the curve is vehicle::spinRateCurveRadS().
    static SpinSpeedProfile spin(installed::spinProfileConfig());
    static motor::Motor yaw(installed::yawHardware()), pitch(installed::pitchHardware());
    static motor::CanBus yaw_bus(DEVICE_DT_GET(DT_NODELABEL(can1)));
    static motor::CanBus pitch_bus(DEVICE_DT_GET(DT_NODELABEL(can2)));
    static motor::Group group(yaw, pitch);
    static const auto yaw_motor_config = installed::yawMotorConfig();
    static const auto pitch_motor_config = installed::pitchMotorConfig();
    static GimbalExecutor gimbal(yaw, pitch, group, yaw_motor_config, installed::yawAxisConfig(), pitch_motor_config,
                                 installed::pitchAxisConfig(), installed::gimbalExecutionConfig());
    // 发射机构与 GM6020 小 Yaw 共用 CAN1：GM6020 走 0x1FF，M3508/M2006 走 0x200，
    // 反馈 0x207 与 0x201/0x202/0x204 互不冲突。摩擦轮与拨盘各自成组，共用一次 commit。
    static motor::Motor friction_left(installed::frictionHardware(0)), friction_right(installed::frictionHardware(1)),
        dial(installed::dialHardware());
    static motor::Group friction_group(friction_left, friction_right), dial_group(dial);
    static ShooterExecutor shooter(friction_left, friction_right, dial, friction_group, dial_group,
                                   installed::frictionMotorConfig(), installed::dialMotorConfig(),
                                   installed::shooterExecutionConfig());
    int command_error = commands.registerSource(remote_source);
#if DT_HAS_ALIAS(navigation_uart) && IS_ENABLED(CONFIG_SKYWALKER_NAVIGATION_RECEIVER)
    if (command_error == 0)
        command_error = commands.registerSource(navigation_source);
#endif
#if DT_HAS_ALIAS(vision_uart) && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER)
    if (command_error == 0)
        command_error = commands.registerSource(vision_source);
#endif
#if IS_ENABLED(CONFIG_SKYWALKER_REFEREE) && IS_ENABLED(CONFIG_SKYWALKER_INTERBOARD_ENDPOINT)
    if (command_error == 0)
        command_error = commands.bindPermissions(referee_source);
#endif
    const int link_error = endpoint.start();
    if (command_error == 0)
        command_error = commands.start();
    const int onboard_error = onboard.start();
    const int head_error = head.start();
    int yaw_error = installed::connections_confirmed ? yaw_bus.attach(yaw, friction_left, friction_right, dial)
                                                     : -ENODEV;
    int pitch_error = installed::connections_confirmed ? pitch_bus.attach(pitch) : -ENODEV;
    if (yaw_error == 0)
        yaw_error = yaw_bus.start();
    if (pitch_error == 0)
        pitch_error = pitch_bus.start();
    const bool yaw_started = yaw_error == 0, pitch_started = pitch_error == 0;
    const int control_error = yaw_started && pitch_started ? gimbal.begin() : -ENODEV;
    const int shooter_error = yaw_started ? shooter.begin() : -ENODEV;
    const int power_error = installed::connections_confirmed ? regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)))
                                                             : -ENODEV;
    diagnostics.event("interboard", link_error);
    diagnostics.event("command", command_error);
    diagnostics.event("carrier_imu", onboard_error);
    diagnostics.event("head_imu", head_error);
    diagnostics.event("yaw_bus", yaw_error);
    diagnostics.event("pitch_bus", pitch_error);
    diagnostics.event("gimbal", control_error);
    // 非零表示分配器参数非法：此时它按 Configuration 拒绝运行，回退旧编码器路径。
    diagnostics.event("dual_yaw", dual_yaw.validate());
    diagnostics.event("shooter", shooter_error);
    diagnostics.event("motor_rail", power_error);
    // 载体 IMU 门控大 Yaw；外置头部 IMU 门控视觉，手动保持旧编码器控制。
    const bool setup_ready = command_error == 0 && yaw_error == 0 && pitch_error == 0 && control_error == 0 &&
                             power_error == 0;
    // 左 Middle 导航档失联不撤权；左 Up 手动档失联仍立即锁定。
    // 巡航拨杆与“解锁后无动作”的快捷解锁只在正式应用启用；
    // 两个字段位于 Config 末尾，其余参数保留默认值。
    samples::control::RcControlAdapter controls({
        .enable_cruise_switches = true,
        .semantic_startup_unlock = true,
    });
    communication::RemoteReceiver::Snapshot remote_frame{};
    CommandSnapshot frame{};
    std::uint64_t next_send_ms = 0;
    std::uint32_t request_sequence = 0;
    std::int64_t deadline = k_uptime_ticks();
    const auto period = k_us_to_ticks_ceil64(installed::gimbal_period_us);
    bool peer_feedback_was_valid = false;
    std::uint64_t last_peer_boot = 0;
    bool was_peer_online = false;
    core::TimeUs previous_cycle_us = 0;
    // 仅记录上一周期已建立且成功输出的视觉 Pitch 参考；视觉被选中不等于轴已执行。
    bool vision_pitch_reference_ready = false;
    // 视觉 Rate 桥接会话：只在视觉驱动的 Rate 周期请求前馈；进入/退出时重建两轴会话。
    bool vision_rate_session_previous = false;
    bool keyboard_spin_latched = false, shift_was_down = false;
    SpinOwner previous_spin_owner = SpinOwner::None;
    bool scan_entry_pending = false, scan_was_selected = false;
    std::uint32_t last_startup_reasons = 0;
    core::OrientationReference last_head_reference{};
    bool head_reference_known = false, head_was_ready = false;
    ControlSource last_head_source = ControlSource::None;
#if defined(CONFIG_SKYWALKER_VEHICLE_DIAGNOSTICS)
    float response_big_yaw_rate = std::numeric_limits<float>::quiet_NaN();
    float response_max_dt_us = 0;
    std::uint32_t response_events = 0;
    int response_error = 0;
    ControlSource response_previous_source = ControlSource::None;
#endif
    for (;;) {
        // Snapshot before taking the validation time: higher-priority IMU
        // or interboard worker updates must not make a new stamp appear in our future.
        const auto carrier_snapshot = onboard.snapshot();
        const auto head_snapshot = head.snapshot();
        const auto peer_state = endpoint.snapshot();
        const auto now_us = core::monotonicTimeUs(), now_ms = now_us / 1000;
        const auto dt_us = previous_cycle_us && now_us > previous_cycle_us ? now_us - previous_cycle_us : 0;
        previous_cycle_us = now_us;
        // 先用管理线程现有快照判定“解锁后是否会产生动作”，再把新鲜遥控帧交给状态机。
        const bool have_action_snapshot = commands.snapshot(frame) == 0;
        (void)remote.snapshot(remote_frame);
        const auto startup_test = installed::assessStartupAction(remote_frame.remote,
                                                                 have_action_snapshot ? &frame.observed : nullptr,
                                                                 command_config, installed::spinProfileConfig(),
                                                                 now_us);
        auto operator_state = controls.update(remote_frame.remote, now_ms, startup_test.noAction());
        const bool peer_revoked = (was_peer_online && !peer_state.online) ||
                                  (last_peer_boot && peer_state.peer.sender_boot_id &&
                                   last_peer_boot != peer_state.peer.sender_boot_id);
        if (peer_state.peer.sender_boot_id)
            last_peer_boot = peer_state.peer.sender_boot_id;
        was_peer_online = peer_state.online;
        if (peer_revoked) {
            controls.withdraw();
            operator_state.armed = false;
            operator_state.run_allowed = false;
            operator_state.manual_active = false;
            operator_state.navigation_active = false;
            operator_state.navigation_hold = false;
            operator_state.autonomous = AutonomousMode::None;
            operator_state.mode = samples::control::RcMode::Safe;
        }
        // 左 Middle 导航档失联时 navigation_hold 维持导航授权；左 Up 手动档失联
        // 仍由 withdraw() 撤销 run_allowed 并锁定。
        const bool operator_authorized = operator_state.fresh || operator_state.navigation_hold;
        const bool autonomous_allowed = operator_state.navigation_active && operator_authorized && setup_ready;
        const bool gimbal_allowed = operator_state.run_allowed || autonomous_allowed;
        const bool cruise_selected = autonomous_allowed && isCruise(operator_state.autonomous);
        // 已授权的模式选择在同一临界区内交给命令线程；返回的行为版本用于拒绝旧快照。
        // selection.valid=true 只表示“本应用提供选择”，不授予运行权。
        OperatorSelection selection{};
        selection.valid = true;
        selection.mode = autonomous_allowed           ? OperatorMode::Navigation
                         : operator_state.run_allowed ? OperatorMode::Manual
                                                      : OperatorMode::Safe;
        selection.autonomous = autonomous_allowed ? operator_state.autonomous : AutonomousMode::None;
        selection.accepted_remote_stamp = operator_state.remote.stamp;
        const auto operator_session = static_cast<std::uint32_t>(operator_state.start_event_id);
        const auto expected_revision = commands.setOperatorGate(gimbal_allowed, operator_session, selection);
        const bool have_decision = commands.snapshot(frame) == 0;
        // 快照必须属于当前会话、当前行为版本和当前主模式；巡航两档视为同一行为，
        // 因此右上/右下互切可以复用同一会话内的云台决策。
        const auto &decision = frame.decision;
        const bool same_autonomous_behavior = decision.autonomous == selection.autonomous ||
                                              (isCruise(decision.autonomous) && isCruise(selection.autonomous));
        const bool decision_matches = have_decision && decision.operator_session == operator_session &&
                                      decision.operator_revision == expected_revision &&
                                      decision.operator_mode == selection.mode &&
                                      (selection.mode != OperatorMode::Navigation || same_autonomous_behavior);
        // The DR16 dial (spring-centred wheel) is the analog spin control. It
        // only scales and signs the chassis spin rate: translation still comes
        // from the sticks and the gimbal command is untouched.
        // 键鼠：Shift 单点锁存小陀螺开关，Q/E 充当拨轮方向（按住即满幅，E 正转、Q 反转）；
        // 拨轮回中的语义不变。解锁撤销时锁存清零，避免下次解锁自动起转。
        const std::uint16_t keyboard_bits = operator_state.remote.keyboard.bits;
        const bool shift_down = ((keyboard_bits >> 4) & 1u) != 0;
        if (shift_down && !shift_was_down && operator_state.run_allowed)
            keyboard_spin_latched = !keyboard_spin_latched;
        if (!operator_state.run_allowed)
            keyboard_spin_latched = false;
        shift_was_down = shift_down;
        const float wheel_dial = samples::control::RcControlAdapter::normalize(operator_state.remote.analog.wheel);
        const float keyboard_dial = ((keyboard_bits >> 7) & 1u) ? 1.0f : ((keyboard_bits >> 6) & 1u) ? -1.0f : 0.0f;
        const float manual_dial = keyboard_dial != 0.0f ? keyboard_dial : wheel_dial;
        // 小陀螺 profile 每 5 ms 只推进一次，并明确归属手动/巡航：Q/E 与拨轮
        // 只在手动档生效，巡航只由当前子模式请求。
        const bool chassis_command_fresh = isFresh(decision.command.chassis.stamp, now_ms,
                                                   command_config.input_timeout_ms);
        const bool chassis_permission_ok = (decision.chassis_reasons & PermissionDenied) == 0;
        const bool body_command_ready = decision_matches && chassis_command_fresh &&
                                        decision.command.chassis.mode == ChassisMode::BodyVelocity &&
                                        chassis_permission_ok;
        const SpinOwner spin_owner = operator_state.run_allowed && operator_state.fresh ? SpinOwner::Manual
                                     : cruise_selected                                  ? SpinOwner::Cruise
                                                                                        : SpinOwner::None;
        if (spin_owner != previous_spin_owner)
            spin.reset();
        previous_spin_owner = spin_owner;
        const bool spin_requested = spin_owner == SpinOwner::Cruise
                                        ? operator_state.autonomous == AutonomousMode::CruiseSpin
                                    : spin_owner == SpinOwner::Manual
                                        ? (operator_state.spin_requested || keyboard_spin_latched)
                                        : false;
        const float spin_intensity = spin_owner == SpinOwner::Manual ? manual_dial : 0.0f;
        const bool spin_allowed = spin_owner != SpinOwner::None && body_command_ready;
        const auto spin_out = spin.update(spin_requested, spin_allowed, spin_intensity, now_us);
        // 手动档已始终合并键鼠，命令来源固定为 KeyboardMouse。来源必须稳定：
        // 忽 Remote 忽 KeyboardMouse 会让回中控制器和惯性外环每帧复位。
        constexpr auto expected_manual_source = ControlSource::KeyboardMouse;
        const auto heading = planarYawHeading(carrier_snapshot, now_us);
        const bool heading_fresh = heading.valid && core::fresh(heading.stamp, now_us, 20000);
        // 外置 IMU 保留原始采样时间；只接受新鲜姿态/角速度和已知的有效参考。
        const auto &head_sample = head_snapshot.sample;
        auto head_orientation = head_sample.orientation.value;
        const bool head_quaternion_valid = core::normalize(head_orientation);
        const auto head_euler = core::euler(head_quaternion_valid ? head_orientation : core::Quaternion{});
        const bool head_orientation_fresh = (head_snapshot.fresh_mask & imu::Orientation) &&
                                            core::fresh(head_sample.orientation.stamp, now_us,
                                                        inertial_config.imu_timeout_us);
        const bool head_gyro_fresh = (head_snapshot.fresh_mask & imu::Gyro) &&
                                     core::fresh(head_sample.gyro_rad_s.stamp, now_us, inertial_config.imu_timeout_us);
        const bool head_quality_ok = head_sample.attitude_quality == imu::AttitudeQuality::Tracking ||
                                     (inertial_config.allow_unknown_quality &&
                                      head_sample.attitude_quality == imu::AttitudeQuality::Unknown);
        const bool head_reference_valid = head_sample.reference.frame_id && head_sample.reference.epoch;
        const unsigned head_reason = head_error                                                              ? 1u
                                     : head_snapshot.state != imu::State::Running                            ? 2u
                                     : !head_orientation_fresh                                               ? 3u
                                     : !head_gyro_fresh                                                      ? 4u
                                     : !head_quality_ok                                                      ? 5u
                                     : !head_reference_valid                                                 ? 6u
                                     : !head_quaternion_valid                                                ? 7u
                                     : !core::finite(head_sample.gyro_rad_s.value)                           ? 8u
                                     : std::fabs(head_euler.pitch) >= inertial_config.max_inertial_pitch_rad ? 9u
                                                                                                             : 0u;
        const bool head_usable = head_reason == 0;
        const bool head_reference_changed = head_usable && head_reference_known &&
                                            last_head_reference != head_sample.reference;
        if (head_usable) {
            last_head_reference = head_sample.reference;
            head_reference_known = true;
        }
        const bool head_ready = head_usable && !head_reference_changed;
        if (!head_ready || !head_was_ready) {
            if (last_head_source == ControlSource::Vision)
                dual_yaw.reset();
            (void)head_control.suspend(now_us, WaitReason::Reference);
        }
        head_was_ready = head_ready;
        GimbalExecutionInputs input{};
        input.command = decision.command.gimbal;
        input.source_stamp = sourceStamp(frame, input.command.source);
        // 模式、会话、行为版本与自主行为类别统一由 decision_matches 表达；
        // 它只说明“这份快照属于当前行为”，不替代命令/来源新鲜度检查。
        const bool gimbal_mode_matches = decision_matches;
        if (!gimbal_allowed || !gimbal_mode_matches ||
            (operator_state.run_allowed && input.command.source != expected_manual_source))
            input.command.mode = GimbalMode::Disabled;
        const float raw_head_rate_rad_s = input.command.yaw_rate_rad_s;
        const float raw_pitch_rate_rad_s = input.command.pitch_rate_rad_s;
        if (input.command.source != last_head_source) {
            dual_yaw.reset();
            (void)head_control.suspend(now_us, WaitReason::Command);
            last_head_source = input.command.source;
        }
        const auto ys = yaw.snapshot();
        const auto ps = pitch.snapshot();
        const bool pitch_valid = ps.feedback_fresh && ps.position_reference_valid &&
                                 bool(ps.feedback.valid & motor::FeedbackPosition) &&
                                 std::isfinite(ps.feedback.position_rad);
        const auto previous_pitch_execution = gimbal.pitchTelemetry();
        const bool pitch_reference_continuous = vision_pitch_reference_ready && pitch_valid && dt_us > 0 &&
                                                dt_us <= 20000 && ps.state == motor::MotorState::Active &&
                                                ps.output_permitted && previous_pitch_execution.output_valid &&
                                                gimbal.pitchTargetDiagnostics().target_valid &&
                                                ps.enable_generation ==
                                                    previous_pitch_execution.motor.enable_generation &&
                                                ps.reference_generation ==
                                                    previous_pitch_execution.motor.reference_generation;
        const bool joint_valid = ys.feedback_fresh && ys.position_reference_valid &&
                                 bool(ys.feedback.valid & motor::FeedbackPosition);
        // 把 Pitch 实测角推给命令线程：巡航扫描用它作为摆头中心。
        commands.setPlantState(ps.feedback.position_rad, pitch_valid);
        const auto &feedback = peer_state.big_yaw_feedback;
        const bool peer_feedback_valid = peer_state.online && feedback.valid &&
                                         forwardedFresh(feedback.stamp, feedback.production_age_ms, now_ms, 250) &&
                                         std::isfinite(feedback.actual_rate_rad_s) &&
                                         std::fabs(feedback.actual_rate_rad_s) <=
                                             installed::big_yaw.velocity_limit_rad_s;
        if (peer_feedback_valid != peer_feedback_was_valid)
            dual_yaw.reset();
        peer_feedback_was_valid = peer_feedback_valid;
        // --- 巡航扫描入口：只在新进入扫描时显式重播种两轴目标 -------------
        // 扫描期间 Yaw 是速率意图、Pitch 是速率往返（方向与折返由仲裁器维护），
        // 因此不能每帧重播种；退出扫描或反馈未就绪时保留 pending，
        // 本周期降级为新鲜 Hold 继续初始化。
        const bool scan_selected = decision_matches && cruise_selected &&
                                   decision.gimbal_behavior == GimbalBehavior::Scan &&
                                   input.command.source == ControlSource::LocalBehavior &&
                                   input.command.mode == GimbalMode::Rate;
        if (!scan_selected)
            scan_entry_pending = false;
        else if (!scan_was_selected)
            scan_entry_pending = true;
        scan_was_selected = scan_selected;
        if (scan_selected && scan_entry_pending) {
            if (joint_valid && pitch_valid && heading_fresh) {
                dual_yaw.reset();
                (void)head_control.suspend(now_us, WaitReason::Command);
                // 两个调用都成功才允许输出扫描速率；丢弃上一视觉会话的积分目标。
                // Pitch 重播种把摆头起点钉在当前实测角，与仲裁器的播种一致。
                if (gimbal.reseedYawTarget() == 0 && gimbal.reseedPitchTarget() == 0)
                    scan_entry_pending = false;
            }
            if (scan_entry_pending) {
                input.command.mode = GimbalMode::Hold;
                input.command.yaw_rate_rad_s = 0;
            }
        }
        // Only the arbitrated target is ever consumed. `selected_vision` is kept
        // by the manager exactly while the final gimbal source is Vision.
        const auto &selected_vision = frame.decision.selected_vision;
        const bool vision_selected = selected_vision.stamp.valid && input.command.source == ControlSource::Vision &&
                                     input.command.mode != GimbalMode::Disabled && gimbal_allowed &&
                                     gimbal_mode_matches;
        // 视觉角与外置 IMU 必须属于同一头部世界参考；不再用板载姿态和关节角估算枪头。
        const bool vision_active = vision_selected && head_ready && joint_valid && pitch_valid &&
                                   selected_vision.value.reference == head_sample.reference;
        if (vision_selected && !vision_active)
            input.command.mode = GimbalMode::Disabled;
        float vision_angle_rad = 0, vision_rate_rad_s = 0, vision_acceleration_rad_s2 = 0;
        float vision_pitch_rad = 0;
        if (vision_selected) {
            const auto &v = selected_vision.value.yaw;
            // AB carries no sample timestamp: the decoder stamps the local byte
            // reception time. Prediction is therefore limited to the measured
            // local age plus the separately verified capture/transport latency;
            // it must not be treated as the vision-side sample age.
            const float local_age_s = now_us >= selected_vision.stamp.time_us
                                          ? float(now_us - selected_vision.stamp.time_us) * 1e-6f
                                          : 0.0f;
            const float age_s = std::min(local_age_s + installed::vision_capture_latency_s, 0.05f);
            vision_angle_rad = v.angle_rad + v.rate_rad_s * age_s + 0.5f * v.acceleration_rad_s2 * age_s * age_s;
            vision_rate_rad_s = v.rate_rad_s + v.acceleration_rad_s2 * age_s;
            vision_acceleration_rad_s2 = v.acceleration_rad_s2;
            const auto &p = selected_vision.value.pitch;
            vision_pitch_rad = p.angle_rad + p.rate_rad_s * age_s + 0.5f * p.acceleration_rad_s2 * age_s * age_s;
        }
        // --- 双 Yaw 分配：每周期唯一一次，必须早于头部外环 --------------------
        // 输入是“已仲裁的头部世界速率意图”（手动）或视觉轨迹（角度/速度/加速度），
        // 输出是可行头部世界参考 + 大 Yaw 世界意图 + 带行程保护的小 Yaw 关节速率。
        DualYawAllocatorInputs alloc_input{};
        alloc_input.head_rate_target_rad_s = raw_head_rate_rad_s;
        alloc_input.joint_rad = ys.feedback.position_rad;
        alloc_input.joint_rate_rad_s = bool(ys.feedback.valid & motor::FeedbackVelocity) ? ys.feedback.velocity_rad_s
                                                                                         : 0.0f;
        alloc_input.joint_stamp = {ys.feedback.timestamp_ms * 1000ULL, ys.feedback.timestamp_ms, joint_valid};
        alloc_input.carrier_rate_rad_s = heading_fresh ? heading.rate_rad_s : 0.0f;
        alloc_input.carrier_stamp = heading.stamp;
        alloc_input.carrier_valid = heading_fresh;
        alloc_input.source_stamp = input.source_stamp;
        // 头部实测世界角只用于重播种/超前角保护；手动模式不因此依赖外置 IMU。
        alloc_input.head_angle_measured_rad = head_usable ? head_euler.yaw : 0.0f;
        alloc_input.head_angle_valid = head_usable;
        // 分配器假定小 Yaw 真正执行它给出的关节速率。仲裁器在左中档（导航/自动，
        // 本机 allow_auto 打开）没有视觉目标时给出的是 GimbalMode::Hold ——
        // 小 Yaw 只锁关节角、不再补偿载体，关节偏角因此永远不变；若此时仍按
        // center_rate 驱动大 Yaw，就会以恒定速率一直转下去（越补越偏）。
        // 所以只有小 Yaw 真正执行运动指令时才让分配器接管大 Yaw。
        const bool yaw_motion_commanded = vision_active || input.command.mode == GimbalMode::Rate ||
                                          input.command.mode == GimbalMode::AbsoluteAngle;
        alloc_input.enabled = gimbal_allowed && yaw_motion_commanded;
        alloc_input.permission_valid = gimbal_allowed && installed::connections_confirmed &&
                                       installed::imu_mounting_confirmed;
        // 大 Yaw 通道是否可用的判定与发送门控分离：真正的 mode/rate 仍在下面
        // big_channel_ready 之后决定，这里只决定本周期是否推进大 Yaw 意图。
        alloc_input.big_yaw_available = yaw_motion_commanded && peer_state.online && peer_feedback_valid &&
                                        heading_fresh && joint_valid;
        alloc_input.vision_active = vision_active;
        alloc_input.vision_angle_rad = vision_angle_rad;
        alloc_input.vision_rate_rad_s = vision_rate_rad_s;
        alloc_input.vision_acceleration_rad_s2 = vision_acceleration_rad_s2;
        // 视觉参考代次变化必须在分配器里重新播种，不能沿用上一参考下的角度。
        alloc_input.vision_epoch = head_usable ? head_sample.reference.epoch : 0;
        const auto allocation = dual_yaw.update(alloc_input, now_us);
        if (allocation.joint_reseed_requested)
            (void)gimbal.reseedYawTarget();
        InertialGimbalInputs inertial_input{};
        inertial_input.head = head_snapshot;
        inertial_input.yaw = ys;
        inertial_input.pitch = ps;
        inertial_input.command = input.command;
        inertial_input.source_stamp = input.source_stamp;
        inertial_input.prerequisites_ready = setup_ready && installed::imu_mounting_confirmed;
        // The received target and the feedback quaternion share the CAN IMU's
        // reference. Never rebuild this target in the carrier's independent zero.
        // 分配器给出可行头部世界角/速率，头部外环只负责把实测姿态闭环到它；
        // 分配器不可用时才退回视觉原始目标，保证失配时仍有旧的视觉行为。
        inertial_input.yaw_reference_valid = vision_active;
        inertial_input.yaw_reference_rad = allocation.head_reference_valid ? allocation.head_angle_rad
                                                                           : vision_angle_rad;
        inertial_input.yaw_reference_rate_rad_s = allocation.head_reference_valid ? allocation.head_reference_rate_rad_s
                                                                                  : vision_rate_rad_s;
        inertial_input.command.mode = vision_active ? GimbalMode::Rate : GimbalMode::Disabled;
        const auto inertial = head_control.update(inertial_input, now_us);
        if (vision_active) {
            input.command = inertial.command;
            input.source_stamp = inertial.source_stamp;
            input.yaw_output_valid = inertial.yaw_output_valid;
            // 头部外环的关节速率仍必须服从分配器实时的行程/制动余量，否则
            // 视觉修正可以把小 Yaw 推过软端点。
            if (allocation.joint_rate_valid)
                input.command.yaw_rate_rad_s = std::clamp(input.command.yaw_rate_rad_s,
                                                          -allocation.outward_budget_neg_rad_s,
                                                          allocation.outward_budget_pos_rad_s);
            // Remove the measured Pitch joint from the *head CAN quaternion*.
            // This leaves the yaw-stage pose in exactly the vision world frame.
            const auto yaw_pose = core::multiply(head_orientation,
                                                 core::fromEuler({0,
                                                                  -installed::pitch.direction *
                                                                      (ps.feedback.position_rad -
                                                                       installed::vision_pitch_horizontal_rad),
                                                                  0}));
            const float cp = std::cos(vision_pitch_rad);
            const core::Vec3 world_ray{cp * std::cos(vision_angle_rad), cp * std::sin(vision_angle_rad),
                                       -std::sin(vision_pitch_rad)};
            const auto joint_ray = core::rotate(core::conjugate(yaw_pose), world_ray);
            // A target behind the current yaw must not produce a 180-degree
            // Pitch command. Aim elevation first; yaw alignment finishes the ray.
            const float target = std::clamp(installed::vision_pitch_horizontal_rad +
                                                installed::pitch.direction *
                                                    std::atan2(-joint_ray.z, std::hypot(joint_ray.x, joint_ray.y)),
                                            installed::pitch_min_rad, installed::pitch_max_rad);
            const float seed = pitch_reference_continuous ? static_cast<float>(gimbal.pitchTargetRad())
                                                          : ps.feedback.position_rad;
            const float dt = dt_us > 0 && dt_us <= 20000 ? float(dt_us) * 1e-6f : 0;
            // Keep Rate mode for BOTH axes: AbsoluteAngle would replace the
            // IMU yaw correction with a world angle interpreted as a joint angle.
            // 恢复周期先让轴从新鲜反馈播种。不能把基于旧目标算出的追赶速率
            // 积分到新种子上；下一次连续执行周期再追踪几何目标。
            input.command.pitch_rate_rad_s = pitch_reference_continuous && dt > 0
                                                 ? std::clamp((target - seed) / dt,
                                                              -installed::manual_max_gimbal_pitch_rate_rad_s,
                                                              installed::manual_max_gimbal_pitch_rate_rad_s)
                                                 : 0;
        }
        else if (input.command.mode == GimbalMode::Rate) {
            // 小 Yaw 关节速率直接来自分配器：已含实测载体补偿、头部加速度窗和
            // 行程度包络；退化输出同样带保护。只有分配器完全不可用时才回退到
            // 旧的“原始头部速率减载体角速度”公式。
            input.command.yaw_rate_rad_s = allocation.joint_rate_valid
                                               ? allocation.joint_rate_rad_s
                                               : installed::small_yaw.direction *
                                                     (raw_head_rate_rad_s -
                                                      (heading_fresh ? heading.rate_rad_s : 0.0f));
            // 手动输入上拨为正（抬头），先转换成姿态/视觉约定的低头正方向。
            // 摇杆与鼠标已合并，在此统一转换以保留两者现有操控方向。
            input.command.pitch_rate_rad_s = -raw_pitch_rate_rad_s * installed::pitch.direction;
        }
        input.pitch_output_valid = pitch_valid;
        // --- 视觉 Rate 桥接：把轴实际积分的关节参考速度交给位置环 -------------
        // 仅当本周期确实由视觉 Rate 路径驱动、且该轴输出有效时才请求；手动、
        // Hold、Disabled 与分配器轨迹路径保持原行为。进入/退出视觉会话时两轴都
        // 重建 Rate 会话，避免上一来源的积分目标被继续积分或当作追赶债务执行。
        const bool vision_rate_session = vision_active && input.command.mode == GimbalMode::Rate;
        const bool rate_session_changed = vision_rate_session != vision_rate_session_previous;
        vision_rate_session_previous = vision_rate_session;
        input.yaw_use_rate_reference = vision_rate_session && input.yaw_output_valid;
        input.pitch_use_rate_reference = vision_rate_session && pitch_valid;
        input.yaw_restart_rate_session = rate_session_changed ||
                                         (vision_rate_session && allocation.joint_reseed_requested);
        input.pitch_restart_rate_session = rate_session_changed || (vision_rate_session && !pitch_reference_continuous);
        input.transport_ready = setup_ready;
        auto status = control_error == 0 ? gimbal.update(input, now_us)
                                         : gimbal.suspend(now_us, WaitReason::Configuration, control_error, true);
        // 执行器可能因漏周期直接返回，也可能只有 Yaw 成功；不能只看 vision_active
        // 或整机 Active。首个成功播种周期允许下一周期追赶，不要求已接受速度前馈。
        vision_pitch_reference_ready = vision_rate_session && status.state == RunState::Active &&
                                       gimbal.pitchTelemetry().output_valid &&
                                       gimbal.pitchTargetDiagnostics().target_valid;
        // 板间大 Yaw 的发送门控仍然在本周期执行结果之后再判断：分配器只提供
        // 世界速率意图，能不能发出去由仲裁、对端在线、对端反馈和本机执行状态决定。
        const auto center_yaw = yaw.snapshot();
        const bool center_joint_valid = center_yaw.feedback_fresh && center_yaw.position_reference_valid &&
                                        bool(center_yaw.feedback.valid & motor::FeedbackPosition);
        const bool yaw_running = gimbal.yawTelemetry().output_valid;
        const bool big_channel_ready = setup_ready && operator_authorized && gimbal_allowed && decision_matches &&
                                       peer_state.online && installed::imu_mounting_confirmed && heading_fresh &&
                                       center_joint_valid && yaw_running;
#if DT_HAS_ALIAS(vision_uart) && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER)
        communication::vision::Feedback vision_feedback{};
        // 全部授权自主子模式（含掉线保持）维持 AutoAim，扫描期间视觉仍能发现目标。
        vision_feedback.mode = autonomous_allowed ? communication::vision::Mode::AutoAim
                                                  : communication::vision::Mode::Idle;
        vision_feedback.reference = head_sample.reference;
        if (head_ready) {
            // Preserve acquisition stamps. No carrier fallback, synthetic yaw,
            // or refreshed timestamps when the external sensor stops reporting.
            vision_feedback.orientation = head_sample.orientation;
            vision_feedback.gyro_rad_s = head_sample.gyro_rad_s;
        }
        // Ballistics remain invalid measurements; the explicitly selected AB
        // bringup option serializes unknown speed/count as zero.
        (void)vision.setFeedback(vision_feedback);
#endif
        // 发射机构：命令来自仲裁器的 ShooterCommand（鼠标点击自动启摩擦轮并延时停转，
        // 视觉请求自动启停供弹并短时保活摩擦轮，右键保留手动预转）。
        // 尚未接入裁判热量模型，先关闭热量与许可门控；摩擦轮/供弹仍受执行器自身的
        // 反馈就绪、云台 Active 和拨盘参考约束，初始化失败时直接停机。
        ShooterExecutionInputs shooter_input{};
        shooter_input.command = frame.decision.command.shooter;
        const bool shooter_source_matches = operator_state.run_allowed
                                                ? shooter_input.command.source == expected_manual_source
                                                : autonomous_allowed &&
                                                      (shooter_input.command.source == ControlSource::Vision ||
                                                       shooter_input.command.source == ControlSource::LocalBehavior);
        if (!gimbal_allowed || !decision_matches || !shooter_source_matches)
            shooter_input.command.mode = ShooterMode::Disabled;
        shooter_input.source_stamp = sourceStamp(frame, shooter_input.command.source);
        shooter_input.transport_ready = setup_ready && shooter_error == 0;
        shooter_input.gimbal = status;
        shooter_input.allow_feed = installed::shooter_constraints_confirmed;
        shooter_input.require_permission = false;
        shooter_input.require_heat = false;
        shooter_input.emergency_stop = !setup_ready;
        (void)shooter.update(shooter_input, now_us);
        if (now_ms >= next_send_ms) {
            next_send_ms = now_ms + 10;
            BigYawRequest request{};
            request.source_sequence = static_cast<std::uint32_t>(input.source_stamp.sequence);
            request.source_age_ms = age(input.source_stamp, now_us);
            request.command_age_ms = age(input.command.stamp, now_ms);
            // 应用层授权：扫描/视觉/手动各自给出授权戳；遥控掉线时可以授权，
            // 但不把“离线”冒充成遥控新鲜。
            const bool local_authority = autonomous_allowed && decision_matches &&
                                         input.command.source == ControlSource::LocalBehavior;
            const bool visual_authority = autonomous_allowed && decision_matches && vision_active;
            const bool manual_authority = operator_state.run_allowed && operator_state.fresh && decision_matches;
            const bool authority_valid = local_authority || visual_authority || manual_authority;
            request.permission = {authority_valid, authority_valid && big_channel_ready, input.command.stamp};
            // Reference v4 world intent: the peer applies motor sign/gyro control.
            // Zero enable obtains initial real feedback before any nonzero follow.
            // Restore the original single readiness gate for mode and rate.
            request.mode = big_channel_ready ? BigYawMode::FollowCenter : BigYawMode::Disabled;
            request.target_rate_rad_s = big_channel_ready && peer_feedback_valid && allocation.enabled
                                            ? allocation.big.rate_rad_s
                                            : 0;
            if (peer_state.online && peer_state.peer.sender_boot_id) {
                GimbalImuYaw sample{};
                sample.valid = heading_fresh;
                sample.yaw_rad = heading.yaw_rad;
                sample.yaw_rate_rad_s = heading.rate_rad_s;
                sample.zero_generation = heading.generation;
                sample.sample_sequence = static_cast<std::uint32_t>(heading.stamp.sequence);
                sample.source_age_ms = heading.stamp.valid && now_us >= heading.stamp.time_us
                                           ? static_cast<std::uint32_t>(
                                                 std::min<std::uint64_t>((now_us - heading.stamp.time_us + 999) / 1000,
                                                                         UINT32_MAX))
                                           : UINT32_MAX;
                sample.receiver_boot_id = peer_state.peer.sender_boot_id;
                sample.stamp = {now_ms, request_sequence + 1, true};
                endpoint.submitGimbalImuYaw(sample);
            }
            request.stamp = {now_ms, ++request_sequence, true};
            endpoint.submitBigYaw(request);
#if defined(CONFIG_SKYWALKER_VEHICLE_DIAGNOSTICS)
            response_big_yaw_rate = request.target_rate_rad_s;
#endif
            auto wheel_command = decision.command.chassis;
            // 普通导航失联保活且没有导航速度指令时，发零速 BodyVelocity 保持底盘
            // 使能。巡航已由仲裁器主动产生合法零平移命令，不依赖这个补丁。
            if (autonomous_allowed && operator_state.navigation_hold && decision_matches &&
                !isCruise(operator_state.autonomous) && wheel_command.mode == ChassisMode::Disabled &&
                wheel_command.stamp.valid) {
                wheel_command.mode = ChassisMode::BodyVelocity;
                wheel_command.vx_m_s = wheel_command.vy_m_s = wheel_command.wz_rad_s = 0;
                wheel_command.source = ControlSource::Autonomous;
            }
            // Reject a cached command from the previous RC mode. 普通导航的 wz 来自
            // 导航帧；巡航与手动的 wz 只由本地小陀螺曲线生成。
            const bool autonomous_command = autonomous_allowed && decision_matches &&
                                            wheel_command.source == ControlSource::Autonomous;
            const bool manual_command = operator_state.run_allowed && operator_state.fresh && decision_matches &&
                                        wheel_command.source == expected_manual_source;
            if (!chassis_command_fresh || (!autonomous_command && !manual_command)) {
                wheel_command.mode = ChassisMode::Disabled;
                wheel_command.vx_m_s = wheel_command.vy_m_s = wheel_command.wz_rad_s = 0;
            }
            else if (wheel_command.mode != ChassisMode::Disabled && chassis_permission_ok &&
                     (manual_command || (autonomous_command && cruise_selected)))
                wheel_command.wz_rad_s = spin_out.applied_rad_s;
            endpoint.submit(wheel_command);
        }
        // Original publication order: submit requests, then commit each local
        // bus once. Shooter shares CAN1 and remains in that same commit.
        if (yaw_started) {
            const int error = yaw_bus.commit().error;
            if (error < 0) {
                status.error = error;
#if defined(CONFIG_SKYWALKER_VEHICLE_DIAGNOSTICS)
                response_events |= 1u << 14;
#endif
            }
        }
        if (pitch_started) {
            const int error = pitch_bus.commit().error;
            if (error < 0) {
                status.error = error;
#if defined(CONFIG_SKYWALKER_VEHICLE_DIAGNOSTICS)
                response_events |= 1u << 15;
#endif
            }
        }
        endpoint.setStatus(status);
        const auto reason = !setup_ready || link_error || onboard_error || (vision_selected && head_error)
                                ? installed::DiagnosticReason::InitFailed
                            // 遥控离线但已授权自主保持时不再报告链路故障。
                            : (!operator_state.fresh && !operator_state.navigation_hold)
                                ? installed::DiagnosticReason::RemoteStale
                            : !gimbal_allowed                    ? installed::DiagnosticReason::Disarmed
                            : !installed::imu_mounting_confirmed ? installed::DiagnosticReason::Configuration
                            : !heading_fresh || (vision_selected && !head_ready) ? installed::DiagnosticReason::ImuStale
                            : !peer_state.online || !peer_feedback_valid ? installed::DiagnosticReason::PeerStale
                            : !joint_valid || !yaw_running               ? installed::DiagnosticReason::FeedbackStale
                            : status.state == RunState::Active           ? installed::DiagnosticReason::Ready
                                                                         : installed::DiagnosticReason::CommandStale;
        (void)status_led.setStatus(
            installed::statusLedFor(reason, status.state == RunState::Active, false, status.error));
#if defined(CONFIG_SKYWALKER_VEHICLE_DIAGNOSTICS)
        const float unavailable = std::numeric_limits<float>::quiet_NaN();
        // Schema 113: 32 channels only. Window events are collected at control
        // rate and cleared only after Diagnostics accepts a complete frame.
        if (startup_test.reasons != last_startup_reasons) {
            last_startup_reasons = startup_test.reasons;
            diagnostics.event("startup_action", static_cast<int>(startup_test.reasons));
        }
        const auto yaw_execution = gimbal.yawTelemetry();
        const auto pitch_execution = gimbal.pitchTelemetry();
        const auto &yaw_target = gimbal.yawTargetDiagnostics();
        const auto &pitch_target = gimbal.pitchTargetDiagnostics();
        const bool cycle_invalid = dt_us == 0 || dt_us > 20000 || status.reason == WaitReason::Cycle;
        const bool yaw_executed = !cycle_invalid && status.state == RunState::Active && input.yaw_output_valid &&
                                  yaw_execution.output_valid;
        const bool pitch_executed = !cycle_invalid && status.state == RunState::Active && input.pitch_output_valid &&
                                    pitch_execution.output_valid;
        const bool yaw_reference_accepted = yaw_executed && yaw_target.reference_accepted;
        const bool pitch_reference_accepted = pitch_executed && pitch_target.reference_accepted;
        const bool yaw_velocity_valid = ys.feedback_fresh && bool(ys.feedback.valid & motor::FeedbackVelocity) &&
                                        std::isfinite(ys.feedback.velocity_rad_s);
        const bool pitch_velocity_valid = ps.feedback_fresh && bool(ps.feedback.valid & motor::FeedbackVelocity) &&
                                          std::isfinite(ps.feedback.velocity_rad_s);
        static_assert(unsigned(ControlSource::LocalBehavior) < 16 && unsigned(GimbalMode::AbsoluteAngle) < 8);
        const std::uint32_t response_state = unsigned(input.command.source) | (unsigned(input.command.mode) << 4) |
                                             (yaw_executed ? 1u << 7 : 0u) | (pitch_executed ? 1u << 8 : 0u) |
                                             (input.yaw_use_rate_reference ? 1u << 9 : 0u) |
                                             (input.pitch_use_rate_reference ? 1u << 10 : 0u) |
                                             (yaw_reference_accepted ? 1u << 11 : 0u) |
                                             (pitch_reference_accepted ? 1u << 12 : 0u) |
                                             (vision_active ? 1u << 13 : 0u) | (head_usable ? 1u << 14 : 0u) |
                                             (joint_valid ? 1u << 15 : 0u) | (pitch_valid ? 1u << 16 : 0u) |
                                             (big_channel_ready ? 1u << 17 : 0u);
        const auto axisEvents = [](const control::PositionMotor::Telemetry &execution,
                                   const GimbalAxis::TargetStatus &target, bool executed, bool observed,
                                   float effort_limit, unsigned axis) {
            const auto &out = execution.output;
            std::uint32_t events = 0;
            if (executed) {
                if (out.velocity_request_limited)
                    events |= 1u << axis;
                if (std::fabs(out.velocity_request_rad_s - out.velocity.velocity_reference_rad_s) >
                    1e-5f * (1 + std::fabs(out.velocity_request_rad_s)))
                    events |= 1u << (2 + axis);
                if (out.position.saturated)
                    events |= 1u << (4 + axis);
                if (out.velocity.regulator.feedback.saturated ||
                    std::fabs(out.velocity.regulator.output) > effort_limit ||
                    std::fabs(execution.effort_command) >= effort_limit)
                    events |= 1u << (6 + axis);
            }
            else
                events |= 1u << (12 + axis);
            if (observed) {
                if (target.mechanical_limited || target.lead_limited)
                    events |= 1u << (8 + axis);
                if (target.restart_reason != AxisRestartReason::None ||
                    (target.target_valid && target.integration_dt_s == 0 && target.reference_requested))
                    events |= 1u << (10 + axis);
                if (execution.error)
                    events |= 1u << (14 + axis);
            }
            return events;
        };
        response_events |= axisEvents(yaw_execution, yaw_target, yaw_executed, !cycle_invalid,
                                      yaw_motor_config.loop.velocity.effort_abs_max, 0);
        response_events |= axisEvents(pitch_execution, pitch_target, pitch_executed, !cycle_invalid,
                                      pitch_motor_config.loop.velocity.effort_abs_max, 1);
        if (allocation.travel_limited ||
            (vision_active && allocation.joint_rate_valid &&
             std::fabs(inertial.command.yaw_rate_rad_s - input.command.yaw_rate_rad_s) > 1e-5f))
            response_events |= 1u << 16;
        if (input.command.source != response_previous_source || head_reference_changed || rate_session_changed)
            response_events |= 1u << 17;
        response_previous_source = input.command.source;
        if (cycle_invalid)
            response_events |= 1u << 18;
        if (!heading_fresh || (vision_selected && !head_ready))
            response_events |= 1u << 19;
        if (allocation.tracking_limited)
            response_events |= 1u << 20;
        if (allocation.big_saturated)
            response_events |= 1u << 21;
        const int cycle_error = status.error                      ? status.error
                                : yaw_execution.error             ? yaw_execution.error
                                : pitch_execution.error           ? pitch_execution.error
                                : vision_selected && !head_usable ? head_snapshot.diagnostics.last_error
                                                                  : 0;
        if (cycle_error)
            response_error = cycle_error;
        response_max_dt_us = std::max(response_max_dt_us, float(dt_us));
        const bool consumed = diagnostics.gimbalResponse(
            {.time_ms = now_ms,
             .state = response_state,
             .error = response_error,
             .signals = {vision_active ? inertial_input.yaw_reference_rad : unavailable, // I9 head world yaw reference
                         head_usable ? head_euler.yaw : unavailable,                     // I10 measured world yaw
                         vision_active ? vision_pitch_rad : unavailable, // I11 predicted world pitch target
                         head_usable ? head_euler.pitch : unavailable,   // I12 measured world pitch
                         yaw_executed && yaw_target.target_valid ? float(gimbal.yawTargetRad()) : unavailable, // I13
                         joint_valid ? ys.feedback.position_rad : unavailable,                                 // I14
                         yaw_reference_accepted ? yaw_target.reference_velocity_rad_s : unavailable,           // I15
                         yaw_executed ? yaw_execution.output.velocity_request_rad_s : unavailable,             // I16
                         yaw_executed ? yaw_execution.output.velocity.velocity_reference_rad_s : unavailable,  // I17
                         yaw_velocity_valid ? ys.feedback.velocity_rad_s : unavailable,                        // I18
                         yaw_executed ? yaw_execution.effort_command : unavailable,                            // I19 A
                         pitch_executed && pitch_target.target_valid ? float(gimbal.pitchTargetRad())
                                                                     : unavailable,                               // I20
                         pitch_valid ? ps.feedback.position_rad : unavailable,                                    // I21
                         pitch_reference_accepted ? pitch_target.reference_velocity_rad_s : unavailable,          // I22
                         pitch_executed ? pitch_execution.output.velocity_request_rad_s : unavailable,            // I23
                         pitch_executed ? pitch_execution.output.velocity.velocity_reference_rad_s : unavailable, // I24
                         pitch_velocity_valid ? ps.feedback.velocity_rad_s : unavailable,                         // I25
                         pitch_executed ? pitch_execution.effort_command : unavailable,                   // I26 Nm
                         alloc_input.carrier_valid ? alloc_input.carrier_rate_rad_s : unavailable,        // I27
                         inertial.carrier_yaw_rate_valid ? inertial.carrier_yaw_rate_rad_s : unavailable, // I28
                         response_big_yaw_rate}, // I29 last submitted, permission-gated big-Yaw world intent
             .max_dt_us = response_max_dt_us,
             .events = response_events});
        if (consumed) {
            response_events = 0;
            response_error = 0;
            response_max_dt_us = 0;
        }
#endif
        deadline += period;
        const auto ticks = k_uptime_ticks();
        if (deadline <= ticks)
            deadline += ((ticks - deadline) / period + 1) * period;
        k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
    }
}
