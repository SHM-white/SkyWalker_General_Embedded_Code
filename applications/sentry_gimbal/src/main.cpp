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
    static installed::Diagnostics diagnostics;
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
    static CommandManager commands([] {
        auto config = installed::manualCommandConfig();
        config.mapper.input_profile = RemoteInputProfile::KeyboardMouseSelectable;
        // 解锁即合并键鼠：不再要求右拨杆上拨选择来源，摇杆离开死区时按轴优先。
        config.mapper.merge_keyboard_mouse = true;
        config.allow_navigation = DT_HAS_ALIAS(navigation_uart) && IS_ENABLED(CONFIG_SKYWALKER_NAVIGATION_RECEIVER);
        config.allow_auto = DT_HAS_ALIAS(vision_uart) && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER);
        config.mapper.auto_on_up = false;
        config.require_referee_for_chassis = true;
        config.require_referee_for_gimbal = true;
        return config;
    }());
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
    static GimbalExecutor gimbal(yaw, pitch, group, installed::yawMotorConfig(), installed::yawAxisConfig(),
                                 installed::pitchMotorConfig(), installed::pitchAxisConfig(),
                                 installed::gimbalExecutionConfig());
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
    samples::control::RcControlAdapter controls;
    communication::RemoteReceiver::Snapshot remote_frame{};
    CommandSnapshot frame{};
    std::uint64_t next_send_ms = 0;
    std::uint32_t request_sequence = 0;
    std::int64_t deadline = k_uptime_ticks();
    const auto period = k_us_to_ticks_ceil64(installed::gimbal_period_us);
    bool peer_feedback_was_valid = false;
    std::uint64_t last_peer_boot = 0;
    bool was_peer_online = false;
    core::TimeUs previous_cycle_us = 0, last_heading_loss_dt_us = 0;
    bool heading_was_fresh = false, have_heading_loss = false;
    HeadingWaitReason last_heading_loss_reason = HeadingWaitReason::None;
    core::TimeUs heading_loss_start_us = 0, heading_loss_duration_us = 0;
    bool vision_pitch_was_active = false;
    bool keyboard_spin_latched = false, shift_was_down = false;
    core::OrientationReference last_head_reference{};
    bool head_reference_known = false, head_was_ready = false;
    ControlSource last_head_source = ControlSource::None;
    for (;;) {
        // Snapshot before taking the validation time: a higher-priority IMU
        // update must not make a new gyro stamp appear to be in our future.
        const auto carrier_snapshot = onboard.snapshot();
        const auto head_snapshot = head.snapshot();
        const auto now_us = core::monotonicTimeUs(), now_ms = now_us / 1000;
        const auto dt_us = previous_cycle_us && now_us > previous_cycle_us ? now_us - previous_cycle_us : 0;
        previous_cycle_us = now_us;
        endpoint.poll(now_ms);
        (void)remote.snapshot(remote_frame);
        auto operator_state = controls.update(remote_frame.remote, now_ms);
        const auto peer_state = endpoint.snapshot();
        const bool peer_revoked = (was_peer_online && !peer_state.online) ||
                                  (last_peer_boot && peer_state.peer.sender_boot_id &&
                                   last_peer_boot != peer_state.peer.sender_boot_id);
        if (peer_state.peer.sender_boot_id)
            last_peer_boot = peer_state.peer.sender_boot_id;
        was_peer_online = peer_state.online;
        if (peer_revoked) {
            controls.withdraw();
            operator_state.run_allowed = false;
            operator_state.navigation_active = false;
        }
        const bool navigation_allowed = operator_state.navigation_active && operator_state.fresh && setup_ready;
        const bool gimbal_allowed = operator_state.run_allowed || navigation_allowed;
        commands.setOperatorGate(gimbal_allowed, static_cast<std::uint32_t>(operator_state.start_event_id));
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
        const auto spin_out = spin.update(operator_state.spin_requested || keyboard_spin_latched,
                                          operator_state.run_allowed,
                                          keyboard_dial != 0.0f ? keyboard_dial : wheel_dial, now_us);
        (void)commands.snapshot(frame);
        // 手动档已始终合并键鼠，命令来源固定为 KeyboardMouse。来源必须稳定：
        // 忽 Remote 忽 KeyboardMouse 会让回中控制器和惯性外环每帧复位。
        constexpr auto expected_manual_source = ControlSource::KeyboardMouse;
        const auto heading = planarYawHeading(carrier_snapshot, now_us);
        const bool heading_fresh = heading.valid && core::fresh(heading.stamp, now_us, 20000);
        const auto heading_reason = !heading.valid   ? heading.reason
                                    : !heading_fresh ? HeadingWaitReason::ControlStale
                                                     : HeadingWaitReason::None;
        // Retain transient losses even when heading recovers before the next
        // diagnostic packet. Motor execution errors remain separate in error.
        if (heading_was_fresh && !heading_fresh) {
            have_heading_loss = true;
            last_heading_loss_reason = heading_reason;
            last_heading_loss_dt_us = dt_us;
            heading_loss_start_us = now_us;
        }
        if (heading_loss_start_us) {
            heading_loss_duration_us = now_us - heading_loss_start_us;
            if (heading_fresh)
                heading_loss_start_us = 0;
        }
        heading_was_fresh = heading_fresh;
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
        input.command = frame.decision.command.gimbal;
        input.source_stamp = sourceStamp(frame, input.command.source);
        const bool gimbal_mode_matches = navigation_allowed ? frame.decision.operator_mode == OperatorMode::Navigation
                                                            : frame.decision.operator_mode == OperatorMode::Manual;
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
        const bool joint_valid = ys.feedback_fresh && ys.position_reference_valid &&
                                 bool(ys.feedback.valid & motor::FeedbackPosition);
        const auto &feedback = peer_state.big_yaw_feedback;
        const bool peer_feedback_valid = peer_state.online && feedback.valid &&
                                         forwardedFresh(feedback.stamp, feedback.production_age_ms, now_ms, 250) &&
                                         std::isfinite(feedback.actual_rate_rad_s) &&
                                         std::fabs(feedback.actual_rate_rad_s) <=
                                             installed::big_yaw.velocity_limit_rad_s;
        if (peer_feedback_valid != peer_feedback_was_valid)
            dual_yaw.reset();
        peer_feedback_was_valid = peer_feedback_valid;
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
        alloc_input.enabled = gimbal_allowed && input.command.mode != GimbalMode::Disabled;
        alloc_input.permission_valid = gimbal_allowed && installed::connections_confirmed &&
                                       installed::imu_mounting_confirmed;
        // 大 Yaw 通道是否可用的判定与发送门控分离：真正的 mode/rate 仍在下面
        // big_channel_ready 之后决定，这里只决定本周期是否推进大 Yaw 意图。
        alloc_input.big_yaw_available = peer_state.online && peer_feedback_valid && heading_fresh && joint_valid;
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
            const float seed = vision_pitch_was_active ? static_cast<float>(gimbal.pitchTargetRad())
                                                       : ps.feedback.position_rad;
            const float dt = dt_us > 0 && dt_us <= 20000 ? float(dt_us) * 1e-6f : 0;
            // Keep Rate mode for BOTH axes: AbsoluteAngle would replace the
            // IMU yaw correction with a world angle interpreted as a joint angle.
            input.command.pitch_rate_rad_s = dt > 0 ? std::clamp((target - seed) / dt,
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
            input.command.pitch_rate_rad_s = raw_pitch_rate_rad_s * installed::pitch.direction;
        }
        vision_pitch_was_active = vision_active;
        input.pitch_output_valid = pitch_valid;
        input.transport_ready = setup_ready;
        auto status = control_error == 0 ? gimbal.update(input, now_us)
                                         : gimbal.suspend(now_us, WaitReason::Configuration, control_error, true);
        // 板间大 Yaw 的发送门控仍然在本周期执行结果之后再判断：分配器只提供
        // 世界速率意图，能不能发出去由仲裁、对端在线、对端反馈和本机执行状态决定。
        const auto center_yaw = yaw.snapshot();
        const bool center_joint_valid = center_yaw.feedback_fresh && center_yaw.position_reference_valid &&
                                        bool(center_yaw.feedback.valid & motor::FeedbackPosition);
        const bool yaw_running = gimbal.yawTelemetry().output_valid;
        const bool big_channel_ready = setup_ready && operator_state.fresh && gimbal_allowed && peer_state.online &&
                                       installed::imu_mounting_confirmed && heading_fresh && center_joint_valid &&
                                       yaw_running;
#if DT_HAS_ALIAS(vision_uart) && IS_ENABLED(CONFIG_SKYWALKER_VISION_RECEIVER)
        communication::vision::Feedback vision_feedback{};
        vision_feedback.mode = navigation_allowed ? communication::vision::Mode::AutoAim
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
        // 发射机构：命令来自仲裁器的 ShooterCommand（鼠标右键摩擦轮、左键发射）。
        // 尚未接入裁判热量模型，先关闭热量与许可门控；摩擦轮/供弹仍受执行器自身的
        // 反馈就绪、云台 Active 和拨盘参考约束，初始化失败时直接停机。
        ShooterExecutionInputs shooter_input{};
        shooter_input.command = frame.decision.command.shooter;
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
            request.permission = {operator_state.fresh, big_channel_ready, operator_state.remote.stamp};
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
            auto wheel_command = frame.decision.command.chassis;
            // Reject a cached command from the previous RC mode. In navigation
            // wz is supplied by the navigation frame, never by the spin profile.
            const bool navigation_command = navigation_allowed && wheel_command.source == ControlSource::Autonomous;
            const bool manual_command = operator_state.run_allowed && wheel_command.source == expected_manual_source;
            if (!navigation_command && !manual_command)
                wheel_command.mode = ChassisMode::Disabled;
            if (manual_command && wheel_command.mode != ChassisMode::Disabled &&
                !(frame.decision.chassis_reasons & PermissionDenied))
                wheel_command.wz_rad_s = spin_out.applied_rad_s;
            endpoint.submit(wheel_command);
        }
        // Original publication order: submit requests, then commit each local
        // bus once. Shooter shares CAN1 and remains in that same commit.
        if (yaw_started) {
            const int error = yaw_bus.commit().error;
            if (error < 0)
                status.error = error;
        }
        if (pitch_started) {
            const int error = pitch_bus.commit().error;
            if (error < 0)
                status.error = error;
        }
        endpoint.setStatus(status);
        const auto reason = !setup_ready || onboard_error || (vision_selected && head_error)
                                ? installed::DiagnosticReason::InitFailed
                            : !operator_state.fresh              ? installed::DiagnosticReason::RemoteStale
                            : !gimbal_allowed                    ? installed::DiagnosticReason::Disarmed
                            : !installed::imu_mounting_confirmed ? installed::DiagnosticReason::Configuration
                            : !heading_fresh || (vision_selected && !head_ready) ? installed::DiagnosticReason::ImuStale
                            : !peer_state.online || !peer_feedback_valid ? installed::DiagnosticReason::PeerStale
                            : !joint_valid || !yaw_running               ? installed::DiagnosticReason::FeedbackStale
                            : status.state == RunState::Active           ? installed::DiagnosticReason::Ready
                                                                         : installed::DiagnosticReason::CommandStale;
        (void)status_led.setStatus(
            installed::statusLedFor(reason, status.state == RunState::Active, false, status.error));
        const float unavailable = std::numeric_limits<float>::quiet_NaN();
        // Fixed channel map documented in docs/dev/双Yaw速度加速度分配与视觉轨迹控制方案.md
        // (schema 111, I19..I48). I38/I43/I48 now describe the dual-Yaw allocator
        // instead of the retired centering controller.
        const std::array<float, 30>
            trace{111,                             // I19 dual-Yaw allocation + CAN vision schema
                  float(unsigned(heading_reason)), // I20 live reason
                  float(unsigned(carrier_snapshot.sample.attitude_quality)), // I21 quality
                  float(heading_fresh),                                      // I22 heading usable
                  heading_fresh ? heading.yaw_rad : unavailable,             // I23 world yaw
                  carrier_snapshot.sample.gyro_rad_s.stamp.valid ? carrier_snapshot.sample.gyro_rad_s.value.z
                                                                 : unavailable, // I24 gyro z
                  carrier_snapshot.sample.accel_m_s2.stamp.valid ? core::norm(carrier_snapshot.sample.accel_m_s2.value)
                                                                 : unavailable,         // I25 accel norm
                  float(head_reference_changed ? 10u : head_reason),                    // I26 head IMU reason
                  head_usable ? head_euler.yaw : unavailable,                           // I27 measured head world yaw
                  head_usable ? head_euler.pitch : unavailable,                         // I28 measured head world pitch
                  inertial.yaw_output_valid ? inertial.yaw_error_rad : unavailable,     // I29 head yaw error
                  inertial.pitch_output_valid ? inertial.pitch_error_rad : unavailable, // I30 head pitch error
                  float((head_orientation_fresh ? 1u : 0u) | (head_gyro_fresh ? 2u : 0u) | (head_quality_ok ? 4u : 0u) |
                        (head_reference_valid ? 8u : 0u) | (head_ready ? 16u : 0u) |
                        (inertial.yaw_output_valid ? 32u : 0u) |
                        (inertial.pitch_output_valid ? 64u : 0u)),     // I31 head/control validity bits
                  float(head_snapshot.diagnostics.io_errors & 0xffff), // I32 head I/O errors
                  ps.feedback.position_rad,                            // I33 pitch joint position rad
                  float(age(head_sample.orientation.stamp, now_us)),   // I34 head orientation age ms
                  float(age(head_sample.gyro_rad_s.stamp, now_us)),    // I35 head gyro age ms
                  ps.feedback.velocity_rad_s,                          // I36 pitch joint velocity rad/s
                  float((!setup_ready ? 1u : 0u) | (!operator_state.fresh ? 2u : 0u) | (!gimbal_allowed ? 4u : 0u) |
                        (!peer_state.online ? 8u : 0u) | (!installed::imu_mounting_confirmed ? 16u : 0u) |
                        (!heading_fresh ? 32u : 0u) | (!joint_valid ? 64u : 0u) | (!yaw_running ? 128u : 0u) |
                        (!head_ready ? 256u : 0u)),                          // I37 disabled prerequisites; zero=ready
                  allocation.big.rate_rad_s,                                 // I38 big-Yaw world intent
                  peer_feedback_valid ? float(feedback.armed) : unavailable, // I39 peer armed
                  float(age(carrier_snapshot.sample.orientation.stamp, now_us)), // I40 orientation age
                  float(age(carrier_snapshot.sample.gyro_rad_s.stamp, now_us)),  // I41 gyro age
                  spin_out.applied_rad_s,                                        // I42 applied variable spin rate
                  std::min(allocation.brake_margin_pos_rad,
                           allocation.brake_margin_neg_rad), // I43 tightest outward travel
                  float(input.command.pitch_rate_rad_s),     // I44 pitch joint rate command
                  float(dt_us),                              // I45 current cycle
                  raw_head_rate_rad_s,                       // I46 raw head world-rate target
                  allocation.joint_rate_rad_s,               // I47 allocated small joint rate
                  float((allocation.enabled ? 1u : 0u) | (allocation.travel_limited ? 2u : 0u) |
                        (allocation.big_saturated ? 4u : 0u) | (allocation.tracking_limited ? 8u : 0u) |
                        (big_channel_ready ? 16u : 0u) | (allocation.joint_reseed_requested ? 32u : 0u) |
                        (allocation.head_reference_valid ? 64u : 0u) | (vision_active ? 512u : 0u) |
                        (static_cast<unsigned>(allocation.reason) << 10))}; // I48 allocator flags/reason

        diagnostics.status({.reason = reason,
                            .valid = joint_valid && (!vision_selected || head_ready),
                            .enabled = status.state == RunState::Active,
                            .target = static_cast<float>(gimbal.yawTargetRad()),
                            .feedback = ys.feedback.position_rad,
                            .age_ms = age(head_sample.orientation.stamp, now_us),
                            .dt_us = float(have_heading_loss ? last_heading_loss_dt_us : dt_us),
                            .effort = gimbal.yawTelemetry().effort_command,
                            .detail = float(unsigned(have_heading_loss ? last_heading_loss_reason : heading_reason)),
                            .error = status.error ? status.error
                                     : head_error ? head_error
                                     : !head_usable && head_snapshot.diagnostics.last_error
                                         ? head_snapshot.diagnostics.last_error
                                         : onboard_error,
                            .extra0 = installed::small_yaw.direction * allocation.center_error_rad,
                            .extra1 = heading_fresh ? heading.rate_rad_s : std::numeric_limits<float>::quiet_NaN(),
                            .extended = true,
                            .extension = trace});
        deadline += period;
        const auto ticks = k_uptime_ticks();
        if (deadline <= ticks)
            deadline += ((ticks - deadline) / period + 1) * period;
        k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
    }
}
