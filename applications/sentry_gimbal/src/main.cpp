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
                      "head CAN IMU inertial yaw/pitch; carrier BMI088 for big yaw; limits=installed profiles");
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
    static communication::vision::AbProtocol vision_protocol({.command_reference = installed::head_reference});
    static communication::vision::VisionReceiver vision(DEVICE_DT_GET(DT_ALIAS(vision_uart)), vision_dma,
                                                        vision_protocol, {});
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
    // Near-horizontal carrier: yaw is integrated in the acquisition thread.
    // Its EKF stays diagnostic-only; head attitude comes from the CAN IMU.
    // The allocator splits one head world-rate target between the big Yaw
    // carrier and the small Yaw joint; it owns no motor and no CAN frame.
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
    diagnostics.event("shooter", shooter_error);
    diagnostics.event("motor_rail", power_error);
    // 载体 IMU 单独门控大 Yaw；头部 IMU 无效时撤销本地惯性输出和大 Yaw 请求。
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
    CarrierAccelerationEstimator carrier_acceleration;
    std::uint32_t last_heading_generation = 0;
    std::uint64_t last_yaw_sequence = 0;
    std::uint32_t vision_trajectory_sequence = 0;
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
        // 忽 Remote 忽 KeyboardMouse 会让分配器和惯性外环每帧复位。
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
        // Accelerations come only from two genuinely new gyro samples; repeated
        // reads keep the filtered value and gaps disable the feedforward.
        const auto &carrier_gyro = carrier_snapshot.sample.gyro_rad_s;
        if (heading.generation != last_heading_generation) {
            // A new software zero invalidates every derivative taken across it.
            last_heading_generation = heading.generation;
            carrier_acceleration.reset();
        }
        carrier_acceleration.update(carrier_gyro.stamp.valid ? carrier_gyro.value.z
                                                             : std::numeric_limits<float>::quiet_NaN(),
                                    carrier_gyro.stamp, 0.02f, 50.0f);
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
        // Allocation runs on the pre-update snapshot: one measured joint state
        // and one carrier gyro sample produce the plan that this cycle's single
        // gimbal.update() consumes.
        const auto ys = yaw.snapshot();
        const auto ps = pitch.snapshot();
        const bool pitch_valid = ps.feedback_fresh && ps.position_reference_valid &&
                                 bool(ps.feedback.valid & motor::FeedbackPosition) &&
                                 std::isfinite(ps.feedback.position_rad);
        auto carrier_orientation = carrier_snapshot.sample.orientation.value;
        const bool carrier_pose_valid = core::fresh(carrier_snapshot.sample.orientation.stamp, now_us, 20000) &&
                                        core::normalize(carrier_orientation);
        const bool joint_valid = ys.feedback_fresh && ys.position_reference_valid &&
                                 bool(ys.feedback.valid & motor::FeedbackPosition);
        // Last completed Yaw telemetry: the request below is gated on it and the
        // freshly computed plan is applied by the update that follows.
        const bool yaw_running = gimbal.yawTelemetry().output_valid;
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
        std::uint32_t vision_epoch = 0;
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
            vision_rate_rad_s = v.rate_rad_s;
            vision_acceleration_rad_s2 = v.acceleration_rad_s2;
            const auto &p = selected_vision.value.pitch;
            vision_pitch_rad = p.angle_rad + p.rate_rad_s * age_s + 0.5f * p.acceleration_rad_s2 * age_s * age_s;
            vision_epoch = (static_cast<std::uint32_t>(selected_vision.value.reference.frame_id) << 16) ^
                           static_cast<std::uint32_t>(selected_vision.value.reference.epoch);
        }
        const bool big_channel_ready = setup_ready && operator_state.fresh && gimbal_allowed && gimbal_mode_matches &&
                                       input.command.mode != GimbalMode::Disabled && peer_state.online &&
                                       installed::imu_mounting_confirmed && heading_fresh && joint_valid &&
                                       yaw_running && head_ready;
        DualYawAllocatorInputs allocation{};
        allocation.head_rate_target_rad_s = raw_head_rate_rad_s;
        allocation.joint_rad = ys.feedback.position_rad;
        const bool joint_rate_valid = (ys.feedback.valid & motor::FeedbackVelocity) != 0 &&
                                      std::isfinite(ys.feedback.velocity_rad_s);
        allocation.joint_rate_rad_s = joint_rate_valid ? ys.feedback.velocity_rad_s : 0.0f;
        // 载体 BMI088 暂不可用时，本地可用头部角速度减关节角速度估计载体运动；
        // 此估计只供本地行程保护，绝不作为新的板间载体航向发布给底盘。
        const auto head_gyro = head_sample.gyro_rad_s.value;
        const float head_yaw_rate = head_ready ? (std::sin(head_euler.roll) * head_gyro.y +
                                                  std::cos(head_euler.roll) * head_gyro.z) /
                                                     std::cos(head_euler.pitch)
                                               : 0;
        allocation.carrier_rate_rad_s = heading_fresh ? heading.rate_rad_s
                                                      : head_yaw_rate - installed::small_yaw.direction *
                                                                            allocation.joint_rate_rad_s;
        allocation.carrier_stamp = heading_fresh ? heading.stamp : head_sample.gyro_rad_s.stamp;
        allocation.carrier_valid = heading_fresh || (head_ready && joint_rate_valid);
        allocation.joint_stamp = {ys.feedback.timestamp_ms * 1000, ys.feedback.timestamp_ms, joint_valid};
        allocation.source_stamp = input.source_stamp;
        allocation.big_yaw_available = big_channel_ready && peer_feedback_valid;
        allocation.enabled = gimbal_allowed && gimbal_mode_matches && head_ready &&
                             (input.command.mode == GimbalMode::Rate || input.command.mode == GimbalMode::Hold ||
                              (vision_active && input.command.mode != GimbalMode::Disabled));
        allocation.permission_valid = gimbal_allowed && gimbal_mode_matches && installed::connections_confirmed &&
                                      installed::imu_mounting_confirmed;
        allocation.vision_active = vision_active;
        allocation.vision_angle_rad = vision_angle_rad;
        allocation.vision_rate_rad_s = vision_rate_rad_s;
        allocation.vision_acceleration_rad_s2 = vision_acceleration_rad_s2;
        allocation.vision_epoch = vision_epoch;
        // 同一外置头部参考同时用于手动、视觉和惯性外环，参考变化时前面已清除旧计划。
        allocation.head_angle_measured_rad = head_euler.yaw;
        allocation.head_angle_valid = head_ready;
        allocation.carrier_acceleration_valid = heading_fresh && carrier_acceleration.valid();
        allocation.carrier_acceleration_rad_s2 = carrier_acceleration.value_rad_s2();
        const auto alloc = dual_yaw.update(allocation, now_us);
        InertialGimbalInputs inertial_input{};
        inertial_input.head = head_snapshot;
        inertial_input.yaw = ys;
        inertial_input.pitch = ps;
        inertial_input.command = input.command;
        inertial_input.source_stamp = input.source_stamp;
        inertial_input.prerequisites_ready = setup_ready && installed::imu_mounting_confirmed;
        inertial_input.yaw_reference_valid = alloc.enabled && alloc.head_reference_valid;
        inertial_input.yaw_reference_rad = alloc.head_angle_rad;
        inertial_input.yaw_reference_rate_rad_s = alloc.head_reference_rate_rad_s;
        // 头部 IMU 只提供 Yaw 世界参考；Pitch 回到纯电机编码器闭环，IMU 仅作观测。
        inertial_input.command.mode = GimbalMode::Rate;
        if (!head_ready || !alloc.enabled || !alloc.head_reference_valid)
            inertial_input.command.mode = GimbalMode::Disabled;
        const auto inertial = head_control.update(inertial_input, now_us);
        input.command = inertial.command;
        input.source_stamp = inertial.source_stamp;
        input.yaw_output_valid = inertial.yaw_output_valid;
        if (alloc.enabled && alloc.vision_mode && carrier_pose_valid) {
            // 视觉：世界瞄准射线经载体姿态与实测 Yaw 关节投影成 Pitch 关节目标。
            input.yaw_trajectory.valid = true;
            input.yaw_trajectory.position_rad = alloc.joint_position_rad;
            input.yaw_trajectory.rate_rad_s = alloc.joint_rate_rad_s;
            input.yaw_trajectory.acceleration_rad_s2 = alloc.joint_acceleration_rad_s2;
            input.yaw_trajectory.stamp = {now_us, ++vision_trajectory_sequence, true};
            auto carrier_euler = core::euler(carrier_orientation);
            carrier_euler.yaw = heading.yaw_rad; // Same startup yaw reference as the allocator.
            const auto yaw_pose = core::multiply(core::fromEuler(carrier_euler),
                                                 core::fromEuler(
                                                     {0, 0,
                                                      installed::small_yaw.direction *
                                                          (ys.feedback.position_rad - installed::yaw_center_rad)}));
            const float cp = std::cos(vision_pitch_rad);
            const core::Vec3 world_ray{cp * std::cos(vision_angle_rad), cp * std::sin(vision_angle_rad),
                                       -std::sin(vision_pitch_rad)};
            const auto joint_ray = core::rotate(core::conjugate(yaw_pose), world_ray);
            const float target = std::clamp(installed::vision_pitch_horizontal_rad +
                                                installed::pitch.direction * std::atan2(-joint_ray.z, joint_ray.x),
                                            installed::pitch_min_rad, installed::pitch_max_rad);
            const float seed = vision_pitch_was_active ? static_cast<float>(gimbal.pitchTargetRad())
                                                       : ps.feedback.position_rad;
            const float step = installed::manual_max_gimbal_pitch_rate_rad_s *
                               static_cast<float>(std::min<core::TimeUs>(dt_us, 20000)) * 1e-6f;
            input.command.mode = GimbalMode::AbsoluteAngle;
            input.command.pitch_target_rad = std::clamp(target, seed - step, seed + step);
        }
        else if (alloc.enabled && alloc.vision_mode) {
            // 载体姿态不可用：保持实测关节，不把世界角直接写进关节目标。
            input.command.mode = GimbalMode::Hold;
        }
        else if (input.command.mode == GimbalMode::Rate) {
            // 编码器闭环拥有 Pitch：直接消费仲裁后的手动关节速度。
            input.command.pitch_rate_rad_s = raw_pitch_rate_rad_s * installed::pitch.direction;
        }
        vision_pitch_was_active = alloc.enabled && alloc.vision_mode;
        input.pitch_output_valid = pitch_valid;
        input.transport_ready = setup_ready && head_error == 0;
        // 外置姿态修正也必须通过同一制动包络；不能在分配器保护之后重新加上向外速度。
        const float inertial_yaw_rate = input.command.yaw_rate_rad_s;
        input.command.yaw_rate_rad_s = std::clamp(inertial_yaw_rate, -alloc.outward_budget_neg_rad_s,
                                                  alloc.outward_budget_pos_rad_s);
        if (allocation.joint_rate_rad_s > alloc.outward_budget_pos_rad_s)
            input.command.yaw_rate_rad_s = std::min(input.command.yaw_rate_rad_s, 0.0f);
        if (allocation.joint_rate_rad_s < -alloc.outward_budget_neg_rad_s)
            input.command.yaw_rate_rad_s = std::max(input.command.yaw_rate_rad_s, 0.0f);
        const bool inertial_travel_limited = input.command.yaw_rate_rad_s != inertial_yaw_rate;
        if ((alloc.joint_reseed_requested || inertial_travel_limited) && control_error == 0)
            (void)gimbal.reseedYawTarget();
        auto status = control_error == 0 ? gimbal.update(input, now_us)
                                         : gimbal.suspend(now_us, WaitReason::Configuration, control_error, true);
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
        // One commit per physical bus, before the inter-board request is built:
        // this cycle's Yaw execution and its CAN result decide whether a nonzero
        // carrier intent may be published at all.
        int yaw_commit_error = 0;
        if (yaw_started) {
            yaw_commit_error = yaw_bus.commit().error;
            if (yaw_commit_error < 0)
                status.error = yaw_commit_error;
        }
        if (pitch_started) {
            const int error = pitch_bus.commit().error;
            if (error < 0)
                status.error = error;
        }
        const auto yaw_telemetry = gimbal.yawTelemetry();
        const bool yaw_updated = yaw_telemetry.target_sequence != last_yaw_sequence;
        last_yaw_sequence = yaw_telemetry.target_sequence;
        // `RunStatus::Active` may come from Pitch alone, so the carrier request
        // needs its own Yaw evidence: executed this cycle, output valid, CAN
        // committed and the joint measurement still usable.
        const bool yaw_cycle_ok = control_error == 0 && head_ready && alloc.enabled && inertial.yaw_output_valid &&
                                  yaw_updated && yaw_telemetry.output_valid && yaw_commit_error == 0 && joint_valid;
        // 授权不包含本周期 Yaw 执行证据：yaw_running/joint_valid 会单周期抶动，
        // 而底盘每收到一帧 Disabled 就会丢掉已保持的世界航向并重新捕获。
        const bool big_follow_authorized = setup_ready && operator_state.fresh && gimbal_allowed &&
                                           gimbal_mode_matches && input.command.mode != GimbalMode::Disabled &&
                                           peer_state.online && installed::imu_mounting_confirmed && heading_fresh &&
                                           head_ready;
        const bool big_follow_executed = yaw_cycle_ok && peer_feedback_valid && alloc.enabled;
        if (now_ms >= next_send_ms) {
            next_send_ms = now_ms + 10;
            BigYawRequest request{};
            request.source_sequence = static_cast<std::uint32_t>(input.source_stamp.sequence);
            request.source_age_ms = age(input.source_stamp, now_us);
            request.command_age_ms = age(input.command.stamp, now_ms);
            request.permission = {operator_state.fresh, big_channel_ready, operator_state.remote.stamp};
            // Reference v4 world intent: the peer applies motor sign/gyro control.
            // Zero enable obtains initial real feedback before any nonzero follow.
            // 模式只反映授权；执行证据短暂缺失时保持 FollowCenter 但发零意图，
            // 否则底盘会丢弃已保持的世界航向。
            request.mode = big_follow_authorized ? BigYawMode::FollowCenter : BigYawMode::Disabled;
            // Allocated world intent. Zero whenever the carrier channel is not
            // authorized, the allocation was rejected, or this cycle's Yaw did
            // not execute and commit.
            request.target_rate_rad_s = big_follow_authorized && big_follow_executed ? alloc.big.rate_rad_s : 0;
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
        endpoint.setStatus(status);
        const auto reason = !setup_ready || onboard_error || head_error  ? installed::DiagnosticReason::InitFailed
                            : !operator_state.fresh                      ? installed::DiagnosticReason::RemoteStale
                            : !gimbal_allowed                            ? installed::DiagnosticReason::Disarmed
                            : !installed::imu_mounting_confirmed         ? installed::DiagnosticReason::Configuration
                            : !heading_fresh || !head_ready              ? installed::DiagnosticReason::ImuStale
                            : !peer_state.online || !peer_feedback_valid ? installed::DiagnosticReason::PeerStale
                            : !joint_valid || !yaw_running               ? installed::DiagnosticReason::FeedbackStale
                            : status.state == RunState::Active           ? installed::DiagnosticReason::Ready
                                                                         : installed::DiagnosticReason::CommandStale;
        (void)status_led.setStatus(
            installed::statusLedFor(reason, status.state == RunState::Active, false, status.error));
        const float unavailable = std::numeric_limits<float>::quiet_NaN();
        // Fixed channel map documented in docs/dev/大Yaw突发跟转诊断操作指南.md
        // (schema 106, I19..I48).
        const std::array<float, 30>
            trace{106,                                                       // I19 head-IMU inertial dual-Yaw schema
                  float(unsigned(heading_reason)),                           // I20 live reason
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
                  alloc.big.rate_rad_s,                                      // I38 big-Yaw world intent
                  peer_feedback_valid ? float(feedback.armed) : unavailable, // I39 peer armed
                  float(age(carrier_snapshot.sample.orientation.stamp, now_us)),    // I40 orientation age
                  float(age(carrier_snapshot.sample.gyro_rad_s.stamp, now_us)),     // I41 gyro age
                  spin_out.applied_rad_s,                                           // I42 applied variable spin rate
                  std::min(alloc.brake_margin_pos_rad, alloc.brake_margin_neg_rad), // I43 tightest outward travel
                  float(input.command.pitch_rate_rad_s),                            // I44 pitch joint rate command
                  float(dt_us),                                                     // I45 current cycle
                  raw_head_rate_rad_s,                                              // I46 raw head world-rate target
                  alloc.head.rate_rad_s,                                            // I47 feasible head world rate
                  float((alloc.enabled ? 1u : 0u) | ((alloc.travel_limited || inertial_travel_limited) ? 2u : 0u) |
                        (alloc.big_saturated ? 4u : 0u) | (alloc.tracking_limited ? 8u : 0u) |
                        (allocation.big_yaw_available ? 16u : 0u) | (alloc.vision_mode ? 512u : 0u) |
                        (alloc.joint_reseed_requested ? 1024u : 0u) |
                        (static_cast<unsigned>(alloc.reason) << 5))}; // I48 allocation flags/reason
        diagnostics.status({.reason = reason,
                            .valid = head_ready && joint_valid,
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
                            .extra0 = alloc.center_error_rad,
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
