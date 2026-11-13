#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <communication/remote/remote_receiver.hpp>
#include <core/clock.hpp>
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>
#include <robotics/command/receiver_sources.hpp>
#include <robotics/vehicle/diagnostics.hpp>
#include <robotics/vehicle/manual_profile.hpp>
#include <robotics/vehicle/bringup_policy.hpp>
#include <robotics/vehicle/calibration.hpp>
#include <robotics/chassis/imu_heading.hpp>
#include <control/attitude_ekf.hpp>
#include <drivers/imu/bmi088_imu.hpp>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include "../../../robotics/common/rc_controls.hpp"
#include <drivers/imu/imu_receiver.hpp>
#include <communication/interboard/uart_interboard_transport.hpp>
#include "../../../robotics/common/operator_controls.hpp"
using namespace skywalker;
using namespace skywalker::robotics;
namespace {
// Fixed peer: unmodified sentry_onlychassis v4, mode=1 means world yaw rate.
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                     samples::control::receiverConfig());
RemoteSource source(remote);
CommandManager commands(vehicle::manualCommandConfig());
vehicle::Diagnostics diagnostics;
// 内置 IMU：板载 BMI088 + 四元数 EKF，向参考底盘发布航向与去偏角速度；
// 本地关节使用编码器闭环，不启动 CAN3 外置 IMU，也不等待它的数据。
// Startup stationary calibration only; slow real turns must not become gyro bias.
control::QuaternionEkf onboard_estimator({.initialization_samples = 800, .adapt_gyro_bias = false});
imu::Bmi088Imu onboard_source(DEVICE_DT_GET(DT_ALIAS(accel0)), DEVICE_DT_GET(DT_ALIAS(gyro0)),
                              {.reference = vehicle::carrier_reference,
                               .sensor_to_body = vehicle::carrier_sensor_to_body,
                               .correct_gyro_bias = true},
                              &onboard_estimator);
imu::ImuReceiver onboard(onboard_source, {.poll_interval_us = 500, .priority = 5});
ImuHeadingReference onboard_heading;
motor::Motor yaw(vehicle::yawHardware()), pitch(vehicle::pitchHardware());
motor::Group motors(yaw, pitch);
motor::CanBus yaw_bus(DEVICE_DT_GET(DT_NODELABEL(can1)));
motor::CanBus pitch_bus(DEVICE_DT_GET(DT_NODELABEL(can2)));
GimbalExecutor gimbal(yaw, pitch, motors, vehicle::yawMotorConfig(), vehicle::yawAxisConfig(),
                      vehicle::pitchMotorConfig(), vehicle::pitchAxisConfig(), vehicle::gimbalExecutionConfig());
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::UartInterBoardTransport transport(DEVICE_DT_GET(DT_ALIAS(interboard_uart)), link_dma);
communication::InterBoardEndpoint endpoint(transport, {.role = BoardRole::GimbalController});
// 双 Yaw 分配器：与正式云台共用同一份配置工厂，只把头部/大 Yaw 世界速率压到
// bringup 允许的上限。旧的 YawCenteringController 已作废（见 yaw_centering.hpp）。
DualYawAllocatorConfig dualYawConfig() {
    auto c = vehicle::dualYawAllocatorConfig();
    // The unmodified peer still clamps the wire world rate at 1 rad/s.
    const float limit = std::min(vehicle::bringup::big_yaw_rate_rad_s, 1.0f);
    c.head_rate_max_rad_s = std::min(c.head_rate_max_rad_s, limit);
    c.head_steady_rate_max_rad_s = std::min(c.head_steady_rate_max_rad_s, c.head_rate_max_rad_s);
    c.big_intent_max_rad_s = limit;
    return c;
}
DualYawMotionAllocator dual_yaw(dualYawConfig());
void communicationTask(void *, void *, void *) {
    for (;;) {
        endpoint.poll(k_uptime_get());
        k_sleep(K_MSEC(1));
    }
}
} // namespace
K_THREAD_DEFINE(dual_yaw_link_worker, 4096, communicationTask, nullptr, nullptr, nullptr, 6, K_FP_REGS, 0);

int main() {
    diagnostics.start("gimbal_dual_yaw", "gimbal",
                      "yaw<=1rad/s,pitch<=0.8rad/s,big_yaw_world<=1rad/s,dual_yaw_allocator");
    const int onboard_error = onboard.start();
    diagnostics.event("onboard.start", onboard_error);
    const int power_error = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
    diagnostics.event("motor_power", power_error);
    // 非零表示分配器参数非法，此时它按 Configuration 拒绝运行并回退编码器公式。
    diagnostics.event("dual_yaw", dual_yaw.validate());
    int source_error = commands.registerSource(source);
    if (!source_error)
        source_error = commands.start();
    diagnostics.event("remote_commands", source_error);
    int setup_error = !vehicle::connections_confirmed ? -EACCES : power_error;
    if (!setup_error)
        setup_error = yaw_bus.attach(yaw);
    if (!setup_error)
        setup_error = pitch_bus.attach(pitch);
    bool yaw_started = false, pitch_started = false;
    if (!setup_error) {
        setup_error = yaw_bus.start();
        yaw_started = setup_error == 0;
    }
    if (!setup_error) {
        setup_error = pitch_bus.start();
        pitch_started = setup_error == 0;
    }
    if (!setup_error)
        setup_error = gimbal.begin();
    diagnostics.event("gimbal.begin", setup_error);
    const bool configured = setup_error == 0;
    samples::control::RcControlAdapter controls;
    communication::RemoteReceiver::Snapshot rc{};
    CommandSnapshot frame{};
    core::TimeUs previous_us = 0;
    samples::control::OperatorControlPublisher operator_publisher;
    bool peer_feedback_was_valid = false;
    std::uint32_t chassis_sequence = 0, big_sequence = 0;
    for (;;) {
        const auto now = core::monotonicTimeUs(), ms = now / 1000;
        const auto dt_us = previous_us && now > previous_us ? now - previous_us : 0;
        previous_us = now;
        (void)remote.snapshot(rc);
        const auto &op = controls.update(rc.remote, ms);
        if (operator_publisher.publish(endpoint, op.run_allowed, false, op.remote.stamp, op.clear_event_id,
                                       op.clear_stamp, ms))
            controls.withdraw();
        commands.setOperatorGate(op.run_allowed, op.clear_event_id);
        (void)commands.snapshot(frame);
        // 零位在解除授权时采集；发到线上的航向只相对于这个软件零位。
        const auto heading = onboard_heading.update(onboard.snapshot(), !op.run_allowed, now);
        const bool heading_fresh = heading.valid && core::fresh(heading.stamp, now, 20000);
        const auto ys = yaw.snapshot();
        const bool joint_valid = ys.feedback_fresh && ys.position_reference_valid &&
                                 bool(ys.feedback.valid & motor::FeedbackPosition);
        const auto peer = endpoint.snapshot();
        const auto &feedback = peer.big_yaw_feedback;
        // The reference peer publishes BigYawFeedback at 100 ms intervals.
        // A 50 ms timeout would repeatedly invalidate feedback and drop the
        // big-Yaw intent before the next remote feedback arrives.
        constexpr auto peer_feedback_timeout_ms = 250U;
        const bool peer_feedback_valid = peer.online && feedback.valid &&
                                         forwardedFresh(feedback.stamp, feedback.production_age_ms, ms,
                                                        peer_feedback_timeout_ms) &&
                                         std::isfinite(feedback.actual_rate_rad_s) &&
                                         std::fabs(feedback.actual_rate_rad_s) <= vehicle::big_yaw.velocity_limit_rad_s;
        if (peer_feedback_valid != peer_feedback_was_valid)
            dual_yaw.reset();
        peer_feedback_was_valid = peer_feedback_valid;
        // 双 Yaw 分配：手动头部世界速率 → 大 Yaw 世界意图 + 小 Yaw 关节速率。
        DualYawAllocatorInputs alloc_input{};
        alloc_input.head_rate_target_rad_s = frame.decision.command.gimbal.yaw_rate_rad_s;
        alloc_input.joint_rad = ys.feedback.position_rad;
        alloc_input.joint_rate_rad_s =
            bool(ys.feedback.valid & motor::FeedbackVelocity) ? ys.feedback.velocity_rad_s : 0.0f;
        alloc_input.joint_stamp = {ys.feedback.timestamp_ms * 1000ULL, ys.feedback.timestamp_ms, joint_valid};
        alloc_input.carrier_rate_rad_s = heading_fresh ? heading.rate_rad_s : 0.0f;
        alloc_input.carrier_stamp = heading.stamp;
        alloc_input.carrier_valid = heading_fresh;
        alloc_input.source_stamp = sourceStamp(frame, frame.decision.command.gimbal.source);
        alloc_input.enabled = op.run_allowed && configured &&
                              frame.decision.command.gimbal.mode != GimbalMode::Disabled;
        alloc_input.permission_valid = vehicle::connections_confirmed && vehicle::imu_mounting_confirmed &&
                                       op.run_allowed && !op.clear_estop;
        alloc_input.big_yaw_available = peer.online && peer_feedback_valid && heading_fresh && joint_valid;
        const auto allocation = dual_yaw.update(alloc_input, now);
        if (allocation.joint_reseed_requested)
            (void)gimbal.reseedYawTarget();
        GimbalExecutionInputs request{};
        request.command = frame.decision.command.gimbal;
        request.source_stamp = sourceStamp(frame, request.command.source);
        // 只有编码器闭环关节。小 Yaw 速率由分配器给出：它已按实测载体速度补偿，
        // 并带头部加速度窗与行程度保护，因此大 Yaw 回中时不需要再手算一次。
        if (request.command.mode == GimbalMode::Rate) {
            request.command.yaw_rate_rad_s =
                allocation.joint_rate_valid
                    ? allocation.joint_rate_rad_s
                    : vehicle::small_yaw.direction * (request.command.yaw_rate_rad_s -
                                                      (heading_fresh ? heading.rate_rad_s : 0.0f));
            request.command.pitch_rate_rad_s *= vehicle::pitch.direction;
        }
        if (!op.run_allowed)
            request.command.mode = GimbalMode::Disabled;
        request.transport_ready = configured && !source_error;
        request.clear_estop = op.clear_estop;
        const auto run = configured ? gimbal.update(request, now)
                                    : gimbal.suspend(now, WaitReason::Configuration, setup_error, true);
        const int yaw_error = yaw_started ? yaw_bus.commit().error : 0;
        const int pitch_error = pitch_started ? pitch_bus.commit().error : 0;
        const bool yaw_running = gimbal.yawTelemetry().output_valid;
        const bool big_channel_ready =
            peer.online && alloc_input.permission_valid && heading_fresh && yaw_running && joint_valid;
        BigYawRequest big_request{};
        // Encoder offset requests WORLD rate. The peer applies gyro compensation
        // and motor direction; do not negate it here. Zero enables initial feedback.
        big_request.mode = big_channel_ready ? BigYawMode::FollowCenter : BigYawMode::Disabled;
        big_request.target_rate_rad_s = big_channel_ready && allocation.enabled ? allocation.big.rate_rad_s : 0;
        big_request.stamp = {ms, ++big_sequence, true};
        big_request.source_sequence = static_cast<std::uint32_t>(request.source_stamp.sequence);
        big_request.source_age_ms = request.source_stamp.valid && now >= request.source_stamp.time_us
                                        ? static_cast<std::uint32_t>(
                                              std::min<std::uint64_t>((now - request.source_stamp.time_us) / 1000,
                                                                      UINT32_MAX))
                                        : UINT32_MAX;
        big_request.command_age_ms = request.command.stamp.valid && ms >= request.command.stamp.timestamp_ms
                                         ? static_cast<std::uint32_t>(
                                               std::min<std::uint64_t>(ms - request.command.stamp.timestamp_ms,
                                                                       UINT32_MAX))
                                         : UINT32_MAX;
        big_request.permission = {op.fresh, big_channel_ready, op.remote.stamp};
        endpoint.submitBigYaw(big_request);
        // 参考底盘的 gyro 大 Yaw 后端必须有云台航向样本才允许使能。
        if (peer.online && peer.peer.sender_boot_id) {
            GimbalImuYaw sample{};
            sample.valid = heading_fresh;
            sample.yaw_rad = heading.yaw_rad;
            sample.yaw_rate_rad_s = heading.rate_rad_s;
            sample.zero_generation = heading.generation;
            sample.sample_sequence = static_cast<std::uint32_t>(heading.stamp.sequence);
            sample.source_age_ms = heading.stamp.valid && now >= heading.stamp.time_us
                                       ? static_cast<std::uint32_t>(
                                             std::min<std::uint64_t>((now - heading.stamp.time_us + 999) / 1000,
                                                                     UINT32_MAX))
                                       : UINT32_MAX;
            sample.receiver_boot_id = peer.peer.sender_boot_id;
            sample.stamp = {ms, big_sequence + 1, true};
            endpoint.submitGimbalImuYaw(sample);
        }
        // 参考底盘的 RobotBigYawControl() 只在底盘控制源被授权时执行；零速 BodyVelocity 只放行通道。
        ChassisCommand chassis{};
        chassis.mode = big_channel_ready ? ChassisMode::BodyVelocity : ChassisMode::Disabled;
        chassis.source = ControlSource::Remote;
        chassis.stamp = {ms, ++chassis_sequence, true};
        endpoint.submit(chassis);
        endpoint.setStatus(run);
        const auto reason = setup_error || source_error || onboard_error ? vehicle::DiagnosticReason::InitFailed
                            : !op.fresh                                  ? vehicle::DiagnosticReason::RemoteStale
                            : !op.run_allowed                            ? vehicle::DiagnosticReason::Disarmed
                            : !vehicle::imu_mounting_confirmed           ? vehicle::DiagnosticReason::Configuration
                            : !heading_fresh                             ? vehicle::DiagnosticReason::ImuStale
                            : !peer.online || !peer_feedback_valid       ? vehicle::DiagnosticReason::PeerStale
                            : !joint_valid || !yaw_running               ? vehicle::DiagnosticReason::FeedbackStale
                                                                         : vehicle::DiagnosticReason::Ready;
        diagnostics.status({.reason = reason,
                            .valid = heading_fresh && joint_valid && peer_feedback_valid,
                            .enabled = big_channel_ready && allocation.enabled && peer_feedback_valid,
                            .target = big_request.target_rate_rad_s,
                            .feedback = peer_feedback_valid
                                            ? vehicle::reference_big_yaw.direction * feedback.actual_rate_rad_s /
                                                  vehicle::reference_big_yaw.gear_ratio
                                            : std::numeric_limits<float>::quiet_NaN(),
                            .age_ms = feedback.valid ? feedback.production_age_ms : UINT32_MAX,
                            .dt_us = float(dt_us),
                            .effort = gimbal.yawTelemetry().effort_command,
                            .detail = float(unsigned(allocation.reason)),
                            .error = setup_error     ? setup_error
                                     : source_error  ? source_error
                                     : onboard_error ? onboard_error
                                     : yaw_error     ? yaw_error
                                     : pitch_error   ? pitch_error
                                                     : run.error,
                            .extra0 = vehicle::small_yaw.direction * allocation.center_error_rad,
                            .extra1 = heading_fresh ? heading.rate_rad_s : std::numeric_limits<float>::quiet_NaN()});
        k_sleep(K_USEC(vehicle::gimbal_period_us));
    }
}
