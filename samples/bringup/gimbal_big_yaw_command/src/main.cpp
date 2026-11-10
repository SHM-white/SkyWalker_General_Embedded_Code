#include <communication/remote/remote_receiver.hpp>
#include "../../../robotics/common/rc_controls.hpp"
#include <algorithm>
#include <cerrno>
#include <limits>
#include <robotics/vehicle/diagnostics.hpp>
#include <robotics/vehicle/bringup_policy.hpp>
#include <robotics/vehicle/calibration.hpp>
#include <core/attitude.hpp>
#include <core/clock.hpp>
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>
#include <drivers/imu/bmi088_imu.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <robotics/chassis/imu_heading.hpp>
#include "../../../robotics/common/operator_controls.hpp"
#include <zephyr/kernel.h>
using namespace skywalker;
using namespace skywalker::robotics;
namespace installed = skywalker::robotics::vehicle;
using installed::DiagnosticReason;
namespace {
communication::AsyncUart::DmaBuffers link_dma __nocache;
communication::ConfiguredInterBoardTransport transport({.kind = communication::InterBoardTransportKind::Uart,
                                                        .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))},
                                                       &link_dma);
communication::InterBoardEndpoint endpoint(transport, {.role = BoardRole::GimbalController});
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                     samples::control::receiverConfig());
// 板载 BMI088（云台板 = dual_imu 样例里的 large_yaw_carrier 安装），走 SPI。
// 参考底盘期望的 0x0404 航向就来自云台板载 BMI088 的 EKF 姿态。
// Startup stationary calibration only; slow real turns must not become gyro bias.
control::QuaternionEkf onboard_estimator({.initialization_samples = 800, .adapt_gyro_bias = false});
imu::Bmi088Imu onboard_source(DEVICE_DT_GET(DT_ALIAS(accel0)), DEVICE_DT_GET(DT_ALIAS(gyro0)),
                              {.reference = installed::carrier_reference,
                               .sensor_to_body = installed::carrier_sensor_to_body,
                               .correct_gyro_bias = true},
                              &onboard_estimator);
imu::ImuReceiver onboard(onboard_source, {.poll_interval_us = 500, .priority = 5});
// 零位在解除授权时采集，只用相对启动零位的航向，不假设绝对朝向。
ImuHeadingReference head_heading;
} // namespace

int main() {
    installed::Diagnostics diagnostics;
    diagnostics.start("gimbal_big_yaw_command", "gimbal", "big-yaw world rate + head imu");
    const int setup = remote.start();
    diagnostics.event("remote.start", setup);
    const int head_error = onboard.start();
    diagnostics.event("head_imu", head_error);
    const bool ready = setup == 0 && head_error == 0;
    samples::control::RcControlAdapter controls;
    samples::control::OperatorControlPublisher authority;
    communication::RemoteReceiver::Snapshot received{};
    std::uint32_t sequence = 0, chassis_sequence = 0;
    for (;;) {
        const auto now_us = core::monotonicTimeUs(), now = now_us / 1000;
        endpoint.poll(now);
        const auto peer = endpoint.snapshot();
        (void)remote.snapshot(received);
        const auto &state = controls.update(received.remote, now);
        const bool revoked = authority.publish(endpoint, state.run_allowed, false, received.remote.stamp,
                                               state.clear_event_id, state.clear_stamp, now);
        if (revoked)
            controls.withdraw();
        // 参考底盘的 gyro 大 Yaw 后端必须有云台头部航向样本才允许使能。
        const auto heading = head_heading.update(onboard.snapshot(), !state.run_allowed, now_us);
        const bool heading_fresh = heading.valid && core::fresh(heading.stamp, now_us, 20000);
        const bool peer_feedback_valid = peer.online && peer.big_yaw_feedback.valid &&
                                         forwardedFresh(peer.big_yaw_feedback.stamp,
                                                        peer.big_yaw_feedback.production_age_ms, now, 250) &&
                                         std::isfinite(peer.big_yaw_feedback.actual_rate_rad_s);
        const bool allowed = ready && state.run_allowed && !revoked && peer.online && heading_fresh;
        const auto age = received.remote.stamp.valid && now >= received.remote.stamp.timestamp_ms
                             ? std::uint32_t(
                                   std::min<std::uint64_t>(now - received.remote.stamp.timestamp_ms, UINT32_MAX))
                             : UINT32_MAX;
        BigYawRequest request{};
        request.mode = allowed ? BigYawMode::FollowCenter : BigYawMode::Disabled;
        // Preserve the current v4 peer convention; do not silently flip one end.
        request.target_rate_rad_s = allowed && peer_feedback_valid
                                        ? -samples::control::RcControlAdapter::normalize(
                                              received.remote.analog.right_x) *
                                              std::min(installed::bringup::big_yaw_rate_rad_s, 1.0f)
                                        : 0;
        request.source_sequence = received.remote.stamp.sequence;
        request.source_age_ms = age;
        request.receiver_boot_id = peer.peer.sender_boot_id;
        request.permission = {state.fresh, allowed, received.remote.stamp};
        request.permission_age_ms = age;
        request.stamp = {now, ++sequence, true};
        endpoint.submitBigYaw(request);
        if (peer.online && peer.peer.sender_boot_id) {
            GimbalImuYaw sample{};
            sample.valid = heading_fresh;
            sample.yaw_rad = heading.yaw_rad;
            sample.yaw_rate_rad_s = heading.rate_rad_s;
            sample.zero_generation = heading.generation;
            sample.sample_sequence = static_cast<std::uint32_t>(heading.stamp.sequence);
            // 向上取整：传输亚毫秒样本不能让它显得更年轻。
            sample.source_age_ms = heading.stamp.valid && now_us >= heading.stamp.time_us
                                       ? static_cast<std::uint32_t>(
                                             std::min<std::uint64_t>((now_us - heading.stamp.time_us + 999) / 1000,
                                                                     UINT32_MAX))
                                       : UINT32_MAX;
            sample.receiver_boot_id = peer.peer.sender_boot_id;
            sample.stamp = {now, sequence + 1, true};
            endpoint.submitGimbalImuYaw(sample);
        }
        // 参考底盘的 RobotBigYawControl() 只在底盘控制源被授权时才会执行，
        // 所以这里发一条零速 BodyVelocity 只用于放行大 Yaw 通道，不给轮子任何速度。
        ChassisCommand chassis{};
        chassis.mode = allowed ? ChassisMode::BodyVelocity : ChassisMode::Disabled;
        chassis.source = ControlSource::Remote;
        chassis.stamp = {now, ++chassis_sequence, true};
        endpoint.submit(chassis);
        diagnostics.status(
            {.reason = !ready                 ? DiagnosticReason::InitFailed
                       : !state.fresh         ? DiagnosticReason::RemoteStale
                       : !peer.online         ? DiagnosticReason::PeerStale
                       : !heading_fresh       ? DiagnosticReason::ImuStale
                       : !allowed             ? DiagnosticReason::Disarmed
                       : !peer_feedback_valid ? DiagnosticReason::PeerStale
                                              : DiagnosticReason::Ready,
             .valid = heading_fresh && peer_feedback_valid,
             .enabled = allowed && peer_feedback_valid,
             .target = request.target_rate_rad_s,
             .feedback = installed::reference_big_yaw.direction * peer.big_yaw_feedback.actual_rate_rad_s /
                         installed::reference_big_yaw.gear_ratio,
             .age_ms = age,
             .detail = float(unsigned(request.mode)),
             .error = !ready ? (setup ? setup : head_error) : peer.error,
             .extra0 = heading_fresh ? heading.yaw_rad : std::numeric_limits<float>::quiet_NaN(),
             .extra1 = heading_fresh ? heading.rate_rad_s : std::numeric_limits<float>::quiet_NaN()});
        k_sleep(K_MSEC(5));
    }
}
