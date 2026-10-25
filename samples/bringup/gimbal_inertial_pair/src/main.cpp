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
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include "../../../robotics/common/rc_controls.hpp"

#include <drivers/imu/imu_receiver.hpp>
using namespace skywalker;
using namespace skywalker::robotics;
namespace {
communication::AsyncUart::DmaBuffers remote_dma __nocache;
communication::RemoteReceiver remote(DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma,
                                     samples::control::receiverConfig());
RemoteSource source(remote);
CommandManager commands(vehicle::manualCommandConfig());
vehicle::Diagnostics diagnostics;
imu::DmImuCanSource head_source(vehicle::headSensorConfig());
imu::ImuReceiver head(head_source, {.poll_interval_us = 1000, .priority = 6});
motor::Motor yaw(vehicle::yawHardware()), pitch(vehicle::pitchHardware());
motor::Group motors(yaw, pitch);
motor::CanBus yaw_bus(DEVICE_DT_GET(DT_NODELABEL(can1)));
motor::CanBus pitch_bus(DEVICE_DT_GET(DT_NODELABEL(can2)));
GimbalExecutor gimbal(yaw, pitch, motors, vehicle::yawMotorConfig(), vehicle::yawAxisConfig(),
                      vehicle::pitchMotorConfig(), vehicle::pitchAxisConfig(), vehicle::gimbalExecutionConfig());
InertialGimbalAdapter::Config inertialConfig() {
    auto c = vehicle::manualInertialConfig();
    c.yaw.max_rate_rad_s = 1.0f;
    c.pitch.max_rate_rad_s = 0.8f;
    return c;
}
InertialGimbalAdapter adapter(inertialConfig());
} // namespace

int main() {
    diagnostics.start("gimbal_inertial_pair", "gimbal", "yaw<=1rad/s,pitch<=0.8rad/s,yaw_effort<=1.2A,pitch_effort<=0.8Nm");
    const int imu_error = head.start();
    diagnostics.event("head.start", imu_error);
    const int power_error = regulator_enable(DEVICE_DT_GET(DT_NODELABEL(power1)));
    diagnostics.event("motor_power", power_error);
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
    for (;;) {
        const auto now = core::monotonicTimeUs(), ms = now / 1000;
        const auto dt_us = previous_us && now > previous_us ? now - previous_us : 0;
        previous_us = now;
        (void)remote.snapshot(rc);
        const auto &op = controls.update(rc.remote, ms);
        commands.setOperatorGate(op.run_allowed, op.clear_event_id);
        (void)commands.snapshot(frame);
        const auto measured_head = head.snapshot();
        const auto fields = imu::Orientation | imu::Gyro;
        const bool imu_fresh = !imu_error && measured_head.state == imu::State::Running &&
                               (measured_head.fresh_mask & fields) == fields &&
                               core::fresh(measured_head.sample.orientation.stamp, now, 20000) &&
                               core::fresh(measured_head.sample.gyro_rad_s.stamp, now, 20000);
        InertialGimbalInputs input{};
        input.head = measured_head;
        input.yaw = yaw.snapshot();
        input.pitch = pitch.snapshot();
        input.command = frame.decision.command.gimbal;
        input.source_stamp = sourceStamp(frame, input.command.source);
        input.prerequisites_ready = configured && vehicle::imu_mounting_confirmed;
        if (!op.run_allowed || !imu_fresh || source_error)
            input.command.mode = GimbalMode::Disabled;
        const auto inertial = adapter.update(input, now);
        GimbalExecutionInputs request{};
        request.command = inertial.command;
        request.source_stamp = inertial.source_stamp;
        request.yaw_output_valid = inertial.yaw_output_valid;
        request.pitch_output_valid = inertial.pitch_output_valid;
        request.transport_ready = configured && !source_error && !imu_error;
        request.clear_estop = op.clear_estop;
        const auto run = configured ? gimbal.update(request, now)
                                    : gimbal.suspend(now, WaitReason::Configuration, setup_error, true);
        const int yaw_error = yaw_started ? yaw_bus.commit().error : 0;
        const int pitch_error = pitch_started ? pitch_bus.commit().error : 0;
        const auto ys = yaw.snapshot(), ps = pitch.snapshot();
        const auto age = measured_head.sample.orientation.stamp.valid &&
                         now >= measured_head.sample.orientation.stamp.time_us
                             ? std::min<std::uint64_t>((now - measured_head.sample.orientation.stamp.time_us) / 1000,
                                                       UINT32_MAX) : UINT32_MAX;
        const auto reason = setup_error || source_error || imu_error ? vehicle::DiagnosticReason::InitFailed
                            : !vehicle::imu_mounting_confirmed ? vehicle::DiagnosticReason::Configuration
                            : !op.fresh ? vehicle::DiagnosticReason::RemoteStale
                            : !op.run_allowed ? vehicle::DiagnosticReason::Disarmed
                            : !imu_fresh ? vehicle::DiagnosticReason::ImuStale
                            : !inertial.yaw_output_valid || !inertial.pitch_output_valid
                                ? vehicle::DiagnosticReason::FeedbackStale
                            : vehicle::DiagnosticReason::Ready;
        diagnostics.status({.reason = reason,
                            .valid = imu_fresh && ys.feedback_fresh && ps.feedback_fresh,
                            .enabled = run.state == RunState::Active,
                            .target = float(gimbal.yawTargetRad()),
                            .feedback = ys.feedback.position_rad,
                            .age_ms = static_cast<std::uint32_t>(age),
                            .dt_us = float(dt_us),
                            .effort = gimbal.yawTelemetry().effort_command,
                            .detail = float(unsigned(run.reason)),
                            .error = setup_error ? setup_error : source_error ? source_error : imu_error ? imu_error :
                                     yaw_error ? yaw_error : pitch_error ? pitch_error : run.error,
                            .extra0 = inertial.head_yaw_rad,
                            .extra1 = inertial.head_pitch_rad});
        k_sleep(K_USEC(vehicle::gimbal_period_us));
    }
}
