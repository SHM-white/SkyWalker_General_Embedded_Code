#include <robotics/vehicle/diagnostics.hpp>
#include <robotics/vehicle/calibration.hpp>
#include <core/clock.hpp>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
using namespace skywalker;
using namespace skywalker::robotics;
namespace installed = skywalker::robotics::vehicle;
using installed::DiagnosticReason;
#include <robotics/vehicle/gimbal_profile.hpp>
#include <drivers/imu/imu_receiver.hpp>
namespace {
imu::DmImuCanSource source(installed::headSensorConfig());
imu::ImuReceiver head(source, {.poll_interval_us=1000, .priority=6});
}
int main() {
    installed::Diagnostics diagnostics;
    diagnostics.start("gimbal_head_imu", "gimbal", "observe only");
    const int setup = head.start();
    diagnostics.event("head.start", setup);
    for (;;) {
        const auto sample = head.snapshot();
        const auto now = core::monotonicTimeUs();
        const auto &orientation = sample.sample.orientation;
        const bool valid = (sample.fresh_mask & imu::Orientation) &&
                           core::fresh(orientation.stamp, now, 20000);
        diagnostics.status({.reason = setup < 0 ? DiagnosticReason::InitFailed :
                            !valid ? DiagnosticReason::ImuStale : DiagnosticReason::Ready,
                            .valid = valid, .enabled = false,
                            .target = orientation.value.w, .feedback = orientation.value.x,
                            .age_ms = orientation.stamp.valid && now >= orientation.stamp.time_us
                                          ? std::uint32_t((now-orientation.stamp.time_us)/1000) : UINT32_MAX,
                            .effort = sample.sample.gyro_rad_s.value.z,
                            .detail = float(sample.fresh_mask),
                            .error = setup < 0 ? setup : sample.diagnostics.last_error,
                            .extra0 = orientation.value.y, .extra1 = orientation.value.z});
        k_sleep(K_MSEC(20));
    }
}
