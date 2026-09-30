#include <drivers/imu/bmi088_imu.hpp>
#include <drivers/imu/imu_heater.hpp>
#include <core/attitude.hpp>
#include <core/clock.hpp>
#include <lib/vofa/vofa.h>
#include <zephyr/logging/log.h>
#include <cerrno>
LOG_MODULE_REGISTER(imu_bench, LOG_LEVEL_INF);
namespace imu = skywalker::imu;
namespace core = skywalker::core;
namespace {
skywalker::control::QuaternionEkf estimator({});
imu::Bmi088Imu source(DEVICE_DT_GET(DT_ALIAS(accel0)), DEVICE_DT_GET(DT_ALIAS(gyro0)), {}, &estimator);
imu::ImuHeater heater({.pwm = PWM_DT_SPEC_GET(DT_ALIAS(imu_heater))});
Vofa vofa{};
}
int main() {
    const int hr = heater.init(), ir = source.init();
    const int vr = vofa_init(&vofa, DEVICE_DT_GET(DT_ALIAS(telemetry_uart)));
    LOG_INF("IMU=%d heater=%d VOFA=%d", ir, hr, vr);
    if (ir < 0) {
        if (!hr)
            heater.disable();
        return ir;
    }
    core::TimeUs next = 0;
    int last = 0;
    for (;;) {
        const int r = source.service();
        if (r != -EAGAIN && r != last) {
            LOG_WRN("IMU sample=%d", r);
            last = r;
        }
        const auto s = source.snapshot();
        const auto now = core::monotonicTimeUs();
        if (!hr)
            heater.update(s.sample.temperature_c, now);
        if (!vr && now >= next) {
            next = now + 10000;
            const auto e = core::euler(s.sample.orientation.value);
            const float v[] =
                {e.roll, e.pitch, e.yaw, s.sample.temperature_c.value, heater.snapshot().duty, float(s.fresh_mask)};
            vofa_send(&vofa, v, 6);
        }
        k_usleep(500);
    }
}
