#include "board_config.hpp"
#include <core/attitude.hpp>
#include <lib/vofa/vofa.h>
#include <zephyr/logging/log.h>
#include <limits>
LOG_MODULE_REGISTER(dual_imu_bench, LOG_LEVEL_INF);
namespace imu = skywalker::imu;
namespace core = skywalker::core;
namespace {
static skywalker::communication::AsyncUart::DmaBuffers external_dma __nocache;
skywalker::control::QuaternionEkf estimator(bench::estimator);
imu::Bmi088Imu onboard_source(bench::accel, bench::gyro, bench::onboard, &estimator);
imu::DmImuRs485Source external_source(bench::external_uart, external_dma, bench::external);
imu::ImuHeater heater(bench::heater);
imu::ImuReceiver onboard(onboard_source, bench::onboard_receiver, &heater);
imu::ImuReceiver external(external_source, bench::external_receiver);
void telemetryTask(void *, void *, void *) {
    static Vofa vofa{};
    const int init = vofa_init(&vofa, bench::telemetry_uart);
    LOG_INF("VOFA init=%d", init);
    if (init < 0)
        return;
    int last = 0;
    constexpr float invalid = std::numeric_limits<float>::quiet_NaN();
    for (;;) {
        const auto a = onboard.snapshot(), b = external.snapshot();
        const auto ae = core::euler(a.sample.orientation.value), be = core::euler(b.sample.orientation.value);
        const auto ag = a.sample.gyro_rad_s.value, bg = b.sample.gyro_rad_s.value;
        const bool aq = a.fresh_mask & imu::Orientation, bq = b.fresh_mask & imu::Orientation;
        const bool av = a.fresh_mask & imu::Gyro, bv = b.fresh_mask & imu::Gyro;
        const float values[] = {aq ? ae.roll : invalid,
                                aq ? ae.pitch : invalid,
                                aq ? ae.yaw : invalid,
                                av ? ag.x : invalid,
                                av ? ag.y : invalid,
                                av ? ag.z : invalid,
                                bq ? be.roll : invalid,
                                bq ? be.pitch : invalid,
                                bq ? be.yaw : invalid,
                                bv ? bg.x : invalid,
                                bv ? bg.y : invalid,
                                bv ? bg.z : invalid,
                                (a.fresh_mask & imu::Temperature) ? a.sample.temperature_c.value : invalid,
                                onboard.status().heater_duty,
                                float(a.fresh_mask),
                                float(b.fresh_mask)};
        static_assert(sizeof(values) / sizeof(float) == VOFA_MAX_FLOATS);
        const int r = vofa_send(&vofa, values, VOFA_MAX_FLOATS);
        if (r != last) {
            if (r < 0)
                LOG_WRN("VOFA send=%d", r);
            last = r;
        }
        k_sleep(K_MSEC(10));
    }
}
}
K_THREAD_DEFINE(telemetry_thread, 4096, telemetryTask, nullptr, nullptr, nullptr, 7, K_FP_REGS, 0);
int main() {
    LOG_INF("Dual IMU bench: onboard EKF + 50 C heater; DM active RS485-2 at 1 Mbit/s");
    const int onboard_result = onboard.start();
    const int external_result = external.start();
    LOG_INF("IMU workers: onboard=%d external=%d", onboard_result, external_result);
    return onboard_result < 0 ? onboard_result : external_result;
}
