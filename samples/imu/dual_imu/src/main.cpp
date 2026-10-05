#include "board_config.hpp"
#include <core/attitude.hpp>
#include <core/clock.hpp>
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
core::TimeUs ageUs(const core::Stamp &stamp, core::TimeUs now) {
    return stamp.valid && now >= stamp.time_us ? now - stamp.time_us : UINT64_MAX;
}
void logObservation(const char *mount, const imu::Snapshot &s, core::TimeUs now) {
    LOG_INF(
        "mount=%s ref=%u/%u quality=%u fresh=%x quat_seq=%llu quat_us=%llu quat_age_us=%llu gyro_seq=%llu gyro_us=%llu gyro_age_us=%llu state=%u error=%d gaps=%u",
        mount, s.sample.reference.frame_id, s.sample.reference.epoch, unsigned(s.sample.attitude_quality), s.fresh_mask,
        s.sample.orientation.stamp.sequence, s.sample.orientation.stamp.time_us, ageUs(s.sample.orientation.stamp, now),
        s.sample.gyro_rad_s.stamp.sequence, s.sample.gyro_rad_s.stamp.time_us, ageUs(s.sample.gyro_rad_s.stamp, now),
        unsigned(s.state), s.diagnostics.last_error, s.diagnostics.transport_gaps);
}
void telemetryTask(void *, void *, void *) {
    static Vofa vofa{};
    const int init = vofa_init(&vofa, bench::telemetry_uart);
    LOG_INF("VOFA init=%d", init);
    int last = 0;
    core::TimeUs next_observation = 0;
    constexpr float invalid = std::numeric_limits<float>::quiet_NaN();
    for (;;) {
        const auto a = onboard.snapshot();
        const auto b = bench::use_external ? external.snapshot() : imu::Snapshot{};
        const auto now = core::monotonicTimeUs();
        if (now >= next_observation) {
            next_observation = now + 200000;
            logObservation(bench::onboard_mount, a, now);
            if (bench::use_external)
                logObservation(bench::external_mount, b, now);
        }
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
        const int r = init == 0 ? vofa_send(&vofa, values, VOFA_MAX_FLOATS) : init;
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
    LOG_INF("IMU observation: onboard=%s external=%s; sensor_to_body requires calibration", bench::onboard_mount,
            bench::use_external ? bench::external_mount : "disabled");
    const int onboard_result = onboard.start();
    const int external_result = bench::use_external ? external.start() : 0;
    LOG_INF("IMU workers: onboard=%d external=%d", onboard_result, external_result);
    return onboard_result < 0 ? onboard_result : external_result;
}
