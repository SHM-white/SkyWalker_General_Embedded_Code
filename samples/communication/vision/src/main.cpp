#include "board_config.hpp"
#include <lib/vofa/vofa.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(vision_bench, LOG_LEVEL_INF);
namespace vision = skywalker::communication::vision;
namespace {
static skywalker::communication::AsyncUart::DmaBuffers dma __nocache;
vision::AbProtocol protocol(bench::protocol);
vision::VisionReceiver receiver(bench::vision_uart, dma, protocol, bench::receiver);
void telemetry(void *, void *, void *) {
    static Vofa vofa{};
    const int init = vofa_init(&vofa, bench::telemetry_uart);
    LOG_INF("VOFA init=%d", init);
    if (init < 0)
        return;
    int last_error = 0, last_send = 0;
    for (;;) {
        const auto s = receiver.snapshot();
        const auto &a = s.link.aim.value;
        const float values[] = {s.link.aim_fresh ? 1.0f : 0.0f,
                                a.control_requested ? 1.0f : 0.0f,
                                a.fire_requested ? 1.0f : 0.0f,
                                a.yaw.angle_rad,
                                a.yaw.rate_rad_s,
                                a.yaw.acceleration_rad_s2,
                                a.pitch.angle_rad,
                                a.pitch.rate_rad_s,
                                a.pitch.acceleration_rad_s2,
                                float(s.link.aim.stamp.sequence),
                                float(s.rx_chunks),
                                float(s.link.protocol.crc_errors),
                                float(s.link.protocol.invalid_frames),
                                float(s.resets),
                                float(s.dropped),
                                float(static_cast<unsigned>(s.state))};
        static_assert(sizeof(values) / sizeof(float) == VOFA_MAX_FLOATS);
        const int r = vofa_send(&vofa, values, VOFA_MAX_FLOATS);
        if (r != last_send) {
            if (r < 0)
                LOG_WRN("VOFA send=%d", r);
            last_send = r;
        }
        if (s.uart_error != last_error) {
            LOG_WRN("Vision UART status=%d", s.uart_error);
            last_error = s.uart_error;
        }
        k_sleep(K_MSEC(10));
    }
}
}
K_THREAD_DEFINE(telemetry_thread, 4096, telemetry, nullptr, nullptr, nullptr, 7, 0, 0);
int main() {
    const int r = receiver.start();
    LOG_INF("Vision receiver start=%d (AB, 115200, RX only)", r);
    return r;
}
