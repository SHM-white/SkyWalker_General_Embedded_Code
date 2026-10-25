#include <robotics/vehicle/diagnostics.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/printk.h>

namespace {
using namespace skywalker::robotics::vehicle;
const device *const diagnostic_uart = DEVICE_DT_GET(DT_ALIAS(diagnostic_uart));
struct Packet { std::uint16_t size = 0; std::uint8_t data[192]{}; };
K_MSGQ_DEFINE(diagnostic_queue, sizeof(Packet), 16, 4);
atomic_t dropped_packets = 0;
int output_error = -EAGAIN;

void enqueue(const std::uint8_t *bytes, std::size_t size) {
    Packet packet{};
    if (size > sizeof(packet.data)) {
        atomic_inc(&dropped_packets);
        return;
    }
    packet.size = size;
    std::memcpy(packet.data, bytes, size);
    if (k_msgq_put(&diagnostic_queue, &packet, K_NO_WAIT) != 0)
        atomic_inc(&dropped_packets);
}
int textOutput(std::uint8_t *bytes, std::size_t size, void *) {
    if constexpr (diagnostic_output == DiagnosticOutput::Log)
        enqueue(bytes, size);
    return size;
}
std::uint8_t log_buffer[192];
LOG_OUTPUT_DEFINE(diagnostic_log_output, textOutput, log_buffer, sizeof(log_buffer));
void processLog(const log_backend *, log_msg_generic *message) {
    if constexpr (diagnostic_output == DiagnosticOutput::Log)
        log_output_msg_process(&diagnostic_log_output, &message->log,
                               LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_TIMESTAMP);
}
void droppedLog(const log_backend *, std::uint32_t count) {
    atomic_add(&dropped_packets, count);
}
void panicLog(const log_backend *) {
    // No synchronous serial writes on a failing execution thread. VOFA never
    // falls back to text; a fatal error may stop the stream entirely.
}
const log_backend_api diagnostic_backend_api{
    .process = processLog,
    .dropped = droppedLog,
    .panic = panicLog,
};
LOG_BACKEND_DEFINE(vehicle_diagnostic_backend, diagnostic_backend_api, true);

int initializeOutput() {
    output_error = device_is_ready(diagnostic_uart) ? 0 : -ENODEV;
    return 0; // Diagnostics failure must not abort initialization of other devices.
}
SYS_INIT(initializeOutput, POST_KERNEL, 99);
void outputTask(void *, void *, void *) {
    Packet packet{};
    for (;;) {
        k_msgq_get(&diagnostic_queue, &packet, K_FOREVER);
        if (output_error) {
            atomic_inc(&dropped_packets);
            continue;
        }
        for (unsigned i = 0; i < packet.size; ++i)
            uart_poll_out(diagnostic_uart, packet.data[i]);
    }
}
K_THREAD_DEFINE(diagnostic_worker, 2048, outputTask, nullptr, nullptr, nullptr, 12, 0, 0);

template<typename... Args> void textLine(const char *format, Args... args) {
    if constexpr (diagnostic_output == DiagnosticOutput::Log) {
        char line[192];
        const int length = snprintk(line, sizeof(line), format, args...);
        if (length > 0)
            enqueue(reinterpret_cast<const std::uint8_t *>(line),
                    std::min<std::size_t>(length, sizeof(line) - 1));
    }
}
} // namespace

namespace skywalker::robotics::vehicle {
int Diagnostics::start(const char *application, const char *role, const char *limits) {
    application_ = application;
    textLine("app=%s role=%s source=%s protocol=4\n", application, role, SKYWALKER_SOURCE_VERSION);
    textLine("hardware=%s calibration=%s uart=%s baud=%u\n", hardware_revision,
             calibration_revision, diagnostic_uart->name, diagnostic_baudrate);
    textLine("resources: diagnostic=UART7/PE8-TX,PE7-RX; remote=UART5/PD2; see installed overlays\n");
    textLine("limits=%s\n", limits);
    event("diagnostic", output_error);
    status({.reason = output_error ? DiagnosticReason::InitFailed : DiagnosticReason::Boot,
            .error = output_error});
    return output_error;
}
void Diagnostics::event(const char *step, int result) {
    if (result < 0 && !first_error_)
        first_error_ = result;
    textLine("%s t=%u step=%s result=%d\n", application_, std::uint32_t(k_uptime_get()), step, result);
}
void Diagnostics::status(const DiagnosticStatus &sample) {
    const auto now = static_cast<std::uint64_t>(k_uptime_get());
    const bool changed = sample.reason != last_reason_;
    if (changed) {
        textLine("%s t=%u reason=%u previous=%u duration_ms=%u seq=%u\n", application_,
                 std::uint32_t(now), unsigned(sample.reason), unsigned(last_reason_),
                 std::uint32_t(now - transition_ms_), sequence_);
        last_reason_ = sample.reason;
        transition_ms_ = now;
    }
    if (now < next_sample_ms_)
        return;
    next_sample_ms_ = now + diagnostic_period_ms;
    ++sequence_;
    const auto dropped = static_cast<std::uint32_t>(atomic_get(&dropped_packets));
    if constexpr (diagnostic_output == DiagnosticOutput::Vofa) {
        const auto time = static_cast<std::uint32_t>(now);
        const float values[] = {
            1, float(sequence_ & 0xffff), float(sequence_ >> 16),
            float(time & 0xffff), float(time >> 16), float(unsigned(sample.reason)),
            float(sample.valid), float(sample.enabled), sample.target,
            sample.valid ? sample.feedback : std::numeric_limits<float>::quiet_NaN(),
            sample.age_ms == UINT32_MAX ? -1.0f : float(sample.age_ms),
            sample.dt_us, sample.effort, sample.detail,
            float(sample.error ? sample.error : first_error_),
            float(dropped & 0xffff), float(dropped >> 16), sample.extra0, sample.extra1};
        std::uint8_t bytes[sizeof(values) + 4];
        static_assert(sizeof(float) == 4);
        for (unsigned i = 0; i < ARRAY_SIZE(values); ++i) {
            std::uint32_t bits;
            std::memcpy(&bits, &values[i], sizeof(bits));
            sys_put_le32(bits, bytes + 4 * i);
        }
        sys_put_le32(0x7f800000, bytes + sizeof(values));
        enqueue(bytes, sizeof(bytes));
    }
    else if (changed || now >= next_log_ms_) {
        next_log_ms_ = now + 1000;
        textLine("%s t=%u seq=%u reason=%u valid=%u enabled=%u age_ms=%u error=%d dropped=%u\n",
                 application_, std::uint32_t(now), sequence_, unsigned(sample.reason), sample.valid,
                 sample.enabled, sample.age_ms, sample.error ? sample.error : first_error_, dropped);
        textLine("target=%f feedback=%f effort=%f dt_us=%f detail=%f extra=%f,%f\n",
                 double(sample.target), sample.valid ? double(sample.feedback) : double(NAN),
                 double(sample.effort), double(sample.dt_us), double(sample.detail),
                 double(sample.extra0), double(sample.extra1));
    }
}
} // namespace skywalker::robotics::vehicle
