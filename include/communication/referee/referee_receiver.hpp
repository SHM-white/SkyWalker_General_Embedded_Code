#pragma once
#include <cerrno>
#include <cstdint>
#include <communication/async_uart.hpp>
#include <communication/referee/referee_service.hpp>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>

namespace skywalker::communication {
// One receiver exclusively owns its UART. Both the receiver and its external
// static __nocache DMA storage must outlive device callbacks (including on error).
// No stop/restart or destruction while the system is running is supported.
class RefereeReceiver {
public:
    enum class State : std::uint8_t { NotStarted, Starting, Running, InitFailed };
    struct Snapshot {
        robotics::RefereeState referee{};
        State state = State::NotStarted;
        std::uint32_t rx_chunks = 0, resets = 0, dropped = 0;
        int uart_error = 0;
    };
    RefereeReceiver(const device *device, AsyncUart::DmaBuffers &dma, std::uint32_t timeout_ms = 500)
        : uart_(device, dma), service_(timeout_ms), timeout_ms_(timeout_ms) {
    }
    RefereeReceiver(const RefereeReceiver &) = delete;
    RefereeReceiver &operator=(const RefereeReceiver &) = delete;
    // Bind before start(); callback runs on the receiver thread and must not block.
    int setFrameHandler(RefereeParser::FrameHandler handler, void *context);
    // Thread context only. 0 schedules the worker; inspect Snapshot for UART
    // initialization results. -EINVAL: configuration; -EALREADY: already started;
    // -EWOULDBLOCK: ISR.
    [[nodiscard]] int start();
    // Thread context, multiple readers allowed. Re-checks online timeout at read time.
    Snapshot snapshot() const;

private:
    static void threadEntry(void *self, void *, void *);
    void run();
    AsyncUart uart_;
    RefereeService service_;
    std::uint32_t timeout_ms_;
    atomic_t started_ = 0;
    mutable k_spinlock lock_{};
    Snapshot value_{};
    k_thread thread_{};
    K_KERNEL_STACK_MEMBER(thread_stack_, CONFIG_SKYWALKER_REFEREE_RX_STACK_SIZE);
};
} // namespace skywalker::communication
