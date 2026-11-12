#pragma once

#include <communication/async_uart.hpp>
#include <communication/navigation/navigation_protocol.hpp>

namespace skywalker::communication::navigation {
// Static receiver and __nocache DMA storage must outlive the UART callbacks.
class NavigationReceiver {
public:
    enum class State : std::uint8_t { NotStarted, Starting, Running, InitFailed };
    struct Snapshot {
        NavigationProtocol::Snapshot protocol{};
        State state = State::NotStarted;
        std::uint32_t rx_chunks = 0, resets = 0, dropped = 0;
        int uart_error = 0;
    };
    NavigationReceiver(const device *uart, AsyncUart::DmaBuffers &dma) : uart_(uart, dma) {
    }
    NavigationReceiver(const NavigationReceiver &) = delete;
    NavigationReceiver &operator=(const NavigationReceiver &) = delete;
    int start();
    Snapshot snapshot() const;

private:
    static void entry(void *, void *, void *);
    void run();
    AsyncUart uart_;
    NavigationProtocol protocol_;
    atomic_t started_ = 0;
    mutable k_spinlock lock_{};
    Snapshot value_{};
    k_thread thread_{};
    K_KERNEL_STACK_MEMBER(stack_, CONFIG_SKYWALKER_NAVIGATION_RX_STACK_SIZE);
};
} // namespace skywalker::communication::navigation
