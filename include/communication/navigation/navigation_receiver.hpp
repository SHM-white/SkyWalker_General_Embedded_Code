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
        std::uint32_t forwarded_frames = 0, forward_dropped = 0, forward_errors = 0;
        int tx_error = 0;
    };
    NavigationReceiver(const device *uart, AsyncUart::DmaBuffers &dma) : uart_(uart, dma) {
        k_msgq_init(&forward_queue_, forward_storage_, sizeof(ForwardFrame), 8);
    }
    NavigationReceiver(const NavigationReceiver &) = delete;
    NavigationReceiver &operator=(const NavigationReceiver &) = delete;
    int start();
    Snapshot snapshot() const;
    // Thread-only, bounded and nonblocking; copies one complete referee frame.
    // This worker remains the sole UART owner for both RX and TX.
    int forwardRefereeFrame(const std::uint8_t *bytes, std::size_t size);

private:
    static void entry(void *, void *, void *);
    void run();
    AsyncUart uart_;
    NavigationProtocol protocol_;
    struct ForwardFrame {
        std::uint8_t bytes[256];
        std::uint16_t size;
        core::TimeUs queued_us;
    };
    alignas(4) char forward_storage_[8 * sizeof(ForwardFrame)]{};
    k_msgq forward_queue_{};
    atomic_t forward_dropped_ = 0;
    atomic_t started_ = 0;
    mutable k_spinlock lock_{};
    Snapshot value_{};
    k_thread thread_{};
    K_KERNEL_STACK_MEMBER(stack_, CONFIG_SKYWALKER_NAVIGATION_RX_STACK_SIZE);
};
} // namespace skywalker::communication::navigation
