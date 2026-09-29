#pragma once
#include <communication/async_uart.hpp>
#include <communication/vision/vision_link.hpp>
namespace skywalker::communication::vision {
// DR16-style standalone worker. No IMU or application dependencies.
// Receiver, protocol and static __nocache DMA storage must outlive callbacks.
class VisionReceiver {
public:
    struct Config {
        VisionLink::Config link{};
        core::TimeUs feedback_period_us = 0; // 0 = RX only; no fabricated feedback.
    };
    enum class State : std::uint8_t { NotStarted, Starting, Running, InitFailed };
    struct Snapshot {
        vision::Snapshot link{};
        State state = State::NotStarted;
        std::uint32_t rx_chunks = 0, resets = 0, dropped = 0, tx_frames = 0, tx_skipped = 0;
        int uart_error = 0, tx_error = 0;
    };
    VisionReceiver(const device *uart, AsyncUart::DmaBuffers &dma, VisionProtocol &p, const Config &c)
        : config_(c), uart_(uart, dma), link_(p, c.link) {
    }
    VisionReceiver(const VisionReceiver &) = delete;
    VisionReceiver &operator=(const VisionReceiver &) = delete;
    int start();
    Snapshot snapshot() const;
    int setFeedback(const Feedback &f) {
        return link_.setFeedback(f);
    }

private:
    static void entry(void *, void *, void *);
    void run();
    const Config config_;
    AsyncUart uart_;
    VisionLink link_;
    atomic_t started_ = 0;
    mutable k_spinlock lock_{};
    Snapshot value_{};
    k_thread thread_{};
    K_KERNEL_STACK_MEMBER(stack_, CONFIG_SKYWALKER_VISION_RX_STACK_SIZE);
};
}
