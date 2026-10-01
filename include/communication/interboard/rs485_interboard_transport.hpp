#pragma once
#include <communication/async_uart.hpp>
#include <communication/interboard/transport_rx_queue.hpp>

namespace skywalker::communication {
// Two-wire, hardware-DE UART. Exactly one coordinator and one responder.
// Coordinator grants a bounded response window in each CRC-protected request.
class Rs485InterBoardTransport final : public InterBoardTransport {
public:
    enum class Role : std::uint8_t { Coordinator, Responder };
    struct Config {
        const device *uart = nullptr;
        Role role = Role::Coordinator;
        std::uint32_t poll_interval_ms = 5;
        std::uint16_t response_window_ms = 20;
        std::uint16_t turnaround_ms = 1;
    };
    Rs485InterBoardTransport(const Config &config, AsyncUart::DmaBuffers &dma)
        : config_(config), uart_(config.uart, dma) {}
    InterBoardTransportKind kind() const override { return InterBoardTransportKind::Rs485; }
    int service(std::uint64_t now_ms) override;
    int read(RxChunk &out) override { return rx_.read(out); }
    int send(const std::uint8_t *, std::size_t, std::uint32_t timeout_ms = 60) override;
    bool txBusy() const override { return pending_size_ != 0 || uart_.txBusy(); }
private:
    static constexpr std::size_t kHeaderSize = 12, kOverhead = 14;
    enum class Phase : std::uint8_t { Idle, Sending, AwaitReply, ReplyGranted };
    int initialize(std::uint64_t now_ms);
    void consume(const AsyncUart::RxChunk &, std::uint64_t now_ms);
    void accept(std::uint64_t first_ms, std::uint64_t last_ms, std::uint64_t now_ms);
    void discard(std::size_t);
    int transmit(std::uint64_t now_ms);
    std::uint32_t wireTimeMs(std::size_t size) const;
    Config config_;
    AsyncUart uart_;
    detail::TransportRxQueue rx_;
    std::uint8_t input_[256]{}, pending_[kTxCapacity]{};
    std::uint64_t input_times_[256]{};
    std::size_t input_size_ = 0, pending_size_ = 0;
    std::uint64_t pending_deadline_ = 0, retry_ms_ = 0, uart_retry_ms_ = 0, next_poll_ms_ = 0;
    std::uint64_t phase_deadline_ = 0, reply_after_ms_ = 0, last_rx_ms_ = 0;
    std::uint64_t startup_until_ms_ = 0;
    std::uint32_t baudrate_ = 0, token_ = 0;
    std::uint16_t granted_window_ = 0;
    Phase phase_ = Phase::Idle;
    bool configured_ = false, ready_ = false, reply_received_ = false;
};
}
