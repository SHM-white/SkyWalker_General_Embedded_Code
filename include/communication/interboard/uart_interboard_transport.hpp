#pragma once
#include <communication/async_uart.hpp>
#include <communication/interboard/interboard_transport.hpp>

namespace skywalker::communication {
class UartInterBoardTransport final : public InterBoardTransport {
public:
    UartInterBoardTransport(const device *uart, AsyncUart::DmaBuffers &dma) : uart_(uart, dma) {
    }
    InterBoardTransportKind kind() const override {
        return InterBoardTransportKind::Uart;
    }
    int service(std::uint64_t now_ms) override;
    int read(RxChunk &) override;
    int send(const std::uint8_t *, std::size_t, std::uint32_t timeout_ms = 60) override;
    bool txBusy() const override {
        return uart_.txBusy();
    }

private:
    AsyncUart uart_;
    std::uint64_t retry_ms_ = 0;
    bool ready_ = false, sending_ = false;
};
}
