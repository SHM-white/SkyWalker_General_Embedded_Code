#pragma once
#include <cerrno>
#include <variant>
#include <communication/interboard/uart_interboard_transport.hpp>
#include <communication/interboard/rs485_interboard_transport.hpp>
#include <communication/interboard/can_interboard_transport.hpp>

namespace skywalker::communication {
// One backend in inline storage: no heap, hot switching or constructor I/O.
class ConfiguredInterBoardTransport final : public InterBoardTransport {
public:
    struct Config {
        InterBoardTransportKind kind = InterBoardTransportKind::Uart;
        const device *uart = nullptr;
        Rs485InterBoardTransport::Config rs485{};
        CanInterBoardTransport::Config can{};
    };
    explicit ConfiguredInterBoardTransport(const Config &config, AsyncUart::DmaBuffers *dma = nullptr)
        : kind_(config.kind) {
        switch (kind_) {
        case InterBoardTransportKind::Uart:
#if defined(CONFIG_SKYWALKER_UART_TRANSPORT)
            if (!dma) { error_ = -EINVAL; break; }
            selected_ = &storage_.emplace<UartInterBoardTransport>(config.uart, *dma);
#endif
            break;
        case InterBoardTransportKind::Rs485:
#if defined(CONFIG_SKYWALKER_INTERBOARD_RS485)
            if (!dma) { error_ = -EINVAL; break; }
            selected_ = &storage_.emplace<Rs485InterBoardTransport>(config.rs485, *dma);
#endif
            break;
        case InterBoardTransportKind::Can:
#if defined(CONFIG_SKYWALKER_INTERBOARD_CAN)
            selected_ = &storage_.emplace<CanInterBoardTransport>(config.can);
#endif
            break;
        default:
            error_ = -EINVAL;
        }
    }
    InterBoardTransportKind kind() const override { return kind_; }
    int service(std::uint64_t now_ms) override { return selected_ ? selected_->service(now_ms) : error_; }
    int read(RxChunk &out) override { return selected_ ? selected_->read(out) : -EAGAIN; }
    int send(const std::uint8_t *p, std::size_t n, std::uint32_t timeout_ms = 60) override {
        return selected_ ? selected_->send(p, n, timeout_ms) : error_;
    }
    bool txBusy() const override { return selected_ && selected_->txBusy(); }
private:
    std::variant<std::monostate
#if defined(CONFIG_SKYWALKER_UART_TRANSPORT)
        , UartInterBoardTransport
#endif
#if defined(CONFIG_SKYWALKER_INTERBOARD_RS485)
        , Rs485InterBoardTransport
#endif
#if defined(CONFIG_SKYWALKER_INTERBOARD_CAN)
        , CanInterBoardTransport
#endif
    > storage_{};
    InterBoardTransportKind kind_;
    InterBoardTransport *selected_ = nullptr;
    int error_ = -ENOTSUP;
};
}
