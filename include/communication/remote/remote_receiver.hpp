#pragma once
#include <communication/async_uart.hpp>
#include <communication/remote/remote_service.hpp>
#include <latest.hpp>

namespace skywalker::communication {
// One receiver exclusively owns its UART. Both the receiver and its external
// static __nocache DMA storage must outlive device callbacks (including on error).
// No stop/restart or destruction while the system is running is supported.
class RemoteReceiver {
public:
    struct Config {
        Dr16Decoder::Config decoder{};
        RemoteService::Config remote{};
    };
    enum class State : std::uint8_t { NotStarted, Starting, Running, InitFailed };
    struct Snapshot {
        robotics::RemoteState remote{};
        State state = State::NotStarted;
        std::uint32_t rx_chunks = 0, resets = 0;
        unsigned dropped = 0;
        int uart_error = 0;
    };

    RemoteReceiver(const device *uart, AsyncUart::DmaBuffers &dma, const Config &config)
        : config_(config), uart_(uart, dma), service_(config.decoder, config.remote) {
    }
    RemoteReceiver(const RemoteReceiver &) = delete;
    RemoteReceiver &operator=(const RemoteReceiver &) = delete;

    // Thread context only. 0 schedules the worker; inspect Snapshot for UART
    // initialization results. -EINVAL: configuration; -EALREADY: already started.
    [[nodiscard]] int start();
    // Thread context only, multiple readers allowed. 0 means copied, not online.
    // On -EAGAIN retain the caller's previous value, but still expire its online
    // flag. Each reader must initialize and retain its own Snapshot{}.
    int snapshot(Snapshot &out);
    // Thread context; exactly one consumer. Original full decoded frames/stamps.
    int nextFrame(robotics::RemoteState &out);

private:
    static void threadEntry(void *self, void *, void *);
    void run();
    const Config config_;
    AsyncUart uart_;
    RemoteService service_;
    Latest<Snapshot> latest_;
    atomic_t started_ = 0;
    k_thread thread_{};
    K_KERNEL_STACK_MEMBER(thread_stack_, CONFIG_SKYWALKER_REMOTE_RX_STACK_SIZE);
};
} // namespace skywalker::communication
