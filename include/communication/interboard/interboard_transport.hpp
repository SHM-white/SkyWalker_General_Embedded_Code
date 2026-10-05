#pragma once
#include <cstddef>
#include <cstdint>

namespace skywalker::communication {
enum class InterBoardTransportKind : std::uint8_t { Uart, Rs485, Can };

// One communication thread owns every method. Objects and callback storage must
// live until reboot: no copying, hot switching, or destruction after service().
class InterBoardTransport {
public:
    // Leave room for the RS485 envelope in AsyncUart's 256-byte DMA buffer.
    static constexpr std::size_t kTxCapacity = 240;
    struct RxChunk {
        std::uint8_t bytes[64]{};
        std::uint16_t size = 0;
        std::uint64_t timestamp_ms = 0;
    };
    virtual ~InterBoardTransport() = default;
    InterBoardTransport(const InterBoardTransport &) = delete;
    InterBoardTransport &operator=(const InterBoardTransport &) = delete;
    virtual InterBoardTransportKind kind() const = 0;
    // Lazily initialize, then advance I/O/recovery on EVERY poll, including errors.
    // 0: locally ready (not peer online); -EAGAIN: recovering; other -errno: error.
    virtual int service(std::uint64_t now_ms) = 0;
    // 0: copied bytes; -EAGAIN: empty; -EOVERFLOW: discard protocol partial frame.
    virtual int read(RxChunk &out) = 0;
    // Copy the entire batch or accept none. 0 is acceptance, not delivery.
    // 1..240 bytes; 1..1000 ms total batch lifetime; -EAGAIN: backpressure.
    virtual int send(const std::uint8_t *, std::size_t, std::uint32_t timeout_ms = 60) = 0;
    virtual bool txBusy() const = 0;

protected:
    InterBoardTransport() = default;
};
}
