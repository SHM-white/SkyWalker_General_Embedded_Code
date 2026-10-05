#include <communication/interboard/uart_interboard_transport.hpp>
#include <cerrno>
#include <cstring>

namespace skywalker::communication {
int UartInterBoardTransport::service(std::uint64_t now) {
    int ret = uart_.service(now);
    if (ret == -EACCES) {
        if (now < retry_ms_)
            return -EAGAIN;
        retry_ms_ = now + 100;
        ret = uart_.init();
    }
    ready_ = ret == 0;
    if (sending_ && !uart_.txBusy()) {
        sending_ = false;
        if (ret == 0 && uart_.txError() < 0)
            ret = uart_.txError();
    }
    return ret;
}
int UartInterBoardTransport::read(RxChunk &out) {
    if (!ready_)
        return -EAGAIN;
    AsyncUart::RxChunk raw{};
    const int ret = uart_.read(raw);
    if (ret < 0)
        return ret;
    RxChunk next{};
    static_assert(sizeof(next.bytes) == sizeof(raw.bytes));
    next.size = raw.size;
    next.timestamp_ms = raw.timestamp_ms;
    std::memcpy(next.bytes, raw.bytes, raw.size);
    out = next;
    return 0;
}
int UartInterBoardTransport::send(const std::uint8_t *p, std::size_t n, std::uint32_t timeout) {
    if (!p || !n || !timeout || timeout > 1000)
        return -EINVAL;
    if (n > kTxCapacity)
        return -EMSGSIZE;
    if (!ready_)
        return -EACCES;
    const int ret = uart_.send(p, n, timeout);
    if (ret == 0)
        sending_ = true;
    return ret;
}
}
