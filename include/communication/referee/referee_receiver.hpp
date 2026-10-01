#pragma once
#include <cerrno>
#include <communication/async_uart.hpp>
#include <communication/referee/referee_service.hpp>
namespace skywalker::communication {
// One owner task polls. This object and DMA buffers must outlive callbacks.
class RefereeReceiver {
public:
    RefereeReceiver(const device *device, AsyncUart::DmaBuffers &dma, RefereeVersion version,
                    std::uint32_t timeout_ms = 500) : uart_(device, dma), service_(version, timeout_ms) {}
    robotics::RefereeState poll(std::uint64_t now_ms) {
        if (now_ms >= retry_ms_) {
            int ret = uart_.service(now_ms);
            if (ret == -EACCES) ret = uart_.init();
            if (ret < 0 && ret != -EAGAIN) { error_ = ret; retry_ms_ = now_ms + 100; }
            if (ret == 0) {
                error_ = 0;
                AsyncUart::RxChunk chunk{};
                for (unsigned budget = 0; budget < 8; ++budget) {
                    ret = uart_.read(chunk);
                    if (ret == -EOVERFLOW) { service_.discardPartial(); continue; }
                    if (ret == -EAGAIN) break;
                    if (ret < 0) { error_ = ret; break; }
                    ret = service_.processBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
                    if (ret < 0) error_ = ret;
                }
            }
        }
        service_.processBytes(nullptr, 0, now_ms);
        robotics::RefereeState result{};
        service_.snapshot(now_ms, result);
        return result;
    }
    int error() const { return error_; }
private:
    AsyncUart uart_;
    RefereeService service_;
    std::uint64_t retry_ms_ = 0;
    int error_ = 0;
};
}
