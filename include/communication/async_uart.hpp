#pragma once
#include <cstddef>
#include <cstdint>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
namespace skywalker::communication {
// One communication task calls init/service/read/send. Callback only owns buffers
// and copies RX chunks. Instances must outlive registered device callbacks.
class AsyncUart {
public:
    struct RxChunk { std::uint8_t bytes[64]{}; std::uint16_t size=0; std::uint64_t timestamp_ms=0; atomic_val_t generation=0; };
    explicit AsyncUart(const device *uart): uart_(uart) {}
    int init();
    int service(std::uint64_t now_ms);
    int read(RxChunk &out); // -EAGAIN empty; -EOVERFLOW: caller MUST discard parser partial bytes.
    int send(const std::uint8_t *,std::size_t size,std::uint32_t timeout_ms=20);
    bool txBusy() const { return atomic_get(&tx_busy_)!=0; }
    atomic_val_t droppedChunks() const { return atomic_get(&dropped_); }
private:
    static void callback(const device *,uart_event *,void *);
    void event(uart_event &);
    int startRx();
    const device *uart_;
    alignas(32) std::uint8_t rx_[2][128]{};
    alignas(32) std::uint8_t tx_[256]{};
    alignas(4) char queue_storage_[8*sizeof(RxChunk)]{};
    k_msgq queue_{};
    atomic_t in_use_=0,rx_disabled_=1,tx_busy_=0,generation_=0,dropped_=0;
    atomic_val_t observed_generation_=0;
    std::uint64_t retry_ms_=0,tx_deadline_ms_=0;
    bool initialized_=false;
};
}
