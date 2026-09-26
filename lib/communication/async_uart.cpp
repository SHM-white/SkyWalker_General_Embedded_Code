#include <algorithm>
#include <cerrno>
#include <cstring>
#include <communication/async_uart.hpp>
namespace skywalker::communication {
static_assert(!IS_ENABLED(CONFIG_DCACHE) || IS_ENABLED(CONFIG_NOCACHE_MEMORY),
              "AsyncUart DMA buffers require CONFIG_NOCACHE_MEMORY when DCACHE is enabled");
int AsyncUart::init() {
    if (initialized_)
        return -EALREADY;
    if (!uart_ || !device_is_ready(uart_))
        return -ENODEV;
    k_msgq_init(&queue_, queue_storage_, sizeof(RxChunk), 8);
    const int ret = uart_callback_set(uart_, callback, this);
    if (ret < 0)
        return ret;
    initialized_ = true;
    return startRx();
}
int AsyncUart::startRx() {
    atomic_set(&in_use_, 1);
    atomic_clear(&rx_disabled_);
    const int ret = uart_rx_enable(uart_, dma_.rx[0], sizeof(dma_.rx[0]), 1000);
    if (ret < 0) {
        atomic_clear(&in_use_);
        atomic_set(&rx_disabled_, 1);
    }
    return ret;
}
int AsyncUart::service(std::uint64_t now) {
    if (!initialized_)
        return -EACCES;
    if (atomic_get(&tx_busy_) && now >= tx_deadline_ms_) {
        tx_deadline_ms_ = now + 100;
        uart_tx_abort(uart_); // Only the completion/abort callback releases dma_.tx.
    }
    if (!atomic_get(&rx_disabled_))
        return 0;
    if (now < retry_ms_)
        return -EAGAIN;
    retry_ms_ = now + 100;
    return startRx();
}
void AsyncUart::callback(const device *, uart_event *e, void *self) {
    static_cast<AsyncUart *>(self)->event(*e);
}
void AsyncUart::event(uart_event &e) {
    switch (e.type) {
    case UART_RX_RDY: {
        const auto *p = e.data.rx.buf + e.data.rx.offset;
        std::size_t remaining = e.data.rx.len;
        while (remaining) {
            RxChunk c{};
            c.size = std::min(remaining, sizeof(c.bytes));
            c.timestamp_ms = k_uptime_get();
            c.generation = atomic_get(&generation_);
            std::memcpy(c.bytes, p, c.size);
            if (k_msgq_put(&queue_, &c, K_NO_WAIT) < 0) {
                atomic_inc(&generation_);
                atomic_inc(&dropped_);
            }
            p += c.size;
            remaining -= c.size;
        }
        break;
    }
    case UART_RX_BUF_REQUEST:
        for (unsigned i = 0; i < 2; ++i)
            if (!atomic_test_and_set_bit(&in_use_, i)) {
                if (uart_rx_buf_rsp(uart_, dma_.rx[i], sizeof(dma_.rx[i])) < 0)
                    atomic_clear_bit(&in_use_, i);
                break;
            }
        break;
    case UART_RX_BUF_RELEASED:
        for (unsigned i = 0; i < 2; ++i)
            if (e.data.rx_buf.buf == dma_.rx[i])
                atomic_clear_bit(&in_use_, i);
        break;
    case UART_RX_STOPPED:
        atomic_inc(&generation_);
        break;
    case UART_RX_DISABLED:
        atomic_inc(&generation_);
        atomic_set(&rx_disabled_, 1);
        break;
    case UART_TX_ABORTED:
    case UART_TX_DONE:
        atomic_clear(&tx_busy_);
        break;
    default:
        break;
    }
}
int AsyncUart::read(RxChunk &out) {
    if (!initialized_)
        return -EACCES;
    const auto gen = atomic_get(&generation_);
    if (observed_generation_ != gen) {
        observed_generation_ = gen;
        return -EOVERFLOW;
    }
    RxChunk next{};
    while (k_msgq_get(&queue_, &next, K_NO_WAIT) == 0) {
        if (next.generation != observed_generation_)
            continue;
        if (atomic_get(&generation_) != observed_generation_)
            return -EOVERFLOW;
        out = next;
        return 0;
    }
    return -EAGAIN;
}
int AsyncUart::send(const std::uint8_t *p, std::size_t n, std::uint32_t timeout) {
    if (!initialized_ || !p || !n || n > sizeof(dma_.tx) || !timeout || timeout > 1000)
        return -EINVAL;
    if (!atomic_cas(&tx_busy_, 0, 1))
        return -EAGAIN;
    tx_deadline_ms_ = static_cast<std::uint64_t>(k_uptime_get()) + timeout;
    std::memcpy(dma_.tx, p, n);
    const int ret = uart_tx(uart_, dma_.tx, n, timeout * 1000);
    if (ret < 0)
        atomic_clear(&tx_busy_);
    return ret;
}
}
