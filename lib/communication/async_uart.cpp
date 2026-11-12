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
    if (ret == -ENOSYS || ret == -ENOTSUP) {
        // USB CDC ACM and DMA-less UARTs only implement the interrupt API.
        const int irq_ret = enterIrqMode();
        if (irq_ret < 0)
            return irq_ret;
        initialized_ = true;
        return 0;
    }
    if (ret < 0)
        return ret;
    initialized_ = true;
    return startRx();
}
int AsyncUart::enterIrqMode() {
    const int ret = uart_irq_callback_user_data_set(uart_, irqCallback, this);
    if (ret < 0)
        return ret;
    irq_mode_ = true;
    atomic_clear(&in_use_);
    atomic_clear(&rx_disabled_);
    uart_irq_tx_disable(uart_);
    uart_irq_rx_enable(uart_);
    return 0;
}
void AsyncUart::irqCallback(const device *dev, void *self) {
    auto *uart = static_cast<AsyncUart *>(self);
    uart_irq_update(dev);
    if (uart_irq_rx_ready(dev) > 0)
        uart->drainIrq();
    if (uart_irq_tx_ready(dev) > 0)
        uart->fillIrq();
}
void AsyncUart::drainIrq() {
    if (!atomic_cas(&rx_draining_, 0, 1))
        return;
    RxChunk chunk{};
    std::uint8_t bytes[sizeof(RxChunk::bytes)];
    for (;;) {
        const int read = uart_fifo_read(uart_, bytes, sizeof(bytes));
        if (read <= 0)
            break;
        chunk = RxChunk{};
        chunk.size = static_cast<std::uint16_t>(read);
        chunk.timestamp_ms = k_uptime_get();
        chunk.generation = atomic_get(&generation_);
        std::memcpy(chunk.bytes, bytes, static_cast<std::size_t>(read));
        if (k_msgq_put(&queue_, &chunk, K_NO_WAIT) < 0) {
            atomic_inc(&generation_);
            atomic_inc(&dropped_);
        }
    }
    atomic_clear(&rx_draining_);
}
int AsyncUart::sendIrq(const std::uint8_t *p, std::size_t n, std::uint32_t timeout_ms) {
    // FIFO access belongs to the driver's callback context. In particular the
    // CDC ACM driver rejects fifo_fill() from this communication thread.
    const auto key = k_spin_lock(&irq_tx_lock_);
    if (atomic_get(&tx_busy_)) {
        k_spin_unlock(&irq_tx_lock_, key);
        return -EAGAIN;
    }
    std::memcpy(dma_.tx, p, n);
    irq_tx_size_ = n;
    irq_tx_offset_ = 0;
    tx_deadline_ms_ = static_cast<std::uint64_t>(k_uptime_get()) + timeout_ms;
    atomic_clear(&tx_error_);
    atomic_set(&tx_busy_, 1);
    uart_irq_tx_enable(uart_);
    k_spin_unlock(&irq_tx_lock_, key);
    return 0;
}
void AsyncUart::fillIrq() {
    const auto key = k_spin_lock(&irq_tx_lock_);
    if (!atomic_get(&tx_busy_)) {
        uart_irq_tx_disable(uart_);
        k_spin_unlock(&irq_tx_lock_, key);
        return;
    }
    const auto remaining = irq_tx_size_ - irq_tx_offset_;
    const int filled = uart_fifo_fill(uart_, dma_.tx + irq_tx_offset_, static_cast<int>(remaining));
    if (filled > 0 && static_cast<std::size_t>(filled) <= remaining)
        irq_tx_offset_ += static_cast<std::size_t>(filled);
    else
        atomic_set(&tx_error_, filled < 0 ? filled : -EIO);
    if (atomic_get(&tx_error_) || irq_tx_offset_ == irq_tx_size_) {
        // A ready TX interrupt with no work would be resubmitted indefinitely
        // by CDC ACM on the cooperative system workqueue, starving control/RX.
        uart_irq_tx_disable(uart_);
        atomic_clear(&tx_busy_);
    }
    k_spin_unlock(&irq_tx_lock_, key);
}
int AsyncUart::startRx() {
    atomic_set(&in_use_, 1);
    atomic_clear(&rx_disabled_);
    const int ret = uart_rx_enable(uart_, dma_.rx[0], sizeof(dma_.rx[0]), 1000);
    if (ret == -ENOSYS || ret == -ENOTSUP) {
        // The callback registration succeeded but this instance cannot do async
        // RX (for example a UART with its DMA channels removed).
        atomic_clear(&in_use_);
        return enterIrqMode();
    }
    if (ret < 0) {
        atomic_clear(&in_use_);
        atomic_set(&rx_disabled_, 1);
    }
    return ret;
}
int AsyncUart::service(std::uint64_t now) {
    if (!initialized_)
        return -EACCES;
    if (irq_mode_) {
        // RX FIFO access is also callback-only for CDC ACM. Service merely
        // expires pending TX when a disconnected/full device makes no progress.
        const auto key = k_spin_lock(&irq_tx_lock_);
        if (atomic_get(&tx_busy_) && now >= tx_deadline_ms_) {
            uart_irq_tx_disable(uart_);
            atomic_set(&tx_error_, -ETIMEDOUT);
            atomic_clear(&tx_busy_);
        }
        k_spin_unlock(&irq_tx_lock_, key);
        return 0;
    }
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
        atomic_set(&tx_error_, -ECANCELED);
        atomic_clear(&tx_busy_);
        break;
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
    if (irq_mode_)
        return sendIrq(p, n, timeout);
    if (!atomic_cas(&tx_busy_, 0, 1))
        return -EAGAIN;
    atomic_clear(&tx_error_);
    tx_deadline_ms_ = static_cast<std::uint64_t>(k_uptime_get()) + timeout;
    std::memcpy(dma_.tx, p, n);
    const int ret = uart_tx(uart_, dma_.tx, n, timeout * 1000);
    if (ret < 0) {
        atomic_set(&tx_error_, ret);
        atomic_clear(&tx_busy_);
    }
    return ret;
}
}
