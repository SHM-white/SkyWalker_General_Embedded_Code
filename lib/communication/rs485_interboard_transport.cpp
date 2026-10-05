#include <communication/interboard/rs485_interboard_transport.hpp>
#include <communication/interboard/interboard_protocol.hpp>
#include <communication/wire.hpp>
#include <zephyr/random/random.h>
#include <algorithm>
#include <cerrno>
#include <cstring>

namespace skywalker::communication {
int Rs485InterBoardTransport::initialize(std::uint64_t now) {
    if (!config_.uart || !device_is_ready(config_.uart))
        return -ENODEV;
    if ((config_.role != Role::Coordinator && config_.role != Role::Responder) || !config_.poll_interval_ms ||
        config_.poll_interval_ms > 1000 || !config_.turnaround_ms || config_.turnaround_ms > 20 ||
        config_.response_window_ms < 5 || config_.response_window_ms > 1000)
        return -EINVAL;
    uart_config c{};
    const int ret = uart_config_get(config_.uart, &c);
    if (ret < 0)
        return ret;
    if (c.flow_ctrl != UART_CFG_FLOW_CTRL_RS485 || c.parity != UART_CFG_PARITY_NONE ||
        c.data_bits != UART_CFG_DATA_BITS_8 || c.stop_bits != UART_CFG_STOP_BITS_1 || !c.baudrate)
        return -ENOTSUP;
    baudrate_ = c.baudrate;
    if (wireTimeMs(kTxCapacity + kOverhead) + config_.turnaround_ms + 3 >= config_.response_window_ms)
        return -EINVAL;
    token_ = sys_rand32_get();
    // Let a previous coordinator's response grant expire after a local reboot.
    next_poll_ms_ = now + 1002;
    startup_until_ms_ = next_poll_ms_;
    configured_ = true;
    return 0;
}
std::uint32_t Rs485InterBoardTransport::wireTimeMs(std::size_t n) const {
    return static_cast<std::uint32_t>((n * 10000ULL + baudrate_ - 1) / baudrate_);
}
void Rs485InterBoardTransport::discard(std::size_t n) {
    input_size_ -= n;
    std::memmove(input_, input_ + n, input_size_);
    std::memmove(input_times_, input_times_ + n, input_size_ * sizeof(input_times_[0]));
}
void Rs485InterBoardTransport::accept(std::uint64_t first, std::uint64_t last, std::uint64_t now) {
    const auto type = input_[3];
    const auto token = wire::loadLe32(input_ + 4);
    const auto size = wire::loadLe16(input_ + 8);
    const auto window = wire::loadLe16(input_ + 10);
    if (config_.role == Role::Coordinator) {
        if (type != 2 || (phase_ != Phase::AwaitReply && phase_ != Phase::Sending) || token != token_ ||
            window != config_.response_window_ms || reply_received_ ||
            (phase_ == Phase::AwaitReply && now >= phase_deadline_))
            return;
        reply_received_ = true;
        rx_.push(input_ + kHeaderSize, size, first);
        // Keep the entire grant quiet, even if a reply completes early.
    }
    else {
        if (type != 1 || phase_ == Phase::Sending || window < 5 || window > 1000)
            return;
        token_ = token;
        granted_window_ = window;
        phase_ = Phase::ReplyGranted;
        reply_after_ms_ = last + config_.turnaround_ms;
        // First-byte time is conservative: never spend queue latency twice.
        phase_deadline_ = first + window;
        rx_.push(input_ + kHeaderSize, size, first);
    }
}
void Rs485InterBoardTransport::consume(const AsyncUart::RxChunk &chunk, std::uint64_t now) {
    last_rx_ms_ = std::max(last_rx_ms_, chunk.timestamp_ms);
    for (std::size_t i = 0; i < chunk.size; ++i) {
        if (input_size_ == sizeof(input_)) {
            discard(1);
            rx_.gap();
        }
        input_[input_size_] = chunk.bytes[i];
        input_times_[input_size_++] = chunk.timestamp_ms;
        while (input_size_) {
            if (input_[0] != 0xd3 || (input_size_ > 1 && input_[1] != 0x91)) {
                discard(1);
                continue;
            }
            if (input_size_ < kHeaderSize)
                break;
            const auto size = wire::loadLe16(input_ + 8);
            if (input_[2] != 1 || (input_[3] != 1 && input_[3] != 2) || size > kTxCapacity) {
                discard(1);
                rx_.gap();
                continue;
            }
            const std::size_t total = size + kOverhead;
            if (input_size_ < total)
                break;
            if (interboardCrc16(input_, total - 2) != wire::loadLe16(input_ + total - 2)) {
                discard(1);
                rx_.gap();
                continue;
            }
            accept(input_times_[0], input_times_[total - 1], now);
            discard(total);
        }
    }
}
int Rs485InterBoardTransport::transmit(std::uint64_t now) {
    const bool coordinator = config_.role == Role::Coordinator;
    const auto duration = wireTimeMs(pending_size_ + kOverhead);
    if (pending_size_ && now + duration + 2 >= pending_deadline_) {
        pending_size_ = 0;
        return -ETIMEDOUT;
    }
    if (!coordinator && now + duration + 3 >= phase_deadline_) {
        phase_ = Phase::Idle;
        return 0; // Missed grant: never send late into the next request.
    }
    std::uint8_t frame[256]{};
    frame[0] = 0xd3;
    frame[1] = 0x91;
    frame[2] = 1;
    frame[3] = coordinator ? 1 : 2;
    const auto next_token = coordinator ? token_ + 1 : token_;
    wire::storeLe32(frame + 4, next_token);
    wire::storeLe16(frame + 8, pending_size_);
    wire::storeLe16(frame + 10, coordinator ? config_.response_window_ms : granted_window_);
    std::memcpy(frame + kHeaderSize, pending_, pending_size_);
    const auto total = pending_size_ + kOverhead;
    wire::storeLe16(frame + total - 2, interboardCrc16(frame, total - 2));
    const auto timeout = coordinator ? duration + 2 : std::min<std::uint64_t>(duration + 2, phase_deadline_ - now - 1);
    const int ret = uart_.send(frame, total, static_cast<std::uint32_t>(timeout));
    if (ret == 0) {
        token_ = next_token;
        pending_size_ = 0;
        phase_ = Phase::Sending;
        reply_received_ = false;
    }
    return ret;
}
int Rs485InterBoardTransport::service(std::uint64_t now) {
    if (!configured_) {
        if (now < retry_ms_)
            return -EAGAIN;
        retry_ms_ = now + 100;
        const int ret = initialize(now);
        if (ret < 0)
            return ret;
    }
    int ret = uart_.service(now);
    if (ret == -EACCES) {
        if (now < uart_retry_ms_)
            return -EAGAIN;
        uart_retry_ms_ = now + 100;
        ret = uart_.init();
    }
    const bool starting = config_.role == Role::Coordinator && now < startup_until_ms_;
    ready_ = ret == 0 && !starting;
    int error = ret;
    if (phase_ == Phase::Sending && !uart_.txBusy()) {
        if (uart_.txError() < 0)
            error = uart_.txError();
        if (config_.role == Role::Coordinator) {
            phase_ = Phase::AwaitReply;
            phase_deadline_ = now + config_.response_window_ms;
        }
        else {
            phase_ = Phase::Idle;
        }
    }
    if (pending_size_ && now >= pending_deadline_) {
        pending_size_ = 0;
        error = -ETIMEDOUT;
    }
    if (ret == 0) {
        AsyncUart::RxChunk chunk{};
        for (unsigned budget = 0; budget < 8; ++budget) {
            const int rr = uart_.read(chunk);
            if (rr == -EAGAIN)
                break;
            if (rr == -EOVERFLOW) {
                input_size_ = 0;
                rx_.gap();
                continue;
            }
            if (rr < 0) {
                error = rr;
                break;
            }
            consume(chunk, now);
        }
    }
    if (input_size_ && now >= input_times_[0] && now - input_times_[0] > wireTimeMs(256) + 5) {
        input_size_ = 0;
        rx_.gap();
    }
    if (phase_ == Phase::AwaitReply && now >= phase_deadline_) {
        phase_ = Phase::Idle;
        next_poll_ms_ = now + config_.poll_interval_ms;
        if (!reply_received_) {
            rx_.gap();
            error = -ETIMEDOUT;
        }
    }
    if (phase_ == Phase::ReplyGranted && now >= phase_deadline_)
        phase_ = Phase::Idle;
    if (ready_ && !uart_.txBusy() && now >= last_rx_ms_ + config_.turnaround_ms) {
        if ((config_.role == Role::Coordinator && phase_ == Phase::Idle && now >= next_poll_ms_) ||
            (config_.role == Role::Responder && phase_ == Phase::ReplyGranted && now >= reply_after_ms_)) {
            const int sent = transmit(now);
            if (sent < 0 && sent != -EAGAIN)
                error = sent;
        }
    }
    return error == 0 && starting ? -EAGAIN : error;
}
int Rs485InterBoardTransport::send(const std::uint8_t *p, std::size_t n, std::uint32_t timeout) {
    if (!p || !n || !timeout || timeout > 1000)
        return -EINVAL;
    if (n > kTxCapacity)
        return -EMSGSIZE;
    if (!ready_)
        return -EACCES;
    if (txBusy())
        return -EAGAIN;
    std::memcpy(pending_, p, n);
    pending_size_ = n;
    pending_deadline_ = static_cast<std::uint64_t>(k_uptime_get()) + timeout;
    return 0;
}
}
