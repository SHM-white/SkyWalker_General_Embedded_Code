#pragma once
#include <communication/interboard/transport_rx_queue.hpp>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

namespace skywalker::communication {
// Classic CAN, including FDCAN controllers in classic mode. Dedicated controller:
// do not share it with motor::CanBus or any other start/stop/recovery owner.
class CanInterBoardTransport final : public InterBoardTransport {
public:
    struct Config {
        const device *can = nullptr;
        std::uint32_t tx_id = 0x600, rx_id = 0x601;
        bool extended_id = false;
        std::uint32_t reassembly_timeout_ms = 60, recovery_retry_ms = 100;
    };
    explicit CanInterBoardTransport(const Config &config) : config_(config) {}
    InterBoardTransportKind kind() const override { return InterBoardTransportKind::Can; }
    int service(std::uint64_t now_ms) override;
    int read(RxChunk &out) override { return rx_.read(out); }
    int send(const std::uint8_t *, std::size_t, std::uint32_t timeout_ms = 60) override;
    bool txBusy() const override { return tx_size_ != 0 || in_flight_; }
private:
    static constexpr std::size_t kPacketCapacity = kTxCapacity + 5, kRawDepth = 64;
    struct RxEvent { can_frame frame{}; std::uint64_t timestamp_ms = 0; };
    static void onRx(const device *, can_frame *, void *);
    static void onTx(const device *, int, void *);
    int initialize();
    int recover(std::uint64_t now_ms);
    int fail(int error, std::uint64_t now_ms);
    void consume(const RxEvent &);
    void resetRx();
    Config config_;
    detail::TransportRxQueue rx_;
    k_spinlock rx_lock_{};
    RxEvent raw_[kRawDepth]{};
    std::size_t raw_head_ = 0, raw_count_ = 0;
    bool raw_gap_ = false;
    atomic_t accept_rx_ = 0, tx_done_ = 0, tx_error_ = 0;
    can_frame tx_frame_{};
    std::uint8_t tx_packet_[kPacketCapacity]{}, rx_packet_[kPacketCapacity]{};
    std::size_t tx_size_ = 0, tx_offset_ = 0, flight_size_ = 0;
    std::size_t rx_size_ = 0, rx_expected_ = 0;
    std::uint8_t tx_token_ = 0, tx_index_ = 0, rx_token_ = 0, rx_index_ = 0;
    std::uint64_t tx_deadline_ = 0, rx_started_ms_ = 0, retry_ms_ = 0;
    int filter_ = -1;
    bool initialized_ = false, recovering_ = false, stopped_ = false, in_flight_ = false;
};
}
