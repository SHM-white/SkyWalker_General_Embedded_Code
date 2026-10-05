#pragma once
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <communication/interboard/interboard_transport.hpp>

namespace skywalker::communication::detail {
// Communication-thread-owned completed batches; never called from an ISR.
class TransportRxQueue {
public:
    void gap() {
        head_ = count_ = offset_ = 0;
        gap_ = true;
    }
    void push(const std::uint8_t *bytes, std::size_t size, std::uint64_t received_ms) {
        if (!size)
            return;
        if (size > InterBoardTransport::kTxCapacity) {
            gap();
            return;
        }
        if (count_ == 2)
            gap();
        auto &batch = batches_[(head_ + count_) % 2];
        std::memcpy(batch.bytes, bytes, size);
        batch.size = size;
        batch.timestamp_ms = received_ms;
        ++count_;
    }
    int read(InterBoardTransport::RxChunk &out) {
        if (gap_) {
            gap_ = false;
            return -EOVERFLOW;
        }
        if (!count_)
            return -EAGAIN;
        const auto &batch = batches_[head_];
        InterBoardTransport::RxChunk next{};
        next.size = std::min(sizeof(next.bytes), batch.size - offset_);
        next.timestamp_ms = batch.timestamp_ms;
        std::memcpy(next.bytes, batch.bytes + offset_, next.size);
        offset_ += next.size;
        if (offset_ == batch.size) {
            offset_ = 0;
            head_ = (head_ + 1) % 2;
            --count_;
        }
        out = next;
        return 0;
    }

private:
    struct Batch {
        std::uint8_t bytes[InterBoardTransport::kTxCapacity]{};
        std::size_t size = 0;
        std::uint64_t timestamp_ms = 0;
    } batches_[2]{};
    std::size_t head_ = 0, count_ = 0, offset_ = 0;
    bool gap_ = false;
};
}
