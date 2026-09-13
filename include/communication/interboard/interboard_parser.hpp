#pragma once
#include <communication/interboard/interboard_protocol.hpp>
namespace skywalker::communication {
// One owner; fixed queue, no allocation. consume(nullptr,0,now) expires partial frames.
class InterBoardParser {
public:
    struct Stats {
        std::uint32_t valid_frames = 0, crc_errors = 0, length_errors = 0, version_errors = 0, unknown_messages = 0,
                      queue_overflows = 0, assembly_timeouts = 0;
    };
    explicit InterBoardParser(std::uint32_t assembly_timeout_ms = 50) : timeout_ms_(assembly_timeout_ms) {
    }
    int reset();
    void discardPartial();
    int consume(const std::uint8_t *, std::size_t, std::uint64_t local_receive_ms);
    bool popFrame(FrameMeta &, std::uint8_t *, std::size_t capacity, std::size_t &payload_size);
    const Stats &stats() const {
        return stats_;
    }

private:
    void scan(std::uint64_t now);
    void discard(std::size_t n);
    std::array<std::uint8_t, kMaxFrame> buffer_{};
    std::array<std::uint64_t, kMaxFrame> receive_ms_{};
    std::array<InterBoardFrame, 4> queue_{};
    std::size_t used_ = 0, head_ = 0, count_ = 0;
    std::uint64_t started_ms_ = 0;
    std::uint32_t timeout_ms_;
    Stats stats_{};
};
}
