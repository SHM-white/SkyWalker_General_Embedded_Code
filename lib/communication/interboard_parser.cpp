#include <cerrno>
#include <cstring>
#include <communication/wire.hpp>
#include <communication/interboard/interboard_parser.hpp>
namespace skywalker::communication {
std::uint16_t interboardCrc16(const std::uint8_t *p, std::size_t n) {
    std::uint16_t crc = 0xffff;
    for (std::size_t i = 0; i < n; ++i) {
        crc ^= std::uint16_t(p[i]) << 8;
        for (unsigned j = 0; j < 8; ++j)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}
int messageIndex(MessageId id) {
    switch (id) {
    case MessageId::Heartbeat:
        return 0;
    case MessageId::ChassisControl:
        return 1;
    case MessageId::ChassisConstraint:
        return 2;
    case MessageId::ChassisFeedback:
        return 3;
    default:
        return -1;
    }
}
int InterBoardParser::reset() {
    used_ = head_ = count_ = 0;
    started_ms_ = 0;
    stats_ = {};
    return 0;
}
void InterBoardParser::discardPartial() {
    used_ = 0;
    started_ms_ = 0;
}
void InterBoardParser::discard(std::size_t n) {
    used_ -= n;
    std::memmove(buffer_.data(), buffer_.data() + n, used_);
    std::memmove(receive_ms_.data(), receive_ms_.data() + n, used_ * sizeof(receive_ms_[0]));
    if (used_)
        started_ms_ = receive_ms_[0];
}
void InterBoardParser::scan(std::uint64_t now) {
    while (used_) {
        auto *p = buffer_.data();
        if (p[0] != 0xa5 || (used_ >= 2 && p[1] != 0x5a)) {
            discard(1);
            continue;
        }
        if (used_ < 12)
            break;
        if (p[2] != 1 || p[3] < 1 || p[3] > 2) {
            ++stats_.version_errors;
            discard(1);
            continue;
        }
        const auto size = wire::loadLe16(p + 6);
        if (size > kMaxPayload) {
            ++stats_.length_errors;
            discard(1);
            continue;
        }
        const std::size_t total = 14 + size;
        if (used_ < total)
            break;
        if (interboardCrc16(p, total - 2) != wire::loadLe16(p + total - 2)) {
            ++stats_.crc_errors;
            discard(1);
            continue;
        }
        const auto id = static_cast<MessageId>(wire::loadLe16(p + 4));
        if (messageIndex(id) < 0)
            ++stats_.unknown_messages;
        else {
            if (count_ == queue_.size()) {
                head_ = (head_ + 1) % queue_.size();
                --count_;
                ++stats_.queue_overflows;
            }
            auto &f = queue_[(head_ + count_) % queue_.size()];
            f.meta = {static_cast<BoardRole>(p[3]), id, wire::loadLe32(p + 8), receive_ms_[total - 1]};
            f.size = size;
            std::memcpy(f.payload.data(), p + 12, size);
            ++count_;
            ++stats_.valid_frames;
        }
        discard(total);
        started_ms_ = now;
    }
}
int InterBoardParser::consume(const std::uint8_t *data, std::size_t size, std::uint64_t now) {
    if (!data && size)
        return -EINVAL;
    if (used_ && (now < started_ms_ || now - started_ms_ > timeout_ms_)) {
        ++stats_.assembly_timeouts;
        discard(1);
        scan(now);
        started_ms_ = now;
    }
    for (std::size_t i = 0; i < size; ++i) {
        if (!used_)
            started_ms_ = now;
        if (used_ == buffer_.size()) {
            discard(1);
            ++stats_.length_errors;
        }
        receive_ms_[used_] = now;
        buffer_[used_++] = data[i];
        scan(now);
    }
    return 0;
}
bool InterBoardParser::popFrame(FrameMeta &meta, std::uint8_t *payload, std::size_t capacity, std::size_t &size) {
    if (!count_ || !payload || capacity < queue_[head_].size)
        return false;
    const auto &f = queue_[head_];
    meta = f.meta;
    size = f.size;
    std::memcpy(payload, f.payload.data(), size);
    head_ = (head_ + 1) % queue_.size();
    --count_;
    return true;
}
}
