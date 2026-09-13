#pragma once
#include <cerrno>
#include <communication/referee/referee_parser.hpp>
namespace skywalker::communication {
class RefereeService {
public:
    explicit RefereeService(RefereeVersion version,std::uint32_t timeout_ms=500): parser_(version),timeout_ms_(timeout_ms) {}
    int processBytes(const std::uint8_t *p,std::size_t n,std::uint64_t now) { return parser_.consume(p,n,now); }
    void discardPartial() { parser_.discardPartial(); }
    int snapshot(std::uint64_t now,robotics::RefereeState &out) const {
        if (!parser_.state().stamp.valid) return -EAGAIN;
        out=parser_.state(); out.online=robotics::isFresh(out.stamp,now,timeout_ms_); return out.online ? 0 : -ESTALE;
    }
private: RefereeParser parser_; std::uint32_t timeout_ms_;
};
}
