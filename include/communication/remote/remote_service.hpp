#pragma once
#include <array>
#include <communication/remote/dr16_decoder.hpp>
namespace skywalker::communication {
// A single input task owns framing and snapshots. UART callbacks only enqueue bytes.
class RemoteService {
public:
    struct Config { std::uint32_t offline_timeout_ms=100, assembly_gap_ms=10; };
    RemoteService(const Dr16Decoder::Config &d,const Config &c): decoder_(d),config_(c) {}
    int processBytes(const std::uint8_t *,std::size_t,std::uint64_t now_ms);
    int snapshot(std::uint64_t now_ms,robotics::RemoteState &out) const;
    bool online(std::uint64_t now_ms) const;
    void discardPartial() { used_=0; }
private:
    Dr16Decoder decoder_; Config config_; robotics::RemoteState latest_{};
    std::array<std::uint8_t,18> buffer_{}; std::size_t used_=0; std::uint64_t last_bytes_ms_=0;
};
}
