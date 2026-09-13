#pragma once
#include <array>
#include <communication/referee/referee_protocol.hpp>
#include <robotics/messages/referee.hpp>
namespace skywalker::communication {
class RefereeParser {
public:
    struct Stats { std::uint32_t valid_frames=0,crc8_errors=0,crc16_errors=0,length_errors=0,unknown_commands=0,assembly_timeouts=0; };
    explicit RefereeParser(RefereeVersion version=RefereeVersion::Unspecified): version_(version) {}
    int reset();
    void discardPartial() { used_=0; }
    int consume(const std::uint8_t *,std::size_t,std::uint64_t timestamp_ms);
    const robotics::RefereeState &state() const { return state_; }
    const Stats &stats() const { return stats_; }
private:
    void scan(std::uint64_t now);
    void discard(std::size_t n);
    void decode(std::uint16_t id,const std::uint8_t *,std::size_t,std::uint64_t now);
    RefereeVersion version_; std::array<std::uint8_t,256> buffer_{}; std::array<std::uint64_t,256> receive_ms_{}; std::size_t used_=0;
    std::uint64_t started_ms_=0; std::uint32_t sequence_=0; Stats stats_{}; robotics::RefereeState state_{};
};
}
