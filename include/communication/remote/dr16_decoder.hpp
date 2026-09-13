#pragma once
#include <cstddef>
#include <robotics/messages/remote.hpp>
namespace skywalker::communication {
class Dr16Decoder {
public:
    static constexpr std::size_t kFrameSize=18;
    struct Config { std::int16_t channel_center=1024, channel_min=364, channel_max=1684, center_deadband=10; };
    explicit Dr16Decoder(const Config &config): config_(config) {}
    int reset();
    int decodeFrame(const std::uint8_t *,std::size_t,std::uint64_t timestamp_ms,robotics::RemoteState &out);
    std::uint32_t validFrameCount() const { return valid_frames_; }
    std::uint32_t invalidFrameCount() const { return invalid_frames_; }
private: Config config_; std::uint32_t valid_frames_=0,invalid_frames_=0,sequence_=0;
};
}
