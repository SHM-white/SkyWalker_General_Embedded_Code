#pragma once
#include <communication/vision/vision_types.hpp>
#include <cstddef>
namespace skywalker::communication::vision {
class CommandSink {
public:
    virtual ~CommandSink() = default;
    virtual int accept(const AimCommand &, core::TimeUs frame_rx_us) = 0;
};
class VisionProtocol {
public:
    virtual ~VisionProtocol() = default;
    virtual int validateConfig() const = 0;
    virtual void reset() = 0;
    virtual void discardPartial() = 0;
    virtual int consume(const std::uint8_t *, std::size_t, core::TimeUs, CommandSink &) = 0;
    virtual int encodeFeedback(const Feedback &, core::TimeUs, std::uint8_t *, std::size_t) = 0;
    virtual ProtocolStatistics statistics() const = 0; // Owner thread only.
};
}
