#pragma once
#include <communication/vision/vision_protocol.hpp>
namespace skywalker::communication::vision {
// The user-selected RM2026-AutoAim AB wire protocol: RX 29, TX 43 bytes.
class AbProtocol final : public VisionProtocol {
public:
    struct Config {
        // AB carries no reference/session metadata; this is an explicit local agreement.
        core::OrientationReference command_reference{1, 1};
        core::TimeUs assembly_timeout_us = 20000;
    };
    static constexpr std::size_t CommandSize = 29, FeedbackSize = 43;
    explicit AbProtocol(const Config &c) : config_(c) {
    }
    int validateConfig() const override;
    void reset() override;
    void discardPartial() override {
        used_ = 0;
    }
    int consume(const std::uint8_t *, std::size_t, core::TimeUs, CommandSink &) override;
    int encodeFeedback(const Feedback &, core::TimeUs, std::uint8_t *, std::size_t) override;
    ProtocolStatistics statistics() const override {
        return stats_;
    }
    static std::uint16_t crc16(const std::uint8_t *, std::size_t);

private:
    void drop();
    Config config_;
    std::uint8_t bytes_[CommandSize]{};
    core::TimeUs times_[CommandSize]{};
    std::size_t used_ = 0;
    ProtocolStatistics stats_{};
};
}
