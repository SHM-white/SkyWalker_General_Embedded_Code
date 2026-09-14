#pragma once
#include <communication/interboard/interboard_codec.hpp>
#include <communication/interboard/interboard_parser.hpp>
namespace skywalker::communication {
// Protocol/session owner. UART bytes and encoded TX frames cross the transport boundary by value.
// All methods must be called by one communication thread; apps publish copies under a mutex.
class InterBoardLink {
public:
    explicit InterBoardLink(BoardRole role) : role_(role) {
    }
    int processRxBytes(const std::uint8_t *, std::size_t, std::uint64_t now_ms);
    void discardPartial() {
        parser_.discardPartial();
    }
    bool peerOnline(std::uint64_t now_ms, std::uint32_t timeout_ms = 200) const;
    int latestHeartbeat(robotics::BoardHeartbeat &) const;
    int latestChassisControl(robotics::RemoteChassisControl &) const;
    int latestChassisConstraint(robotics::ChassisConstraint &) const;
    int latestChassisFeedback(robotics::ChassisFeedbackSummary &) const;
    const InterBoardParser::Stats &stats() const {
        return parser_.stats();
    }
    std::uint32_t rejectedFrames() const {
        return rejected_;
    }

private:
    void accept(const FrameMeta &, const std::uint8_t *, std::size_t);
    BoardRole role_;
    InterBoardParser parser_{};
    std::array<robotics::MessageStamp, 4> sequences_{};
    robotics::BoardHeartbeat heartbeat_{};
    robotics::RemoteChassisControl control_{};
    robotics::ChassisConstraint constraint_{};
    robotics::ChassisFeedbackSummary feedback_{};
    std::uint32_t rejected_ = 0;
};
}
