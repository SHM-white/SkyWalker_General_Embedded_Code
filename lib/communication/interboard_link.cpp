#include <cerrno>
#include <communication/interboard/interboard_link.hpp>
namespace skywalker::communication {
using namespace robotics;
void InterBoardLink::accept(const FrameMeta &m, const std::uint8_t *p, std::size_t n) {
    if (m.sender_role == role_) {
        ++rejected_;
        return;
    }
    const int idx = messageIndex(m.message_id);
    if (idx < 0)
        return;
    BoardHeartbeat hb{};
    if (m.message_id == MessageId::Heartbeat) {
        if (InterBoardCodec::decodeHeartbeat(m, p, n, hb) < 0) {
            ++rejected_;
            return;
        }
        if (!heartbeat_.stamp.valid || hb.sender_boot_id != heartbeat_.sender_boot_id) {
            sequences_ = {};
            control_ = {};
            constraint_ = {};
            feedback_ = {};
            capabilities_ = {};
            big_yaw_request_ = {};
            big_yaw_feedback_ = {};
        }
    }
    else if (!heartbeat_.stamp.valid) {
        ++rejected_;
        return;
    }
    if (sequences_[idx].valid && !sequenceAfter(m.frame_sequence, sequences_[idx].sequence)) {
        ++rejected_;
        return;
    }
    int ret = 0;
    switch (m.message_id) {
    case MessageId::Heartbeat:
        heartbeat_ = hb;
        break;
    case MessageId::ChassisControl: {
        RemoteChassisControl c{};
        ret = InterBoardCodec::decodeChassisControl(m, p, n, c);
        if (ret == 0 &&
            (!control_.stamp.valid || sequenceAfter(c.command.stamp.sequence, control_.command.stamp.sequence)))
            control_ = c;
        break;
    }
    case MessageId::ChassisConstraint:
        ret = InterBoardCodec::decodeChassisConstraint(m, p, n, constraint_);
        break;
    case MessageId::ChassisFeedback:
        ret = InterBoardCodec::decodeChassisFeedback(m, p, n, feedback_);
        break;
    case MessageId::CapabilitiesV2: {
        InterBoardCapabilities c{};
        ret = InterBoardCodec::decodeCapabilities(m, p, n, c);
        if (ret == 0 && c.sender_boot_id == heartbeat_.sender_boot_id) capabilities_ = c;
        else if (ret == 0) ret = -ESTALE;
        break;
    }
    case MessageId::BigYawRequestV2: {
        BigYawRequest r{};
        ret = InterBoardCodec::decodeBigYawRequest(m, p, n, r);
        if (ret == 0 && (!big_yaw_request_.stamp.valid ||
            sequenceAfter(r.stamp.sequence, big_yaw_request_.stamp.sequence))) big_yaw_request_ = r;
        break;
    }
    case MessageId::BigYawFeedbackV2:
        ret = InterBoardCodec::decodeBigYawFeedback(m, p, n, big_yaw_feedback_);
        break;
    default:
        return;
    }
    if (ret < 0) {
        ++rejected_;
        return;
    }
    sequences_[idx] = {m.local_receive_ms, m.frame_sequence, true};
}
int InterBoardLink::processRxBytes(const std::uint8_t *p, std::size_t n, std::uint64_t now) {
    if ((!p && n) || (role_ != BoardRole::GimbalController && role_ != BoardRole::ChassisController))
        return -EINVAL;
    auto drain = [&]() {
        FrameMeta m{};
        std::uint8_t payload[kMaxPayload];
        std::size_t size = 0;
        while (parser_.popFrame(m, payload, sizeof(payload), size))
            accept(m, payload, size);
    };
    parser_.consume(nullptr, 0, now);
    drain();
    for (std::size_t i = 0; i < n; ++i) {
        parser_.consume(p + i, 1, now);
        drain();
    }
    return 0;
}
bool InterBoardLink::peerOnline(std::uint64_t now, std::uint32_t timeout) const {
    return isFresh(heartbeat_.stamp, now, timeout);
}
int InterBoardLink::latestHeartbeat(BoardHeartbeat &out) const {
    if (!heartbeat_.stamp.valid)
        return -EAGAIN;
    out = heartbeat_;
    return 0;
}
int InterBoardLink::latestChassisControl(RemoteChassisControl &out) const {
    if (!control_.stamp.valid)
        return -EAGAIN;
    out = control_;
    return 0;
}
int InterBoardLink::latestChassisConstraint(ChassisConstraint &out) const {
    if (!constraint_.stamp.valid)
        return -EAGAIN;
    out = constraint_;
    return 0;
}
int InterBoardLink::latestChassisFeedback(ChassisFeedbackSummary &out) const {
    if (!feedback_.stamp.valid)
        return -EAGAIN;
    out = feedback_;
    return 0;
}

int InterBoardLink::latestCapabilities(InterBoardCapabilities &out) const {
    if (!capabilities_.stamp.valid) return -EAGAIN;
    out = capabilities_; return 0;
}
int InterBoardLink::latestBigYawRequest(BigYawRequest &out) const {
    if (!big_yaw_request_.stamp.valid) return -EAGAIN;
    out = big_yaw_request_; return 0;
}
int InterBoardLink::latestBigYawFeedback(BigYawFeedback &out) const {
    if (!big_yaw_feedback_.stamp.valid) return -EAGAIN;
    out = big_yaw_feedback_; return 0;
}
}
