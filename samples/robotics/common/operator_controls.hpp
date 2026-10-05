#pragma once
#include <communication/interboard/interboard_endpoint.hpp>
#include <algorithm>

namespace skywalker::samples::control {
class OperatorControlPublisher {
public:
    // A peer reboot or loss revokes the sender's local RC release gate. The
    // original clear event is bound to the peer that existed when it was made.
    bool publish(communication::InterBoardEndpoint &endpoint, bool run_allowed, bool emergency,
                 const robotics::MessageStamp &source, std::uint32_t clear_id,
                 const robotics::MessageStamp &clear_stamp, std::uint64_t now_ms) {
        const auto peer = endpoint.snapshot();
        const auto peer_boot = peer.peer.sender_boot_id;
        const bool revoked = (was_online_ && !peer.online) || (peer_boot_ && peer_boot && peer_boot != peer_boot_);
        if (peer_boot)
            peer_boot_ = peer_boot;
        was_online_ = peer.online;
        if (clear_id != last_clear_id_) {
            last_clear_id_ = clear_id;
            clear_peer_boot_ = peer.online ? peer_boot : 0;
        }
        robotics::OperatorControl request{};
        request.run_allowed = run_allowed && !revoked && peer.online && peer_boot &&
                              robotics::isFresh(source, now_ms, 100);
        request.emergency_stop = emergency;
        request.receiver_boot_id = peer_boot;
        request.source_sequence = source.sequence;
        request.source_age_ms = age(source, now_ms);
        if (clear_peer_boot_ && clear_peer_boot_ == peer_boot && !request.run_allowed && !emergency &&
            age(clear_stamp, now_ms) <= 100) {
            request.clear_event_id = clear_id;
            request.clear_event_age_ms = age(clear_stamp, now_ms);
        }
        request.stamp = {now_ms, ++sequence_, true};
        (void)endpoint.submitOperatorControl(request);
        return revoked;
    }

private:
    static std::uint32_t age(const robotics::MessageStamp &stamp, std::uint64_t now_ms) {
        return stamp.valid && now_ms >= stamp.timestamp_ms
                   ? static_cast<std::uint32_t>(std::min<std::uint64_t>(now_ms - stamp.timestamp_ms, UINT32_MAX))
                   : UINT32_MAX;
    }
    std::uint64_t peer_boot_ = 0, clear_peer_boot_ = 0;
    std::uint32_t sequence_ = 0, last_clear_id_ = 0;
    bool was_online_ = false;
};
struct PeerOperatorState {
    bool run_allowed = false, emergency_stop = false, clear_estop = false;
};
class OperatorControlConsumer {
public:
    PeerOperatorState update(const communication::InterBoardEndpoint::Snapshot &peer, std::uint64_t now_ms) {
        PeerOperatorState state{};
        const auto &request = peer.operator_control;
        if (!peer.operator_control_valid)
            return state;
        state.emergency_stop = request.emergency_stop;
        state.run_allowed = request.run_allowed && !state.emergency_stop;
        const bool delivery_fresh = request.stamp.valid && now_ms >= request.stamp.timestamp_ms &&
                                    now_ms - request.stamp.timestamp_ms <= 100 &&
                                    request.clear_event_age_ms <= 100 - (now_ms - request.stamp.timestamp_ms);
        if (!state.run_allowed && !state.emergency_stop && request.clear_event_id && delivery_fresh &&
            (request.sender_boot_id != cleared_boot_ || request.clear_event_id != cleared_id_)) {
            cleared_boot_ = request.sender_boot_id;
            cleared_id_ = request.clear_event_id;
            state.clear_estop = true;
        }
        return state;
    }

private:
    std::uint64_t cleared_boot_ = 0;
    std::uint32_t cleared_id_ = 0;
};
} // namespace skywalker::samples::control
