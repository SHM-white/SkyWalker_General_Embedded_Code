#include <algorithm>
#include <cerrno>
#include <limits>
#include <zephyr/random/random.h>
#include <communication/interboard/interboard_endpoint.hpp>
namespace skywalker::communication {
using namespace robotics;
namespace {
std::uint32_t age(const MessageStamp &stamp, std::uint64_t now) {
    return stamp.valid && now >= stamp.timestamp_ms
        ? static_cast<std::uint32_t>(std::min<std::uint64_t>(now - stamp.timestamp_ms, UINT32_MAX)) : UINT32_MAX;
}
}
void InterBoardEndpoint::submit(const ChassisCommand &command) {
    const auto key = k_spin_lock(&lock_);
    outgoing_.command = command;
    k_spin_unlock(&lock_, key);
}
void InterBoardEndpoint::setReferee(const RefereeState &referee) {
    const auto key = k_spin_lock(&lock_);
    outgoing_.referee = referee;
    k_spin_unlock(&lock_, key);
}
void InterBoardEndpoint::setStatus(const RunStatus &status) {
    const auto key = k_spin_lock(&lock_);
    outgoing_.status = status;
    k_spin_unlock(&lock_, key);
}
InterBoardEndpoint::Snapshot InterBoardEndpoint::snapshot() const {
    const auto key = k_spin_lock(&lock_);
    const auto result = published_;
    k_spin_unlock(&lock_, key);
    return result;
}
void InterBoardEndpoint::poll(std::uint64_t now) {
    if (!boot_id_) boot_id_ = sys_rand64_get() | 1ULL;
    if (now >= retry_ms_) {
        int ret = uart_.service(now);
        if (ret == -EACCES) ret = uart_.init();
        if (ret < 0 && ret != -EAGAIN) { error_ = ret; retry_ms_ = now + 100; }
        if (ret == 0) {
            error_ = 0;
            AsyncUart::RxChunk chunk{};
            for (unsigned budget = 0; budget < 8; ++budget) {
                ret = uart_.read(chunk);
                if (ret == -EOVERFLOW) { link_.discardPartial(); continue; }
                if (ret == -EAGAIN) break;
                if (ret < 0) { error_ = ret; break; }
                link_.processRxBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
            }
        }
    }
    link_.processRxBytes(nullptr, 0, now);
    Outgoing outgoing{};
    {
        const auto key = k_spin_lock(&lock_);
        outgoing = outgoing_;
        k_spin_unlock(&lock_, key);
    }
    Snapshot next{};
    next.local_boot_id = boot_id_;
    link_.latestHeartbeat(next.peer);
    next.online = link_.peerOnline(now, config_.heartbeat_timeout_ms);
    if (peer_online_ && !next.online) link_.invalidateControl();
    if (next.online && (!peer_online_ || peer_boot_ != next.peer.sender_boot_id ||
                        peer_generation_ != next.peer.resume_generation)) {
        // Bind only a producer command observed AFTER this context change.
        baseline_sequence_ = outgoing.command.stamp.sequence;
        have_baseline_ = outgoing.command.stamp.valid;
        peer_boot_ = next.peer.sender_boot_id;
        peer_generation_ = next.peer.resume_generation;
    }
    peer_online_ = next.online;
    if (next.online) {
        link_.latestChassisControl(next.control);
        link_.latestChassisConstraint(next.constraint);
        link_.latestChassisFeedback(next.feedback);
    }
    if (now >= next_tx_ms_ && !uart_.txBusy()) {
        next_tx_ms_ = now + 10;
        std::uint8_t bytes[256]{};
        std::size_t used = 0;
        const auto append = [&](int length) {
            if (length > 0) used += static_cast<std::size_t>(length);
            else if (length < 0) error_ = length;
        };
        if (now >= next_heartbeat_ms_) {
            next_heartbeat_ms_ = now + 20;
            const auto feedback = wireFeedback(outgoing.status);
            BoardHeartbeat heartbeat{};
            heartbeat.role = config_.role;
            heartbeat.sender_boot_id = boot_id_;
            heartbeat.sender_uptime_ms = static_cast<std::uint32_t>(now);
            heartbeat.resume_generation = outgoing.status.generation;
            heartbeat.ready = feedback.ready;
            heartbeat.safety_state = feedback.safety_state;
            heartbeat.active_reasons = feedback.active_reasons;
            heartbeat.sync_requested = !next.online;
            append(InterBoardCodec::encodeHeartbeat(heartbeat, ++heartbeat_sequence_, bytes + used, sizeof(bytes) - used));
            if (config_.role == BoardRole::ChassisController)
                append(InterBoardCodec::encodeChassisFeedback(feedback, ++feedback_sequence_, bytes + used, sizeof(bytes) - used));
            else {
                ChassisConstraint constraint{};
                constraint.output = outgoing.referee.robot.chassis_output;
                constraint.output_age_ms = age(constraint.output.stamp, now);
                constraint.power_valid = outgoing.referee.power.limit_stamp.valid && outgoing.referee.power.stamp.valid;
                constraint.power_limit_w = outgoing.referee.power.chassis_power_limit_w;
                constraint.buffer_energy_j = outgoing.referee.power.buffer_energy_j;
                constraint.power_age_ms = std::max(age(outgoing.referee.power.limit_stamp, now), age(outgoing.referee.power.stamp, now));
                append(InterBoardCodec::encodeChassisConstraint(constraint, ++constraint_sequence_, bytes + used, sizeof(bytes) - used));
            }
        }
        if (config_.role == BoardRole::GimbalController && next.online &&
            isFresh(outgoing.command.stamp, now, config_.command_timeout_ms) &&
            (outgoing.command.mode == ChassisMode::Disabled || !have_baseline_ ||
             sequenceAfter(outgoing.command.stamp.sequence, baseline_sequence_))) {
            RemoteChassisControl control{};
            control.command = outgoing.command;
            control.stamp = outgoing.command.stamp;
            control.receiver_boot_id = peer_boot_;
            control.resume_generation = peer_generation_;
            control.global_action = control.command.mode == ChassisMode::Disabled ? SafetyAction::Disable : SafetyAction::Active;
            append(InterBoardCodec::encodeChassisControl(control, ++control_sequence_, bytes + used, sizeof(bytes) - used));
        }
        if (used) {
            const int ret = uart_.send(bytes, used);
            if (ret < 0) error_ = ret;
        }
    }
    next.error = error_;
    const auto key = k_spin_lock(&lock_);
    published_ = next;
    k_spin_unlock(&lock_, key);
}
}
