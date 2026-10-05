#include <algorithm>
#include <cerrno>
#include <cstring>
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
std::uint32_t addAge(std::uint32_t a, std::uint32_t b) {
    return a > UINT32_MAX - b ? UINT32_MAX : a + b;
}
}
void InterBoardEndpoint::submit(const ChassisCommand &command) {
    const auto key = k_spin_lock(&lock_); outgoing_.command = command; k_spin_unlock(&lock_, key);
}
int InterBoardEndpoint::submitOperatorControl(const OperatorControl &control) {
    if (config_.role != BoardRole::GimbalController) return -EACCES;
    if ((control.run_allowed && control.emergency_stop) ||
        (control.clear_event_id && (control.run_allowed || control.emergency_stop)) ||
        ((control.run_allowed || control.clear_event_id) && (!control.stamp.valid || !control.receiver_boot_id)))
        return -EINVAL;
    const auto key = k_spin_lock(&lock_);
    outgoing_.operator_control = control;
    if (!control.clear_event_id) outgoing_.operator_control.clear_event_age_ms = UINT32_MAX;
    k_spin_unlock(&lock_, key);
    return 0;
}
void InterBoardEndpoint::setReferee(const RefereeState &referee) {
    const auto key = k_spin_lock(&lock_); outgoing_.referee = referee; k_spin_unlock(&lock_, key);
}
void InterBoardEndpoint::setStatus(const RunStatus &status) {
    const auto key = k_spin_lock(&lock_); outgoing_.status = status; k_spin_unlock(&lock_, key);
}
void InterBoardEndpoint::submitBigYaw(const BigYawRequest &request) {
    const auto key = k_spin_lock(&lock_); outgoing_.big_yaw_request = request; k_spin_unlock(&lock_, key);
}
void InterBoardEndpoint::setBigYawFeedback(const BigYawFeedback &feedback) {
    const auto key = k_spin_lock(&lock_); outgoing_.big_yaw_feedback = feedback; k_spin_unlock(&lock_, key);
}
InterBoardEndpoint::Snapshot InterBoardEndpoint::snapshot() const {
    const auto key = k_spin_lock(&lock_); const auto value = published_; k_spin_unlock(&lock_, key); return value;
}
void InterBoardEndpoint::poll(std::uint64_t now) {
    if ((config_.role != BoardRole::GimbalController && config_.role != BoardRole::ChassisController) ||
        !config_.command_timeout_ms || !config_.heartbeat_timeout_ms || !config_.tx_timeout_ms ||
        config_.tx_timeout_ms > 1000 || !config_.status_timeout_ms) {
        const auto key = k_spin_lock(&lock_); published_ = {}; published_.transport = transport_.kind();
        published_.error = -EINVAL; k_spin_unlock(&lock_, key); return;
    }
    if (!boot_id_) boot_id_ = sys_rand64_get() | 1ULL;
    // Advance backend recovery even when no application message can be sent.
    const int serviced = transport_.service(now);
    bool transport_ready = serviced == 0;
    if (serviced != -EAGAIN) error_ = serviced;
    InterBoardTransport::RxChunk chunk{};
    for (unsigned budget = 0; budget < 8; ++budget) {
        const int ret = transport_.read(chunk);
        if (ret == -EOVERFLOW) { link_.discardPartial(); continue; }
        if (ret == -EAGAIN) break;
        if (ret < 0) { error_ = ret; transport_ready = false; break; }
        (void)link_.processRxBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
    }
    (void)link_.processRxBytes(nullptr, 0, now);
    Outgoing outgoing{};
    { const auto key = k_spin_lock(&lock_); outgoing = outgoing_; k_spin_unlock(&lock_, key); }
    Snapshot next{};
    next.transport = transport_.kind(); next.local_boot_id = boot_id_;
    (void)link_.latestHeartbeat(next.peer);
    next.online = link_.peerOnline(now, config_.heartbeat_timeout_ms);
    if (peer_online_ && !next.online) link_.invalidateControl();
    if (next.online && (!peer_online_ || peer_boot_ != next.peer.sender_boot_id)) {
        baseline_sequence_ = outgoing.command.stamp.sequence;
        have_baseline_ = outgoing.command.stamp.valid;
        big_yaw_baseline_sequence_ = outgoing.big_yaw_request.stamp.sequence;
        have_big_yaw_baseline_ = outgoing.big_yaw_request.stamp.valid;
        peer_boot_ = next.peer.sender_boot_id;
    }
    peer_online_ = next.online;
    if (next.online) {
        (void)link_.latestChassisControl(next.control);
        (void)link_.latestChassisConstraint(next.constraint);
        (void)link_.latestChassisFeedback(next.feedback);
        (void)link_.latestOperatorControl(next.operator_control);
        (void)link_.latestBigYawRequest(next.big_yaw_request);
        (void)link_.latestBigYawFeedback(next.big_yaw_feedback);
    }
    const auto &op = next.operator_control;
    next.operator_control_valid = next.online && op.sender_boot_id == next.peer.sender_boot_id &&
        op.receiver_boot_id == boot_id_ && forwardedFresh(op.stamp, op.source_age_ms, now, config_.command_timeout_ms);
    if (!forwardedFresh(next.big_yaw_request.stamp, next.big_yaw_request.command_age_ms, now, config_.command_timeout_ms) ||
        !forwardedFresh(next.big_yaw_request.stamp, next.big_yaw_request.source_age_ms, now, config_.command_timeout_ms))
        next.big_yaw_request = {};
    if (!next.big_yaw_feedback.valid || !forwardedFresh(next.big_yaw_feedback.stamp,
        next.big_yaw_feedback.production_age_ms, now, config_.status_timeout_ms)) {
        next.big_yaw_feedback.valid = false; next.big_yaw_feedback.ready = false; next.big_yaw_feedback.armed = false;
    }
    if (config_.role == BoardRole::ChassisController &&
        (!next.operator_control_valid || !op.run_allowed || op.emergency_stop)) {
        next.control.command.mode = ChassisMode::Disabled;
        next.control.global_action = SafetyAction::Disable;
        next.big_yaw_request.mode = BigYawMode::Disabled;
    }
    if (transport_ready && now >= next_tx_ms_ && !transport_.txBusy()) {
        std::uint8_t batch[InterBoardTransport::kTxCapacity]{};
        std::size_t used = 0;
        std::array<bool, kMessageCount> included{};
        // Encode each due message into its own bounded frame. A full batch
        // leaves unsent messages due; the next poll re-encodes fresh ages.
        const auto append = [&](MessageId id, auto encode) {
            const auto index = static_cast<std::size_t>(messageIndex(id));
            if (now < next_message_ms_[index]) return;
            std::uint8_t frame[kMaxFrame]{};
            const int n = encode(++wire_sequence_[index], frame, sizeof(frame));
            if (n < 0) { error_ = n; return; }
            if (!n) return;
            if (static_cast<std::size_t>(n) > sizeof(batch)) { error_ = -EMSGSIZE; return; }
            if (static_cast<std::size_t>(n) > sizeof(batch) - used) return;
            std::memcpy(batch + used, frame, n); used += static_cast<std::size_t>(n); included[index] = true;
        };
        append(MessageId::Heartbeat, [&](auto seq, auto *bytes, auto cap) {
            const auto feedback = wireFeedback(outgoing.status, now, config_.status_timeout_ms);
            BoardHeartbeat heartbeat{};
            heartbeat.role = config_.role; heartbeat.sender_boot_id = boot_id_;
            heartbeat.sender_uptime_ms = static_cast<std::uint32_t>(now);
            heartbeat.ready = feedback.ready;
            heartbeat.safety_state = feedback.safety_state; heartbeat.active_reasons = feedback.active_reasons;
            heartbeat.sync_requested = !next.online;
            return InterBoardCodec::encodeHeartbeat(heartbeat, seq, bytes, cap);
        });
        if (config_.role == BoardRole::GimbalController) {
            if (next.online) append(MessageId::OperatorControl, [&](auto seq, auto *bytes, auto cap) {
                auto control = outgoing.operator_control;
                const auto elapsed = age(control.stamp, now);
                control.source_age_ms = addAge(control.source_age_ms, elapsed);
                control.clear_event_age_ms = addAge(control.clear_event_age_ms, elapsed);
                control.sender_boot_id = boot_id_;
                // A stale or differently bound value can only send a stop.
                // Never rebind a clear request to a newly booted receiver.
                if (control.receiver_boot_id != peer_boot_ || control.source_age_ms > config_.command_timeout_ms) {
                    control.run_allowed = false; control.clear_event_id = 0;
                    control.source_age_ms = UINT32_MAX; control.receiver_boot_id = peer_boot_;
                }
                if (control.clear_event_age_ms > config_.command_timeout_ms) control.clear_event_id = 0;
                if (!control.clear_event_id) control.clear_event_age_ms = UINT32_MAX;
                return InterBoardCodec::encodeOperatorControl(control, seq, bytes, cap);
            });
            if (next.online && isFresh(outgoing.command.stamp, now, config_.command_timeout_ms) &&
                (outgoing.command.mode == ChassisMode::Disabled || !have_baseline_ ||
                 sequenceAfter(outgoing.command.stamp.sequence, baseline_sequence_)))
                append(MessageId::ChassisControl, [&](auto seq, auto *bytes, auto cap) {
                    RemoteChassisControl control{};
                    control.command = outgoing.command; control.stamp = outgoing.command.stamp;
                    control.receiver_boot_id = peer_boot_;
                    control.global_action = control.command.mode == ChassisMode::Disabled ? SafetyAction::Disable : SafetyAction::Active;
                    return InterBoardCodec::encodeChassisControl(control, seq, bytes, cap);
                });
            if (next.online && isFresh(outgoing.big_yaw_request.stamp, now, config_.command_timeout_ms) &&
                (outgoing.big_yaw_request.mode == BigYawMode::Disabled || !have_big_yaw_baseline_ ||
                 sequenceAfter(outgoing.big_yaw_request.stamp.sequence, big_yaw_baseline_sequence_)))
                append(MessageId::BigYawRequest, [&](auto seq, auto *bytes, auto cap) {
                    auto request = outgoing.big_yaw_request;
                    const auto elapsed = age(request.stamp, now);
                    request.source_age_ms = addAge(request.source_age_ms, elapsed);
                    request.command_age_ms = addAge(request.command_age_ms, elapsed);
                    request.permission_age_ms = age(request.permission.stamp, now);
                    request.receiver_boot_id = peer_boot_;
                    return InterBoardCodec::encodeBigYawRequest(request, seq, bytes, cap);
                });
            append(MessageId::ChassisConstraint, [&](auto seq, auto *bytes, auto cap) {
                ChassisConstraint constraint{};
                constraint.output = outgoing.referee.robot.chassis_output;
                constraint.output_age_ms = age(constraint.output.stamp, now);
                constraint.power_valid = outgoing.referee.power.limit_stamp.valid && outgoing.referee.power.stamp.valid;
                constraint.power_limit_w = outgoing.referee.power.chassis_power_limit_w;
                constraint.buffer_energy_j = outgoing.referee.power.buffer_energy_j;
                constraint.power_age_ms = std::max(age(outgoing.referee.power.limit_stamp, now), age(outgoing.referee.power.stamp, now));
                return InterBoardCodec::encodeChassisConstraint(constraint, seq, bytes, cap);
            });
        } else {
            append(MessageId::ChassisFeedback, [&](auto seq, auto *bytes, auto cap) {
                return InterBoardCodec::encodeChassisFeedback(wireFeedback(outgoing.status, now, config_.status_timeout_ms), seq, bytes, cap);
            });
            append(MessageId::BigYawFeedback, [&](auto seq, auto *bytes, auto cap) {
                auto feedback = outgoing.big_yaw_feedback;
                feedback.production_age_ms = age(feedback.stamp, now); feedback.source_sequence = feedback.stamp.sequence;
                if (!isFresh(feedback.stamp, now, config_.status_timeout_ms)) {
                    feedback.valid = false; feedback.ready = false; feedback.armed = false;
                    feedback.execution_state = ExecutionState::Waiting; feedback.active_reasons |= FeedbackStale;
                }
                return InterBoardCodec::encodeBigYawFeedback(feedback, seq, bytes, cap);
            });
        }
        if (used) {
            const int ret = transport_.send(batch, used, config_.tx_timeout_ms);
            if (ret == 0) {
                next_tx_ms_ = now + 1;
                for (std::size_t i = 0; i < included.size(); ++i) if (included[i])
                    next_message_ms_[i] = now + ((i == 1 || i == 4 || i == 5) ? 10 : 20);
            } else if (ret != -EAGAIN) error_ = ret;
        }
    }
    next.error = error_; next.parser_stats = link_.stats(); next.rejected_frames = link_.rejectedFrames();
    const auto key = k_spin_lock(&lock_); published_ = next; k_spin_unlock(&lock_, key);
}
}
