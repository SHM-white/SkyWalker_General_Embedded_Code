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
void InterBoardEndpoint::submitBigYaw(const BigYawRequest &request) {
    const auto key = k_spin_lock(&lock_); outgoing_.big_yaw_request = request; k_spin_unlock(&lock_, key);
}
void InterBoardEndpoint::setBigYawFeedback(const BigYawFeedback &feedback) {
    const auto key = k_spin_lock(&lock_); outgoing_.big_yaw_feedback = feedback; k_spin_unlock(&lock_, key);
}
InterBoardEndpoint::Snapshot InterBoardEndpoint::snapshot() const {
    const auto key = k_spin_lock(&lock_);
    const auto result = published_;
    k_spin_unlock(&lock_, key);
    return result;
}
void InterBoardEndpoint::poll(std::uint64_t now) {
    if ((config_.role != BoardRole::GimbalController && config_.role != BoardRole::ChassisController) ||
        !config_.command_timeout_ms || !config_.heartbeat_timeout_ms ||
        !config_.tx_timeout_ms || config_.tx_timeout_ms > 1000 || !config_.status_timeout_ms) {
        const auto key = k_spin_lock(&lock_);
        published_ = {};
        published_.transport = transport_.kind();
        published_.error = -EINVAL;
        k_spin_unlock(&lock_, key);
        return;
    }
    if (!boot_id_) boot_id_ = sys_rand64_get() | 1ULL;
    // Backends own recovery deadlines; never starve in-flight I/O after an error.
    const int serviced = transport_.service(now);
    bool transport_ready = serviced == 0;
    if (serviced != -EAGAIN) error_ = serviced;
    InterBoardTransport::RxChunk chunk{};
    for (unsigned budget = 0; budget < 8; ++budget) {
        const int ret = transport_.read(chunk);
        if (ret == -EOVERFLOW) { link_.discardPartial(); continue; }
        if (ret == -EAGAIN) break;
        if (ret < 0) { error_ = ret; transport_ready = false; break; }
        link_.processRxBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
    }
    link_.processRxBytes(nullptr, 0, now);
    Outgoing outgoing{};
    {
        const auto key = k_spin_lock(&lock_);
        outgoing = outgoing_;
        k_spin_unlock(&lock_, key);
    }
    Snapshot next{};
    next.transport = transport_.kind();
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
    link_.latestCapabilities(next.capabilities);
    next.big_yaw_compatible = config_.enable_big_yaw && next.online &&
        isFresh(next.capabilities.stamp, now, config_.heartbeat_timeout_ms) &&
        next.capabilities.sender_boot_id == next.peer.sender_boot_id &&
        config_.big_yaw_contract_version == kBigYawContractVersion &&
        next.capabilities.contract_version == config_.big_yaw_contract_version &&
        (next.capabilities.features & kBigYawCapability);
    if (next.big_yaw_compatible) {
        link_.latestBigYawRequest(next.big_yaw_request);
        link_.latestBigYawFeedback(next.big_yaw_feedback);
        if (!forwardedFresh(next.big_yaw_request.stamp, next.big_yaw_request.command_age_ms, now, config_.command_timeout_ms) ||
            !forwardedFresh(next.big_yaw_request.stamp, next.big_yaw_request.source_age_ms, now, config_.command_timeout_ms))
            next.big_yaw_request = {};
        if (!next.big_yaw_feedback.valid || !forwardedFresh(next.big_yaw_feedback.stamp,
            next.big_yaw_feedback.production_age_ms, now, config_.status_timeout_ms)) {
            next.big_yaw_feedback.valid = false; next.big_yaw_feedback.ready = false; next.big_yaw_feedback.armed = false;
        }
    }
    const bool axis_context = next.big_yaw_compatible && next.big_yaw_feedback.valid && next.big_yaw_feedback.resume_generation;
    if (axis_context && (!big_yaw_context_valid_ || big_yaw_peer_boot_ != next.peer.sender_boot_id ||
                         big_yaw_peer_generation_ != next.big_yaw_feedback.resume_generation)) {
        big_yaw_baseline_sequence_ = outgoing.big_yaw_request.source_sequence;
        have_big_yaw_baseline_ = outgoing.big_yaw_request.stamp.valid;
        big_yaw_peer_boot_ = next.peer.sender_boot_id;
        big_yaw_peer_generation_ = next.big_yaw_feedback.resume_generation;
    }
    big_yaw_context_valid_ = axis_context;
    if (transport_ready && now >= next_tx_ms_ && !transport_.txBusy()) {
        std::uint8_t bytes[InterBoardTransport::kTxCapacity]{};
        std::size_t used = 0;
        int encode_error = 0;
        const bool heartbeat_due = now >= next_heartbeat_ms_;
        const auto append = [&](int length) {
            if (length > 0) used += static_cast<std::size_t>(length);
            else if (length < 0) encode_error = length;
        };
        if (heartbeat_due) {
            const auto feedback = wireFeedback(outgoing.status, now, config_.status_timeout_ms);
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
        if (heartbeat_due && config_.enable_big_yaw) {
            InterBoardCapabilities capability{}; capability.sender_boot_id = boot_id_;
            capability.contract_version = config_.big_yaw_contract_version;
            append(InterBoardCodec::encodeCapabilities(capability, config_.role, ++capabilities_sequence_, bytes + used, sizeof(bytes) - used));
            if (config_.role == BoardRole::ChassisController) {
                auto feedback = outgoing.big_yaw_feedback;
                feedback.production_age_ms = age(feedback.stamp, now);
                feedback.source_sequence = feedback.stamp.sequence;
                if (!isFresh(feedback.stamp, now, config_.status_timeout_ms)) {
                    feedback.valid = false; feedback.ready = false; feedback.armed = false;
                    feedback.execution_state = ExecutionState::Waiting; feedback.active_reasons |= FeedbackStale;
                }
                append(InterBoardCodec::encodeBigYawFeedback(feedback, ++big_yaw_feedback_sequence_, bytes + used, sizeof(bytes) - used));
            }
        }
        if (config_.role == BoardRole::GimbalController && axis_context &&
            isFresh(outgoing.big_yaw_request.stamp, now, config_.command_timeout_ms) &&
            (outgoing.big_yaw_request.mode == BigYawMode::Disabled || !have_big_yaw_baseline_ ||
             sequenceAfter(outgoing.big_yaw_request.source_sequence, big_yaw_baseline_sequence_))) {
            auto request = outgoing.big_yaw_request;
            const auto producer_age = age(request.stamp, now);
            const auto add_age = [](std::uint32_t a, std::uint32_t b) {
                return a > UINT32_MAX - b ? UINT32_MAX : a + b;
            };
            request.source_age_ms = add_age(request.source_age_ms, producer_age);
            request.command_age_ms = add_age(request.command_age_ms, producer_age);
            request.permission_age_ms = age(request.permission.stamp, now);
            request.receiver_boot_id = big_yaw_peer_boot_; request.resume_generation = big_yaw_peer_generation_;
            append(InterBoardCodec::encodeBigYawRequest(request, ++big_yaw_request_sequence_, bytes + used, sizeof(bytes) - used));
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
        if (encode_error < 0) error_ = encode_error;
        else if (used) {
            const int ret = transport_.send(bytes, used, config_.tx_timeout_ms);
            if (ret == 0) {
                next_tx_ms_ = now + 10;
                if (heartbeat_due) next_heartbeat_ms_ = now + 20;
            } else if (ret != -EAGAIN) error_ = ret;
        }
    }
    next.error = error_;
    next.parser_stats = link_.stats();
    next.rejected_frames = link_.rejectedFrames();
    const auto key = k_spin_lock(&lock_);
    published_ = next;
    k_spin_unlock(&lock_, key);
}
}
