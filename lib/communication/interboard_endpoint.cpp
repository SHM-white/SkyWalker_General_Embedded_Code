#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <core/clock.hpp>
#include <zephyr/random/random.h>
#include <communication/interboard/interboard_endpoint.hpp>
namespace skywalker::communication {
using namespace robotics;
namespace {
std::uint32_t age(const MessageStamp &stamp, std::uint64_t now) {
    return stamp.valid && now >= stamp.timestamp_ms
               ? static_cast<std::uint32_t>(std::min<std::uint64_t>(now - stamp.timestamp_ms, UINT32_MAX))
               : UINT32_MAX;
}
std::uint32_t addAge(std::uint32_t a, std::uint32_t b) {
    return a > UINT32_MAX - b ? UINT32_MAX : a + b;
}
}
void InterBoardEndpoint::submit(const ChassisCommand &command) {
    const auto key = k_spin_lock(&lock_);
    outgoing_.command = command;
    k_spin_unlock(&lock_, key);
}
int InterBoardEndpoint::submitOperatorControl(const OperatorControl &control) {
    if (config_.role != BoardRole::GimbalController)
        return -EACCES;
    if ((control.run_allowed && control.emergency_stop) ||
        (control.clear_event_id && (control.run_allowed || control.emergency_stop)) ||
        ((control.run_allowed || control.clear_event_id) && (!control.stamp.valid || !control.receiver_boot_id)))
        return -EINVAL;
    const auto key = k_spin_lock(&lock_);
    outgoing_.operator_control = control;
    if (!control.clear_event_id)
        outgoing_.operator_control.clear_event_age_ms = UINT32_MAX;
    k_spin_unlock(&lock_, key);
    return 0;
}
void InterBoardEndpoint::setReferee(const RefereeState &referee) {
    const auto key = k_spin_lock(&lock_);
    outgoing_.referee = referee;
    k_spin_unlock(&lock_, key);
}
int InterBoardEndpoint::submitRefereeFrame(const std::uint8_t *bytes, std::size_t size, std::uint64_t receive_ms) {
    if (k_is_in_isr())
        return -EWOULDBLOCK;
    if (config_.role != BoardRole::ChassisController)
        return -EACCES;
    if (!bytes || size < 9 || size > kRefereeFrameCapacity || bytes[0] != 0xA5)
        return -EINVAL;
    const auto key = k_spin_lock(&lock_);
    int result = 0;
    if (!published_.online || !published_.peer.sender_boot_id)
        result = -ENOTCONN;
    else if (referee_tx_count_ == referee_tx_.size())
        result = -ENOSPC;
    else {
        auto &frame = referee_tx_[(referee_tx_head_ + referee_tx_count_) % referee_tx_.size()];
        frame = {};
        std::memcpy(frame.bytes.data(), bytes, size);
        frame.size = static_cast<std::uint16_t>(size);
        frame.receive_ms = receive_ms;
        frame.receiver_boot_id = published_.peer.sender_boot_id;
        frame.transfer_id = ++referee_transfer_id_;
        ++referee_tx_count_;
    }
    k_spin_unlock(&lock_, key);
    return result;
}
int InterBoardEndpoint::readRefereeFrame(RefereeRelayFrame &out) {
    if (k_is_in_isr())
        return -EWOULDBLOCK;
    if (config_.role != BoardRole::GimbalController)
        return -EACCES;
    const auto now = core::monotonicTimeUs() / 1000;
    const auto key = k_spin_lock(&lock_);
    int result = -EAGAIN;
    while (referee_rx_count_) {
        const auto &frame = referee_rx_[referee_rx_head_];
        referee_rx_head_ = (referee_rx_head_ + 1) % referee_rx_.size();
        --referee_rx_count_;
        if (!published_.online || frame.sender_boot_id != published_.peer.sender_boot_id ||
            frame.receiver_boot_id != published_.local_boot_id || now < frame.receive_ms || frame.source_age_ms > 300 ||
            now - frame.receive_ms > 300 - frame.source_age_ms)
            continue;
        out = frame;
        result = 0;
        break;
    }
    k_spin_unlock(&lock_, key);
    return result;
}
void InterBoardEndpoint::receiveRefereeFragment(const RefereeFragment &fragment) {
    if (config_.role != BoardRole::GimbalController || fragment.receiver_boot_id != boot_id_)
        return;
    if (!fragment.offset) {
        referee_assembly_ = {};
        referee_assembly_.size = fragment.total_size;
        referee_assembly_.transfer_id = fragment.transfer_id;
        referee_assembly_.sender_boot_id = fragment.sender_boot_id;
        referee_assembly_.receiver_boot_id = fragment.receiver_boot_id;
        referee_rx_offset_ = 0;
        referee_assembly_start_ms_ = fragment.receive_ms;
    }
    if (fragment.transfer_id != referee_assembly_.transfer_id ||
        fragment.sender_boot_id != referee_assembly_.sender_boot_id || fragment.total_size != referee_assembly_.size ||
        fragment.offset != referee_rx_offset_ || fragment.receive_ms < referee_assembly_start_ms_ ||
        fragment.receive_ms - referee_assembly_start_ms_ > 100) {
        referee_assembly_.size = 0;
        referee_rx_offset_ = 0;
        return;
    }
    std::memcpy(referee_assembly_.bytes.data() + referee_rx_offset_, fragment.bytes.data(), fragment.size);
    referee_rx_offset_ += fragment.size;
    if (referee_rx_offset_ != referee_assembly_.size)
        return;
    referee_assembly_.receive_ms = fragment.receive_ms;
    referee_assembly_.source_age_ms = fragment.source_age_ms;
    const auto key = k_spin_lock(&lock_);
    if (referee_rx_count_ < referee_rx_.size()) {
        referee_rx_[(referee_rx_head_ + referee_rx_count_) % referee_rx_.size()] = referee_assembly_;
        ++referee_rx_count_;
    }
    k_spin_unlock(&lock_, key);
    referee_assembly_.size = 0;
    referee_rx_offset_ = 0;
}
void InterBoardEndpoint::setStatus(const RunStatus &status) {
    const auto key = k_spin_lock(&lock_);
    outgoing_.status = status;
    k_spin_unlock(&lock_, key);
}
void InterBoardEndpoint::submitGimbalImuYaw(const GimbalImuYaw &sample) {
    if (config_.role != BoardRole::GimbalController)
        return;
    const auto key = k_spin_lock(&lock_);
    outgoing_.gimbal_imu = sample;
    k_spin_unlock(&lock_, key);
}
void InterBoardEndpoint::submitBigYaw(const BigYawRequest &request) {
    const auto key = k_spin_lock(&lock_);
    outgoing_.big_yaw_request = request;
    k_spin_unlock(&lock_, key);
}
void InterBoardEndpoint::setBigYawFeedback(const BigYawFeedback &feedback) {
    const auto key = k_spin_lock(&lock_);
    outgoing_.big_yaw_feedback = feedback;
    k_spin_unlock(&lock_, key);
}
InterBoardEndpoint::Snapshot InterBoardEndpoint::snapshot() const {
    const auto key = k_spin_lock(&lock_);
    const auto value = published_;
    k_spin_unlock(&lock_, key);
    return value;
}
void InterBoardEndpoint::poll(std::uint64_t now) {
    if ((config_.role != BoardRole::GimbalController && config_.role != BoardRole::ChassisController) ||
        !config_.command_timeout_ms || !config_.heartbeat_timeout_ms || !config_.tx_timeout_ms ||
        config_.tx_timeout_ms > 1000 || !config_.status_timeout_ms || !config_.heartbeat_period_ms ||
        !config_.control_period_ms || !config_.imu_period_ms || !config_.aux_period_ms) {
        const auto key = k_spin_lock(&lock_);
        published_ = {};
        published_.transport = transport_.kind();
        published_.error = -EINVAL;
        k_spin_unlock(&lock_, key);
        return;
    }
    if (!boot_id_)
        boot_id_ = sys_rand64_get() | 1ULL;
    // Advance backend recovery even when no application message can be sent.
    const int serviced = transport_.service(now);
    bool transport_ready = serviced == 0;
    if (serviced != -EAGAIN)
        error_ = serviced;
    InterBoardTransport::RxChunk chunk{};
    for (unsigned budget = 0; budget < 8; ++budget) {
        const int ret = transport_.read(chunk);
        if (ret == -EOVERFLOW) {
            link_.discardPartial();
            continue;
        }
        if (ret == -EAGAIN)
            break;
        if (ret < 0) {
            error_ = ret;
            transport_ready = false;
            break;
        }
        (void)link_.processRxBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
    }
    (void)link_.processRxBytes(nullptr, 0, now);
    Outgoing outgoing{};
    {
        const auto key = k_spin_lock(&lock_);
        outgoing = outgoing_;
        k_spin_unlock(&lock_, key);
    }
    Snapshot next{};
    next.transport = transport_.kind();
    next.local_boot_id = boot_id_;
    (void)link_.latestHeartbeat(next.peer);
    next.online = link_.peerOnline(now, config_.heartbeat_timeout_ms);
    if (peer_online_ && !next.online)
        link_.invalidateControl();
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
        (void)link_.latestGimbalImuYaw(next.gimbal_imu);
    }
    if (!transport_ready || !next.online || next.gimbal_imu.receiver_boot_id != boot_id_ ||
        next.gimbal_imu.sender_boot_id != next.peer.sender_boot_id ||
        !forwardedFresh(next.gimbal_imu.stamp, next.gimbal_imu.source_age_ms, now, 50))
        next.gimbal_imu.valid = false;
    const auto &op = next.operator_control;
    next.operator_control_valid = next.online && op.sender_boot_id == next.peer.sender_boot_id &&
                                  op.receiver_boot_id == boot_id_ &&
                                  forwardedFresh(op.stamp, op.source_age_ms, now, config_.command_timeout_ms);
    if (!forwardedFresh(next.big_yaw_request.stamp,
                        std::max(next.big_yaw_request.command_age_ms, next.big_yaw_request.source_age_ms), now,
                        config_.command_timeout_ms))
        next.big_yaw_request = {};
    if (!next.big_yaw_feedback.valid ||
        !forwardedFresh(next.big_yaw_feedback.stamp, next.big_yaw_feedback.production_age_ms, now,
                        config_.status_timeout_ms)) {
        next.big_yaw_feedback.valid = false;
        next.big_yaw_feedback.ready = false;
        next.big_yaw_feedback.armed = false;
    }
    if (transport_ready && now >= next_tx_ms_ && !transport_.txBusy()) {
        std::array<std::uint32_t, kMessageCount> periods{};
        periods.fill(config_.aux_period_ms);
        periods[messageIndex(MessageId::Heartbeat)] = config_.heartbeat_period_ms;
        periods[messageIndex(MessageId::ChassisControl)] = config_.control_period_ms;
        periods[messageIndex(MessageId::BigYawRequest)] = config_.control_period_ms;
        periods[messageIndex(MessageId::BigYawFeedback)] = config_.control_period_ms;
        periods[messageIndex(MessageId::GimbalImuYaw)] = config_.imu_period_ms;
        periods[messageIndex(MessageId::RefereeData)] = 1;
        std::size_t referee_fragment_size = 0;
        std::uint8_t batch[InterBoardTransport::kTxCapacity]{};
        std::size_t used = 0;
        std::array<bool, kMessageCount> included{};
        // Encode each due message into its own bounded frame. A full batch
        // leaves unsent messages due; the next poll re-encodes fresh ages.
        const auto append = [&](MessageId id, auto encode) {
            const auto index = static_cast<std::size_t>(messageIndex(id));
            if (now < next_message_ms_[index])
                return;
            std::uint8_t frame[kMaxFrame]{};
            const int n = encode(++wire_sequence_[index], frame, sizeof(frame));
            if (n < 0) {
                error_ = n;
                return;
            }
            if (!n)
                return;
            if (static_cast<std::size_t>(n) > sizeof(batch)) {
                error_ = -EMSGSIZE;
                return;
            }
            if (static_cast<std::size_t>(n) > sizeof(batch) - used)
                return;
            std::memcpy(batch + used, frame, n);
            used += static_cast<std::size_t>(n);
            included[index] = true;
        };
        append(MessageId::Heartbeat, [&](auto seq, auto *bytes, auto cap) {
            const auto feedback = wireFeedback(outgoing.status, now, config_.status_timeout_ms);
            BoardHeartbeat heartbeat{};
            heartbeat.role = config_.role;
            heartbeat.sender_boot_id = boot_id_;
            heartbeat.sender_uptime_ms = static_cast<std::uint32_t>(now);
            heartbeat.ready = feedback.ready;
            heartbeat.safety_state = feedback.safety_state;
            heartbeat.active_reasons = feedback.active_reasons;
            heartbeat.sync_requested = !next.online;
            return InterBoardCodec::encodeHeartbeat(heartbeat, seq, bytes, cap);
        });
        if (config_.role == BoardRole::GimbalController) {
            // Optional bench authorization: send only when a producer supplied
            // it for this peer session. Do not rebind an old release on retry.
            if (next.online && outgoing.operator_control.stamp.valid &&
                outgoing.operator_control.receiver_boot_id == peer_boot_)
                append(MessageId::OperatorControl, [&](auto seq, auto *bytes, auto cap) {
                    auto request = outgoing.operator_control;
                    const bool current = forwardedFresh(request.stamp, request.source_age_ms, now,
                                                        config_.command_timeout_ms);
                    const auto elapsed = age(request.stamp, now);
                    request.sender_boot_id = boot_id_;
                    request.source_age_ms = addAge(request.source_age_ms, elapsed);
                    request.run_allowed = request.run_allowed && current;
                    if (!current || !request.clear_event_id ||
                        !forwardedFresh(request.stamp, request.clear_event_age_ms, now, config_.command_timeout_ms)) {
                        request.clear_event_id = 0;
                        request.clear_event_age_ms = UINT32_MAX;
                    }
                    else
                        request.clear_event_age_ms = addAge(request.clear_event_age_ms, elapsed);
                    return InterBoardCodec::encodeOperatorControl(request, seq, bytes, cap);
                });
            if (next.online && isFresh(outgoing.command.stamp, now, config_.command_timeout_ms) &&
                (outgoing.command.mode == ChassisMode::Disabled || !have_baseline_ ||
                 sequenceAfter(outgoing.command.stamp.sequence, baseline_sequence_)))
                append(MessageId::ChassisControl, [&](auto seq, auto *bytes, auto cap) {
                    RemoteChassisControl control{};
                    control.command = outgoing.command;
                    control.stamp = outgoing.command.stamp;
                    control.receiver_boot_id = peer_boot_;
                    control.global_action = control.command.mode == ChassisMode::Disabled ? SafetyAction::Disable
                                                                                          : SafetyAction::Active;
                    return InterBoardCodec::encodeChassisControl(control, seq, bytes, cap);
                });
            append(MessageId::BigYawRequest, [&](auto seq, auto *bytes, auto cap) {
                auto request = outgoing.big_yaw_request;
                const bool current = forwardedFresh(request.stamp,
                                                    std::max(request.source_age_ms, request.command_age_ms), now,
                                                    config_.command_timeout_ms) &&
                                     (!have_big_yaw_baseline_ ||
                                      sequenceAfter(request.stamp.sequence, big_yaw_baseline_sequence_));
                const auto elapsed = age(request.stamp, now);
                request.source_age_ms = addAge(request.source_age_ms, elapsed);
                request.command_age_ms = addAge(request.command_age_ms, elapsed);
                request.permission_age_ms = age(request.permission.stamp, now);
                request.receiver_boot_id = peer_boot_;
                if (!next.online || !current || request.mode != BigYawMode::FollowCenter ||
                    !std::isfinite(request.target_rate_rad_s)) {
                    request.mode = BigYawMode::Disabled;
                    request.target_rate_rad_s = 0;
                    request.permission.enabled = false;
                }
                // Keep sending stops at the normal cadence, even without a peer
                // heartbeat or a live producer. Never renew producer stamps/ages.
                return InterBoardCodec::encodeBigYawRequest(request, seq, bytes, cap);
            });
            if (next.online && outgoing.gimbal_imu.receiver_boot_id == peer_boot_)
                append(MessageId::GimbalImuYaw, [&](auto seq, auto *bytes, auto cap) {
                    auto sample = outgoing.gimbal_imu;
                    sample.source_age_ms = addAge(sample.source_age_ms, age(sample.stamp, now));
                    if (!forwardedFresh(sample.stamp, outgoing.gimbal_imu.source_age_ms, now, 50))
                        sample.valid = false;
                    return InterBoardCodec::encodeGimbalImuYaw(sample, seq, bytes, cap);
                });
            append(MessageId::ChassisConstraint, [&](auto seq, auto *bytes, auto cap) {
                ChassisConstraint constraint{};
                constraint.output = outgoing.referee.robot.chassis_output;
                constraint.output_age_ms = age(constraint.output.stamp, now);
                constraint.power_valid = outgoing.referee.power.limit_stamp.valid && outgoing.referee.power.stamp.valid;
                constraint.power_limit_w = outgoing.referee.power.chassis_power_limit_w;
                constraint.buffer_energy_j = outgoing.referee.power.buffer_energy_j;
                constraint.power_age_ms = std::max(age(outgoing.referee.power.limit_stamp, now),
                                                   age(outgoing.referee.power.stamp, now));
                return InterBoardCodec::encodeChassisConstraint(constraint, seq, bytes, cap);
            });
        }
        else {
            append(MessageId::ChassisFeedback, [&](auto seq, auto *bytes, auto cap) {
                return InterBoardCodec::encodeChassisFeedback(wireFeedback(outgoing.status, now,
                                                                           config_.status_timeout_ms),
                                                              seq, bytes, cap);
            });
            append(MessageId::BigYawFeedback, [&](auto seq, auto *bytes, auto cap) {
                auto feedback = outgoing.big_yaw_feedback;
                feedback.production_age_ms = age(feedback.stamp, now);
                feedback.source_sequence = feedback.stamp.sequence;
                if (!isFresh(feedback.stamp, now, config_.status_timeout_ms)) {
                    feedback.valid = false;
                    feedback.ready = false;
                    feedback.armed = false;
                    feedback.execution_state = ExecutionState::Waiting;
                    feedback.active_reasons |= FeedbackStale;
                }
                return InterBoardCodec::encodeBigYawFeedback(feedback, seq, bytes, cap);
            });
            // Referee traffic uses only space left after heartbeat/control feedback.
            if (referee_pending_valid_ &&
                (!next.online || referee_pending_.receiver_boot_id != peer_boot_ || now < referee_pending_.receive_ms ||
                 now - referee_pending_.receive_ms > 300))
                referee_pending_valid_ = false;
            if (next.online && !referee_pending_valid_) {
                const auto key = k_spin_lock(&lock_);
                while (referee_tx_count_) {
                    referee_pending_ = referee_tx_[referee_tx_head_];
                    referee_tx_head_ = (referee_tx_head_ + 1) % referee_tx_.size();
                    --referee_tx_count_;
                    if (referee_pending_.receiver_boot_id != peer_boot_ || now < referee_pending_.receive_ms ||
                        now - referee_pending_.receive_ms > 300)
                        continue;
                    referee_pending_valid_ = true;
                    referee_tx_offset_ = 0;
                    break;
                }
                k_spin_unlock(&lock_, key);
            }
            if (referee_pending_valid_)
                append(MessageId::RefereeData, [&](auto seq, auto *bytes, auto cap) {
                    RefereeFragment fragment{};
                    fragment.sender_boot_id = boot_id_;
                    fragment.receiver_boot_id = referee_pending_.receiver_boot_id;
                    fragment.transfer_id = referee_pending_.transfer_id;
                    fragment.source_age_ms = static_cast<std::uint32_t>(now - referee_pending_.receive_ms);
                    fragment.total_size = referee_pending_.size;
                    fragment.offset = static_cast<std::uint16_t>(referee_tx_offset_);
                    fragment.size = std::min(kRefereeFragmentCapacity, referee_pending_.size - referee_tx_offset_);
                    std::memcpy(fragment.bytes.data(), referee_pending_.bytes.data() + referee_tx_offset_,
                                fragment.size);
                    referee_fragment_size = fragment.size;
                    return InterBoardCodec::encodeRefereeFragment(fragment, seq, bytes, cap);
                });
        }
        if (used) {
            const int ret = transport_.send(batch, used, config_.tx_timeout_ms);
            if (ret == 0) {
                if (included[messageIndex(MessageId::RefereeData)]) {
                    referee_tx_offset_ += referee_fragment_size;
                    if (referee_tx_offset_ == referee_pending_.size)
                        referee_pending_valid_ = false;
                }
                next_tx_ms_ = now + 1;
                for (std::size_t i = 0; i < included.size(); ++i)
                    if (included[i])
                        next_message_ms_[i] = now + periods[i];
            }
            else if (ret != -EAGAIN)
                error_ = ret;
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
