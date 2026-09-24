#include <drivers/motor/can_bus.hpp>

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <limits>
#include <variant>

#include <drivers/motor/dji_protocol.hpp>
#include <drivers/motor/dm_protocol.hpp>
#include <drivers/motor/group.hpp>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(skywalker_motor_bus);

namespace skywalker::motor {
namespace {

#ifndef CONFIG_SKYWALKER_MOTOR_MAX_BUSES
#define CONFIG_SKYWALKER_MOTOR_MAX_BUSES 3
#endif
#ifndef CONFIG_SKYWALKER_MOTOR_IO_PRIORITY
#define CONFIG_SKYWALKER_MOTOR_IO_PRIORITY 5
#endif
constexpr std::uint32_t kDmSafetyProbeIntervalMs = 10;

k_spinlock owner_lock{};
CanBus *owners[CONFIG_SKYWALKER_MOTOR_MAX_BUSES]{};

std::uint64_t nowMs() {
    return std::max<std::uint64_t>(1, static_cast<std::uint64_t>(k_uptime_get()));
}

bool elapsed(std::uint64_t now, std::uint64_t then, std::uint32_t limit) {
    return then == 0 || now < then || now - then > limit;
}

} // namespace

bool CanBus::isDji(const Motor &motor) {
    return std::holds_alternative<dji::Config>(motor.config_);
}

// Static topology is checked before taking ownership of a controller. The
// descriptors are values; this code never constructs a brand-specific bus.
int CanBus::describeMotor(const Motor &motor, std::uint16_t &rx_id, std::uint16_t &tx_id,
                          std::uint8_t &slot) {
    if (const auto *cfg = std::get_if<dji::Config>(&motor.config_)) {
        dji::Descriptor descriptor{};
        const int ret = dji::describe(*cfg, descriptor);
        if (ret < 0)
            return ret;
        rx_id = descriptor.feedback_id;
        tx_id = descriptor.command_id;
        slot = descriptor.command_slot;
        return 0;
    }
    const auto &cfg = std::get<dm::Config>(motor.config_);
    dm::Descriptor descriptor{};
    const int ret = dm::describe(cfg, descriptor);
    if (ret < 0)
        return ret;
    rx_id = descriptor.master_id;
    tx_id = descriptor.control_id;
    slot = cfg.id;
    return 0;
}

CanBus::CanBus(const device *can, BusOptions options) : can_(can), options_(options) {
    k_sem_init(&wake_sem_, 0, 1);
}

int CanBus::attach(Motor &motor) {
    Motor *one[] = {&motor};
    return attachBatch(one, 1);
}

int CanBus::attachBatch(Motor *const *batch, std::size_t count) {
    if (batch == nullptr || count == 0)
        return -EINVAL;
    if (status_.state != BusState::Unstarted)
        return -EACCES;
    if (count > kMaxMotors - motor_count_)
        return -ENOSPC;
    for (std::size_t i = 0; i < count; ++i) {
        if (batch[i] == nullptr || batch[i]->bus_ != nullptr)
            return -EALREADY;
        for (std::size_t j = 0; j < i; ++j) {
            if (batch[i] == batch[j])
                return -EALREADY;
        }
    }
    for (std::size_t i = 0; i < count; ++i) {
        batch[i]->bus_ = this;
        motors_[motor_count_++] = batch[i];
    }
    return 0;
}

int CanBus::validateTopology() {
    if (can_ == nullptr || !device_is_ready(can_))
        return -ENODEV;
    if (motor_count_ == 0 || options_.tx_timeout_ms == 0 || options_.recovery_retry_ms == 0)
        return -EINVAL;

    unit_count_ = 0;
    route_count_ = 0;
    for (std::size_t i = 0; i < motor_count_; ++i) {
        Motor &motor = *motors_[i];
        const int config_error = motor.validateConfig();
        if (config_error < 0)
            return config_error;
        if (motor.group_conflict_ || (motor.group_ != nullptr && motor.group_->topologyValid() < 0))
            return -EINVAL;

        std::uint16_t rx_id = 0, tx_id = 0;
        std::uint8_t slot = 0;
        int ret = describeMotor(motor, rx_id, tx_id, slot);
        if (ret < 0)
            return ret;
        if (rx_id == tx_id)
            return -EADDRINUSE;

        for (std::size_t j = 0; j < i; ++j) {
            std::uint16_t other_rx = 0, other_tx = 0;
            std::uint8_t other_slot = 0;
            ret = describeMotor(*motors_[j], other_rx, other_tx, other_slot);
            if (ret < 0)
                return ret;
            const bool both_dji = isDji(motor) && isDji(*motors_[j]);
            const bool both_dm = !isDji(motor) && !isDji(*motors_[j]);
            if (rx_id == other_tx || tx_id == other_rx) {
                LOG_ERR("CAN ID TX/RX collision: 0x%x / 0x%x", tx_id, rx_id);
                return -EADDRINUSE;
            }
            if (tx_id == other_tx && !(both_dji && slot != other_slot)) {
                LOG_ERR("CAN TX collision: 0x%x", tx_id);
                return -EADDRINUSE;
            }
            if (rx_id == other_rx && !(both_dm && slot != other_slot)) {
                LOG_ERR("CAN RX collision: 0x%x", rx_id);
                return -EADDRINUSE;
            }
        }

        bool route_exists = false;
        for (std::size_t r = 0; r < route_count_; ++r) {
            route_exists |= routes_[r].id == rx_id;
        }
        if (!route_exists) {
            if (route_count_ >= kMaxMotors)
                return -ENOSPC;
            routes_[route_count_++] = Route{rx_id, -1};
        }

        if (isDji(motor)) {
            bool unit_exists = false;
            for (std::size_t u = 0; u < unit_count_; ++u) {
                unit_exists |= units_[u].kind == UnitKind::Dji && units_[u].command_id == tx_id;
            }
            if (!unit_exists)
                units_[unit_count_++] = TxUnit{UnitKind::Dji, tx_id, i};
        }
        else {
            units_[unit_count_++] = TxUnit{UnitKind::Dm, tx_id, i};
        }
    }
    return 0;
}

int CanBus::installRoutes() {
    for (std::size_t i = 0; i < route_count_; ++i) {
        can_filter filter{};
        filter.id = routes_[i].id;
        filter.mask = CAN_STD_ID_MASK;
        filter.flags = 0;
        const int handle = can_add_rx_filter(can_, onRx, this, &filter);
        if (handle < 0)
            return handle;
        routes_[i].filter_id = handle;
    }
    return 0;
}

void CanBus::rollbackStart() {
    if (controller_started_) {
        (void)can_stop(can_);
        controller_started_ = false;
    }
    for (std::size_t i = 0; i < route_count_; ++i) {
        if (routes_[i].filter_id >= 0) {
            can_remove_rx_filter(can_, routes_[i].filter_id);
            routes_[i].filter_id = -1;
        }
    }
    const auto key = k_spin_lock(&owner_lock);
    for (CanBus *&owner : owners) {
        if (owner == this)
            owner = nullptr;
    }
    k_spin_unlock(&owner_lock, key);
}

int CanBus::start() {
    if (status_.state == BusState::Running || status_.state == BusState::Recovering)
        return -EALREADY;
    const int check = validateTopology();
    if (check < 0) {
        status_.state = BusState::ConfigBlocked;
        status_.last_error = check;
        return check;
    }

    bool claimed = false;
    const auto key = k_spin_lock(&owner_lock);
    for (CanBus *owner : owners) {
        if (owner != nullptr && owner->can_ == can_) {
            k_spin_unlock(&owner_lock, key);
            return -EBUSY;
        }
    }
    for (CanBus *&owner : owners) {
        if (owner == nullptr) {
            owner = this;
            claimed = true;
            break;
        }
    }
    k_spin_unlock(&owner_lock, key);
    if (!claimed)
        return -ENOSPC;

    int ret = installRoutes();
    if (ret == 0) {
        ret = can_start(can_);
        if (ret == 0)
            controller_started_ = true;
        else if (ret == -EALREADY)
            ret = -EBUSY;
    }
    if (ret < 0) {
        rollbackStart();
        status_.last_error = ret;
        return ret;
    }

    status_.state = BusState::Running;
    status_.last_error = 0;
    for (std::size_t i = 0; i < motor_count_; ++i)
        motors_[i]->markStarted();
    k_thread_create(&thread_, thread_stack_, K_KERNEL_STACK_SIZEOF(thread_stack_),
                    threadEntry, this, nullptr, nullptr,
                    CONFIG_SKYWALKER_MOTOR_IO_PRIORITY, 0, K_NO_WAIT);
    thread_started_ = true;
    wake();
    return 0;
}

CommitResult CanBus::commit() {
    const auto state_key = k_spin_lock(&state_lock_);
    const BusState state = status_.state;
    k_spin_unlock(&state_lock_, state_key);
    if (state != BusState::Running)
        return {state == BusState::Recovering ? -EAGAIN : -EACCES, 0};

    StagedCommand next[kMaxMotors]{};
    for (std::size_t i = 0; i < motor_count_; ++i)
        next[i] = motors_[i]->copyStaged();

    const auto key = k_spin_lock(&publication_lock_);
    if (published_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
        k_spin_unlock(&publication_lock_, key);
        return {-EOVERFLOW, 0};
    }
    const bool superseded = targets_remaining_ > 0;
    ++published_sequence_;
    for (std::size_t i = 0; i < motor_count_; ++i)
        published_[i] = next[i];
    targets_remaining_ = unit_count_;
    const std::uint64_t sequence = published_sequence_;
    k_spin_unlock(&publication_lock_, key);
    if (superseded) {
        const auto state_key = k_spin_lock(&state_lock_);
        ++status_.superseded_batches;
        k_spin_unlock(&state_lock_, state_key);
    }
    wake();
    return {0, sequence};
}

BusStatus CanBus::status() const {
    const auto key = k_spin_lock(&state_lock_);
    BusStatus copy = status_;
    k_spin_unlock(&state_lock_, key);
    return copy;
}

void CanBus::wake() {
    k_sem_give(&wake_sem_);
}

std::uint64_t CanBus::recordCallbackOrder() {
    const auto key = k_spin_lock(&callback_order_lock_);
    // Exhaustion fails closed: order zero cannot confirm a protocol handshake.
    const std::uint64_t order = next_callback_order_;
    if (next_callback_order_ != std::numeric_limits<std::uint64_t>::max())
        ++next_callback_order_;
    k_spin_unlock(&callback_order_lock_, key);
    return order == std::numeric_limits<std::uint64_t>::max() ? 0 : order;
}

void CanBus::onRx(const device *, can_frame *frame, void *context) {
    auto *self = static_cast<CanBus *>(context);
    if (self == nullptr || frame == nullptr)
        return;
    const std::uint64_t callback_order = self->recordCallbackOrder();
    const auto state_key = k_spin_lock(&self->state_lock_);
    const auto generation = self->bus_generation_;
    const bool running = self->status_.state == BusState::Running;
    k_spin_unlock(&self->state_lock_, state_key);
    if (!running)
        return;
    RxEvent event{*frame, nowMs(), generation, callback_order};
    const auto key = k_spin_lock(&self->rx_lock_);
    if (self->rx_count_ == kRxDepth) {
        self->rx_overflowed_ = true;
    }
    else {
        self->rx_queue_[self->rx_tail_] = event;
        self->rx_tail_ = (self->rx_tail_ + 1) % kRxDepth;
        ++self->rx_count_;
    }
    k_spin_unlock(&self->rx_lock_, key);
    self->wake();
}

void CanBus::onTxDone(const device *, int error, void *context) {
    auto *self = static_cast<CanBus *>(context);
    if (self == nullptr)
        return;
    const std::uint64_t callback_order = self->recordCallbackOrder();
    const auto key = k_spin_lock(&self->tx_lock_);
    if (self->in_flight_.busy && !self->in_flight_.callback_seen) {
        self->in_flight_.callback_seen = true;
        self->in_flight_.callback_error = error;
        self->in_flight_.completed_ms = nowMs();
        self->in_flight_.completed_order = callback_order;
    }
    k_spin_unlock(&self->tx_lock_, key);
    self->wake();
}

void CanBus::threadEntry(void *context, void *, void *) {
    static_cast<CanBus *>(context)->ioMain();
}

void CanBus::processRx(const RxEvent &event) {
    if (event.bus_generation != bus_generation_)
        return;
    if ((event.frame.flags & (CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF)) != 0 || event.frame.dlc != 8) {
        const auto key = k_spin_lock(&state_lock_);
        ++status_.rx_invalid_frames;
        k_spin_unlock(&state_lock_, key);
        return;
    }
    bool accepted = false;
    for (std::size_t i = 0; i < motor_count_; ++i) {
        Motor &motor = *motors_[i];
        std::uint16_t rx_id = 0, tx_id = 0;
        std::uint8_t slot = 0;
        if (describeMotor(motor, rx_id, tx_id, slot) < 0 || rx_id != event.frame.id)
            continue;
        if (std::holds_alternative<dji::Config>(motor.config_)) {
            dji::RawFeedback raw{};
            if (dji::decodeFeedback(event.frame, raw)) {
                raw.timestamp_ms = event.received_ms;
                accepted |= motor.acceptDjiFeedback(raw, event.received_ms) == 0;
            }
            continue;
        }
        const auto &cfg = std::get<dm::Config>(motor.config_);
        dm::DecodedFeedback decoded{};
        if (dm::decodeFeedback(event.frame, cfg.id, cfg.limits, decoded) == 0) {
            decoded.raw.timestamp_ms = event.received_ms;
            if (motor.acceptDmFeedback(decoded, event.received_ms, event.callback_order) == 0) {
                accepted = true;
                const auto snapshot = motor.snapshot();
                if (snapshot.stop.progress == StopProgress::DriveConfirmed &&
                    decoded.raw.status == dm::DriveStatus::Disabled)
                    motor.markSafePrepared(snapshot.stop.request_generation);
            }
        }
    }
    if (!accepted) {
        const auto key = k_spin_lock(&state_lock_);
        ++status_.rx_invalid_frames;
        k_spin_unlock(&state_lock_, key);
    }
}

void CanBus::processTx() {
    InFlight completed{};
    const auto key = k_spin_lock(&tx_lock_);
    if (!in_flight_.busy || !in_flight_.callback_seen) {
        k_spin_unlock(&tx_lock_, key);
        return;
    }
    completed = in_flight_;
    in_flight_.busy = false;
    in_flight_.callback_seen = false;
    k_spin_unlock(&tx_lock_, key);

    if (completed.callback_error == 0 &&
        (completed.completed_ms < completed.submitted_ms ||
         completed.completed_ms - completed.submitted_ms > options_.tx_timeout_ms))
        completed.callback_error = -ETIMEDOUT;

    const auto state_key = k_spin_lock(&state_lock_);
    status_.last_tx = {true, completed.sequence, completed.frame.id, completed.purpose,
                       completed.callback_error, completed.completed_ms};
    if (completed.callback_error < 0)
        status_.last_error = completed.callback_error;
    k_spin_unlock(&state_lock_, state_key);

    if (completed.callback_error < 0) {
        enterRecovery(completed.callback_error, FaultReason::TransportError);
        return;
    }
    if (completed.bus_generation != bus_generation_)
        return;
    if (completed.purpose == TxPurpose::SafeOutput)
        updateStopAfterTx(completed);
    else if (completed.purpose == TxPurpose::Enable) {
        Motor &motor = *motors_[units_[completed.unit_index].motor_index];
        motor.markEnableTxComplete(completed.enable_generation, completed.completed_ms,
                                   completed.completed_order);
    }
    else if (completed.purpose == TxPurpose::ClearFault) {
        motors_[units_[completed.unit_index].motor_index]->markClearTxComplete(
            completed.completed_ms, completed.completed_order);
    }
    else if (completed.purpose == TxPurpose::Probe) {
        neutral_done_generation_[units_[completed.unit_index].motor_index] = completed.enable_generation;
    }
}

void CanBus::checkDeadlines(std::uint64_t now_ms) {
    const auto key = k_spin_lock(&tx_lock_);
    const bool timed_out = in_flight_.busy && !in_flight_.callback_seen &&
                           elapsed(now_ms, in_flight_.submitted_ms, options_.tx_timeout_ms);
    k_spin_unlock(&tx_lock_, key);
    if (timed_out) {
        enterRecovery(-ETIMEDOUT, FaultReason::TransportError);
        return;
    }

    can_state state{};
    const int controller_error = can_get_state(can_, &state, nullptr);
    if (controller_error < 0 || state == CAN_STATE_BUS_OFF || state == CAN_STATE_STOPPED) {
        enterRecovery(controller_error < 0 ? controller_error
                      : state == CAN_STATE_BUS_OFF ? -ENETUNREACH : -ENETDOWN,
                      FaultReason::TransportError);
        return;
    }

    for (std::size_t i = 0; i < motor_count_; ++i) {
        Motor &motor = *motors_[i];
        const MotorSnapshot snapshot = motor.snapshot();
        if (std::holds_alternative<dm::Config>(motor.config_) &&
            snapshot.stop.progress == StopProgress::TxComplete) {
            const auto motor_key = k_spin_lock(&motor.lock_);
            const auto safety_completed_ms = motor.safe_first_tx_completed_ms_;
            k_spin_unlock(&motor.lock_, motor_key);
            if (elapsed(now_ms, safety_completed_ms, motor.info().timing.enable_timeout_ms))
                motor.markStopped(StopProgress::Unreachable, 0, snapshot.stop.request_generation);
        }
        if (snapshot.state != MotorState::Active && snapshot.state != MotorState::Enabling)
            continue;
        const Timing timing = motor.info().timing;
        if (elapsed(now_ms, snapshot.feedback.timestamp_ms, timing.feedback_timeout_ms)) {
            motor.raiseFault({FaultReason::FeedbackExpired, -ETIMEDOUT, &motor, now_ms});
            continue;
        }
        if (snapshot.state == MotorState::Enabling) {
            const auto motor_key = k_spin_lock(&motor.lock_);
            const auto requested_ms = motor.enable_requested_at_ms_;
            const bool safe = motor.safe_prepared_;
            const bool tx_done = motor.enable_tx_done_;
            const bool feedback_after_tx = motor.enable_tx_completed_order_ != 0u &&
                                           motor.feedback_event_order_ > motor.enable_tx_completed_order_;
            k_spin_unlock(&motor.lock_, motor_key);
            if (elapsed(now_ms, requested_ms, timing.enable_timeout_ms)) {
                motor.raiseFault({FaultReason::EnableTimeout, -ETIMEDOUT, &motor, now_ms});
                continue;
            }
            if (!safe || !motor.feedbackFresh(now_ms))
                continue;
            if (std::holds_alternative<dji::Config>(motor.config_) ||
                (tx_done && snapshot.native_drive_status_valid &&
                 snapshot.native_drive_status == static_cast<std::uint32_t>(dm::DriveStatus::Enabled) &&
                 feedback_after_tx &&
                 neutral_done_generation_[i] == snapshot.enable_generation)) {
                motor.markPrepared(snapshot.enable_generation);
            }
            continue;
        }

        StagedCommand published{};
        const auto publication_key = k_spin_lock(&publication_lock_);
        published = published_[i];
        k_spin_unlock(&publication_lock_, publication_key);
        std::uint64_t activated_ms = 0;
        const auto motor_key = k_spin_lock(&motor.lock_);
        activated_ms = motor.activated_ms_;
        k_spin_unlock(&motor.lock_, motor_key);
        const bool current_command = published.valid &&
                                     published.enable_generation == snapshot.enable_generation;
        if ((current_command && elapsed(now_ms, published.written_ms, timing.command_timeout_ms)) ||
            (!current_command && elapsed(now_ms, activated_ms, timing.command_timeout_ms))) {
            motor.raiseFault({FaultReason::CommandExpired, -ETIMEDOUT, &motor, now_ms});
        }
    }
}

bool CanBus::unitHasPendingSafety(const TxUnit &unit) const {
    for (std::size_t i = 0; i < motor_count_; ++i) {
        Motor &motor = *motors_[i];
        if (unit.kind == UnitKind::Dm && i != unit.motor_index)
            continue;
        if (unit.kind == UnitKind::Dji) {
            if (!isDji(motor))
                continue;
            std::uint16_t rx = 0, tx = 0;
            std::uint8_t slot = 0;
            if (describeMotor(motor, rx, tx, slot) < 0 || tx != unit.command_id)
                continue;
        }
        const auto key = k_spin_lock(&motor.lock_);
        const bool pending = motor.safe_pending_;
        k_spin_unlock(&motor.lock_, key);
        if (pending)
            return true;
    }
    return false;
}

bool CanBus::unitNeedsDmSafetyProbe(const TxUnit &unit, std::uint64_t now_ms) const {
    if (unit.kind != UnitKind::Dm)
        return false;
    const Motor &motor = *motors_[unit.motor_index];
    const auto key = k_spin_lock(&motor.lock_);
    const bool safe_state = motor.snapshot_.stop.progress == StopProgress::TxComplete ||
                            motor.snapshot_.stop.progress == StopProgress::Unreachable ||
                            motor.snapshot_.stop.progress == StopProgress::DriveConfirmed;
    const bool due = safe_state && !motor.safe_pending_ &&
                     motor.safe_tx_done_ && motor.safe_tx_completed_ms_ != 0 &&
                     motor.snapshot_.state != MotorState::Active &&
                     motor.snapshot_.state != MotorState::Enabling &&
                     now_ms >= motor.safe_tx_completed_ms_ &&
                     now_ms - motor.safe_tx_completed_ms_ >= kDmSafetyProbeIntervalMs;
    k_spin_unlock(&motor.lock_, key);
    return due;
}

int CanBus::buildTarget(const TxUnit &unit, std::uint64_t now_ms, can_frame &out) {
    if (unit.kind == UnitKind::Dji) {
        std::int16_t slots[4]{};
        for (std::size_t i = 0; i < motor_count_; ++i) {
            Motor &motor = *motors_[i];
            const auto *cfg = std::get_if<dji::Config>(&motor.config_);
            if (cfg == nullptr)
                continue;
            dji::Descriptor descriptor{};
            if (dji::describe(*cfg, descriptor) < 0 || descriptor.command_id != unit.command_id)
                continue;
            StagedCommand command{};
            const auto key = k_spin_lock(&publication_lock_);
            command = published_[i];
            k_spin_unlock(&publication_lock_, key);
            const MotorSnapshot snapshot = motor.snapshot();
            if (!motor.active() || !command.valid || command.command.kind != CommandKind::Current ||
                command.enable_generation != snapshot.enable_generation ||
                elapsed(now_ms, command.written_ms, cfg->timing.command_timeout_ms) ||
                elapsed(now_ms, snapshot.feedback.timestamp_ms, cfg->timing.feedback_timeout_ms))
                continue;
            const float scaled = command.command.primary *
                                 (descriptor.model == dji::Model::M2006C610 ? 10000.0f : 16384.0f) /
                                 descriptor.protocol_current_max_a;
            slots[descriptor.command_slot] = static_cast<std::int16_t>(std::lround(scaled));
        }
        return dji::buildCommandFrame(out, unit.command_id, slots);
    }

    Motor &motor = *motors_[unit.motor_index];
    const auto &cfg = std::get<dm::Config>(motor.config_);
    const MotorSnapshot snapshot = motor.snapshot();
    StagedCommand command{};
    const auto key = k_spin_lock(&publication_lock_);
    command = published_[unit.motor_index];
    k_spin_unlock(&publication_lock_, key);
    if (!motor.active() || !command.valid || command.enable_generation != snapshot.enable_generation ||
        elapsed(now_ms, command.written_ms, cfg.timing.command_timeout_ms) ||
        elapsed(now_ms, snapshot.feedback.timestamp_ms, cfg.timing.feedback_timeout_ms))
        return -ENOENT;
    switch (command.command.kind) {
    case CommandKind::Torque: {
        dm::MitCommand mit{};
        mit.torque_ff_nm = command.command.primary;
        return dm::buildMitFrame(cfg.id, cfg.limits, mit, out);
    }
    case CommandKind::Mit:
        return dm::buildMitFrame(cfg.id, cfg.limits, command.command.mit, out);
    case CommandKind::Velocity:
        return dm::buildVelocityFrame(cfg.id, command.command.primary, out);
    case CommandKind::PositionVelocity:
        return dm::buildPositionVelocityFrame(cfg.id, command.command.primary, command.command.secondary, out);
    default:
        return -ENOTSUP;
    }
}

int CanBus::buildSafety(const TxUnit &unit, can_frame &out) {
    if (unit.kind == UnitKind::Dji)
        return buildTarget(unit, nowMs(), out); // inactive slots become zero; healthy slots retain their target.
    const auto &cfg = std::get<dm::Config>(motors_[unit.motor_index]->config_);
    return dm::buildSpecialFrame(cfg.mode, cfg.id, dm::SpecialCommand::Disable, out);
}

int CanBus::submit(const can_frame &frame, TxPurpose purpose, std::size_t unit_index,
                   std::uint64_t sequence, std::uint64_t now_ms) {
    const auto key = k_spin_lock(&tx_lock_);
    if (in_flight_.busy) {
        k_spin_unlock(&tx_lock_, key);
        return -EBUSY;
    }
    if (next_operation_id_ == std::numeric_limits<std::uint64_t>::max()) {
        k_spin_unlock(&tx_lock_, key);
        return -EOVERFLOW;
    }
    in_flight_ = {};
    in_flight_.frame = frame;
    in_flight_.purpose = purpose;
    in_flight_.unit_index = unit_index;
    in_flight_.sequence = sequence;
    in_flight_.bus_generation = bus_generation_;
    in_flight_.operation_id = next_operation_id_++;
    in_flight_.submitted_ms = now_ms;
    if (purpose == TxPurpose::Enable || purpose == TxPurpose::Probe)
        in_flight_.enable_generation = motors_[units_[unit_index].motor_index]->snapshot().enable_generation;
    if (purpose == TxPurpose::SafeOutput) {
        for (std::size_t i = 0; i < motor_count_; ++i)
            in_flight_.stop_generations[i] = motors_[i]->snapshot().stop.request_generation;
    }
    in_flight_.busy = true;
    k_spin_unlock(&tx_lock_, key);

    // The slot and callback context remain owned until completion or confirmed
    // controller cancellation. CAN operations never run while holding a spinlock.
    const int ret = can_send(can_, &in_flight_.frame, K_NO_WAIT, onTxDone, this);
    if (ret < 0) {
        const auto cancel_key = k_spin_lock(&tx_lock_);
        in_flight_.busy = false;
        k_spin_unlock(&tx_lock_, cancel_key);
        const auto state_key = k_spin_lock(&state_lock_);
        status_.last_tx = {true, sequence, frame.id, purpose, ret, nowMs()};
        status_.last_error = ret;
        k_spin_unlock(&state_lock_, state_key);
        enterRecovery(ret, FaultReason::TransportError);
        return ret;
    }
    const auto state_key = k_spin_lock(&state_lock_);
    status_.latest_submitted_sequence = sequence;
    k_spin_unlock(&state_lock_, state_key);
    return 0;
}

void CanBus::updateStopAfterTx(const InFlight &completed) {
    const TxUnit &unit = units_[completed.unit_index];
    for (std::size_t i = 0; i < motor_count_; ++i) {
        Motor &motor = *motors_[i];
        if (unit.kind == UnitKind::Dm && i != unit.motor_index)
            continue;
        if (unit.kind == UnitKind::Dji) {
            std::uint16_t rx = 0, tx = 0;
            std::uint8_t slot = 0;
            if (!isDji(motor) || describeMotor(motor, rx, tx, slot) < 0 || tx != unit.command_id)
                continue;
        }
        const auto snapshot = motor.snapshot();
        if (snapshot.stop.progress == StopProgress::Pending ||
            (unit.kind == UnitKind::Dm &&
             (snapshot.stop.progress == StopProgress::TxComplete ||
              snapshot.stop.progress == StopProgress::Unreachable ||
              snapshot.stop.progress == StopProgress::DriveConfirmed) &&
             snapshot.state != MotorState::Active && snapshot.state != MotorState::Enabling))
            motor.markStopped(StopProgress::TxComplete, 0, completed.stop_generations[i],
                              completed.completed_ms, completed.completed_order);
        if (unit.kind == UnitKind::Dji)
            motor.markSafePrepared(completed.stop_generations[i]);
    }
}

void CanBus::pumpTx(std::uint64_t now_ms) {
    const auto busy_key = k_spin_lock(&tx_lock_);
    const bool busy = in_flight_.busy;
    k_spin_unlock(&tx_lock_, busy_key);
    if (busy)
        return;

    // Safety requests are persistent endpoint metadata; a newer commit cannot
    // overwrite them. One DJI frame may settle several pending slots.
    for (std::size_t u = 0; u < unit_count_; ++u) {
        if (!unitHasPendingSafety(units_[u]))
            continue;
        can_frame frame{};
        const int ret = buildSafety(units_[u], frame);
        if (ret < 0) {
            enterRecovery(ret, FaultReason::TransportError);
            return;
        }
        (void)submit(frame, TxPurpose::SafeOutput, u, 0, now_ms);
        return;
    }

    for (std::size_t i = 0; i < motor_count_; ++i) {
        Motor &motor = *motors_[i];
        if (!isDji(motor))
            continue;
        const auto key = k_spin_lock(&motor.lock_);
        const bool clear = motor.clear_pending_;
        k_spin_unlock(&motor.lock_, key);
        if (clear) {
            motor.markFaultCleared();
            return;
        }
    }

    for (std::size_t u = 0; u < unit_count_; ++u) {
        const TxUnit &unit = units_[u];
        if (unit.kind != UnitKind::Dm)
            continue;
        Motor &motor = *motors_[unit.motor_index];
        const auto key = k_spin_lock(&motor.lock_);
        const bool clear = motor.clear_pending_ && !motor.clear_tx_done_;
        const bool enable = motor.enable_pending_ && !motor.enable_tx_done_;
        k_spin_unlock(&motor.lock_, key);
        if (!clear && !enable)
            continue;
        const auto &cfg = std::get<dm::Config>(motor.config_);
        can_frame frame{};
        const auto command = clear ? dm::SpecialCommand::ClearError : dm::SpecialCommand::Enable;
        const int ret = dm::buildSpecialFrame(cfg.mode, cfg.id, command, frame);
        if (ret < 0) {
            motor.raiseFault({FaultReason::TransportError, ret, &motor, now_ms});
            return;
        }
        (void)submit(frame, clear ? TxPurpose::ClearFault : TxPurpose::Enable, u, 0, now_ms);
        return;
    }

    for (std::size_t u = 0; u < unit_count_; ++u) {
        const TxUnit &unit = units_[u];
        if (unit.kind != UnitKind::Dm)
            continue;
        Motor &motor = *motors_[unit.motor_index];
        const MotorSnapshot snapshot = motor.snapshot();
        if (snapshot.state != MotorState::Enabling ||
            neutral_done_generation_[unit.motor_index] == snapshot.enable_generation ||
            !snapshot.native_drive_status_valid ||
            snapshot.native_drive_status != static_cast<std::uint32_t>(dm::DriveStatus::Enabled))
            continue;
        const auto key = k_spin_lock(&motor.lock_);
        const bool enabled_after_tx = motor.enable_tx_done_ &&
                                      motor.enable_tx_completed_order_ != 0u &&
                                      motor.feedback_event_order_ > motor.enable_tx_completed_order_;
        const auto native_position = std::get<Motor::DmRuntime>(motor.protocol_state_).last_native_position_rad;
        k_spin_unlock(&motor.lock_, key);
        if (!enabled_after_tx)
            continue;
        const auto &cfg = std::get<dm::Config>(motor.config_);
        can_frame frame{};
        int ret = 0;
        switch (cfg.mode) {
        case dm::ControlMode::Mit:
            ret = dm::buildMitFrame(cfg.id, cfg.limits, dm::MitCommand{}, frame);
            break;
        case dm::ControlMode::Velocity:
            ret = dm::buildVelocityFrame(cfg.id, 0.0f, frame);
            break;
        case dm::ControlMode::PositionVelocity:
            ret = dm::buildPositionVelocityFrame(cfg.id, native_position, 0.0f, frame);
            break;
        }
        if (ret < 0) {
            motor.raiseFault({FaultReason::InvalidCommand, ret, &motor, now_ms});
            return;
        }
        (void)submit(frame, TxPurpose::Probe, u, 0, now_ms);
        return;
    }

    // Disabled DM drives may need a host command to produce fresh feedback.
    // Alternate periodic safety probes with committed targets so either kind
    // of work progresses on a mixed bus under sustained load.
    if (target_before_safety_probe_ && pumpTarget(now_ms)) {
        target_before_safety_probe_ = false;
        return;
    }
    for (std::size_t attempt = 0; attempt < unit_count_; ++attempt) {
        const std::size_t u = (safety_probe_cursor_ + attempt) % unit_count_;
        if (!unitNeedsDmSafetyProbe(units_[u], now_ms))
            continue;
        can_frame frame{};
        const int ret = buildSafety(units_[u], frame);
        if (ret < 0) {
            enterRecovery(ret, FaultReason::TransportError);
            return;
        }
        safety_probe_cursor_ = (u + 1) % unit_count_;
        target_before_safety_probe_ = true;
        (void)submit(frame, TxPurpose::SafeOutput, u, 0, now_ms);
        return;
    }
    if (!target_before_safety_probe_ && pumpTarget(now_ms)) {
        target_before_safety_probe_ = false;
        return;
    }
}

bool CanBus::pumpTarget(std::uint64_t now_ms) {
    for (std::size_t attempts = 0; attempts < unit_count_; ++attempts) {
        TxUnit unit{};
        std::size_t index = 0;
        std::uint64_t sequence = 0;
        const auto key = k_spin_lock(&publication_lock_);
        if (targets_remaining_ == 0) {
            k_spin_unlock(&publication_lock_, key);
            return false;
        }
        index = target_cursor_;
        unit = units_[index];
        sequence = published_sequence_;
        target_cursor_ = (target_cursor_ + 1) % unit_count_;
        --targets_remaining_;
        k_spin_unlock(&publication_lock_, key);
        can_frame frame{};
        const int ret = buildTarget(unit, now_ms, frame);
        if (ret == -ENOENT)
            continue;
        if (ret < 0) {
            motors_[unit.motor_index]->raiseFault({FaultReason::InvalidCommand, ret,
                                                    motors_[unit.motor_index], now_ms});
            continue;
        }
        (void)submit(frame, TxPurpose::Target, index, sequence, now_ms);
        return true;
    }
    return false;
}

void CanBus::enterRecovery(int error, FaultReason reason) {
    const auto key = k_spin_lock(&state_lock_);
    if (status_.state == BusState::Recovering) {
        k_spin_unlock(&state_lock_, key);
        return;
    }
    status_.state = BusState::Recovering;
    status_.last_error = error;
    ++bus_generation_;
    next_recovery_ms_ = nowMs();
    k_spin_unlock(&state_lock_, key);
    const auto publication_key = k_spin_lock(&publication_lock_);
    targets_remaining_ = 0;
    for (std::size_t i = 0; i < motor_count_; ++i)
        published_[i] = {};
    k_spin_unlock(&publication_lock_, publication_key);
    for (std::size_t i = 0; i < motor_count_; ++i)
        motors_[i]->raiseFault({reason, error, motors_[i], nowMs()});
    wake();
}

void CanBus::recoverController(std::uint64_t now_ms) {
    if (now_ms < next_recovery_ms_)
        return;
    if (controller_started_) {
        const int stop_error = can_stop(can_);
        if (stop_error < 0 && stop_error != -EALREADY) {
            const auto key = k_spin_lock(&state_lock_);
            status_.last_error = stop_error;
            k_spin_unlock(&state_lock_, key);
            next_recovery_ms_ = now_ms + options_.recovery_retry_ms;
            return;
        }
        controller_started_ = false;
        processTx(); // can_stop is required to complete/abort a pending callback.
    }
    const auto tx_key = k_spin_lock(&tx_lock_);
    const bool still_busy = in_flight_.busy;
    k_spin_unlock(&tx_lock_, tx_key);
    if (still_busy) {
        next_recovery_ms_ = now_ms + options_.recovery_retry_ms;
        return;
    }
    // A successful stop terminates pending callbacks; starting the controller
    // resets its bus-off state. can_recover() is invalid while stopped.
    const int ret = can_start(can_);
    if (ret < 0 && ret != -EALREADY) {
        next_recovery_ms_ = now_ms + options_.recovery_retry_ms;
        const auto key = k_spin_lock(&state_lock_);
        status_.last_error = ret;
        k_spin_unlock(&state_lock_, key);
        return;
    }
    controller_started_ = true;
    for (std::size_t i = 0; i < motor_count_; ++i)
        motors_[i]->requestDisable();
    const auto key = k_spin_lock(&state_lock_);
    status_.state = BusState::Running;
    k_spin_unlock(&state_lock_, key);
}

void CanBus::ioMain() {
    for (;;) {
        processTx();
        bool overflowed = false;
        {
            const auto key = k_spin_lock(&rx_lock_);
            overflowed = rx_overflowed_;
            rx_overflowed_ = false;
            k_spin_unlock(&rx_lock_, key);
        }
        if (overflowed) {
            const auto key = k_spin_lock(&state_lock_);
            ++status_.rx_overflows;
            k_spin_unlock(&state_lock_, key);
            enterRecovery(-ENOBUFS, FaultReason::RxOverflow);
        }

        // Bound RX work per pass so a saturated receive path cannot starve
        // TX completion, deadline checks, or safety output.
        for (std::size_t n = 0; n < kRxDepth / 2; ++n) {
            RxEvent event{};
            bool available = false;
            const auto key = k_spin_lock(&rx_lock_);
            if (rx_count_ != 0) {
                event = rx_queue_[rx_head_];
                rx_head_ = (rx_head_ + 1) % kRxDepth;
                --rx_count_;
                available = true;
            }
            k_spin_unlock(&rx_lock_, key);
            if (!available)
                break;
            processRx(event);
        }

        const auto now = nowMs();
        if (status().state == BusState::Recovering) {
            recoverController(now);
        }
        else {
            checkDeadlines(now);
            if (status().state == BusState::Running)
                pumpTx(now);
        }

        // The semaphore covers new work; the short timeout also bounds
        // feedback, command, enable, and controller-state checks when idle.
        (void)k_sem_take(&wake_sem_, K_MSEC(2));
    }
}

} // namespace skywalker::motor
