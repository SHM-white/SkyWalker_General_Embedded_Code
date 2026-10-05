#include <drivers/motor/can_bus.hpp>

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <limits>
#include <variant>

#include <drivers/motor/dji_protocol.hpp>
#include <drivers/motor/dm_protocol.hpp>
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
BUILD_ASSERT(CONFIG_SKYWALKER_MOTOR_IO_PRIORITY >= 0 &&
                 CONFIG_SKYWALKER_MOTOR_IO_PRIORITY < CONFIG_NUM_PREEMPT_PRIORITIES,
             "Motor I/O requires a valid preemptible priority");

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
int CanBus::describeMotor(const Motor &motor, std::uint16_t &rx_id, std::uint16_t &tx_id, std::uint8_t &slot) {
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
    k_thread_create(&thread_, thread_stack_, K_KERNEL_STACK_SIZEOF(thread_stack_), threadEntry, this, nullptr, nullptr,
                    CONFIG_SKYWALKER_MOTOR_IO_PRIORITY, 0, K_NO_WAIT);
    thread_started_ = true;
    wake();
    return 0;
}

CommitResult CanBus::commit() {
    const auto state_key = k_spin_lock(&state_lock_);
    const BusState state = status_.state;
    k_spin_unlock(&state_lock_, state_key);
    if (state != BusState::Running && state != BusState::Recovering)
        return {-EACCES, 0};

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
    if (completed.bus_generation != bus_generation_)
        return;
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
    if (completed.purpose == TxPurpose::SafeOutput) {
        updateStopAfterTx(completed);
    } else if (completed.purpose == TxPurpose::Enable || completed.purpose == TxPurpose::ClearFault) {
        Motor &motor = *motors_[units_[completed.unit_index].motor_index];
        motor.markAttemptTxComplete(completed.purpose == TxPurpose::Enable ? Motor::AttemptKind::Enable
                                                                        : Motor::AttemptKind::ClearFault,
                                    completed.protocol_generation, completed.completed_ms, completed.completed_order);
    } else if (completed.purpose == TxPurpose::Probe) {
        Motor &motor = *motors_[units_[completed.unit_index].motor_index];
        const auto motor_key = k_spin_lock(&motor.lock_);
        if (motor.protocol_generation_ == completed.protocol_generation &&
            motor.attempt_ != Motor::AttemptKind::None)
            motor.probe_sent_ = true;
        k_spin_unlock(&motor.lock_, motor_key);
    }
}

void CanBus::checkDeadlines() {
    const auto tx_key = k_spin_lock(&tx_lock_);
    const bool timed_out = in_flight_.busy && !in_flight_.callback_seen &&
                           elapsed(nowMs(), in_flight_.submitted_ms, options_.tx_timeout_ms);
    k_spin_unlock(&tx_lock_, tx_key);
    if (timed_out) {
        enterRecovery(-ETIMEDOUT, FaultReason::TransportError);
        return;
    }
    can_state controller_state{};
    const int controller_error = can_get_state(can_, &controller_state, nullptr);
    if (controller_error < 0 || controller_state == CAN_STATE_BUS_OFF || controller_state == CAN_STATE_STOPPED) {
        enterRecovery(controller_error < 0 ? controller_error :
                      controller_state == CAN_STATE_BUS_OFF ? -ENETUNREACH : -ENETDOWN,
                      FaultReason::TransportError);
        return;
    }
    for (std::size_t i = 0; i < motor_count_; ++i) {
        Motor &motor = *motors_[i];
        auto snapshot = motor.snapshot();
        const auto timing = motor.info().timing;
        auto now = nowMs();
        if (!snapshot.feedback_fresh && snapshot.state == MotorState::Active) {
            motor.raiseFault({FaultReason::FeedbackExpired, -ETIMEDOUT, &motor, now});
            snapshot = motor.snapshot();
        }
        if (isDji(motor)) {
            if (snapshot.enabled_requested && snapshot.feedback_fresh && snapshot.state != MotorState::Active) {
                const auto key = k_spin_lock(&motor.lock_);
                const auto generation = motor.protocol_generation_;
                k_spin_unlock(&motor.lock_, key);
                motor.markPrepared(generation);
            }
        } else {
            const auto key = k_spin_lock(&motor.lock_);
            now = nowMs();
            if (motor.attempt_ != Motor::AttemptKind::None &&
                elapsed(now, motor.attempt_started_ms_, timing.enable_timeout_ms)) {
                motor.snapshot_.last_fault = {FaultReason::EnableTimeout, -ETIMEDOUT, &motor, now};
                motor.snapshot_.output_permitted = false;
                if (motor.protocol_generation_ < std::numeric_limits<std::uint64_t>::max())
                    ++motor.protocol_generation_;
                motor.attempt_ = Motor::AttemptKind::None;
                motor.attempt_tx_done_ = false;
                motor.probe_sent_ = false;
                motor.next_retry_ms_ = now + timing.retry_interval_ms;
                const bool fresh = !elapsed(now, motor.snapshot_.feedback.timestamp_ms, timing.feedback_timeout_ms);
                const bool fault = fresh && motor.snapshot_.native_drive_status_valid &&
                                   dm::isFaultStatus(static_cast<dm::DriveStatus>(motor.snapshot_.native_drive_status));
                motor.snapshot_.state = fault ? MotorState::Fault : fresh ? MotorState::Disabled : MotorState::Offline;
            }
            if (motor.snapshot_.enabled_requested && motor.snapshot_.state != MotorState::Active &&
                motor.attempt_ == Motor::AttemptKind::None && now >= motor.next_retry_ms_) {
                if (motor.protocol_generation_ < std::numeric_limits<std::uint64_t>::max()) {
                    ++motor.protocol_generation_;
                    const bool fault = !elapsed(now, motor.snapshot_.feedback.timestamp_ms, timing.feedback_timeout_ms) &&
                                       motor.snapshot_.native_drive_status_valid &&
                                       dm::isFaultStatus(static_cast<dm::DriveStatus>(motor.snapshot_.native_drive_status));
                    motor.attempt_ = fault ? Motor::AttemptKind::ClearFault : Motor::AttemptKind::Enable;
                    motor.attempt_tx_done_ = false;
                    motor.probe_sent_ = false;
                    motor.attempt_started_ms_ = now;
                    motor.attempt_tx_completed_ms_ = 0;
                    motor.attempt_tx_completed_order_ = 0;
                    motor.snapshot_.state = fault ? MotorState::Fault : MotorState::Enabling;
                    ++motor.snapshot_.retry_count;
                } else {
                    motor.snapshot_.last_fault = {FaultReason::EnableTimeout, -EOVERFLOW, &motor, now};
                    motor.next_retry_ms_ = now + timing.retry_interval_ms;
                }
            }
            if (!motor.snapshot_.enabled_requested && motor.stop_tx_done_ &&
                motor.snapshot_.stop.progress == StopProgress::TxComplete &&
                elapsed(now, motor.stop_first_tx_ms_, timing.enable_timeout_ms))
                motor.snapshot_.stop.progress = StopProgress::Unreachable;
            k_spin_unlock(&motor.lock_, key);
        }
        const auto publication_key = k_spin_lock(&publication_lock_);
        const auto &command = published_[i];
        const auto key = k_spin_lock(&motor.lock_);
        now = nowMs();
        const bool valid = motor.snapshot_.enabled_requested && motor.snapshot_.state == MotorState::Active &&
                           motor.snapshot_.output_permitted && command.valid &&
                           command.cancellation_generation == motor.cancellation_generation_ &&
                           !elapsed(now, command.written_ms, timing.command_timeout_ms) &&
                           !elapsed(now, motor.snapshot_.feedback.timestamp_ms, timing.feedback_timeout_ms) &&
                           (!command.computed_effort ||
                            (command.sampled_enable_generation == motor.snapshot_.enable_generation &&
                             command.revision > motor.invalidated_effort_revision_));
        if (motor.output_command_valid_ != valid)
            targets_remaining_ = unit_count_;
        if (motor.output_command_valid_ && !valid && motor.snapshot_.state == MotorState::Active &&
            command.valid && elapsed(now, command.written_ms, timing.command_timeout_ms))
            motor.snapshot_.last_fault = {FaultReason::CommandExpired, -ETIMEDOUT, &motor, now};
        motor.output_command_valid_ = valid;
        k_spin_unlock(&motor.lock_, key);
        k_spin_unlock(&publication_lock_, publication_key);
    }
}

void CanBus::captureCandidate(std::size_t unit_index, TxPurpose purpose) {
    candidate_ = {};
    candidate_.unit_index = unit_index;
    candidate_.purpose = purpose;
    candidate_.bus_generation = bus_generation_;
    const auto publication_key = k_spin_lock(&publication_lock_);
    candidate_.sequence = published_sequence_;
    for (std::size_t i = 0; i < motor_count_; ++i)
        candidate_.motors[i].command = published_[i];
    k_spin_unlock(&publication_lock_, publication_key);
    const auto &unit = units_[unit_index];
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
        auto &entry = candidate_.motors[i];
        const auto key = k_spin_lock(&motor.lock_);
        entry.included = true;
        entry.enabled_requested = motor.snapshot_.enabled_requested;
        entry.enable_generation = motor.snapshot_.enable_generation;
        entry.cancellation_generation = motor.cancellation_generation_;
        entry.protocol_generation = motor.protocol_generation_;
        entry.invalidated_effort_revision = motor.invalidated_effort_revision_;
        entry.stop_generation = motor.snapshot_.stop.request_generation;
        entry.feedback_ms = motor.snapshot_.feedback.timestamp_ms;
        entry.state = motor.snapshot_.state;
        entry.output_permitted = motor.snapshot_.output_permitted;
        entry.native_position_rad = motor.snapshot_.native_position_rad;
        entry.native_position_valid = motor.snapshot_.native_position_valid;
        entry.safe_action = purpose == TxPurpose::SafeOutput && !entry.enabled_requested;
        k_spin_unlock(&motor.lock_, key);
    }
}

int CanBus::buildNeutral() {
    const auto &unit = units_[candidate_.unit_index];
    const auto &cfg = std::get<dm::Config>(motors_[unit.motor_index]->config_);
    const auto &entry = candidate_.motors[unit.motor_index];
    switch (cfg.mode) {
    case dm::ControlMode::Mit:
        return dm::buildMitFrame(cfg.id, cfg.limits, dm::MitCommand{}, candidate_.frame);
    case dm::ControlMode::Velocity:
        return dm::buildVelocityFrame(cfg.id, 0.0f, candidate_.frame);
    case dm::ControlMode::PositionVelocity:
        if (!entry.native_position_valid)
            return -ENOENT;
        return dm::buildPositionVelocityFrame(cfg.id, entry.native_position_rad, 0.0f, candidate_.frame);
    }
    return -ENOTSUP;
}

int CanBus::buildTarget(std::uint64_t now) {
    const auto &unit = units_[candidate_.unit_index];
    std::int16_t slots[4]{};
    for (std::size_t i = 0; i < motor_count_; ++i) {
        auto &entry = candidate_.motors[i];
        if (!entry.included)
            continue;
        Motor &motor = *motors_[i];
        const auto &command = entry.command;
        const auto timing = motor.info().timing;
        entry.motion = entry.enabled_requested && entry.state == MotorState::Active && entry.output_permitted &&
                       command.valid && command.cancellation_generation == entry.cancellation_generation &&
                       !elapsed(now, command.written_ms, timing.command_timeout_ms) &&
                       !elapsed(now, entry.feedback_ms, timing.feedback_timeout_ms) &&
                       (!command.computed_effort ||
                        (command.sampled_enable_generation == entry.enable_generation &&
                         command.revision > entry.invalidated_effort_revision));
        if (!entry.motion) {
            if (unit.kind == UnitKind::Dm)
                return entry.enabled_requested && entry.state == MotorState::Active ? buildNeutral() : -ENOENT;
            continue;
        }
        entry.safe_action = false;
        if (unit.kind == UnitKind::Dji) {
            const auto &cfg = std::get<dji::Config>(motor.config_);
            dji::Descriptor descriptor{};
            if (command.command.kind != CommandKind::Current || dji::describe(cfg, descriptor) < 0)
                return -EINVAL;
            const float scaled = command.command.primary *
                                 (descriptor.model == dji::Model::M2006C610 ? 10000.0f : 16384.0f) /
                                 descriptor.protocol_current_max_a;
            slots[descriptor.command_slot] = static_cast<std::int16_t>(std::lround(scaled));
        } else {
            const auto &cfg = std::get<dm::Config>(motor.config_);
            switch (command.command.kind) {
            case CommandKind::Torque: {
                dm::MitCommand mit{};
                mit.torque_ff_nm = command.command.primary;
                return dm::buildMitFrame(cfg.id, cfg.limits, mit, candidate_.frame);
            }
            case CommandKind::Mit:
                return dm::buildMitFrame(cfg.id, cfg.limits, command.command.mit, candidate_.frame);
            case CommandKind::Velocity:
                return dm::buildVelocityFrame(cfg.id, command.command.primary, candidate_.frame);
            case CommandKind::PositionVelocity:
                return dm::buildPositionVelocityFrame(cfg.id, command.command.primary, command.command.secondary,
                                                      candidate_.frame);
            default:
                return -ENOTSUP;
            }
        }
    }
    return dji::buildCommandFrame(candidate_.frame, unit.command_id, slots);
}

int CanBus::buildSafety() {
    const auto &unit = units_[candidate_.unit_index];
    if (unit.kind == UnitKind::Dji)
        return buildTarget(nowMs());
    const auto &cfg = std::get<dm::Config>(motors_[unit.motor_index]->config_);
    return dm::buildSpecialFrame(cfg.mode, cfg.id, dm::SpecialCommand::Disable, candidate_.frame);
}

int CanBus::submitCandidate() {
    k_spinlock_key_t motor_keys[kMaxMotors]{};
    for (std::size_t i = 0; i < motor_count_; ++i)
        if (candidate_.motors[i].included)
            motor_keys[i] = k_spin_lock(&motors_[i]->lock_);
    const auto tx_key = k_spin_lock(&tx_lock_);
    const auto now = nowMs();
    int error = in_flight_.busy ? -EBUSY : 0;
    if (next_operation_id_ == std::numeric_limits<std::uint64_t>::max())
        error = -EOVERFLOW;
    if (candidate_.bus_generation != bus_generation_)
        error = -EAGAIN;
    for (std::size_t i = 0; error == 0 && i < motor_count_; ++i) {
        const auto &entry = candidate_.motors[i];
        if (!entry.included)
            continue;
        const Motor &motor = *motors_[i];
        const auto &view = motor.snapshot_;
        if (motor.cancellation_generation_ != entry.cancellation_generation ||
            motor.protocol_generation_ != entry.protocol_generation ||
            view.enabled_requested != entry.enabled_requested || view.state != entry.state ||
            view.enable_generation != entry.enable_generation || view.output_permitted != entry.output_permitted)
            error = -EAGAIN;
        const auto timing = motor.info().timing;
        if (entry.motion && (elapsed(now, entry.command.written_ms, timing.command_timeout_ms) ||
                             elapsed(now, view.feedback.timestamp_ms, timing.feedback_timeout_ms) ||
                             (entry.command.computed_effort &&
                              entry.command.revision <= motor.invalidated_effort_revision_)))
            error = -EAGAIN;
        if (candidate_.purpose == TxPurpose::Enable &&
            (!view.enabled_requested || motor.attempt_ != Motor::AttemptKind::Enable || motor.attempt_tx_done_))
            error = -EAGAIN;
        if (candidate_.purpose == TxPurpose::ClearFault &&
            (!view.enabled_requested || motor.attempt_ != Motor::AttemptKind::ClearFault || motor.attempt_tx_done_))
            error = -EAGAIN;
        if (candidate_.purpose == TxPurpose::Probe &&
            (!view.enabled_requested || motor.attempt_ == Motor::AttemptKind::None || !motor.attempt_tx_done_))
            error = -EAGAIN;
        if (entry.safe_action && view.stop.request_generation != entry.stop_generation)
            error = -EAGAIN;
    }
    const auto sequence = candidate_.purpose == TxPurpose::Target ? candidate_.sequence : 0;
    if (error == 0) {
        in_flight_ = {};
        in_flight_.frame = candidate_.frame;
        in_flight_.purpose = candidate_.purpose;
        in_flight_.unit_index = candidate_.unit_index;
        in_flight_.sequence = sequence;
        in_flight_.bus_generation = candidate_.bus_generation;
        in_flight_.operation_id = next_operation_id_++;
        in_flight_.submitted_ms = now;
        in_flight_.protocol_generation = candidate_.motors[units_[candidate_.unit_index].motor_index].protocol_generation;
        for (std::size_t i = 0; i < motor_count_; ++i)
            if (candidate_.motors[i].included && candidate_.motors[i].safe_action)
                in_flight_.stop_generations[i] = candidate_.motors[i].stop_generation;
        in_flight_.busy = true;
    }
    k_spin_unlock(&tx_lock_, tx_key);
    for (std::size_t i = motor_count_; i-- > 0;)
        if (candidate_.motors[i].included)
            k_spin_unlock(&motors_[i]->lock_, motor_keys[i]);
    if (error != 0) {
        if (error == -EAGAIN)
            wake();
        return error;
    }
    const int result = can_send(can_, &in_flight_.frame, K_NO_WAIT, onTxDone, this);
    if (result < 0) {
        const auto cancel_key = k_spin_lock(&tx_lock_);
        in_flight_.busy = false;
        k_spin_unlock(&tx_lock_, cancel_key);
        const auto state_key = k_spin_lock(&state_lock_);
        status_.last_tx = {true, sequence, candidate_.frame.id, candidate_.purpose, result, nowMs()};
        status_.last_error = result;
        k_spin_unlock(&state_lock_, state_key);
        enterRecovery(result, FaultReason::TransportError);
        return result;
    }
    const auto state_key = k_spin_lock(&state_lock_);
    status_.latest_submitted_sequence = sequence;
    k_spin_unlock(&state_lock_, state_key);
    return 0;
}

void CanBus::updateStopAfterTx(const InFlight &completed) {
    for (std::size_t i = 0; i < motor_count_; ++i)
        if (completed.stop_generations[i] != 0)
            motors_[i]->markStopped(StopProgress::TxComplete, 0, completed.stop_generations[i],
                                   completed.completed_ms, completed.completed_order);
}

bool CanBus::pumpProtocol(std::uint64_t now, bool stops_only) {
    for (std::size_t attempt = 0; attempt < unit_count_; ++attempt) {
        const std::size_t index = (protocol_cursor_ + attempt) % unit_count_;
        const auto &unit = units_[index];
        bool stop = false;
        bool clear = false;
        bool enable = false;
        bool probe = false;
        bool clear_probe = false;
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
            const auto key = k_spin_lock(&motor.lock_);
            if (!motor.snapshot_.enabled_requested) {
                const bool fresh = !elapsed(now, motor.snapshot_.feedback.timestamp_ms, motor.info().timing.feedback_timeout_ms);
                const bool needs_confirmation = motor.snapshot_.stop.progress != StopProgress::DriveConfirmed || !fresh;
                stop |= motor.stop_pending_ ||
                        (!stops_only && unit.kind == UnitKind::Dm && needs_confirmation && now >= motor.next_retry_ms_);
            } else if (!stops_only && unit.kind == UnitKind::Dm) {
                clear = motor.attempt_ == Motor::AttemptKind::ClearFault && !motor.attempt_tx_done_;
                enable = motor.attempt_ == Motor::AttemptKind::Enable && !motor.attempt_tx_done_;
                probe = motor.attempt_ != Motor::AttemptKind::None && motor.attempt_tx_done_ && !motor.probe_sent_;
                clear_probe = probe && motor.attempt_ == Motor::AttemptKind::ClearFault;
            }
            k_spin_unlock(&motor.lock_, key);
        }
        if (!stop && !clear && !enable && !probe)
            continue;
        captureCandidate(index, stop ? TxPurpose::SafeOutput : clear ? TxPurpose::ClearFault :
                                enable ? TxPurpose::Enable : TxPurpose::Probe);
        int result = 0;
        if (stop)
            result = buildSafety();
        else if (clear_probe) {
            const auto &cfg = std::get<dm::Config>(motors_[unit.motor_index]->config_);
            result = dm::buildSpecialFrame(cfg.mode, cfg.id, dm::SpecialCommand::Disable, candidate_.frame);
        } else if (probe)
            result = buildNeutral();
        else {
            const auto &cfg = std::get<dm::Config>(motors_[unit.motor_index]->config_);
            result = dm::buildSpecialFrame(cfg.mode, cfg.id,
                                           clear ? dm::SpecialCommand::ClearError : dm::SpecialCommand::Enable,
                                           candidate_.frame);
        }
        protocol_cursor_ = (index + 1) % unit_count_;
        if (result == -ENOENT) {
            // Without a position measurement, never invent a hold position.
            Motor &motor = *motors_[unit.motor_index];
            const auto key = k_spin_lock(&motor.lock_);
            if (motor.protocol_generation_ == candidate_.motors[unit.motor_index].protocol_generation)
                motor.probe_sent_ = true;
            k_spin_unlock(&motor.lock_, key);
            continue;
        }
        if (result < 0) {
            Motor &motor = *motors_[unit.motor_index];
            const auto key = k_spin_lock(&motor.lock_);
            motor.snapshot_.last_fault = {FaultReason::InvalidCommand, result, &motor, now};
            motor.next_retry_ms_ = now + motor.info().timing.retry_interval_ms;
            motor.attempt_ = Motor::AttemptKind::None;
            k_spin_unlock(&motor.lock_, key);
            continue;
        }
        (void)submitCandidate();
        return true;
    }
    return false;
}

void CanBus::pumpTx(std::uint64_t now) {
    const auto key = k_spin_lock(&tx_lock_);
    const bool busy = in_flight_.busy;
    k_spin_unlock(&tx_lock_, key);
    if (busy)
        return;
    if (pumpProtocol(now, true))
        return;
    if (protocol_before_target_ && pumpProtocol(now, false)) {
        protocol_before_target_ = false;
        return;
    }
    if (pumpTarget(now)) {
        protocol_before_target_ = true;
        return;
    }
    if (pumpProtocol(now, false))
        protocol_before_target_ = false;
}

bool CanBus::pumpTarget(std::uint64_t now) {
    for (std::size_t attempts = 0; attempts < unit_count_; ++attempts) {
        const auto key = k_spin_lock(&publication_lock_);
        if (targets_remaining_ == 0) {
            k_spin_unlock(&publication_lock_, key);
            return false;
        }
        const std::size_t index = target_cursor_;
        target_cursor_ = (target_cursor_ + 1) % unit_count_;
        --targets_remaining_;
        k_spin_unlock(&publication_lock_, key);
        captureCandidate(index, TxPurpose::Target);
        const int built = buildTarget(nowMs());
        if (built == -ENOENT)
            continue;
        if (built < 0) {
            Motor &motor = *motors_[units_[index].motor_index];
            const auto motor_key = k_spin_lock(&motor.lock_);
            motor.snapshot_.last_fault = {FaultReason::InvalidCommand, built, &motor, now};
            k_spin_unlock(&motor.lock_, motor_key);
            continue;
        }
        const int submitted = submitCandidate();
        if (submitted == -EAGAIN || submitted == -EBUSY) {
            const auto retry_key = k_spin_lock(&publication_lock_);
            if (published_sequence_ == candidate_.sequence)
                targets_remaining_ = std::min(unit_count_, targets_remaining_ + 1);
            k_spin_unlock(&publication_lock_, retry_key);
        } else if (submitted == -EOVERFLOW) {
            enterRecovery(submitted, FaultReason::TransportError);
        }
        return true;
    }
    return false;
}

void CanBus::enterRecovery(int error, FaultReason reason) {
    // Only the I/O worker enters recovery. Capture outside the spinlock and
    // before recoverController(): can_start() resets the controller statistics.
    if (status().state == BusState::Recovering)
        return;
    BusRecoverySnapshot recovery{};
    recovery.occurred_ms = nowMs();
    recovery.reason = reason;
    recovery.error = error;
    recovery.query_error = can_get_state(can_, &recovery.controller_state, &recovery.error_counts);
#ifdef CONFIG_CAN_STATS
    recovery.stats_valid = true;
    recovery.bit0 = can_stats_get_bit0_errors(can_);
    recovery.bit1 = can_stats_get_bit1_errors(can_);
    recovery.stuff = can_stats_get_stuff_errors(can_);
    recovery.crc = can_stats_get_crc_errors(can_);
    recovery.form = can_stats_get_form_errors(can_);
    recovery.ack = can_stats_get_ack_errors(can_);
#endif
    const auto key = k_spin_lock(&state_lock_);
    if (status_.state == BusState::Recovering) {
        k_spin_unlock(&state_lock_, key);
        return;
    }
    recovery.count = status_.last_recovery.count + 1;
    recovery.last_tx = status_.last_tx;
    status_.last_recovery = recovery;
    status_.state = BusState::Recovering;
    status_.last_error = error;
    recovery_restarted_ = false;
    ++bus_generation_;
    next_recovery_ms_ = nowMs();
    k_spin_unlock(&state_lock_, key);
    for (std::size_t i = 0; i < motor_count_; ++i)
        motors_[i]->raiseFault({reason, error, motors_[i], nowMs()});
    wake();
}

void CanBus::recoverController(std::uint64_t now_ms) {
    if (now_ms < next_recovery_ms_)
        return;
    if (!recovery_restarted_ && controller_started_) {
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
    if (!recovery_restarted_) {
        // Restart once, then let the controller complete automatic bus-off
        // recovery. can_start() alone does not establish a usable bus.
        const int ret = can_start(can_);
        if (ret < 0 && ret != -EALREADY) {
            next_recovery_ms_ = nowMs() + options_.recovery_retry_ms;
            const auto key = k_spin_lock(&state_lock_);
            status_.last_error = ret;
            k_spin_unlock(&state_lock_, key);
            return;
        }
        controller_started_ = true;
        recovery_restarted_ = true;
        next_recovery_ms_ = nowMs() + options_.recovery_retry_ms;
        return;
    }

    can_state state{};
    const int state_error = can_get_state(can_, &state, nullptr);
    if (state_error < 0 || state == CAN_STATE_BUS_OFF || state == CAN_STATE_STOPPED) {
        // Do not keep restarting a bus-off controller: that prevents its
        // hardware recovery from progressing and can starve other threads.
        if (state_error < 0 || state == CAN_STATE_STOPPED)
            recovery_restarted_ = false;
        next_recovery_ms_ = nowMs() + options_.recovery_retry_ms;
        const auto key = k_spin_lock(&state_lock_);
        status_.last_error = state_error < 0 ? state_error :
            state == CAN_STATE_BUS_OFF ? -ENETUNREACH : -ENETDOWN;
        k_spin_unlock(&state_lock_, key);
        return;
    }
    const auto publication_key = k_spin_lock(&publication_lock_);
    targets_remaining_ = unit_count_;
    k_spin_unlock(&publication_lock_, publication_key);
    const auto key = k_spin_lock(&state_lock_);
    status_.state = BusState::Running;
    status_.last_error = 0;
    k_spin_unlock(&state_lock_, key);
    wake();
}

std::uint32_t CanBus::nextWaitMs(std::uint64_t now) const {
    if (status().state == BusState::Recovering)
        return next_recovery_ms_ <= now ? 0 : static_cast<std::uint32_t>(
            std::min<std::uint64_t>(next_recovery_ms_ - now, options_.recovery_retry_ms));
    std::uint64_t wait = 2;
    const auto due = [&](std::uint64_t deadline) {
        wait = std::min(wait, deadline <= now ? std::uint64_t{0} : deadline - now);
    };
    const auto rx_key = k_spin_lock(&rx_lock_);
    const bool rx_ready = rx_count_ != 0 || rx_overflowed_;
    k_spin_unlock(&rx_lock_, rx_key);
    if (rx_ready)
        return 0;
    const auto tx_key = k_spin_lock(&tx_lock_);
    const bool busy = in_flight_.busy;
    const bool completed = busy && in_flight_.callback_seen;
    if (busy)
        due(in_flight_.submitted_ms + options_.tx_timeout_ms + 1);
    k_spin_unlock(&tx_lock_, tx_key);
    if (completed)
        return 0;
    for (std::size_t i = 0; i < motor_count_; ++i) {
        const Motor &motor = *motors_[i];
        const auto key = k_spin_lock(&motor.lock_);
        const auto &view = motor.snapshot_;
        const auto timing = motor.info().timing;
        if (view.state == MotorState::Active &&
            !elapsed(now, view.feedback.timestamp_ms, timing.feedback_timeout_ms))
            due(view.feedback.timestamp_ms + timing.feedback_timeout_ms + 1);
        if (motor.attempt_ != Motor::AttemptKind::None)
            due(motor.attempt_started_ms_ + timing.enable_timeout_ms + 1);
        if (!busy) {
            if (!view.enabled_requested && motor.stop_pending_)
                wait = 0;
            if (!isDji(motor)) {
                if (view.enabled_requested && view.state != MotorState::Active && motor.attempt_ == Motor::AttemptKind::None)
                    due(motor.next_retry_ms_);
                if (motor.attempt_ != Motor::AttemptKind::None && !motor.attempt_tx_done_)
                    wait = 0;
                if (motor.attempt_ != Motor::AttemptKind::None && motor.attempt_tx_done_ && !motor.probe_sent_)
                    wait = 0;
                if (!view.enabled_requested &&
                    (view.stop.progress != StopProgress::DriveConfirmed ||
                     elapsed(now, view.feedback.timestamp_ms, timing.feedback_timeout_ms)))
                    due(motor.next_retry_ms_);
            }
        }
        k_spin_unlock(&motor.lock_, key);
    }
    if (!busy) {
        const auto key = k_spin_lock(&publication_lock_);
        if (targets_remaining_ != 0)
            wait = 0;
        k_spin_unlock(&publication_lock_, key);
    }
    return static_cast<std::uint32_t>(wait);
}

void CanBus::ioMain() {
    unsigned immediate_passes = 0;
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
            checkDeadlines();
            if (status().state == BusState::Running)
                pumpTx(now);
        }

        const auto wait_ms = nextWaitMs(nowMs());
        if (wait_ms == 0) {
            // k_yield() only gives equal/higher priorities a turn. Bound a
            // sustained ready-work burst so remote RX and logging can run.
            if (++immediate_passes >= 8) {
                immediate_passes = 0;
                k_sleep(K_TICKS(1));
            } else {
                k_yield();
            }
            continue;
        }
        immediate_passes = 0;
        (void)k_sem_take(&wake_sem_, K_MSEC(wait_ms));
    }
}

} // namespace skywalker::motor
