#include <drivers/motor/motor.hpp>

#include <cmath>
#include <cstdint>
#include <errno.h>
#include <limits>
#include <type_traits>

#include <zephyr/kernel.h>

#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>

namespace skywalker::motor {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;
constexpr std::uint16_t kDjiEncoderTicks = 8192;

std::uint64_t nowMs() {
    const std::int64_t now = k_uptime_get();
    return now <= 0 ? 1u : static_cast<std::uint64_t>(now);
}

bool isFresh(std::uint64_t stamp, std::uint32_t limit, std::uint64_t now) {
    return stamp != 0u && now >= stamp && now - stamp <= limit;
}

bool validTiming(const Timing &timing) {
    return timing.feedback_timeout_ms > 0u && timing.command_timeout_ms > 0u &&
           timing.recovery_stable_ms > 0u && timing.enable_timeout_ms > 0u;
}

float djiProtocolCurrentMax(dji::Model model) {
    switch (model) {
    case dji::Model::M3508C620:
        return 20.0f;
    case dji::Model::M2006C610:
        return 10.0f;
    case dji::Model::GM6020Current:
        return 3.0f;
    }
    return 0.0f;
}

std::int16_t djiRawCurrentMax(dji::Model model) {
    return model == dji::Model::M2006C610 ? 10000 : 16384;
}

bool validDjiModel(dji::Model model) {
    return model == dji::Model::M3508C620 || model == dji::Model::M2006C610 ||
           model == dji::Model::GM6020Current;
}

bool validDmMode(dm::ControlMode mode) {
    return mode == dm::ControlMode::Mit || mode == dm::ControlMode::Velocity ||
           mode == dm::ControlMode::PositionVelocity;
}

int checkCommand(const dji::Config &config, const Command &command) {
    if (command.kind != CommandKind::Current)
        return -ENOTSUP;
    if (!std::isfinite(command.primary))
        return -EINVAL;
    if (std::fabs(command.primary) > config.current_limit_a)
        return -ERANGE;
    return 0;
}

int checkCommand(const dm::Config &config, const Command &command) {
    const dm::Limits &limits = config.limits;
    switch (command.kind) {
    case CommandKind::Torque:
        if (config.mode != dm::ControlMode::Mit)
            return -ENOTSUP;
        if (!std::isfinite(command.primary))
            return -EINVAL;
        return std::fabs(command.primary) <= config.torque_limit_nm ? 0 : -ERANGE;
    case CommandKind::Mit: {
        if (config.mode != dm::ControlMode::Mit)
            return -ENOTSUP;
        const dm::MitCommand &mit = command.mit;
        if (!std::isfinite(mit.position_rad) || !std::isfinite(mit.velocity_rad_s) ||
            !std::isfinite(mit.kp) || !std::isfinite(mit.kd) || !std::isfinite(mit.torque_ff_nm))
            return -EINVAL;
        return std::fabs(mit.position_rad) <= limits.position_max_rad &&
                       std::fabs(mit.velocity_rad_s) <= limits.velocity_max_rad_s &&
                       mit.kp >= 0.0f && mit.kp <= 500.0f && mit.kd >= 0.0f && mit.kd <= 5.0f &&
                       std::fabs(mit.torque_ff_nm) <= config.torque_limit_nm
                   ? 0
                   : -ERANGE;
    }
    case CommandKind::Velocity:
        if (config.mode != dm::ControlMode::Velocity)
            return -ENOTSUP;
        if (!std::isfinite(command.primary))
            return -EINVAL;
        return std::fabs(command.primary) <= limits.velocity_max_rad_s ? 0 : -ERANGE;
    case CommandKind::PositionVelocity:
        if (config.mode != dm::ControlMode::PositionVelocity)
            return -ENOTSUP;
        if (!std::isfinite(command.primary) || !std::isfinite(command.secondary))
            return -EINVAL;
        return std::fabs(command.primary) <= limits.position_max_rad && command.secondary >= 0.0f &&
                       command.secondary <= limits.velocity_max_rad_s
                   ? 0
                   : -ERANGE;
    default:
        return -ENOTSUP;
    }
}

} // namespace

namespace dji {

Config gm6020(const Gm6020Options &options) {
    return {Model::GM6020Current, options.id, options.current_limit_a, 1.0f,
            options.encoder_zero_ticks, options.current_mode_confirmed, options.timing};
}

Config m3508(const M3508Options &options) {
    return {Model::M3508C620, options.id, options.current_limit_a, options.gear_ratio, 0, true, options.timing};
}

Config m2006(const M2006Options &options) {
    return {Model::M2006C610, options.id, options.current_limit_a, options.gear_ratio, 0, true, options.timing};
}

int describe(const Config &config, Descriptor &out) {
    if (!validDjiModel(config.model) || config.id == 0u ||
        config.id > (config.model == Model::GM6020Current ? 7u : 8u))
        return -ERANGE;
    Descriptor next{};
    next.model = config.model;
    next.motor_id = config.id;
    next.feedback_id = static_cast<std::uint16_t>(
        (config.model == Model::GM6020Current ? 0x204u : 0x200u) + config.id);
    next.command_id = config.id <= 4u ? (config.model == Model::GM6020Current ? 0x1FEu : 0x200u)
                                      : (config.model == Model::GM6020Current ? 0x2FEu : 0x1FFu);
    next.command_slot = config.id <= 4u ? config.id - 1u : config.id - 5u;
    next.protocol_current_max_a = djiProtocolCurrentMax(config.model);
    next.configured_current_limit_a = config.current_limit_a;
    next.gear_ratio = config.gear_ratio;
    next.temperature_valid = config.model != Model::M2006C610;
    out = next;
    return 0;
}

} // namespace dji

namespace dm {

namespace {
Config makeConfig(const J4310Options &options, ControlMode mode) {
    return {Model::J4310_2EC_V1_1, mode, options.id, options.master_id,
            {options.position_max_rad, options.velocity_max_rad_s, options.torque_max_nm},
            options.torque_limit_nm, options.timing};
}
} // namespace

Config j4310Mit(const J4310Options &options) {
    return makeConfig(options, ControlMode::Mit);
}

Config j4310Velocity(const J4310Options &options) {
    return makeConfig(options, ControlMode::Velocity);
}

Config j4310PositionVelocity(const J4310Options &options) {
    return makeConfig(options, ControlMode::PositionVelocity);
}

int describe(const Config &config, Descriptor &out) {
    if (config.model != Model::J4310_2EC_V1_1 || !validDmMode(config.mode))
        return -EINVAL;
    if (config.id == 0u || config.id > 15u || config.master_id > CAN_STD_ID_MASK)
        return -ERANGE;
    Descriptor next{};
    next.model = config.model;
    next.mode = config.mode;
    next.motor_id = config.id;
    next.master_id = config.master_id;
    next.control_id = static_cast<std::uint16_t>(config.id +
        (config.mode == ControlMode::Mit ? 0u : config.mode == ControlMode::PositionVelocity ? 0x100u : 0x200u));
    next.limits = config.limits;
    next.torque_limit_nm = config.torque_limit_nm;
    out = next;
    return 0;
}

} // namespace dm

Motor::Motor(dji::Config config) : config_(config), protocol_state_(DjiRuntime{}) {}
Motor::Motor(dm::Config config) : config_(config), protocol_state_(DmRuntime{}) {}

int Motor::validateConfig() const {
    return std::visit([](const auto &config) -> int {
        if (!validTiming(config.timing))
            return -ERANGE;
        using T = std::decay_t<decltype(config)>;
        if constexpr (std::is_same_v<T, dji::Config>) {
#if !defined(CONFIG_SKYWALKER_MOTOR_DJI)
            return -ENOTSUP;
#endif
            if (!validDjiModel(config.model) || config.id == 0u ||
                config.id > (config.model == dji::Model::GM6020Current ? 7u : 8u))
                return -ERANGE;
            if (!std::isfinite(config.current_limit_a) || config.current_limit_a <= 0.0f ||
                config.current_limit_a > djiProtocolCurrentMax(config.model) ||
                !std::isfinite(config.gear_ratio) || config.gear_ratio <= 0.0f ||
                config.encoder_zero_ticks >= kDjiEncoderTicks)
                return -ERANGE;
            if (config.model == dji::Model::GM6020Current && !config.current_mode_confirmed)
                return -EINVAL;
        }
        else {
#if !defined(CONFIG_SKYWALKER_MOTOR_DM)
            return -ENOTSUP;
#endif
            if (config.model != dm::Model::J4310_2EC_V1_1 || !validDmMode(config.mode))
                return -EINVAL;
            const dm::Limits &limits = config.limits;
            if (config.id == 0u || config.id > 15u || config.master_id > CAN_STD_ID_MASK ||
                !std::isfinite(limits.position_max_rad) || limits.position_max_rad <= 0.0f ||
                !std::isfinite(limits.velocity_max_rad_s) || limits.velocity_max_rad_s <= 0.0f ||
                !std::isfinite(limits.torque_max_nm) || limits.torque_max_nm <= 0.0f ||
                !std::isfinite(config.torque_limit_nm) || config.torque_limit_nm <= 0.0f ||
                config.torque_limit_nm > limits.torque_max_nm)
                return -ERANGE;
        }
        return 0;
    }, config_);
}

MotorInfo Motor::info() const {
    return std::visit([](const auto &config) {
        MotorInfo out{};
        out.timing = config.timing;
        using T = std::decay_t<decltype(config)>;
        if constexpr (std::is_same_v<T, dji::Config>) {
            out.capabilities = CommandCurrent | FeedbackPosition | FeedbackVelocity | FeedbackCurrent;
            if (config.model != dji::Model::M2006C610)
                out.capabilities |= FeedbackTemperature;
            if (config.model == dji::Model::GM6020Current && config.gear_ratio == 1.0f)
                out.capabilities |= FeedbackAbsolutePosition;
            out.current_limit_a = config.current_limit_a;
        }
        else {
            out.capabilities = FeedbackPosition | FeedbackVelocity | FeedbackTorque | FeedbackTemperature;
            switch (config.mode) {
            case dm::ControlMode::Mit:
                out.capabilities |= CommandTorque | CommandMit;
                break;
            case dm::ControlMode::Velocity:
                out.capabilities |= CommandVelocity;
                break;
            case dm::ControlMode::PositionVelocity:
                out.capabilities |= CommandPositionVelocity;
                break;
            }
            out.torque_limit_nm = config.torque_limit_nm;
        }
        return out;
    }, config_);
}

MotorSnapshot Motor::snapshot() const {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    MotorSnapshot out = snapshot_;
    const bool started = started_;
    k_spin_unlock(&lock_, key);
    const std::uint64_t now = nowMs();
    out.feedback_fresh = started && isFresh(out.feedback.timestamp_ms, info().timing.feedback_timeout_ms, now);
    if (!out.feedback_fresh)
        out.output_permitted = false;
    return out;
}

bool Motor::feedbackFresh(std::uint64_t now_ms) const {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const std::uint64_t stamp = snapshot_.feedback.timestamp_ms;
    const bool started = started_;
    k_spin_unlock(&lock_, key);
    return started && isFresh(stamp, info().timing.feedback_timeout_ms, now_ms);
}

bool Motor::readyLocked(std::uint64_t now) const {
    const bool drive_disabled = !std::holds_alternative<dm::Config>(config_) ||
                                (snapshot_.native_drive_status_valid &&
                                 snapshot_.native_drive_status == static_cast<std::uint32_t>(dm::DriveStatus::Disabled));
    return started_ && safe_prepared_ && drive_disabled && snapshot_.state == MotorState::Disabled &&
           isFresh(snapshot_.feedback.timestamp_ms, info().timing.feedback_timeout_ms, now) &&
           feedback_stable_since_ms_ != 0u && now >= feedback_stable_since_ms_ &&
           now - feedback_stable_since_ms_ >= info().timing.recovery_stable_ms;
}

bool Motor::ready() const {
    const std::uint64_t now = nowMs();
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const bool ready_now = readyLocked(now);
    k_spin_unlock(&lock_, key);
    return ready_now;
}

bool Motor::active() const {
    const MotorSnapshot view = snapshot();
    return view.state == MotorState::Active && view.output_permitted && view.feedback_fresh &&
           (group_ == nullptr || group_->permits(view.enable_generation));
}

bool Motor::busStarted() const {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const bool result = started_;
    k_spin_unlock(&lock_, key);
    return result;
}

void Motor::wakeBus() {
    if (bus_ != nullptr)
        bus_->wake();
}

void Motor::markStarted() {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    started_ = true;
    safe_pending_ = true;
    snapshot_.state = MotorState::Offline;
    snapshot_.stop.progress = StopProgress::Pending;
    snapshot_.stop.request_generation = 1;
    snapshot_.stop.tx_error = 0;
    safe_first_tx_completed_ms_ = 0;
    safe_tx_completed_order_ = 0;
    feedback_event_order_ = 0;
    k_spin_unlock(&lock_, key);
    wakeBus();
}

int Motor::requestEnable(std::uint64_t generation) {
    if (generation == 0u || generation == std::numeric_limits<std::uint64_t>::max())
        return -EOVERFLOW;
    if (!busStarted())
        return -EACCES;
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const std::uint64_t now = nowMs();
    int ret = 0;
    if (snapshot_.state == MotorState::Enabling || snapshot_.state == MotorState::Active)
        ret = -EALREADY;
    else if (snapshot_.state == MotorState::Fault)
        ret = -EIO;
    else if (!readyLocked(now))
        ret = -EAGAIN;
    else if (std::holds_alternative<dji::Config>(config_) &&
             snapshot_.stop.request_generation == std::numeric_limits<std::uint64_t>::max())
        ret = -EOVERFLOW;
    // A bound controller's latest feedback must still satisfy its limits
    // at the exact transition to Enabling.
    else if ((ret = checkProducerSafetyLocked()) == 0) {
        snapshot_.state = MotorState::Enabling;
        snapshot_.output_permitted = false;
        snapshot_.enable_generation = generation;
        snapshot_.stop.progress = StopProgress::None;
        snapshot_.stop.tx_error = 0;
        staged_.valid = false;
        enable_pending_ = true;
        activated_ms_ = 0;
        enable_tx_done_ = false;
        enable_tx_completed_ms_ = 0;
        enable_tx_completed_order_ = 0;
        enable_requested_at_ms_ = now;
        if (std::holds_alternative<dji::Config>(config_)) {
            ++snapshot_.stop.request_generation;
            snapshot_.stop.progress = StopProgress::Pending;
            safe_prepared_ = false;
            safe_pending_ = true;
            safe_tx_done_ = false;
            safe_tx_completed_ms_ = 0;
            safe_tx_completed_order_ = 0;
            safe_first_tx_completed_ms_ = 0;
        }
    }
    k_spin_unlock(&lock_, key);
    if (ret == 0)
        wakeBus();
    return ret;
}

void Motor::requestDisable() {
    const std::uint64_t now = nowMs();
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    snapshot_.output_permitted = false;
    if (snapshot_.state != MotorState::Fault)
        snapshot_.state = isFresh(snapshot_.feedback.timestamp_ms, info().timing.feedback_timeout_ms, now)
                              ? MotorState::Disabled : MotorState::Offline;
    staged_.valid = false;
    enable_pending_ = false;
    enable_tx_done_ = false;
    enable_tx_completed_order_ = 0;
    clear_pending_ = false;
    clear_tx_done_ = false;
    clear_tx_completed_ms_ = 0;
    clear_tx_completed_order_ = 0;
    activated_ms_ = 0;
    safe_prepared_ = false;
    safe_pending_ = true;
    safe_tx_done_ = false;
    safe_tx_completed_ms_ = 0;
    safe_tx_completed_order_ = 0;
    safe_first_tx_completed_ms_ = 0;
    if (snapshot_.stop.request_generation < std::numeric_limits<std::uint64_t>::max())
        ++snapshot_.stop.request_generation;
    snapshot_.stop.progress = StopProgress::Pending;
    snapshot_.stop.tx_error = 0;
    k_spin_unlock(&lock_, key);
    wakeBus();
}

int Motor::requestClearFault() {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    int ret = 0;
    if (snapshot_.output_permitted || snapshot_.state == MotorState::Active ||
        snapshot_.state == MotorState::Enabling)
        ret = -EBUSY;
    else if (snapshot_.state != MotorState::Fault)
        ret = -EALREADY;
    else if (clear_pending_)
        ret = -EALREADY;
    else {
        clear_pending_ = true;
        clear_tx_done_ = false;
        clear_tx_completed_ms_ = 0;
        clear_tx_completed_order_ = 0;
    }
    k_spin_unlock(&lock_, key);
    if (ret == 0)
        wakeBus();
    return ret;
}

int Motor::enable() {
    if (group_ != nullptr)
        return -EACCES;
    const MotorSnapshot view = snapshot();
    if (view.enable_generation >= std::numeric_limits<std::uint64_t>::max() - 1u)
        return -EOVERFLOW;
    return requestEnable(view.enable_generation + 1u);
}

int Motor::disable() {
    if (group_ != nullptr)
        return -EACCES;
    if (!busStarted())
        return -EACCES;
    requestDisable();
    return 0;
}

int Motor::clearFault() {
    if (group_ != nullptr)
        return -EACCES;
    if (!busStarted())
        return -EACCES;
    return requestClearFault();
}

void Motor::grantGroupActive(std::uint64_t generation) {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (enable_pending_ && snapshot_.state == MotorState::Enabling &&
        snapshot_.enable_generation == generation) {
        snapshot_.state = MotorState::Active;
        snapshot_.output_permitted = true;
        enable_pending_ = false;
        activated_ms_ = nowMs();
    }
    k_spin_unlock(&lock_, key);
}

void Motor::markSafePrepared(std::uint64_t expected_stop_generation) {
    const std::uint64_t now = nowMs();
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const bool stop_confirmed = std::holds_alternative<dji::Config>(config_)
                                    ? snapshot_.stop.progress == StopProgress::TxComplete
                                    : snapshot_.stop.progress == StopProgress::DriveConfirmed;
    if ((expected_stop_generation != 0u &&
         snapshot_.stop.request_generation != expected_stop_generation) || !stop_confirmed) {
        k_spin_unlock(&lock_, key);
        return;
    }
    safe_prepared_ = true;
    safe_pending_ = false;
    safe_tx_done_ = true;
    if (snapshot_.state == MotorState::Offline &&
        isFresh(snapshot_.feedback.timestamp_ms, info().timing.feedback_timeout_ms, now))
        snapshot_.state = MotorState::Disabled;
    k_spin_unlock(&lock_, key);
}

void Motor::markEnableTxComplete(std::uint64_t generation, std::uint64_t completed_ms,
                                 std::uint64_t completed_order) {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (enable_pending_ && snapshot_.state == MotorState::Enabling &&
        snapshot_.enable_generation == generation) {
        enable_tx_done_ = true;
        enable_tx_completed_ms_ = completed_ms;
        enable_tx_completed_order_ = completed_order;
    }
    k_spin_unlock(&lock_, key);
}

void Motor::markClearTxComplete(std::uint64_t completed_ms, std::uint64_t completed_order) {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (clear_pending_ && snapshot_.state == MotorState::Fault) {
        clear_tx_done_ = true;
        clear_tx_completed_ms_ = completed_ms;
        clear_tx_completed_order_ = completed_order;
    }
    k_spin_unlock(&lock_, key);
}

void Motor::markPrepared(std::uint64_t generation) {
    Group *group = nullptr;
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (enable_pending_ && snapshot_.state == MotorState::Enabling &&
        snapshot_.enable_generation == generation) {
        if (group_ == nullptr) {
            snapshot_.state = MotorState::Active;
            snapshot_.output_permitted = true;
            enable_pending_ = false;
            activated_ms_ = nowMs();
        }
        else {
            group = group_;
        }
    }
    k_spin_unlock(&lock_, key);
    if (group != nullptr)
        group->memberPrepared(*this, generation);
}

void Motor::markStopped(StopProgress progress, int tx_error, std::uint64_t request_generation,
                        std::uint64_t completed_ms, std::uint64_t completed_order) {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (snapshot_.stop.request_generation == request_generation) {
        // Repeated DM safety probes belong to the same stop generation. Keep
        // Unreachable or DriveConfirmed visible until newer feedback changes it.
        if ((snapshot_.stop.progress != StopProgress::Unreachable &&
             snapshot_.stop.progress != StopProgress::DriveConfirmed) ||
            progress != StopProgress::TxComplete)
            snapshot_.stop.progress = progress;
        snapshot_.stop.tx_error = tx_error;
        if (progress == StopProgress::TxComplete || progress == StopProgress::DriveConfirmed) {
            if (safe_first_tx_completed_ms_ == 0)
                safe_first_tx_completed_ms_ = completed_ms;
            safe_pending_ = false;
            safe_tx_done_ = true;
            safe_tx_completed_ms_ = completed_ms;
            safe_tx_completed_order_ = completed_order;
        }
    }
    k_spin_unlock(&lock_, key);
}

void Motor::markFaultCleared() {
    const std::uint64_t now = nowMs();
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    clear_pending_ = false;
    clear_tx_done_ = false;
    clear_tx_completed_ms_ = 0;
    clear_tx_completed_order_ = 0;
    if (snapshot_.state == MotorState::Fault) {
        snapshot_.state = isFresh(snapshot_.feedback.timestamp_ms, info().timing.feedback_timeout_ms, now)
                              ? MotorState::Disabled : MotorState::Offline;
        snapshot_.output_permitted = false;
        staged_.valid = false;
        safe_prepared_ = false;
        safe_pending_ = true;
        safe_tx_done_ = false;
        safe_tx_completed_ms_ = 0;
        safe_tx_completed_order_ = 0;
        safe_first_tx_completed_ms_ = 0;
        if (snapshot_.stop.request_generation < std::numeric_limits<std::uint64_t>::max())
            ++snapshot_.stop.request_generation;
        snapshot_.stop.progress = StopProgress::Pending;
        snapshot_.stop.tx_error = 0;
    }
    k_spin_unlock(&lock_, key);
    wakeBus();
}

int Motor::bindProducer(const void *producer, float velocity_abs_max_rad_s, float temperature_max_c,
                        std::uint32_t required_feedback, bool require_position_reference) {
    if (producer == nullptr || !std::isfinite(velocity_abs_max_rad_s) || velocity_abs_max_rad_s <= 0.0f ||
        !std::isfinite(temperature_max_c) || temperature_max_c < 0.0f ||
        (required_feedback & FeedbackVelocity) == 0u ||
        (require_position_reference && (required_feedback & FeedbackPosition) == 0u))
        return -EINVAL;
    if (temperature_max_c > 0.0f)
        required_feedback |= FeedbackTemperature;
    if ((info().capabilities & required_feedback) != required_feedback)
        return -ENOTSUP;
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    int ret = 0;
    if (!started_)
        ret = -EACCES;
    else if (snapshot_.state == MotorState::Active || snapshot_.state == MotorState::Enabling)
        ret = -EBUSY;
    else if (producer_ == producer)
        ret = -EALREADY;
    else if (producer_ != nullptr)
        ret = -EBUSY;
    else {
        producer_ = producer;
        producer_safety_ = {velocity_abs_max_rad_s, temperature_max_c, required_feedback,
                            require_position_reference};
    }
    k_spin_unlock(&lock_, key);
    return ret;
}

int Motor::checkProducerSafetyLocked() const {
    if (producer_ == nullptr)
        return 0;
    const Feedback &feedback = snapshot_.feedback;
    if ((feedback.valid & producer_safety_.required_feedback) != producer_safety_.required_feedback ||
        (producer_safety_.require_position_reference && !snapshot_.position_reference_valid))
        return -ENODATA;
    if (!std::isfinite(feedback.velocity_rad_s) ||
        ((producer_safety_.required_feedback & FeedbackPosition) != 0u &&
         !std::isfinite(feedback.position_rad)) ||
        ((producer_safety_.required_feedback & FeedbackAbsolutePosition) != 0u &&
         !std::isfinite(feedback.absolute_position_rad)))
        return -EINVAL;
    if (std::fabs(feedback.velocity_rad_s) > producer_safety_.velocity_abs_max_rad_s)
        return -ERANGE;
    if (producer_safety_.temperature_max_c <= 0.0f)
        return 0;
    if (!std::isfinite(feedback.temperature_c))
        return -EINVAL;
    if (feedback.temperature_c >= producer_safety_.temperature_max_c)
        return -ERANGE;
    if (std::holds_alternative<dm::Config>(config_)) {
        if (!snapshot_.native_temperatures_valid)
            return -ENODATA;
        if (!std::isfinite(snapshot_.native_mos_temperature_c) ||
            !std::isfinite(snapshot_.native_rotor_temperature_c))
            return -EINVAL;
        if (snapshot_.native_mos_temperature_c >= producer_safety_.temperature_max_c ||
            snapshot_.native_rotor_temperature_c >= producer_safety_.temperature_max_c)
            return -ERANGE;
    }
    return 0;
}

void Motor::rejectControl(int error) {
    if (error < 0 && active())
        raiseFault({FaultReason::ControlRejected, error, this, nowMs()});
}

int Motor::stage(const Command &command, const void *producer) {
    const k_spinlock_key_t identity_key = k_spin_lock(&lock_);
    const bool authorized = producer_ == producer;
    k_spin_unlock(&lock_, identity_key);
    if (!authorized)
        return -EACCES;
    if (!active())
        return -EACCES;
    const int check = std::visit([&](const auto &config) { return checkCommand(config, command); }, config_);
    if (check < 0) {
        raiseFault({FaultReason::InvalidCommand, check, this, nowMs()});
        return check;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    int ret = 0;
    if (producer_ != producer || snapshot_.state != MotorState::Active || !snapshot_.output_permitted)
        ret = -EACCES;
    else if (staged_.revision == std::numeric_limits<std::uint64_t>::max())
        ret = -EOVERFLOW;
    else {
        staged_.command = command;
        staged_.written_ms = nowMs();
        staged_.enable_generation = snapshot_.enable_generation;
        ++staged_.revision;
        staged_.valid = true;
    }
    k_spin_unlock(&lock_, key);
    if (ret == -EOVERFLOW)
        raiseFault({FaultReason::InvalidCommand, ret, this, nowMs()});
    return ret;
}

StagedCommand Motor::copyStaged() const {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const StagedCommand copy = staged_;
    k_spin_unlock(&lock_, key);
    return copy;
}

int Motor::setCurrent(float ampere) {
    Command command{};
    command.kind = CommandKind::Current;
    command.primary = ampere;
    return stage(command);
}

int Motor::setCurrentFrom(const void *producer, float ampere) {
    Command command{};
    command.kind = CommandKind::Current;
    command.primary = ampere;
    return stage(command, producer);
}

int Motor::setTorque(float newton_meter) {
    Command command{};
    command.kind = CommandKind::Torque;
    command.primary = newton_meter;
    return stage(command);
}

int Motor::setTorqueFrom(const void *producer, float newton_meter) {
    Command command{};
    command.kind = CommandKind::Torque;
    command.primary = newton_meter;
    return stage(command, producer);
}

int Motor::setMit(const dm::MitCommand &mit) {
    Command command{};
    command.kind = CommandKind::Mit;
    command.mit = mit;
    return stage(command);
}

int Motor::setVelocity(float rad_s) {
    Command command{};
    command.kind = CommandKind::Velocity;
    command.primary = rad_s;
    return stage(command);
}

int Motor::setPositionVelocity(float rad, float max_rad_s) {
    Command command{};
    command.kind = CommandKind::PositionVelocity;
    command.primary = rad;
    command.secondary = max_rad_s;
    return stage(command);
}

int Motor::reseedPosition(double known_position_rad) {
    if (!std::isfinite(known_position_rad))
        return -EINVAL;
    if (std::fabs(known_position_rad) > static_cast<double>(std::numeric_limits<float>::max()))
        return -ERANGE;
    if ((info().capabilities & FeedbackPosition) == 0u)
        return -ENOTSUP;
    const std::uint64_t now = nowMs();
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    int ret = 0;
    if (snapshot_.state == MotorState::Active || snapshot_.state == MotorState::Enabling)
        ret = -EBUSY;
    else if (!isFresh(snapshot_.feedback.timestamp_ms, info().timing.feedback_timeout_ms, now))
        ret = -EAGAIN;
    else if (snapshot_.reference_generation == std::numeric_limits<std::uint64_t>::max())
        ret = -EOVERFLOW;
    else if (auto *dji_state = std::get_if<DjiRuntime>(&protocol_state_)) {
        const auto &config = std::get<dji::Config>(config_);
        const double ticks = known_position_rad * static_cast<double>(config.gear_ratio) * kDjiEncoderTicks / kTwoPi;
        if (!std::isfinite(ticks) || std::fabs(ticks) >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
            ret = -ERANGE;
        else
            dji_state->total_encoder_ticks = static_cast<std::int64_t>(std::llround(ticks));
    }
    else {
        auto &dm_state = std::get<DmRuntime>(protocol_state_);
        dm_state.position_offset_rad = known_position_rad - dm_state.accumulated_native_rad;
    }
    if (ret == 0) {
        snapshot_.feedback.position_rad = static_cast<float>(known_position_rad);
        snapshot_.feedback.valid |= FeedbackPosition;
        snapshot_.position_reference_valid = true;
        ++snapshot_.reference_generation;
    }
    k_spin_unlock(&lock_, key);
    return ret;
}

int Motor::acceptDjiFeedback(const dji::RawFeedback &raw, std::uint64_t received_ms) {
    if (received_ms == 0u || raw.encoder >= kDjiEncoderTicks || !std::holds_alternative<dji::Config>(config_))
        return -EBADMSG;
    const dji::Config &config = std::get<dji::Config>(config_);
    Feedback next{};
    next.velocity_rad_s = static_cast<float>(raw.speed_rpm) * static_cast<float>(kTwoPi / 60.0) / config.gear_ratio;
    next.valid = FeedbackVelocity;
    if (std::abs(static_cast<int>(raw.current_raw)) <= djiRawCurrentMax(config.model)) {
        next.current_a = static_cast<float>(raw.current_raw) * djiProtocolCurrentMax(config.model) /
                         static_cast<float>(djiRawCurrentMax(config.model));
        next.valid |= FeedbackCurrent;
    }
    if (config.model != dji::Model::M2006C610) {
        next.temperature_c = static_cast<float>(raw.temperature_raw);
        next.valid |= FeedbackTemperature;
    }
    if (config.model == dji::Model::GM6020Current && config.gear_ratio == 1.0f) {
        std::int32_t ticks = static_cast<std::int32_t>(raw.encoder) - config.encoder_zero_ticks;
        if (ticks >= static_cast<std::int32_t>(kDjiEncoderTicks / 2u))
            ticks -= kDjiEncoderTicks;
        if (ticks < -static_cast<std::int32_t>(kDjiEncoderTicks / 2u))
            ticks += kDjiEncoderTicks;
        next.absolute_position_rad = static_cast<float>(ticks * kTwoPi / kDjiEncoderTicks);
        next.valid |= FeedbackAbsolutePosition;
    }
    next.timestamp_ms = received_ms;

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (snapshot_.feedback.timestamp_ms != 0u && received_ms <= snapshot_.feedback.timestamp_ms) {
        k_spin_unlock(&lock_, key);
        return -ESTALE;
    }
    DjiRuntime &state = std::get<DjiRuntime>(protocol_state_);
    const bool gap = state.has_encoder &&
        received_ms - snapshot_.feedback.timestamp_ms > config.timing.feedback_timeout_ms;
    if (gap) {
        if (snapshot_.position_reference_valid &&
            snapshot_.reference_generation < std::numeric_limits<std::uint64_t>::max())
            ++snapshot_.reference_generation;
        snapshot_.position_reference_valid = false;
        feedback_stable_since_ms_ = received_ms;
    }
    if (!state.has_encoder) {
        state.has_encoder = true;
        snapshot_.position_reference_valid = true;
        feedback_stable_since_ms_ = received_ms;
    }
    else if (!gap && snapshot_.position_reference_valid) {
        std::int32_t delta = static_cast<std::int32_t>(raw.encoder) - state.last_encoder;
        if (delta > static_cast<std::int32_t>(kDjiEncoderTicks / 2u))
            delta -= kDjiEncoderTicks;
        if (delta < -static_cast<std::int32_t>(kDjiEncoderTicks / 2u))
            delta += kDjiEncoderTicks;
        state.total_encoder_ticks += delta;
    }
    state.last_encoder = raw.encoder;
    if (snapshot_.position_reference_valid) {
        next.position_rad = static_cast<float>(state.total_encoder_ticks * kTwoPi /
                                                (kDjiEncoderTicks * static_cast<double>(config.gear_ratio)));
        next.valid |= FeedbackPosition;
    }
    snapshot_.feedback = next;
    snapshot_.native_dji_feedback = raw;
    snapshot_.native_dji_feedback.timestamp_ms = received_ms;
    snapshot_.native_dji_feedback_valid = true;
    if (started_ && snapshot_.state == MotorState::Offline && safe_prepared_)
        snapshot_.state = MotorState::Disabled;
    k_spin_unlock(&lock_, key);
    return 0;
}

int Motor::acceptDmFeedback(const dm::DecodedFeedback &decoded, std::uint64_t received_ms,
                            std::uint64_t callback_order) {
    if (received_ms == 0u || callback_order == 0u || !std::holds_alternative<dm::Config>(config_))
        return -EINVAL;
    const dm::Config &config = std::get<dm::Config>(config_);
    if (decoded.raw.motor_id != config.id)
        return -ENOENT;
    Feedback next{};
    next.velocity_rad_s = decoded.velocity_rad_s;
    next.torque_nm = decoded.torque_nm;
    next.temperature_c = static_cast<float>(decoded.raw.rotor_temperature_c);
    next.valid = FeedbackVelocity | FeedbackTorque | FeedbackTemperature;
    next.timestamp_ms = received_ms;

    bool drive_fault = false;
    bool unexpected_disabled = false;
    bool clear_confirmed = false;
    bool retry_disable = false;
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (callback_order <= feedback_event_order_ ||
        (snapshot_.feedback.timestamp_ms != 0u && received_ms < snapshot_.feedback.timestamp_ms)) {
        k_spin_unlock(&lock_, key);
        return -ESTALE;
    }
    DmRuntime &state = std::get<DmRuntime>(protocol_state_);
    const bool gap = state.has_native_position &&
        received_ms - snapshot_.feedback.timestamp_ms > config.timing.feedback_timeout_ms;
    if (gap) {
        if (snapshot_.position_reference_valid &&
            snapshot_.reference_generation < std::numeric_limits<std::uint64_t>::max())
            ++snapshot_.reference_generation;
        snapshot_.position_reference_valid = false;
        feedback_stable_since_ms_ = received_ms;
        state.accumulated_native_rad = 0.0;
    }
    if (!state.has_native_position) {
        state.has_native_position = true;
        state.accumulated_native_rad = 0.0;
        state.position_offset_rad = 0.0;
        snapshot_.position_reference_valid = true;
        feedback_stable_since_ms_ = received_ms;
    }
    else if (!gap && snapshot_.position_reference_valid) {
        const double p_max = static_cast<double>(config.limits.position_max_rad);
        double delta = static_cast<double>(decoded.position_rad) -
                       static_cast<double>(state.last_native_position_rad);
        if (delta > p_max)
            delta -= 2.0 * p_max;
        else if (delta < -p_max)
            delta += 2.0 * p_max;
        state.accumulated_native_rad += delta;
    }
    state.last_native_position_rad = decoded.position_rad;
    if (snapshot_.position_reference_valid) {
        next.position_rad = static_cast<float>(state.position_offset_rad +
                                               state.accumulated_native_rad);
        next.valid |= FeedbackPosition;
    }
    snapshot_.feedback = next;
    feedback_event_order_ = callback_order;
    snapshot_.native_mos_temperature_c = static_cast<float>(decoded.raw.mos_temperature_c);
    snapshot_.native_rotor_temperature_c = static_cast<float>(decoded.raw.rotor_temperature_c);
    snapshot_.native_temperatures_valid = true;
    snapshot_.native_position_rad = decoded.position_rad;
    snapshot_.native_position_valid = true;
    snapshot_.native_drive_status = static_cast<std::uint32_t>(decoded.raw.status);
    snapshot_.native_drive_status_valid = true;
    // RX and TX callbacks may occur in the same uptime millisecond. Confirm
    // only feedback whose callback followed the safety TX callback.
    if (decoded.raw.status == dm::DriveStatus::Disabled && safe_tx_done_ &&
        safe_tx_completed_order_ != 0u && callback_order > safe_tx_completed_order_ &&
        (snapshot_.stop.progress == StopProgress::TxComplete ||
         snapshot_.stop.progress == StopProgress::Unreachable))
        snapshot_.stop.progress = StopProgress::DriveConfirmed;
    if (started_ && snapshot_.state == MotorState::Offline && safe_prepared_ &&
        decoded.raw.status == dm::DriveStatus::Disabled)
        snapshot_.state = MotorState::Disabled;
    unexpected_disabled = snapshot_.state == MotorState::Active && decoded.raw.status == dm::DriveStatus::Disabled;
    drive_fault = dm::isFaultStatus(decoded.raw.status);
    clear_confirmed = clear_pending_ && clear_tx_done_ &&
                      clear_tx_completed_order_ != 0u && callback_order > clear_tx_completed_order_ &&
                      decoded.raw.status == dm::DriveStatus::Disabled;
    if (decoded.raw.status == dm::DriveStatus::Enabled && !snapshot_.output_permitted &&
        !enable_pending_ && !clear_pending_) {
        const bool was_confirmed = safe_prepared_ ||
                                   snapshot_.stop.progress == StopProgress::DriveConfirmed;
        safe_prepared_ = false;
        if (was_confirmed && !safe_pending_ &&
            snapshot_.stop.request_generation < std::numeric_limits<std::uint64_t>::max()) {
            ++snapshot_.stop.request_generation;
            snapshot_.stop.progress = StopProgress::Pending;
            snapshot_.stop.tx_error = 0;
            safe_pending_ = true;
            safe_tx_done_ = false;
            safe_tx_completed_ms_ = 0;
            safe_tx_completed_order_ = 0;
            safe_first_tx_completed_ms_ = 0;
            retry_disable = true;
        }
    }
    k_spin_unlock(&lock_, key);

    if (retry_disable)
        wakeBus();
    if (clear_confirmed)
        markFaultCleared();
    if (drive_fault)
        raiseFault({FaultReason::DriveFault, -EIO, this, received_ms});
    else if (unexpected_disabled)
        raiseFault({FaultReason::UnexpectedDisabled, -EHOSTDOWN, this, received_ms});
    return 0;
}

void Motor::raiseFault(const FaultInfo &fault) {
    FaultInfo record = fault;
    if (record.source_motor == nullptr)
        record.source_motor = this;
    if (record.occurred_ms == 0u)
        record.occurred_ms = nowMs();
    const k_spinlock_key_t check_key = k_spin_lock(&lock_);
    const bool duplicate = snapshot_.state == MotorState::Fault &&
                           snapshot_.last_fault.reason == record.reason &&
                           snapshot_.last_fault.source_motor == record.source_motor;
    k_spin_unlock(&lock_, check_key);
    if (duplicate)
        return;
    if (group_ != nullptr)
        group_->trip(*this, record);
    else
        requestDisable();

    const k_spinlock_key_t key = k_spin_lock(&lock_);
    snapshot_.last_fault = record;
    snapshot_.output_permitted = false;
    staged_.valid = false;
    enable_pending_ = false;
    switch (record.reason) {
    case FaultReason::FeedbackExpired:
    case FaultReason::TransportError:
    case FaultReason::RxOverflow:
        snapshot_.state = MotorState::Offline;
        if (snapshot_.position_reference_valid &&
            snapshot_.reference_generation < std::numeric_limits<std::uint64_t>::max())
            ++snapshot_.reference_generation;
        snapshot_.position_reference_valid = false;
        feedback_stable_since_ms_ = 0;
        break;
    case FaultReason::CommandExpired:
        snapshot_.state = MotorState::Disabled;
        break;
    default:
        snapshot_.state = MotorState::Fault;
        break;
    }
    k_spin_unlock(&lock_, key);
    wakeBus();
}

} // namespace skywalker::motor
