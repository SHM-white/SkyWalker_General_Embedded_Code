#include <core/clock.hpp>
#include <robotics/command/command_manager.hpp>
#include <zephyr/ztest.h>

#include <array>
#include <cerrno>
#include <limits>

namespace {
using namespace skywalker;
using namespace skywalker::robotics;

// The service owns sampling. Tests only publish a bounded one-shot sample or
// select an error; -EAGAIN afterwards exercises the manager's real cache.
class Source final : public ICommandSource {
public:
    explicit Source(SourceRole role = SourceRole::Operator, int startup = 0)
        : role_(role), startup_(startup) {}

    SourceRole role() const override { return role_; }
    int start() override
    {
        ++starts;
        return startup_;
    }

    int sample(SourceSample &out) override
    {
        const auto key = k_spin_lock(&lock_);
        const int result = result_;
        if (result != -EAGAIN) out = pending_;
        if (result == 0) result_ = -EAGAIN;
        k_spin_unlock(&lock_, key);
        return result;
    }

    void publish(const SourceValue &value)
    {
        const auto key = k_spin_lock(&lock_);
        pending_ = {value, {}};
        result_ = 0;
        k_spin_unlock(&lock_, key);
    }

    void fail(int error)
    {
        const auto key = k_spin_lock(&lock_);
        pending_ = {};
        pending_.diagnostics.state = 73;
        pending_.diagnostics.dropped = 4;
        pending_.diagnostics.dropped_available = true;
        result_ = error;
        k_spin_unlock(&lock_, key);
    }

    int starts = 0;

private:
    const SourceRole role_;
    const int startup_;
    k_spinlock lock_{};
    SourceSample pending_{};
    int result_ = -EAGAIN;
};

class Permission final : public IPermissionSource {
public:
    explicit Permission(int startup = 0) : startup_(startup) {}
    int start() override
    {
        ++starts;
        return startup_;
    }

    int sample(std::uint64_t, RefereeState &out, SourceDiagnostics &diagnostics) override
    {
        const auto key = k_spin_lock(&lock_);
        const int result = result_;
        if (result != -EAGAIN) {
            out = pending_;
            diagnostics = diagnostics_;
        }
        if (result == 0) result_ = -EAGAIN;
        k_spin_unlock(&lock_, key);
        return result;
    }

    void publish(std::uint32_t sequence)
    {
        const auto now_ms = core::monotonicTimeUs() / 1000;
        const auto key = k_spin_lock(&lock_);
        pending_ = {};
        const OutputPermission allowed{true, true, {now_ms, sequence, true}};
        pending_.robot.chassis_output = allowed;
        pending_.robot.gimbal_output = allowed;
        pending_.robot.shooter_output = allowed;
        pending_.stamp = allowed.stamp;
        pending_.online = true;
        diagnostics_ = {};
        result_ = 0;
        k_spin_unlock(&lock_, key);
    }

    void fail(int error)
    {
        const auto key = k_spin_lock(&lock_);
        pending_ = {};
        diagnostics_ = {};
        diagnostics_.state = 91;
        result_ = error;
        k_spin_unlock(&lock_, key);
    }

    int starts = 0;

private:
    const int startup_;
    k_spinlock lock_{};
    RefereeState pending_{};
    SourceDiagnostics diagnostics_{};
    int result_ = -EAGAIN;
};

CommandManager::Config noPermission()
{
    CommandManager::Config config{};
    config.require_referee_for_motion = false;
    return config;
}

CommandManager::Config invalidConfig()
{
    auto config = noPermission();
    config.max_gimbal_yaw_rate_rad_s = std::numeric_limits<float>::quiet_NaN();
    return config;
}

CommandManager::Config serviceConfig()
{
    CommandManager::Config config{};
    config.input_timeout_ms = 150;
    config.permission_timeout_ms = 100;
    return config;
}

RemoteState remote(std::uint32_t sequence)
{
    RemoteState value{};
    value.online = true;
    value.left_switch = value.right_switch = RcSwitch::Middle;
    value.analog.right_x = 660;
    value.analog.left_y = 660;
    value.stamp = {core::monotonicTimeUs() / 1000, sequence, true};
    return value;
}

void disabled(const RobotCommand &command)
{
    zassert_equal(command.chassis.mode, ChassisMode::Disabled);
    zassert_equal(command.gimbal.mode, GimbalMode::Disabled);
    zassert_equal(command.shooter.mode, ShooterMode::Disabled);
}

template <typename Predicate>
CommandSnapshot awaitSnapshot(CommandManager &manager, Predicate condition)
{
    CommandSnapshot snapshot{};
    const auto deadline = k_uptime_get() + 1000;
    do {
        if (manager.snapshot(snapshot) == 0 && condition(snapshot)) return snapshot;
        k_sleep(K_MSEC(1));
    } while (k_uptime_get() < deadline);
    zassert_true(false, "service did not publish the expected state within 1 s");
    return snapshot;
}

struct Reader {
    CommandManager *manager = nullptr;
    bool use_snapshot = false;
    k_sem done{};
    std::array<RobotCommand, 16> commands{};
    std::array<int, 16> results{};
    std::array<std::uint32_t, 16> source_sequences{};
};

K_THREAD_STACK_DEFINE(reader_stack_a, 4096);
K_THREAD_STACK_DEFINE(reader_stack_b, 4096);
k_thread reader_thread_a{}, reader_thread_b{};

void readerEntry(void *pointer, void *, void *)
{
    auto &reader = *static_cast<Reader *>(pointer);
    for (std::size_t i = 0; i < reader.commands.size(); ++i) {
        if (reader.use_snapshot) {
            CommandSnapshot value{};
            reader.results[i] = reader.manager->snapshot(value);
            reader.commands[i] = value.decision.command;
            reader.source_sequences[i] = value.observed.remote.stamp.sequence;
        } else {
            reader.results[i] = reader.manager->current(reader.commands[i]);
        }
        // Interleave the two consumers without advancing simulated time. At
        // 100 Hz, repeated 1 ms sleeps consume whole ticks and can expire the
        // one-shot source while this test is asserting fresh read semantics.
        k_yield();
    }
    k_sem_give(&reader.done);
}

void independentReaders(CommandManager &manager, std::uint32_t source_sequence)
{
    Reader current_reader{}, snapshot_reader{};
    current_reader.manager = snapshot_reader.manager = &manager;
    snapshot_reader.use_snapshot = true;
    k_sem_init(&current_reader.done, 0, 1);
    k_sem_init(&snapshot_reader.done, 0, 1);
    k_thread_create(&reader_thread_a, reader_stack_a, K_THREAD_STACK_SIZEOF(reader_stack_a),
                    readerEntry, &current_reader, nullptr, nullptr, 6, 0, K_NO_WAIT);
    k_thread_create(&reader_thread_b, reader_stack_b, K_THREAD_STACK_SIZEOF(reader_stack_b),
                    readerEntry, &snapshot_reader, nullptr, nullptr, 6, 0, K_NO_WAIT);
    zassert_ok(k_sem_take(&current_reader.done, K_SECONDS(1)));
    zassert_ok(k_sem_take(&snapshot_reader.done, K_SECONDS(1)));
    zassert_ok(k_thread_join(&reader_thread_a, K_SECONDS(1)));
    zassert_ok(k_thread_join(&reader_thread_b, K_SECONDS(1)));

    for (const auto *reader : {&current_reader, &snapshot_reader}) {
        for (std::size_t i = 0; i < reader->commands.size(); ++i) {
            zassert_ok(reader->results[i]);
            const auto &value = reader->commands[i];
            zassert_true(value.stamp.valid);
            zassert_equal(value.gimbal.mode, GimbalMode::Rate);
            zassert_equal(value.chassis.mode, ChassisMode::BodyVelocity);
            zassert_within(value.gimbal.yaw_rate_rad_s, -3.0f, 1.0e-6f);
            zassert_within(value.chassis.vx_m_s, 3.0f, 1.0e-6f);
            zassert_equal(value.gimbal.stamp.sequence, value.stamp.sequence);
            zassert_equal(value.chassis.stamp.timestamp_ms, value.stamp.timestamp_ms);
            if (i > 0) zassert_true(value.stamp.sequence >= reader->commands[i - 1].stamp.sequence);
            if (reader->use_snapshot) zassert_equal(reader->source_sequences[i], source_sequence);
        }
    }
}
} // namespace

ZTEST(command_service, test_registration_startup_and_failed_lifetime)
{
    static CommandManager unstarted{noPermission()};
    RobotCommand untouched{};
    untouched.stamp.sequence = 777;
    zassert_equal(unstarted.current(untouched), -EAGAIN);
    zassert_equal(untouched.stamp.sequence, 777);
    CommandSnapshot untouched_snapshot{};
    untouched_snapshot.observed.now_us = 12345;
    zassert_equal(unstarted.snapshot(untouched_snapshot), -EAGAIN);
    zassert_equal(untouched_snapshot.observed.now_us, 12345);

    static CommandManager missing_operator{noPermission()};
    static Source invalid_role{static_cast<SourceRole>(255)};
    static Source after_start;
    zassert_equal(missing_operator.registerSource(invalid_role), -EINVAL);
    zassert_equal(missing_operator.start(), -EINVAL);
    auto failed = awaitSnapshot(missing_operator, [](const auto &) { return true; });
    disabled(failed.decision.command);
    zassert_false(failed.decision.command.stamp.valid);
    zassert_true(failed.decision.reasons() & RcUnavailable);
    zassert_equal(missing_operator.registerSource(after_start), -EBUSY);
    zassert_equal(missing_operator.start(), -EALREADY);

    static CommandManager missing_permission{CommandManager::Config{}};
    static Source needs_permission;
    static Permission too_late;
    zassert_ok(missing_permission.registerSource(needs_permission));
    zassert_equal(missing_permission.start(), -ENODEV);
    zassert_equal(needs_permission.starts, 0);
    failed = awaitSnapshot(missing_permission, [](const auto &) { return true; });
    disabled(failed.decision.command);
    zassert_true(failed.decision.reasons() & PermissionMissing);
    zassert_equal(missing_permission.bindPermissions(too_late), -EBUSY);

    static CommandManager invalid{invalidConfig()};
    static Source invalid_config_source;
    zassert_ok(invalid.registerSource(invalid_config_source));
    zassert_equal(invalid.start(), -EINVAL);
    zassert_equal(invalid_config_source.starts, 0);
    failed = awaitSnapshot(invalid, [](const auto &) { return true; });
    zassert_true(failed.decision.reasons() & InvalidManagerConfig);

    static CommandManager source_failure{noPermission()};
    static Source good_operator;
    static Source duplicate_operator;
    static Source broken_aim{SourceRole::Aim, -EIO};
    static Permission unused_permission;
    zassert_ok(source_failure.registerSource(good_operator));
    zassert_equal(source_failure.registerSource(good_operator), -EEXIST);
    zassert_equal(source_failure.registerSource(duplicate_operator), -EEXIST);
    zassert_ok(source_failure.registerSource(broken_aim));
    zassert_ok(source_failure.bindPermissions(unused_permission));
    zassert_equal(source_failure.bindPermissions(unused_permission), -EEXIST);
    zassert_equal(source_failure.start(), -EIO);
    zassert_equal(good_operator.starts, 1);
    zassert_equal(broken_aim.starts, 1);
    zassert_equal(unused_permission.starts, 0);
    failed = awaitSnapshot(source_failure, [](const auto &) { return true; });
    disabled(failed.decision.command);
    zassert_equal(failed.decision.error, -EIO);
    zassert_equal(failed.vision.error, -EIO);
    zassert_true(failed.decision.reasons() & InvalidInputs);
    zassert_equal(source_failure.start(), -EALREADY);
    zassert_equal(source_failure.registerSource(duplicate_operator), -EBUSY);

    static CommandManager permission_failure{CommandManager::Config{}};
    static Source permission_operator;
    static Permission broken_permission{-EPROTO};
    zassert_ok(permission_failure.registerSource(permission_operator));
    zassert_ok(permission_failure.bindPermissions(broken_permission));
    zassert_equal(permission_failure.start(), -EPROTO);
    zassert_equal(permission_operator.starts, 1);
    zassert_equal(broken_permission.starts, 1);
    failed = awaitSnapshot(permission_failure, [](const auto &) { return true; });
    disabled(failed.decision.command);
    zassert_equal(failed.permission.error, -EPROTO);
    zassert_true(failed.decision.reasons() & PermissionMissing);
}

ZTEST(command_service, test_background_cache_consumers_expiry_and_recovery)
{
    static Source operator_source;
    static Permission permission_source;
    static CommandManager manager{serviceConfig()};
    zassert_ok(manager.registerSource(operator_source));
    zassert_ok(manager.bindPermissions(permission_source));
    zassert_ok(manager.start());
    auto value = awaitSnapshot(manager, [](const auto &s) {
        return s.decision.command.stamp.valid && (s.decision.reasons() & RcUnavailable);
    });
    disabled(value.decision.command);

    const auto original_remote = remote(10);
    operator_source.publish(original_remote);
    permission_source.publish(20);
    value = awaitSnapshot(manager, [](const auto &s) {
        return s.observed.remote.stamp.sequence == 10 && s.decision.command.gimbal.mode == GimbalMode::Rate;
    });
    const auto first_command = value.decision.command.stamp;
    value = awaitSnapshot(manager, [](const auto &s) { return s.remote.sample_error == -EAGAIN; });
    zassert_equal(value.observed.remote.stamp.timestamp_ms, original_remote.stamp.timestamp_ms);
    zassert_equal(value.observed.remote.stamp.sequence, 10);
    independentReaders(manager, 10);
    zassert_equal(operator_source.starts, 1);
    zassert_equal(permission_source.starts, 1);

    value = awaitSnapshot(manager, [](const auto &s) { return s.decision.reasons() & RcUnavailable; });
    disabled(value.decision.command);
    zassert_true(value.observed.remote.stamp.valid);
    zassert_equal(value.observed.remote.stamp.timestamp_ms, original_remote.stamp.timestamp_ms);
    zassert_equal(value.observed.remote.stamp.sequence, 10);
    zassert_true(value.decision.command.stamp.sequence > first_command.sequence);
    zassert_true(value.decision.command.stamp.timestamp_ms > first_command.timestamp_ms);

    operator_source.publish(remote(11));
    permission_source.publish(21);
    awaitSnapshot(manager, [](const auto &s) {
        return s.observed.remote.stamp.sequence == 11 && s.decision.command.gimbal.mode == GimbalMode::Rate;
    });
    operator_source.fail(-EIO);
    value = awaitSnapshot(manager, [](const auto &s) { return s.remote.sample_error == -EIO; });
    disabled(value.decision.command);
    zassert_false(value.observed.remote.stamp.valid);
    zassert_equal(value.remote.state, 73);
    zassert_equal(value.remote.dropped, 4);
    zassert_true(value.remote.dropped_available);

    auto stale = remote(12);
    stale.stamp.timestamp_ms -= serviceConfig().input_timeout_ms + 1;
    operator_source.publish(stale);
    value = awaitSnapshot(manager, [](const auto &s) { return s.observed.remote.stamp.sequence == 12; });
    disabled(value.decision.command);
    zassert_true(value.decision.reasons() & RcUnavailable);
    operator_source.publish(remote(13));
    permission_source.publish(23);
    awaitSnapshot(manager, [](const auto &s) {
        return s.observed.remote.stamp.sequence == 13 && s.decision.command.gimbal.mode == GimbalMode::Rate;
    });

    operator_source.publish(core::Measurement<communication::vision::AimCommand>{});
    value = awaitSnapshot(manager, [](const auto &s) { return s.remote.sample_error == -EINVAL; });
    disabled(value.decision.command);
    zassert_false(value.observed.remote.stamp.valid);
    operator_source.publish(remote(14));
    permission_source.publish(24);
    value = awaitSnapshot(manager, [](const auto &s) {
        return s.observed.remote.stamp.sequence == 14 && s.decision.command.gimbal.mode == GimbalMode::Rate;
    });
    const auto permission_stamp = value.observed.referee.robot.gimbal_output.stamp;
    value = awaitSnapshot(manager, [](const auto &s) { return s.decision.gimbal_reasons & PermissionStale; });
    zassert_equal(value.decision.command.gimbal.mode, GimbalMode::Disabled);
    zassert_equal(value.decision.requested.gimbal.mode, GimbalMode::Rate);
    zassert_false(value.decision.gimbal_reasons & RcUnavailable);
    zassert_equal(value.permission.sample_error, -EAGAIN);
    zassert_equal(value.observed.referee.robot.gimbal_output.stamp.timestamp_ms, permission_stamp.timestamp_ms);
    zassert_equal(value.observed.referee.robot.gimbal_output.stamp.sequence, permission_stamp.sequence);

    operator_source.publish(remote(15));
    permission_source.publish(25);
    awaitSnapshot(manager, [](const auto &s) {
        return s.observed.remote.stamp.sequence == 15 && s.decision.command.gimbal.mode == GimbalMode::Rate;
    });
    permission_source.fail(-EPROTO);
    value = awaitSnapshot(manager, [](const auto &s) { return s.permission.sample_error == -EPROTO; });
    disabled(value.decision.command);
    zassert_false(value.observed.referee.robot.gimbal_output.stamp.valid);
    zassert_equal(value.permission.state, 91);
    zassert_true(value.decision.gimbal_reasons & PermissionMissing);
    operator_source.publish(remote(16));
    permission_source.publish(26);
    awaitSnapshot(manager, [](const auto &s) {
        return s.observed.remote.stamp.sequence == 16 && s.decision.command.gimbal.mode == GimbalMode::Rate;
    });
    zassert_equal(manager.start(), -EALREADY);
    zassert_equal(manager.registerSource(operator_source), -EBUSY);
}

ZTEST_SUITE(command_service, nullptr, nullptr, nullptr, nullptr, nullptr);
