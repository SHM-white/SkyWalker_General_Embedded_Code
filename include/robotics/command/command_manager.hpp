#pragma once
#include <array>
#include <robotics/command/command_arbiter.hpp>
#include <robotics/command/command_source.hpp>
#include <zephyr/kernel.h>

namespace skywalker::robotics {
// Static lifetime including on startup failure. No stop/restart/destruction.
// One startup-thread owner registers/binds/starts. Multiple execution threads
// independently read non-consuming snapshots. All public calls are thread-only.
class CommandManager {
public:
    using Config = CommandArbiter::Config;
    explicit CommandManager(const Config &config);
    CommandManager(const CommandManager &) = delete;
    CommandManager &operator=(const CommandManager &) = delete;
    // Before start only; one source per role. -EEXIST: duplicate, -EINVAL: role,
    // -ENOSPC: capacity, -EBUSY: startup already attempted.
    [[nodiscard]] int registerSource(ICommandSource &source);
    [[nodiscard]] int bindPermissions(IPermissionSource &source);
    // -EINVAL: config/missing operator; -ENODEV: required permission missing;
    // -EALREADY: repeated attempt; otherwise propagates source startup errors.
    [[nodiscard]] int start();
    // 0 copies the current value, possibly the same sequence as a previous read.
    // -EAGAIN before first publication; errors leave out unchanged. Reading never
    // refreshes timestamps: consumers must still enforce command expiry.
    int current(RobotCommand &out) const;
    int snapshot(CommandSnapshot &out) const;
private:
    struct Slot {
        ICommandSource *source = nullptr;
        SourceRole role = SourceRole::Operator;
        SourceSample cached{};
    };
    static void entry(void *self, void *, void *);
    void run();
    void publish(const CommandSnapshot &value);
    int failStart(int error, std::uint32_t reason);
    CommandArbiter arbiter_;
    const bool require_permissions_;
    std::array<Slot, 2> sources_{};
    std::size_t source_count_ = 0;
    IPermissionSource *permissions_ = nullptr;
    RefereeState permission_cache_{};
    SourceDiagnostics permission_diagnostics_{};
    bool start_attempted_ = false;
    mutable k_spinlock output_lock_{};
    CommandSnapshot output_{};
    bool have_output_ = false;
    k_thread thread_{};
    K_KERNEL_STACK_MEMBER(stack_, CONFIG_SKYWALKER_COMMAND_STACK_SIZE);
};
}
