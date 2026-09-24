#pragma once

#include <cstddef>
#include <cstdint>

#include <zephyr/kernel.h>

#include <drivers/motor/motor.hpp>

namespace skywalker::motor {

#if defined(CONFIG_SKYWALKER_MOTOR_MAX_GROUP_MEMBERS)
inline constexpr std::size_t kMaxGroupMembers = CONFIG_SKYWALKER_MOTOR_MAX_GROUP_MEMBERS;
#else
inline constexpr std::size_t kMaxGroupMembers = 16;
#endif

struct GroupStatus {
    bool ready = false;
    bool active = false;
    bool enable_pending = false;
    std::uint64_t enable_generation = 0;
    const Motor *blocking_member = nullptr;
    FaultInfo last_fault{};
};

class Group {
public:
    template <class... Others> explicit Group(Motor &first, Others &...others) {
        addMember(first);
        (addMember(others), ...);
    }

    Group(const Group &) = delete;
    Group &operator=(const Group &) = delete;
    Group(Group &&) = delete;
    Group &operator=(Group &&) = delete;

    bool ready() const;
    bool active() const;
    [[nodiscard]] int enable();
    void disable();
    [[nodiscard]] int clearFault();
    GroupStatus status() const;

private:
    friend class Motor;
    friend class CanBus;

    void addMember(Motor &member);
    int topologyValid() const;
    bool permits(std::uint64_t generation) const;
    void trip(Motor &source, const FaultInfo &fault);
    void memberPrepared(Motor &member, std::uint64_t enable_generation);

    Motor *members_[kMaxGroupMembers]{};
    bool prepared_[kMaxGroupMembers]{};
    std::size_t member_count_ = 0;
    int config_error_ = 0;

    mutable struct k_spinlock lock_{};
    bool active_ = false;
    bool enable_pending_ = false;
    bool stopping_ = false;
    bool fault_latched_ = false;
    std::uint64_t enable_generation_ = 0;
    FaultInfo last_fault_{};
};

} // namespace skywalker::motor
