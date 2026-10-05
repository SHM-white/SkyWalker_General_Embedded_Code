#pragma once

#include <cstddef>
#include <span>

#include <drivers/motor/motor.hpp>

namespace skywalker::motor {

#if defined(CONFIG_SKYWALKER_MOTOR_MAX_GROUP_MEMBERS)
inline constexpr std::size_t kMaxGroupMembers = CONFIG_SKYWALKER_MOTOR_MAX_GROUP_MEMBERS;
#else
inline constexpr std::size_t kMaxGroupMembers = 16;
#endif

struct GroupStatus {
    std::size_t member_count = 0;
    std::size_t enabled_count = 0;
    std::size_t active_count = 0;
    std::size_t offline_count = 0;
    std::size_t fault_count = 0;
};

// A fixed member list for explicit batch operations. It owns no motor state.
class Group {
public:
    template <class... Others> explicit Group(Motor &first, Others &...others) {
        addMember(first);
        (addMember(others), ...);
    }
    explicit Group(std::span<Motor *const> members);
    Group(const Group &) = delete;
    Group &operator=(const Group &) = delete;
    Group(Group &&) = delete;
    Group &operator=(Group &&) = delete;

    [[nodiscard]] int enable();
    void disable();
    GroupStatus status() const;

private:
    void addMember(Motor &member);
    Motor *members_[kMaxGroupMembers]{};
    std::size_t member_count_ = 0;
    int config_error_ = 0;
};

} // namespace skywalker::motor
