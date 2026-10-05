#include <drivers/motor/group.hpp>

#include <cerrno>

namespace skywalker::motor {

Group::Group(std::span<Motor *const> members) {
    if (members.empty())
        config_error_ = -EINVAL;
    for (Motor *member : members) {
        if (member != nullptr)
            addMember(*member);
        else if (config_error_ == 0)
            config_error_ = -EINVAL;
    }
}

void Group::addMember(Motor &member) {
    for (std::size_t i = 0; i < member_count_; ++i) {
        if (members_[i] == &member) {
            if (config_error_ == 0)
                config_error_ = -EINVAL;
            return;
        }
    }
    if (member_count_ == kMaxGroupMembers) {
        if (config_error_ == 0)
            config_error_ = -ENOSPC;
        return;
    }
    members_[member_count_++] = &member;
}

int Group::enable() {
    int first_error = config_error_;
    for (std::size_t i = 0; i < member_count_; ++i) {
        const int error = members_[i]->enable();
        if (error < 0 && first_error == 0)
            first_error = error;
    }
    return first_error;
}

void Group::disable() {
    for (std::size_t i = 0; i < member_count_; ++i)
        (void)members_[i]->disable();
}

GroupStatus Group::status() const {
    GroupStatus result{};
    result.member_count = member_count_;
    for (std::size_t i = 0; i < member_count_; ++i) {
        const auto snapshot = members_[i]->snapshot();
        result.enabled_count += snapshot.enabled_requested;
        result.active_count += snapshot.state == MotorState::Active && snapshot.output_permitted;
        result.offline_count += !snapshot.feedback_fresh;
        result.fault_count += snapshot.state == MotorState::Fault;
    }
    return result;
}

} // namespace skywalker::motor
