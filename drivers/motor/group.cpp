#include <drivers/motor/group.hpp>

#include <cerrno>
#include <limits>

namespace skywalker::motor {

void Group::addMember(Motor &member) {
    // Topology is built before any bus starts. Keep the first owner so both
    // groups can detect an overlap during start validation.
    if (member.group_ != nullptr) {
        if (member.group_ != this) {
            member.group_conflict_ = true;
        }
        if (config_error_ == 0) {
            config_error_ = -EINVAL;
        }
        return;
    }

    member.group_ = this;
    if (member_count_ == kMaxGroupMembers) {
        if (config_error_ == 0) {
            config_error_ = -ENOSPC;
        }
        return;
    }
    members_[member_count_++] = &member;
}

int Group::topologyValid() const {
    if (config_error_ < 0) {
        return config_error_;
    }
    if (member_count_ == 0) {
        return -EINVAL;
    }
    for (std::size_t i = 0; i < member_count_; ++i) {
        const Motor &member = *members_[i];
        if (member.group_ != this || member.group_conflict_) {
            return -EINVAL;
        }
        if (member.bus_ == nullptr) {
            return -EACCES;
        }
    }
    return 0;
}

bool Group::permits(std::uint64_t generation) const {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const bool permitted = active_ && enable_generation_ == generation;
    k_spin_unlock(&lock_, key);
    return permitted;
}

bool Group::ready() const {
    if (topologyValid() < 0) {
        return false;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    bool all_ready = config_error_ == 0 && member_count_ != 0 && !stopping_ &&
                     !enable_pending_ && !active_;
    if (all_ready) {
        for (std::size_t i = 0; i < member_count_; ++i) {
            if (!members_[i]->busStarted() || !members_[i]->ready()) {
                all_ready = false;
                break;
            }
        }
    }
    k_spin_unlock(&lock_, key);
    return all_ready;
}

bool Group::active() const {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    const bool permitted = active_;
    const std::uint64_t generation = enable_generation_;
    k_spin_unlock(&lock_, key);
    if (!permitted) {
        return false;
    }
    for (std::size_t i = 0; i < member_count_; ++i) {
        if (!members_[i]->active()) {
            return false;
        }
    }
    return permits(generation);
}

int Group::enable() {
    const int topology_error = topologyValid();
    if (topology_error < 0) {
        return topology_error;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (config_error_ < 0 || member_count_ == 0) {
        const int error = config_error_ < 0 ? config_error_ : -EINVAL;
        k_spin_unlock(&lock_, key);
        return error;
    }
    if (stopping_) {
        k_spin_unlock(&lock_, key);
        return -EAGAIN;
    }
    if (active_ || enable_pending_) {
        k_spin_unlock(&lock_, key);
        return -EALREADY;
    }
    if (enable_generation_ >= std::numeric_limits<std::uint64_t>::max() - 1u) {
        k_spin_unlock(&lock_, key);
        return -EOVERFLOW;
    }

    // Preflight every member before changing the generation or requesting
    // any protocol handshake. All Motor calls here are bounded and local.
    for (std::size_t i = 0; i < member_count_; ++i) {
        Motor &member = *members_[i];
        if (!member.busStarted()) {
            k_spin_unlock(&lock_, key);
            return -EACCES;
        }
        if (!member.ready()) {
            const int error = member.snapshot().state == MotorState::Fault ? -EIO : -EAGAIN;
            k_spin_unlock(&lock_, key);
            return error;
        }
    }

    ++enable_generation_;
    enable_pending_ = true;
    fault_latched_ = false;
    for (std::size_t i = 0; i < member_count_; ++i) {
        prepared_[i] = false;
    }

    int error = 0;
    for (std::size_t i = 0; i < member_count_; ++i) {
        error = members_[i]->requestEnable(enable_generation_);
        if (error < 0) {
            enable_pending_ = false;
            stopping_ = true;
            break;
        }
    }
    k_spin_unlock(&lock_, key);

    if (error < 0) {
        for (std::size_t i = 0; i < member_count_; ++i) {
            members_[i]->requestDisable();
        }
        const k_spinlock_key_t done_key = k_spin_lock(&lock_);
        stopping_ = false;
        k_spin_unlock(&lock_, done_key);
    }
    return error;
}

void Group::disable() {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    active_ = false;
    enable_pending_ = false;
    const bool needs_stop = !stopping_;
    stopping_ = true;
    k_spin_unlock(&lock_, key);

    if (!needs_stop) {
        return;
    }
    for (std::size_t i = 0; i < member_count_; ++i) {
        members_[i]->requestDisable();
    }
    const k_spinlock_key_t done_key = k_spin_lock(&lock_);
    stopping_ = false;
    k_spin_unlock(&lock_, done_key);
}

int Group::clearFault() {
    const int topology_error = topologyValid();
    if (topology_error < 0) {
        return topology_error;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (config_error_ < 0 || member_count_ == 0) {
        const int error = config_error_ < 0 ? config_error_ : -EINVAL;
        k_spin_unlock(&lock_, key);
        return error;
    }
    if (active_ || enable_pending_ || stopping_) {
        k_spin_unlock(&lock_, key);
        return -EBUSY;
    }

    int first_error = 0;
    for (std::size_t i = 0; i < member_count_; ++i) {
        if (members_[i]->snapshot().state != MotorState::Fault) {
            continue;
        }
        const int error = members_[i]->requestClearFault();
        if (error < 0 && first_error == 0) {
            first_error = error;
        }
    }
    k_spin_unlock(&lock_, key);
    return first_error;
}

GroupStatus Group::status() const {
    GroupStatus result{};
    bool prepared[kMaxGroupMembers]{};
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    result.active = active_;
    result.enable_pending = enable_pending_;
    result.enable_generation = enable_generation_;
    result.last_fault = last_fault_;
    for (std::size_t i = 0; i < member_count_; ++i) {
        prepared[i] = prepared_[i];
    }
    k_spin_unlock(&lock_, key);

    result.ready = ready();
    if (result.active) {
        result.active = active();
    }
    for (std::size_t i = 0; i < member_count_; ++i) {
        const Motor &member = *members_[i];
        const bool blocked = result.enable_pending ? !prepared[i]
                             : result.active         ? !member.active()
                                                     : !member.ready();
        if (blocked) {
            result.blocking_member = &member;
            break;
        }
    }
    return result;
}

void Group::trip(Motor &source, const FaultInfo &fault) {
    if (source.group_ != this) {
        return;
    }
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!fault_latched_) {
        last_fault_ = fault;
        last_fault_.source_motor = &source;
        fault_latched_ = true;
    }
    active_ = false;
    enable_pending_ = false;
    const bool needs_stop = !stopping_;
    stopping_ = true;
    k_spin_unlock(&lock_, key);

    if (!needs_stop) {
        return;
    }
    for (std::size_t i = 0; i < member_count_; ++i) {
        members_[i]->requestDisable();
    }
    const k_spinlock_key_t done_key = k_spin_lock(&lock_);
    stopping_ = false;
    k_spin_unlock(&lock_, done_key);
}

void Group::memberPrepared(Motor &member, std::uint64_t enable_generation) {
    const k_spinlock_key_t key = k_spin_lock(&lock_);
    if (!enable_pending_ || stopping_ || enable_generation != enable_generation_) {
        k_spin_unlock(&lock_, key);
        return;
    }

    std::size_t index = member_count_;
    for (std::size_t i = 0; i < member_count_; ++i) {
        if (members_[i] == &member) {
            index = i;
            break;
        }
    }
    if (index == member_count_ || prepared_[index]) {
        k_spin_unlock(&lock_, key);
        return;
    }

    const MotorSnapshot member_status = member.snapshot();
    if (member_status.state != MotorState::Enabling ||
        member_status.enable_generation != enable_generation) {
        k_spin_unlock(&lock_, key);
        return;
    }
    prepared_[index] = true;
    for (std::size_t i = 0; i < member_count_; ++i) {
        if (!prepared_[i]) {
            k_spin_unlock(&lock_, key);
            return;
        }
    }

    // Group lock precedes Motor locks. Permission opens only after every
    // member has entered the same generation.
    for (std::size_t i = 0; i < member_count_; ++i) {
        members_[i]->grantGroupActive(enable_generation);
    }
    for (std::size_t i = 0; i < member_count_; ++i) {
        const MotorSnapshot current = members_[i]->snapshot();
        if (current.state != MotorState::Active ||
            current.enable_generation != enable_generation) {
            enable_pending_ = false;
            stopping_ = true;
            k_spin_unlock(&lock_, key);
            for (std::size_t j = 0; j < member_count_; ++j) {
                members_[j]->requestDisable();
            }
            const k_spinlock_key_t done_key = k_spin_lock(&lock_);
            stopping_ = false;
            k_spin_unlock(&lock_, done_key);
            return;
        }
    }
    active_ = true;
    enable_pending_ = false;
    k_spin_unlock(&lock_, key);
}

} // namespace skywalker::motor
