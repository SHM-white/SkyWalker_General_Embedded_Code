#pragma once

#include <cerrno>
#include <zephyr/kernel.h>

namespace skywalker::robotics {

// Copy-only, non-consuming cache. Producer-owned stamps stay untouched on both
// publication and reading; transports must check the retained production age.
// Thread context only. Keep snapshots small enough for a bounded spinlock copy.
template <class T> class SnapshotCache {
public:
    SnapshotCache() = default;
    SnapshotCache(const SnapshotCache &) = delete;
    SnapshotCache &operator=(const SnapshotCache &) = delete;

    int publish(const T &value) {
        if (k_is_in_isr()) return -EWOULDBLOCK;
        const auto key = k_spin_lock(&lock_);
        value_ = value;
        available_ = true;
        k_spin_unlock(&lock_, key);
        return 0;
    }

    // Failed reads leave the destination unchanged. Any number of consumers can
    // read the same publication independently without renewing its lifetime.
    int snapshot(T &out) const {
        if (k_is_in_isr()) return -EWOULDBLOCK;
        const auto key = k_spin_lock(&lock_);
        const int ret = available_ ? 0 : -EAGAIN;
        if (ret == 0) out = value_;
        k_spin_unlock(&lock_, key);
        return ret;
    }

private:
    mutable k_spinlock lock_{};
    T value_{};
    bool available_ = false;
};

} // namespace skywalker::robotics
