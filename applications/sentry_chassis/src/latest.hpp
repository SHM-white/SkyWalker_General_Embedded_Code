#pragma once
#include <cerrno>
#include <zephyr/kernel.h>
// Application-owned coherent snapshots; no I/O or callbacks while holding the lock.
template <class T> class Latest {
public:
    Latest() {
        k_mutex_init(&mutex_);
    }
    int put(const T &value) {
        if (k_mutex_lock(&mutex_, K_NO_WAIT) < 0)
            return -EAGAIN;
        value_ = value;
        k_mutex_unlock(&mutex_);
        return 0;
    }
    int get(T &value) {
        if (k_mutex_lock(&mutex_, K_NO_WAIT) < 0)
            return -EAGAIN;
        value = value_;
        k_mutex_unlock(&mutex_);
        return 0;
    }

private:
    k_mutex mutex_{};
    T value_{};
};
