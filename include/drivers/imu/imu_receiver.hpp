#pragma once
#include <drivers/imu/imu.hpp>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

namespace skywalker::imu {
class ImuHeater;

// Owns the source's acquisition thread and optionally drives its heater.
// Receiver, source, estimator, heater and DMA storage must outlive the worker.
// Do not call source.init/service or heater.init/update/disable from other threads.
// No stop/restart or destruction while running is supported.
class ImuReceiver {
public:
    struct Config {
        std::uint32_t poll_interval_us = 1000;
        int priority = CONFIG_SKYWALKER_IMU_RX_PRIORITY;
    };
    struct Status {
        bool started = false, init_complete = false, heater_attached = false;
        int init_error = 0, service_error = 0, heater_init_error = 0;
        int heater_error = 0, heater_disable_error = 0;
        float heater_duty = 0;
    };
    ImuReceiver(ImuSource &source, const Config &config, ImuHeater *heater = nullptr)
        : source_(source), config_(config), heater_(heater) {
    }
    ImuReceiver(const ImuReceiver &) = delete;
    ImuReceiver &operator=(const ImuReceiver &) = delete;
    // Asynchronous initialization: 0 means the worker was started, not valid data.
    int start();
    // Source computes freshness at read time; no I/O or duplicate sample cache.
    Snapshot snapshot() const {
        return source_.snapshot();
    }
    // Independent worker/heater diagnostics, not atomic with snapshot().
    // service_error retains the latest result other than -EAGAIN.
    Status status() const;

private:
    static void entry(void *, void *, void *);
    void run();
    void publish(const Status &status);
    ImuSource &source_;
    const Config config_;
    ImuHeater *const heater_;
    atomic_t started_ = 0;
    mutable k_spinlock lock_{};
    Status status_{};
    k_thread thread_{};
    K_KERNEL_STACK_MEMBER(stack_, CONFIG_SKYWALKER_IMU_RX_STACK_SIZE);
};
} // namespace skywalker::imu
