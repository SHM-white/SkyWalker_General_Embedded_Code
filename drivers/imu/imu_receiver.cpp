#include <drivers/imu/imu_receiver.hpp>
#ifdef CONFIG_SKYWALKER_IMU_HEATER
#include <core/clock.hpp>
#include <drivers/imu/imu_heater.hpp>
#endif
#include <zephyr/logging/log.h>
#include <cerrno>
#include <limits>

LOG_MODULE_REGISTER(imu_receiver, LOG_LEVEL_INF);

namespace skywalker::imu {
int ImuReceiver::start() {
    if (!config_.poll_interval_us ||
        config_.poll_interval_us > std::uint32_t(std::numeric_limits<std::int32_t>::max()) ||
        config_.priority < 0 || config_.priority >= CONFIG_NUM_PREEMPT_PRIORITIES)
        return -EINVAL;
#ifndef CONFIG_SKYWALKER_IMU_HEATER
    if (heater_)
        return -ENOTSUP;
#endif
    if (!atomic_cas(&started_, 0, 1))
        return -EALREADY;
    Status initial{};
    initial.started = true;
    initial.heater_attached = heater_ != nullptr;
    publish(initial);
    uint32_t options = 0;
#if defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING)
    options |= K_FP_REGS;
#endif
    k_thread_create(&thread_, stack_, K_KERNEL_STACK_SIZEOF(stack_), entry, this, nullptr, nullptr,
                    config_.priority, options, K_NO_WAIT);
    return 0;
}

ImuReceiver::Status ImuReceiver::status() const {
    const auto key = k_spin_lock(&lock_);
    const auto copy = status_;
    k_spin_unlock(&lock_, key);
    return copy;
}

void ImuReceiver::publish(const Status &status) {
    const auto key = k_spin_lock(&lock_);
    status_ = status;
    k_spin_unlock(&lock_, key);
}

void ImuReceiver::entry(void *self, void *, void *) {
    static_cast<ImuReceiver *>(self)->run();
}

void ImuReceiver::run() {
    auto current = status();
#ifdef CONFIG_SKYWALKER_IMU_HEATER
    if (heater_)
        current.heater_init_error = heater_->init();
#endif
    current.init_error = source_.init();
    current.service_error = current.init_error;
    current.init_complete = true;
    LOG_INF("IMU %p init=%d heater init=%d", static_cast<void *>(this), current.init_error,
            current.heater_init_error);
    // Keep servicing after an init error: RS485 can recover its UART internally.
    // Never re-run init here, which could register duplicate UART callbacks.
    for (;;) {
        const int result = source_.service();
        if (result != -EAGAIN) {
            if (result != current.service_error)
                LOG_WRN("IMU %p service=%d", static_cast<void *>(this), result);
            current.service_error = result;
        }
#ifdef CONFIG_SKYWALKER_IMU_HEATER
        if (heater_) {
            if (current.heater_init_error == 0) {
                const auto sample = source_.snapshot();
                auto temperature = sample.sample.temperature_c;
                if (sample.state != State::Running || !(sample.fresh_mask & Temperature))
                    temperature.stamp.valid = false;
                // Also check expiry when service() has no new sample or fails.
                heater_->update(temperature, core::monotonicTimeUs());
            }
            const auto heat = heater_->snapshot();
            if (heat.last_error != current.heater_error || heat.disable_error != current.heater_disable_error)
                LOG_WRN("IMU %p heater=%d disable=%d", static_cast<void *>(this), heat.last_error,
                        heat.disable_error);
            current.heater_error = heat.last_error;
            current.heater_disable_error = heat.disable_error;
            current.heater_duty = heat.duty;
        }
#endif
        publish(current);
        k_usleep(config_.poll_interval_us);
    }
}
} // namespace skywalker::imu
