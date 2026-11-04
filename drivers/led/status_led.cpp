#include <drivers/led/status_led.hpp>

#if SKYWALKER_STATUS_LED_AVAILABLE

#include <errno.h>
#include <zephyr/drivers/led_strip.h>

namespace skywalker::led {

StatusLed::StatusLed() : device_(DEVICE_DT_GET(DT_ALIAS(led_strip))) {}

int StatusLed::start() {
    if (!device_is_ready(device_) || led_strip_length(device_) < 1)
        return -ENODEV;
    if (!atomic_cas(&started_, 0, 1))
        return -EALREADY;

    k_sem_init(&changed_, 0, 1);
    k_thread_create(&thread_, stack_, K_KERNEL_STACK_SIZEOF(stack_), entry,
                    this, nullptr, nullptr, CONFIG_SKYWALKER_STATUS_LED_PRIORITY,
                    0, K_NO_WAIT);
    (void)setStatus(LedStatus::Boot);
    return 0;
}

int StatusLed::play(const LedPattern &pattern, bool restart) {
    if (!pattern.steps || !pattern.count)
        return -EINVAL;
    for (std::size_t i = 0; i < pattern.count; ++i) {
        if (pattern.steps[i].duration_ms == 0)
            return -EINVAL;
    }
    if (!atomic_get(&started_))
        return -EAGAIN;

    const auto key = k_spin_lock(&lock_);
    const bool changed = request_.kind != RequestKind::Pattern || request_.pattern != &pattern;
    if (changed || restart) {
        request_.kind = RequestKind::Pattern;
        request_.pattern = &pattern;
        ++request_.version;
    }
    k_spin_unlock(&lock_, key);
    if (changed || restart)
        k_sem_give(&changed_);
    return 0;
}

int StatusLed::setStatus(LedStatus status, bool restart) {
    return play(patterns::forStatus(status), restart);
}

int StatusLed::setColor(Color color) {
    if (!atomic_get(&started_))
        return -EAGAIN;

    const auto key = k_spin_lock(&lock_);
    const bool changed = request_.kind != RequestKind::Solid || !(request_.color == color);
    if (changed) {
        request_.kind = RequestKind::Solid;
        request_.pattern = nullptr;
        request_.color = color;
        ++request_.version;
    }
    k_spin_unlock(&lock_, key);
    if (changed)
        k_sem_give(&changed_);
    return 0;
}

int StatusLed::lastError() const {
    return atomic_get(&last_error_);
}

StatusLed::Request StatusLed::snapshot() const {
    const auto key = k_spin_lock(&lock_);
    const auto value = request_;
    k_spin_unlock(&lock_, key);
    return value;
}

void StatusLed::write(Color color) {
    struct led_rgb pixel{};
    pixel.r = color.r;
    pixel.g = color.g;
    pixel.b = color.b;
    atomic_set(&last_error_, led_strip_update_rgb(device_, &pixel, 1));
}

void StatusLed::entry(void *self, void *, void *) {
    static_cast<StatusLed *>(self)->run();
}

void StatusLed::run() {
    std::uint32_t observed_version = UINT32_MAX;
    std::size_t index = 0;
    for (;;) {
        const auto request = snapshot();
        if (request.version != observed_version) {
            observed_version = request.version;
            index = 0;
        }
        if (request.kind == RequestKind::Solid) {
            write(request.color);
            (void)k_sem_take(&changed_, K_FOREVER);
            continue;
        }

        const auto &pattern = *request.pattern;
        const auto &step = pattern.steps[index];
        write(step.color);
        if (!pattern.repeat && index + 1 == pattern.count) {
            // One-shot: retain final color until another request arrives.
            (void)k_sem_take(&changed_, K_FOREVER);
            continue;
        }
        if (k_sem_take(&changed_, K_MSEC(step.duration_ms)) == -EAGAIN)
            index = (index + 1 == pattern.count) ? 0 : index + 1;
    }
}

} // namespace skywalker::led

#endif // SKYWALKER_STATUS_LED_AVAILABLE
