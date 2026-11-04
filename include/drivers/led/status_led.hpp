#pragma once

#include <drivers/led/led_patterns.hpp>
#include <zephyr/devicetree.h>

// C-board and other boards without an enabled led-strip alias use a no-op
// implementation. Neither DEVICE_DT_GET nor led_strip symbols are evaluated.
#if defined(CONFIG_SKYWALKER_STATUS_LED) && DT_NODE_EXISTS(DT_ALIAS(led_strip)) && \
    DT_NODE_HAS_STATUS(DT_ALIAS(led_strip), okay)
#define SKYWALKER_STATUS_LED_AVAILABLE 1
#else
#define SKYWALKER_STATUS_LED_AVAILABLE 0
#endif

#if SKYWALKER_STATUS_LED_AVAILABLE
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#endif

namespace skywalker::led {

#if SKYWALKER_STATUS_LED_AVAILABLE

// Static-lifetime object: its low-priority thread runs for the firmware lifetime.
// Calls never perform SPI I/O on the caller thread. Do not destroy after start().
class StatusLed {
public:
    static constexpr bool available = true;

    StatusLed();
    StatusLed(const StatusLed &) = delete;
    StatusLed &operator=(const StatusLed &) = delete;

    int start();
    // Only pass a static-lifetime pattern and step array. Replaying the same
    // pattern is idempotent unless restart is explicitly requested.
    int play(const LedPattern &pattern, bool restart = false);
    int setStatus(LedStatus status, bool restart = false);
    int setColor(Color color);
    int off() { return setColor({}); }
    int lastError() const;

private:
    enum class RequestKind : std::uint8_t { Solid, Pattern };
    struct Request {
        const LedPattern *pattern = nullptr;
        Color color{};
        RequestKind kind = RequestKind::Solid;
        std::uint32_t version = 0;
    };

    static void entry(void *self, void *, void *);
    void run();
    Request snapshot() const;
    void write(Color color);

    const struct device *const device_;
    atomic_t started_ = 0;
    atomic_t last_error_ = 0;
    mutable k_spinlock lock_{};
    Request request_{};
    k_sem changed_{};
    k_thread thread_{};
    K_KERNEL_STACK_MEMBER(stack_, CONFIG_SKYWALKER_STATUS_LED_STACK_SIZE);
};

#else

// Safe stub for C-board / boards without WS2812 / feature disabled.
// Existing code may call the same API without #ifdef around every use.
class StatusLed {
public:
    static constexpr bool available = false;
    int start() { return 0; }
    int play(const LedPattern &, bool = false) { return 0; }
    int setStatus(LedStatus, bool = false) { return 0; }
    int setColor(Color) { return 0; }
    int off() { return 0; }
    int lastError() const { return 0; }
};

#endif

} // namespace skywalker::led
