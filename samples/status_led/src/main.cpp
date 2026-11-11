#include <drivers/led/status_led.hpp>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

using namespace skywalker::led;

namespace {

StatusLed status_led;

constexpr Color channels[] = {patterns::red, patterns::green, patterns::blue, patterns::white};

// White PWM ladder: 8-bit channel values as seen by the eye.
constexpr Color white_levels[] = {{16, 16, 16}, {32, 32, 32}, {64, 64, 64}, {128, 128, 128}, {255, 255, 255}};

void hold(Color color, int duration_ms) {
    (void)status_led.setColor(color);
    k_sleep(K_MSEC(duration_ms));
}

} // namespace

int main() {
    if constexpr (!StatusLed::available) {
        printk("status_led: feature disabled or no led-strip alias; nothing to show\n");
        for (;;)
            k_sleep(K_SECONDS(5));
    }

    const int error = status_led.start();
    printk("status_led: start=%d\n", error);
    if (error < 0) {
        printk("status_led: WS2812 device not ready; check SPI6 and the led-strip alias\n");
        for (;;)
            k_sleep(K_SECONDS(5));
    }

    for (;;) {
        printk("status_led: hardware test (red/green/blue/white/off, 300 ms each)\n");
        (void)status_led.setStatus(LedStatus::HardwareTest, true);
        k_sleep(K_MSEC(2000));

        printk("status_led: solid channels at full scale\n");
        for (const auto color : channels)
            hold(color, 700);

        printk("status_led: white ramp 16/32/64/128/255\n");
        for (const auto level : white_levels)
            hold(level, 400);

        (void)status_led.off();
        k_sleep(K_SECONDS(1));
    }
}
