#pragma once

#include <cstddef>
#include <cstdint>

namespace skywalker::led {

struct Color {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
};

constexpr bool operator==(Color a, Color b) {
    return a.r == b.r && a.g == b.g && a.b == b.b;
}

// One color is held for duration_ms; 0 ms steps are invalid.
struct LedStep {
    Color color{};
    std::uint16_t duration_ms = 0;
};

// The step array and the Pattern object must outlive playback.
// Patterns normally live as static constexpr objects in flash.
struct LedPattern {
    const LedStep *steps = nullptr;
    std::size_t count = 0;
    bool repeat = false;

    template <std::size_t N>
    constexpr LedPattern(const LedStep (&items)[N], bool looping) : steps(items), count(N), repeat(looping) {
    }
};

enum class LedStatus : std::uint8_t {
    Boot,
    Initializing,
    Ready,
    Running,
    Disarmed,
    LinkLost,
    Recovering,
    Fault,
    EStop,
    HardwareTest
};

} // namespace skywalker::led
