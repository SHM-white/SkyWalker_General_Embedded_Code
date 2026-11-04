#pragma once

#include <drivers/led/led_pattern.hpp>

namespace skywalker::led::patterns {

// Deliberately dim: one indicator pixel on the MC02, not a lighting strip.
inline constexpr Color off{0, 0, 0};
inline constexpr Color red{22, 0, 0};
inline constexpr Color green{0, 16, 0};
inline constexpr Color blue{0, 0, 20};
inline constexpr Color cyan{0, 15, 15};
inline constexpr Color amber{22, 9, 0};
inline constexpr Color purple{12, 0, 18};
inline constexpr Color white{16, 16, 16};

inline constexpr LedStep off_steps[] = {{off, 1000}};
inline constexpr LedPattern Off{off_steps, false};

inline constexpr LedStep boot_steps[] = {{purple, 100}, {off, 100},    {purple, 100},
                                         {off, 100},    {purple, 100}, {off, 100}};
inline constexpr LedPattern Boot{boot_steps, false};

inline constexpr LedStep initializing_steps[] = {{blue, 150}, {off, 850}};
inline constexpr LedPattern Initializing{initializing_steps, true};

inline constexpr LedStep ready_steps[] = {{green, 1000}};
inline constexpr LedPattern Ready{ready_steps, false};

inline constexpr LedStep running_steps[] = {{cyan, 100}, {off, 900}};
inline constexpr LedPattern Running{running_steps, true};

inline constexpr LedStep disarmed_steps[] = {{amber, 1000}};
inline constexpr LedPattern Disarmed{disarmed_steps, false};

inline constexpr LedStep link_lost_steps[] = {{amber, 120}, {off, 120}, {amber, 120}, {off, 640}};
inline constexpr LedPattern LinkLost{link_lost_steps, true};

inline constexpr LedStep recovering_steps[] = {{blue, 350}, {off, 350}};
inline constexpr LedPattern Recovering{recovering_steps, true};

inline constexpr LedStep fault_steps[] = {{red, 150}, {off, 150}};
inline constexpr LedPattern Fault{fault_steps, true};

inline constexpr LedStep estop_steps[] = {{red, 1000}};
inline constexpr LedPattern EStop{estop_steps, false};

inline constexpr LedStep hardware_test_steps[] = {{red, 300}, {green, 300}, {blue, 300}, {white, 300}, {off, 300}};
inline constexpr LedPattern HardwareTest{hardware_test_steps, false};

inline constexpr const LedPattern &forStatus(LedStatus status) {
    switch (status) {
    case LedStatus::Boot:
        return Boot;
    case LedStatus::Initializing:
        return Initializing;
    case LedStatus::Ready:
        return Ready;
    case LedStatus::Running:
        return Running;
    case LedStatus::Disarmed:
        return Disarmed;
    case LedStatus::LinkLost:
        return LinkLost;
    case LedStatus::Recovering:
        return Recovering;
    case LedStatus::Fault:
        return Fault;
    case LedStatus::EStop:
        return EStop;
    case LedStatus::HardwareTest:
        return HardwareTest;
    }
    return Off;
}

} // namespace skywalker::led::patterns
