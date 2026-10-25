#pragma once
#include <cstdint>

namespace skywalker::robotics::vehicle {
enum class DiagnosticOutput { Log, Vofa };
// Change this value only, then rebuild and flash the selected application.
inline constexpr DiagnosticOutput diagnostic_output = DiagnosticOutput::Log;
inline constexpr std::uint32_t diagnostic_baudrate = 115200;
inline constexpr std::uint32_t diagnostic_period_ms = 20;
inline constexpr const char *hardware_revision = "mc02-installed-20261025-candidate";
inline constexpr const char *calibration_revision = "vehicle-20261025-unverified";
} // namespace skywalker::robotics::vehicle
