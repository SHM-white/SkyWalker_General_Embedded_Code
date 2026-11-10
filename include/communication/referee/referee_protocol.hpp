#pragma once
#include <cstddef>
#include <cstdint>
namespace skywalker::communication {
// Selected season is fixed at build time by CONFIG_SKYWALKER_REFEREE_RM20XX.
// Shared decode outcome. Wire framing/statistics are owned by RefereeParser.
enum class RefereeDecodeResult { Decoded, UnknownCommand, InvalidLength };
std::uint8_t refereeCrc8(const std::uint8_t *, std::size_t);
std::uint16_t refereeCrc16(const std::uint8_t *, std::size_t);
}
