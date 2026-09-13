#pragma once
#include <cstddef>
#include <cstdint>
namespace skywalker::communication {
// Explicit selection: no payload semantics are inferred from frame length alone.
enum class RefereeVersion { Unspecified, Rm2026V1_3 };
std::uint8_t refereeCrc8(const std::uint8_t *,std::size_t);
std::uint16_t refereeCrc16(const std::uint8_t *,std::size_t);
}
