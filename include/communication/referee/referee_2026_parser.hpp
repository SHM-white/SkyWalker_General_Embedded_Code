#pragma once
#include <cstddef>
#include <cstdint>
#include <communication/referee/referee_protocol.hpp>
#include <robotics/messages/referee.hpp>

namespace skywalker::communication {
// RM2026 payload decoder. It knows only this season's command IDs and layouts.
// The caller verifies framing/CRC and passes a candidate state; failures must not mutate it.
class Referee2026Parser final {
public:
    static RefereeDecodeResult decode(std::uint16_t command_id, const std::uint8_t *payload, std::size_t length,
                                      const robotics::MessageStamp &stamp, robotics::RefereeState &state);
};
} // namespace skywalker::communication
