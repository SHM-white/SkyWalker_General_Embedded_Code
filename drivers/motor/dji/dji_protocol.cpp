#include <errno.h>
#include <cstddef>

#include <drivers/motor/dji_protocol.hpp>

namespace skywalker::motor::dji {
namespace {

std::uint16_t readBe16(const std::uint8_t *p)
{
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(p[0]) << 8) | static_cast<std::uint16_t>(p[1]));
}

std::int16_t signedFromBits(std::uint16_t bits)
{
    std::int32_t value = static_cast<std::int32_t>(bits);
    if (value >= 0x8000) value -= 0x10000;
    return static_cast<std::int16_t>(value);
}

} // namespace

bool decodeFeedback(const struct can_frame &frame, RawFeedback &out)
{
    if (frame.dlc != 8u) return false;
    if ((frame.flags & (CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF)) != 0u)
    {
        return false;
    }

    RawFeedback next{};
    next.encoder = readBe16(&frame.data[0]);
    next.speed_rpm = signedFromBits(readBe16(&frame.data[2]));
    next.current_raw = signedFromBits(readBe16(&frame.data[4]));
    next.temperature_raw = frame.data[6];
    out = next;
    return true;
}

int buildCommandFrame(struct can_frame &frame, std::uint16_t command_id, const std::int16_t command_raw[4])
{
    if (command_raw == nullptr) return -EINVAL;
    if (command_id > CAN_STD_ID_MASK) return -ERANGE;

    struct can_frame next{};
    next.id = command_id;
    next.flags = 0u;
    next.dlc = 8u;

    for (std::size_t i = 0; i < 4u; ++i) {
        const std::uint16_t bits = static_cast<std::uint16_t>(command_raw[i]);
        next.data[i * 2u] = static_cast<std::uint8_t>((bits >> 8) & 0xFFu);
        next.data[i * 2u + 1u] = static_cast<std::uint8_t>(bits & 0xFFu);
    }

    frame = next;
    return 0;
}

} // namespace skywalker::motor::dji
