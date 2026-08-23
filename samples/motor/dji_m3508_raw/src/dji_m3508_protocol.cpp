#include "dji_m3508_protocol.hpp"

namespace skywalker::motor::dji {

    static std::uint16_t readBe16(const std::uint8_t *p) {
        return (static_cast<std::uint16_t>(p[0]) << 8) | static_cast<std::uint16_t>(p[1]);
    }

    bool decodeM3508Feedback(const struct can_frame &frame, M3508Feedback &out) {
        if (frame.dlc != 8) {
            return false;
        }

        out.encoder = readBe16(&frame.data[0]);
        out.rpm = static_cast<std::int16_t>(readBe16(&frame.data[2]));
        out.current_raw = static_cast<std::int16_t>(readBe16(&frame.data[4]));
        out.temperature = frame.data[6];

        return true;
    }

    void buildGroupCurrentFrame(can_frame &frame, std::uint16_t command_id, const std::int16_t current[4])
    {
        frame.id = command_id;
        frame.flags = 0;
        frame.dlc = 8;
        for (int i = 0; i < 4; i++)
        {
            frame.data[i * 2] = static_cast<std::uint8_t>((current[i] >> 8) & 0xFF);
            frame.data[i * 2 + 1] = static_cast<std::uint8_t>(current[i] & 0xFF);
        }
    }
}

