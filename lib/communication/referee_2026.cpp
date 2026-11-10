#include <communication/referee/referee_2026_parser.hpp>
#include <communication/wire.hpp>

namespace skywalker::communication {
RefereeDecodeResult Referee2026Parser::decode(std::uint16_t command_id, const std::uint8_t *payload,
                                              std::size_t length, const robotics::MessageStamp &stamp,
                                              robotics::RefereeState &state) {
    // RoboMaster 2026 V1.3.0, 0x0201: 13-byte robot status.
    if (command_id == 0x0201) {
        if (length != 13)
            return RefereeDecodeResult::InvalidLength;
        state.robot.robot_id = payload[0];
        const std::uint8_t outputs = payload[12];
        state.robot.gimbal_output = {true, (outputs & 0x01) != 0, stamp};
        state.robot.chassis_output = {true, (outputs & 0x02) != 0, stamp};
        state.robot.shooter_output = {true, (outputs & 0x04) != 0, stamp};
        state.power.chassis_power_limit_w = wire::loadLe16(payload + 10);
        state.power.limit_stamp = stamp;
        return RefereeDecodeResult::Decoded;
    }
    // 0x0202: bytes 0..7 are reserved, never report measured power from them.
    if (command_id == 0x0202) {
        if (length != 14)
            return RefereeDecodeResult::InvalidLength;
        state.power.buffer_energy_j = wire::loadLe16(payload + 8);
        state.power.stamp = stamp;
        return RefereeDecodeResult::Decoded;
    }
    return RefereeDecodeResult::UnknownCommand;
}
} // namespace skywalker::communication
