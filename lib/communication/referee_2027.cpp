#include <communication/referee/referee_2027_parser.hpp>
#include <communication/wire.hpp>

namespace skywalker::communication {
RefereeDecodeResult Referee2027Parser::decode(std::uint16_t command_id, const std::uint8_t *payload,
                                              std::size_t length, const robotics::MessageStamp &stamp,
                                              robotics::RefereeState &state) {
    // RoboMaster 2027 V1.0.0, table 1-12: 17-byte robot status.
    if (command_id == 0x0201) {
        if (length != 17)
            return RefereeDecodeResult::InvalidLength;
        state.robot.robot_id = payload[0];
        state.power.chassis_power_limit_w = wire::loadLe16(payload + 10);
        state.power.limit_stamp = stamp;
        state.robot.bullet_speed_limit_m_s = wire::loadFloatLe(payload + 12);
        state.robot.bullet_speed_limit_stamp = stamp;
        const std::uint8_t outputs = payload[16];
        state.robot.gimbal_output = {true, (outputs & 0x01) != 0, stamp};
        state.robot.chassis_output = {true, (outputs & 0x02) != 0, stamp};
        state.robot.shooter_output = {true, (outputs & 0x04) != 0, stamp};
        return RefereeDecodeResult::Decoded;
    }
    // 0x0202, table 1-13: bytes 0..7 are still reserved.
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
