#include <array>
#include <cerrno>
#include <cstdint>
#include <communication/referee/referee_parser.hpp>
#include <communication/referee/referee_2026_parser.hpp>
#include <communication/referee/referee_2027_parser.hpp>
#include <communication/referee/referee_protocol.hpp>
#include <communication/wire.hpp>
#include <zephyr/ztest.h>

namespace {
using namespace skywalker::communication;
std::array<std::uint8_t, 22> robotFrame() {
    std::array<std::uint8_t, 22> bytes{};
    bytes[0] = 0xa5;
    wire::storeLe16(bytes.data() + 1, 13);
    bytes[3] = 4;
    bytes[4] = refereeCrc8(bytes.data(), 4);
    wire::storeLe16(bytes.data() + 5, 0x0201);
    bytes[7] = 9; // robot_id
    wire::storeLe16(bytes.data() + 7 + 10, 120);
    bytes[7 + 12] = 0x05; // gimbal + shooter, chassis denied
    wire::storeLe16(bytes.data() + 20, refereeCrc16(bytes.data(), 20));
    return bytes;
}
std::array<std::uint8_t, 26> robotFrame2027() {
    std::array<std::uint8_t, 26> bytes{};
    bytes[0] = 0xa5;
    wire::storeLe16(bytes.data() + 1, 17);
    bytes[3] = 5;
    bytes[4] = refereeCrc8(bytes.data(), 4);
    wire::storeLe16(bytes.data() + 5, 0x0201);
    bytes[7] = 7; // RM2027 sentry robot ID
    wire::storeLe16(bytes.data() + 7 + 10, 200);
    wire::storeFloatLe(bytes.data() + 7 + 12, 30.5f);
    bytes[7 + 16] = 0x05; // gimbal and shooter enabled, chassis disabled
    wire::storeLe16(bytes.data() + 24, refereeCrc16(bytes.data(), 24));
    return bytes;
}
std::array<std::uint8_t, 23> powerFrame() {
    std::array<std::uint8_t, 23> bytes{};
    bytes[0] = 0xa5;
    wire::storeLe16(bytes.data() + 1, 14);
    bytes[4] = refereeCrc8(bytes.data(), 4);
    wire::storeLe16(bytes.data() + 5, 0x0202);
    wire::storeLe16(bytes.data() + 7 + 8, 42);
    wire::storeLe16(bytes.data() + 21, refereeCrc16(bytes.data(), 21));
    return bytes;
}
}
ZTEST(protocol_referee, test_valid_robot_state_decodes_exact_permissions_and_power_limit) {
    using namespace skywalker::communication;
    RefereeParser parser(RefereeVersion::Rm2026V1_3);
    auto robot = robotFrame();
    zassert_ok(parser.consume(robot.data(), 4, 30));
    zassert_false(parser.state().stamp.valid);
    zassert_ok(parser.consume(robot.data() + 4, robot.size() - 4, 31));
    const auto &state = parser.state();
    zassert_true(state.online);
    zassert_equal(state.robot.robot_id, 9u);
    zassert_false(state.robot.chassis_output.enabled);
    zassert_true(state.robot.gimbal_output.enabled);
    zassert_true(state.robot.shooter_output.enabled);
    zassert_equal(state.power.chassis_power_limit_w, 120.0f);
    zassert_equal(state.stamp.timestamp_ms, 31u);
    zassert_equal(parser.stats().valid_frames, 1u);
    zassert_equal(state.power.limit_stamp.sequence, state.stamp.sequence);
    zassert_false(state.robot.bullet_speed_limit_stamp.valid);
}

ZTEST(protocol_referee, test_power_buffer_does_not_refresh_robot_permission) {
    using namespace skywalker::communication;
    RefereeParser parser(RefereeVersion::Rm2026V1_3);
    const auto robot = robotFrame();
    const auto power = powerFrame();
    zassert_ok(parser.consume(robot.data(), robot.size(), 50));
    const auto old = parser.state().robot.chassis_output.stamp;
    zassert_ok(parser.consume(power.data(), power.size(), 60));
    zassert_equal(parser.state().power.buffer_energy_j, 42.0f);
    zassert_equal(parser.state().robot.chassis_output.stamp.sequence, old.sequence);
    zassert_equal(parser.state().robot.chassis_output.stamp.timestamp_ms, 50u);
    zassert_equal(parser.state().power.stamp.timestamp_ms, 60u);
    zassert_equal(parser.state().stamp.sequence, 2u);
}

ZTEST(protocol_referee, test_crc_errors_and_unspecified_version_fail_closed) {
    using namespace skywalker::communication;
    RefereeParser parser(RefereeVersion::Rm2026V1_3);
    auto robot = robotFrame();
    auto bad = robot;
    bad[4] ^= 0x01;
    zassert_ok(parser.consume(bad.data(), bad.size(), 10));
    zassert_true(parser.stats().crc8_errors > 0);
    bad = robot;
    bad[8] ^= 0x01;
    zassert_ok(parser.consume(bad.data(), bad.size(), 20));
    zassert_true(parser.stats().crc16_errors > 0);
    zassert_false(parser.state().stamp.valid);
    zassert_ok(parser.consume(robot.data(), robot.size(), 30));
    zassert_true(parser.state().stamp.valid);
    RefereeParser unknown;
    zassert_ok(unknown.consume(robot.data(), robot.size(), 30));
    zassert_equal(unknown.stats().unknown_commands, 1u);
    zassert_false(unknown.state().stamp.valid);
    zassert_ok(parser.reset());
    zassert_false(parser.state().stamp.valid);
    zassert_equal(parser.stats().valid_frames, 0u);
}
ZTEST(protocol_referee, test_rm2027_robot_status_permissions_and_speed_limit) {
    using namespace skywalker::communication;
    RefereeParser parser(RefereeVersion::Rm2027V1_0);
    const auto robot = robotFrame2027();
    zassert_ok(parser.consume(robot.data(), 9, 100));
    zassert_false(parser.state().stamp.valid);
    zassert_ok(parser.consume(robot.data() + 9, robot.size() - 9, 101));
    const auto &state = parser.state();
    zassert_equal(state.robot.robot_id, 7u);
    zassert_true(state.robot.gimbal_output.enabled);
    zassert_false(state.robot.chassis_output.enabled);
    zassert_true(state.robot.shooter_output.enabled);
    zassert_equal(state.power.chassis_power_limit_w, 200.0f);
    zassert_equal(state.robot.bullet_speed_limit_m_s, 30.5f);
    zassert_true(state.robot.bullet_speed_limit_stamp.valid);
    zassert_equal(state.robot.bullet_speed_limit_stamp.timestamp_ms, 101u);
    zassert_equal(parser.stats().length_errors, 0u);
    zassert_equal(parser.stats().valid_frames, 1u);
}

ZTEST(protocol_referee, test_rm2027_power_buffer_and_field_freshness) {
    using namespace skywalker::communication;
    RefereeParser parser(RefereeVersion::Rm2027V1_0);
    const auto robot = robotFrame2027();
    const auto power = powerFrame();
    zassert_ok(parser.consume(robot.data(), robot.size(), 10));
    const auto permission_stamp = parser.state().robot.chassis_output.stamp;
    const auto speed_stamp = parser.state().robot.bullet_speed_limit_stamp;
    zassert_ok(parser.consume(power.data(), power.size(), 20));
    zassert_equal(parser.state().power.buffer_energy_j, 42.0f);
    zassert_equal(parser.state().power.stamp.timestamp_ms, 20u);
    zassert_equal(parser.state().robot.chassis_output.stamp.sequence, permission_stamp.sequence);
    zassert_equal(parser.state().robot.bullet_speed_limit_stamp.sequence, speed_stamp.sequence);
    zassert_equal(parser.state().power.chassis_power_w, 0.0f); // reserved bytes are not a measurement
}

ZTEST(protocol_referee, test_robot_status_length_is_version_specific) {
    using namespace skywalker::communication;
    const auto old_frame = robotFrame();
    const auto new_frame = robotFrame2027();
    RefereeParser old_parser(RefereeVersion::Rm2026V1_3);
    RefereeParser new_parser(RefereeVersion::Rm2027V1_0);
    zassert_ok(old_parser.consume(new_frame.data(), new_frame.size(), 10));
    zassert_ok(new_parser.consume(old_frame.data(), old_frame.size(), 10));
    zassert_equal(old_parser.stats().length_errors, 1u);
    zassert_equal(new_parser.stats().length_errors, 1u);
    zassert_false(old_parser.state().stamp.valid);
    zassert_false(new_parser.state().stamp.valid);
}
ZTEST(protocol_referee, test_season_decoders_are_independent) {
    using namespace skywalker;
    const auto old_frame = robotFrame();
    const auto new_frame = robotFrame2027();
    const robotics::MessageStamp stamp{123, 5, true};
    robotics::RefereeState old_state{};
    robotics::RefereeState new_state{};

    zassert_true(communication::Referee2026Parser::decode(0x0201, old_frame.data() + 7, 13, stamp, old_state) ==
                 communication::RefereeDecodeResult::Decoded);
    zassert_equal(old_state.robot.robot_id, 9u);
    zassert_true(old_state.robot.gimbal_output.enabled);
    zassert_false(old_state.robot.chassis_output.enabled);
    zassert_false(old_state.robot.bullet_speed_limit_stamp.valid);

    zassert_true(communication::Referee2027Parser::decode(0x0201, new_frame.data() + 7, 17, stamp, new_state) ==
                 communication::RefereeDecodeResult::Decoded);
    zassert_equal(new_state.robot.robot_id, 7u);
    zassert_equal(new_state.robot.bullet_speed_limit_m_s, 30.5f);
    zassert_true(new_state.robot.bullet_speed_limit_stamp.valid);

    // Each decoder independently owns its command lengths; rejects the other
    // version without modifying previously published fields.
    zassert_true(communication::Referee2026Parser::decode(0x0201, new_frame.data() + 7, 17, stamp, old_state) ==
                 communication::RefereeDecodeResult::InvalidLength);
    zassert_true(communication::Referee2027Parser::decode(0x0201, old_frame.data() + 7, 13, stamp, new_state) ==
                 communication::RefereeDecodeResult::InvalidLength);
    zassert_equal(old_state.robot.robot_id, 9u);
    zassert_equal(new_state.robot.robot_id, 7u);
    zassert_true(communication::Referee2026Parser::decode(0x1234, old_frame.data() + 7, 13, stamp, old_state) ==
                 communication::RefereeDecodeResult::UnknownCommand);
    zassert_true(communication::Referee2027Parser::decode(0x1234, new_frame.data() + 7, 17, stamp, new_state) ==
                 communication::RefereeDecodeResult::UnknownCommand);
}

ZTEST_SUITE(protocol_referee, nullptr, nullptr, nullptr, nullptr, nullptr);
