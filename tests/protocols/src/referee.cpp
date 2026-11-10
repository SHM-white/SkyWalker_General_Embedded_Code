#include <array>
#include <cstdint>
#include <communication/referee/referee_parser.hpp>
#include <communication/referee/referee_protocol.hpp>
#if defined(CONFIG_SKYWALKER_REFEREE_RM2026)
#include <communication/referee/referee_2026_parser.hpp>
#elif defined(CONFIG_SKYWALKER_REFEREE_RM2027)
#include <communication/referee/referee_2027_parser.hpp>
#else
#error "Select one referee protocol for the test"
#endif
#include <communication/wire.hpp>
#include <zephyr/ztest.h>

namespace {
using namespace skywalker::communication;
#if defined(CONFIG_SKYWALKER_REFEREE_RM2026)
using SeasonParser = Referee2026Parser;
#else
using SeasonParser = Referee2027Parser;
#endif

std::array<std::uint8_t, 22> robotFrame2026() {
    std::array<std::uint8_t, 22> bytes{};
    bytes[0] = 0xa5;
    wire::storeLe16(bytes.data() + 1, 13);
    bytes[3] = 4;
    bytes[4] = refereeCrc8(bytes.data(), 4);
    wire::storeLe16(bytes.data() + 5, 0x0201);
    bytes[7] = 9;
    wire::storeLe16(bytes.data() + 7 + 10, 120);
    bytes[7 + 12] = 0x05;
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
    bytes[7] = 7;
    wire::storeLe16(bytes.data() + 7 + 10, 200);
    wire::storeFloatLe(bytes.data() + 7 + 12, 30.5f);
    bytes[7 + 16] = 0x05;
    wire::storeLe16(bytes.data() + 24, refereeCrc16(bytes.data(), 24));
    return bytes;
}

auto selectedRobotFrame() {
#if defined(CONFIG_SKYWALKER_REFEREE_RM2026)
    return robotFrame2026();
#else
    return robotFrame2027();
#endif
}

auto incompatibleRobotFrame() {
#if defined(CONFIG_SKYWALKER_REFEREE_RM2026)
    return robotFrame2027();
#else
    return robotFrame2026();
#endif
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
} // namespace

ZTEST(protocol_referee, test_selected_season_robot_status) {
    RefereeParser parser;
    const auto frame = selectedRobotFrame();
    zassert_ok(parser.consume(frame.data(), 4, 30));
    zassert_false(parser.state().stamp.valid);
    zassert_ok(parser.consume(frame.data() + 4, frame.size() - 4, 31));
    const auto &state = parser.state();
    zassert_true(state.online);
#if defined(CONFIG_SKYWALKER_REFEREE_RM2026)
    zassert_equal(state.robot.robot_id, 9u);
    zassert_equal(state.power.chassis_power_limit_w, 120.0f);
    zassert_false(state.robot.bullet_speed_limit_stamp.valid);
#else
    zassert_equal(state.robot.robot_id, 7u);
    zassert_equal(state.power.chassis_power_limit_w, 200.0f);
    zassert_equal(state.robot.bullet_speed_limit_m_s, 30.5f);
    zassert_true(state.robot.bullet_speed_limit_stamp.valid);
    zassert_equal(state.robot.bullet_speed_limit_stamp.timestamp_ms, 31u);
#endif
    zassert_true(state.robot.gimbal_output.enabled);
    zassert_false(state.robot.chassis_output.enabled);
    zassert_true(state.robot.shooter_output.enabled);
    zassert_equal(state.stamp.timestamp_ms, 31u);
    zassert_equal(state.power.limit_stamp.sequence, state.stamp.sequence);
    zassert_equal(parser.stats().valid_frames, 1u);
}

ZTEST(protocol_referee, test_power_buffer_does_not_refresh_permissions) {
    RefereeParser parser;
    const auto robot = selectedRobotFrame();
    const auto power = powerFrame();
    zassert_ok(parser.consume(robot.data(), robot.size(), 50));
    const auto permission_stamp = parser.state().robot.chassis_output.stamp;
    const auto speed_stamp = parser.state().robot.bullet_speed_limit_stamp;
    zassert_ok(parser.consume(power.data(), power.size(), 60));
    const auto &state = parser.state();
    zassert_equal(state.power.buffer_energy_j, 42.0f);
    zassert_equal(state.robot.chassis_output.stamp.sequence, permission_stamp.sequence);
    zassert_equal(state.robot.chassis_output.stamp.timestamp_ms, 50u);
    zassert_equal(state.robot.bullet_speed_limit_stamp.sequence, speed_stamp.sequence);
    zassert_equal(state.power.stamp.timestamp_ms, 60u);
    zassert_equal(state.power.chassis_power_w, 0.0f);
    zassert_equal(state.stamp.sequence, 2u);
}

ZTEST(protocol_referee, test_crc_unknown_command_and_reset) {
    RefereeParser parser;
    const auto frame = selectedRobotFrame();
    auto bad = frame;
    bad[4] ^= 1;
    zassert_ok(parser.consume(bad.data(), bad.size(), 10));
    zassert_true(parser.stats().crc8_errors > 0);
    bad = frame;
    bad[8] ^= 1;
    zassert_ok(parser.consume(bad.data(), bad.size(), 20));
    zassert_true(parser.stats().crc16_errors > 0);
    zassert_false(parser.state().stamp.valid);
    // Valid frame with a command unknown to this selected decoder.
    bad = frame;
    wire::storeLe16(bad.data() + 5, 0x1234);
    wire::storeLe16(bad.data() + bad.size() - 2, refereeCrc16(bad.data(), bad.size() - 2));
    zassert_ok(parser.consume(bad.data(), bad.size(), 25));
    zassert_equal(parser.stats().unknown_commands, 1u);
    zassert_false(parser.state().stamp.valid);
    zassert_ok(parser.consume(frame.data(), frame.size(), 30));
    zassert_true(parser.state().stamp.valid);
    zassert_ok(parser.reset());
    zassert_false(parser.state().stamp.valid);
    zassert_equal(parser.stats().valid_frames, 0u);
}

ZTEST(protocol_referee, test_other_season_is_rejected_without_changing_state) {
    RefereeParser parser;
    const auto frame = selectedRobotFrame();
    const auto other = incompatibleRobotFrame();
    zassert_ok(parser.consume(frame.data(), frame.size(), 10));
    const auto stamp = parser.state().stamp;
    const auto robot_id = parser.state().robot.robot_id;
    zassert_ok(parser.consume(other.data(), other.size(), 20));
    zassert_equal(parser.stats().length_errors, 1u);
    zassert_equal(parser.state().robot.robot_id, robot_id);
    zassert_equal(parser.state().stamp.sequence, stamp.sequence);
    zassert_equal(parser.state().stamp.timestamp_ms, stamp.timestamp_ms);
}

ZTEST(protocol_referee, test_selected_decoder_is_directly_testable) {
    using namespace skywalker::communication;
    const auto frame = selectedRobotFrame();
    const auto other = incompatibleRobotFrame();
    const skywalker::robotics::MessageStamp stamp{123, 5, true};
    skywalker::robotics::RefereeState state{};
    zassert_true(SeasonParser::decode(0x0201, frame.data() + 7, frame.size() - 9, stamp, state) ==
                 RefereeDecodeResult::Decoded);
    const auto robot_id = state.robot.robot_id;
    zassert_true(SeasonParser::decode(0x0201, other.data() + 7, other.size() - 9, stamp, state) ==
                 RefereeDecodeResult::InvalidLength);
    zassert_equal(state.robot.robot_id, robot_id);
    zassert_true(SeasonParser::decode(0x1234, frame.data() + 7, frame.size() - 9, stamp, state) ==
                 RefereeDecodeResult::UnknownCommand);
    zassert_equal(state.robot.robot_id, robot_id);
}
ZTEST_SUITE(protocol_referee, nullptr, nullptr, nullptr, nullptr, nullptr);
