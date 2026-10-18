#include <array>
#include <cerrno>
#include <cstdint>
#include <communication/referee/referee_parser.hpp>
#include <communication/referee/referee_protocol.hpp>
#include <communication/wire.hpp>
#include <zephyr/ztest.h>

namespace {
using namespace skywalker::communication;
std::array<std::uint8_t, 22> robotFrame() {
    std::array<std::uint8_t, 22> bytes{};
    bytes[0] = 0xa5;
    wire::storeLe16(bytes.data()+1, 13);
    bytes[3] = 4;
    bytes[4] = refereeCrc8(bytes.data(), 4);
    wire::storeLe16(bytes.data()+5, 0x0201);
    bytes[7] = 9; // robot_id
    wire::storeLe16(bytes.data()+7+10, 120);
    bytes[7+12] = 0x05; // gimbal + shooter, chassis denied
    wire::storeLe16(bytes.data()+20, refereeCrc16(bytes.data(), 20));
    return bytes;
}
std::array<std::uint8_t, 23> powerFrame() {
    std::array<std::uint8_t, 23> bytes{};
    bytes[0] = 0xa5;
    wire::storeLe16(bytes.data()+1, 14);
    bytes[4] = refereeCrc8(bytes.data(), 4);
    wire::storeLe16(bytes.data()+5, 0x0202);
    wire::storeLe16(bytes.data()+7+8, 42);
    wire::storeLe16(bytes.data()+21, refereeCrc16(bytes.data(), 21));
    return bytes;
}
}
ZTEST(protocol_referee, test_valid_robot_state_decodes_exact_permissions_and_power_limit) {
    using namespace skywalker::communication;
    RefereeParser parser(RefereeVersion::Rm2026V1_3);
    auto robot = robotFrame();
    zassert_ok(parser.consume(robot.data(), 4, 30));
    zassert_false(parser.state().stamp.valid);
    zassert_ok(parser.consume(robot.data()+4, robot.size()-4, 31));
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
ZTEST_SUITE(protocol_referee, nullptr, nullptr, nullptr, nullptr, nullptr);
