#include <array>
#include <cerrno>
#include <cstdint>
#include <limits>
#include <drivers/motor/dji_protocol.hpp>
#include <drivers/motor/dm_protocol.hpp>
#include <zephyr/ztest.h>

namespace {
using namespace skywalker::motor;
constexpr dm::Limits limits{12.5f, 30.0f, 10.0f};
}

ZTEST(motor_protocol, test_dji_command_frame_golden_bytes_and_signed_values) {
    can_frame frame{};
    constexpr std::int16_t command[4] = {0, -1, 1234, -32768};
    zassert_ok(dji::buildCommandFrame(frame, 0x200, command));
    zassert_equal(frame.id, 0x200u);
    zassert_equal(frame.dlc, 8u);
    constexpr std::uint8_t expected[8]={0x00,0x00,0xff,0xff,0x04,0xd2,0x80,0x00};
    zassert_mem_equal(frame.data, expected, sizeof(expected));
    zassert_equal(dji::buildCommandFrame(frame, 0x200, nullptr), -EINVAL);
    zassert_equal(dji::buildCommandFrame(frame, 0x800, command), -ERANGE);
    zassert_mem_equal(frame.data, expected, sizeof(expected));
}

ZTEST(motor_protocol, test_dji_feedback_decode_and_invalid_frame_transaction) {
    can_frame frame{};
    frame.dlc=8;
    constexpr std::uint8_t bytes[8]={0x1a,0xbc,0xff,0x9c,0x80,0x01,55,0};
    for (unsigned i=0; i<8; ++i) frame.data[i]=bytes[i];
    dji::RawFeedback decoded{};
    zassert_true(dji::decodeFeedback(frame, decoded));
    zassert_equal(decoded.encoder, 0x1abcu);
    zassert_equal(decoded.speed_rpm, -100);
    zassert_equal(decoded.current_raw, -32767);
    zassert_equal(decoded.temperature_raw, 55u);
    frame.flags=CAN_FRAME_IDE;
    zassert_false(dji::decodeFeedback(frame, decoded));
    zassert_equal(decoded.speed_rpm, -100);
    frame.flags=0;
    frame.dlc=7;
    zassert_false(dji::decodeFeedback(frame, decoded));
    zassert_equal(decoded.encoder, 0x1abcu);
}

ZTEST(motor_protocol, test_dm_ids_and_special_command_layout) {
    std::uint16_t id=0;
    zassert_ok(dm::controlFrameId(dm::ControlMode::Mit, 1, id));
    zassert_equal(id, 0x001u);
    zassert_ok(dm::controlFrameId(dm::ControlMode::PositionVelocity, 15, id));
    zassert_equal(id, 0x10fu);
    zassert_ok(dm::controlFrameId(dm::ControlMode::Velocity, 2, id));
    zassert_equal(id, 0x202u);
    zassert_equal(dm::controlFrameId(dm::ControlMode::Mit, 0, id), -ERANGE);
    zassert_equal(dm::controlFrameId(dm::ControlMode::Mit, 16, id), -ERANGE);
    can_frame frame{};
    zassert_ok(dm::buildSpecialFrame(dm::ControlMode::Mit, 3, dm::SpecialCommand::Enable, frame));
    zassert_equal(frame.id, 3u);
    zassert_equal(frame.dlc, 8u);
    for (unsigned i=0; i<7; ++i) zassert_equal(frame.data[i], 0xffu);
    zassert_equal(frame.data[7], 0xfcu);
    zassert_equal(dm::buildSpecialFrame(dm::ControlMode::Mit, 3,
                   static_cast<dm::SpecialCommand>(0x30), frame), -EINVAL);
    zassert_equal(frame.data[7], 0xfcu);
}

ZTEST(motor_protocol, test_dm_velocity_and_position_velocity_ieee754_frames) {
    can_frame frame{};
    zassert_ok(dm::buildVelocityFrame(2, 1.0f, frame));
    zassert_equal(frame.id, 0x202u);
    zassert_equal(frame.dlc, 4u);
    constexpr std::uint8_t one[]={0,0,0x80,0x3f};
    zassert_mem_equal(frame.data, one, sizeof(one));
    zassert_ok(dm::buildPositionVelocityFrame(2, -2.5f, 1.0f, frame));
    zassert_equal(frame.id, 0x102u);
    zassert_equal(frame.dlc, 8u);
    constexpr std::uint8_t expected[]={0,0,0x20,0xc0,0,0,0x80,0x3f};
    zassert_mem_equal(frame.data, expected, sizeof(expected));
    zassert_equal(dm::buildVelocityFrame(2, std::numeric_limits<float>::infinity(), frame), -EINVAL);
    zassert_mem_equal(frame.data, expected, sizeof(expected));
}

ZTEST(motor_protocol, test_dm_mit_boundaries_and_invalid_input_preserves_output) {
    can_frame frame{};
    dm::MitCommand command{};
    command.position_rad=-limits.position_max_rad;
    command.velocity_rad_s=limits.velocity_max_rad_s;
    command.kp=0;
    command.kd=5;
    command.torque_ff_nm=limits.torque_max_nm;
    zassert_ok(dm::buildMitFrame(1, limits, command, frame));
    zassert_equal(frame.id, 1u);
    zassert_equal(frame.dlc, 8u);
    constexpr std::uint8_t expected[8]={0x00,0x00,0xff,0xf0,0x00,0xff,0xff,0xff};
    zassert_mem_equal(frame.data, expected, sizeof(expected));
    command.position_rad=13.0f;
    zassert_equal(dm::buildMitFrame(1, limits, command, frame), -ERANGE);
    zassert_mem_equal(frame.data, expected, sizeof(expected));
    zassert_equal(dm::buildMitFrame(1, {}, command, frame), -EINVAL);
}

ZTEST(motor_protocol, test_dm_feedback_status_and_id_mismatch) {
    can_frame frame{};
    frame.dlc=8;
    frame.data[0]=0x12; // enabled status, motor 2
    frame.data[1]=0x80;
    frame.data[2]=0x00;
    frame.data[3]=0x80;
    frame.data[4]=0x08;
    frame.data[5]=0x00;
    frame.data[6]=45;
    frame.data[7]=46;
    dm::DecodedFeedback feedback{};
    zassert_ok(dm::decodeFeedback(frame, 2, limits, feedback));
    zassert_equal(feedback.raw.status, dm::DriveStatus::Enabled);
    zassert_equal(feedback.raw.motor_id, 2u);
    zassert_equal(feedback.raw.mos_temperature_c, 45u);
    zassert_equal(feedback.raw.rotor_temperature_c, 46u);
    zassert_within(feedback.position_rad, 0, 0.001f);
    zassert_equal(dm::decodeFeedback(frame, 1, limits, feedback), -ENOENT);
    zassert_equal(feedback.raw.motor_id, 2u);
    frame.data[0]=0xa2;
    zassert_ok(dm::decodeFeedback(frame, 2, limits, feedback));
    zassert_equal(feedback.raw.status, dm::DriveStatus::OverCurrent);
    zassert_true(dm::isFaultStatus(feedback.raw.status));
    zassert_false(dm::isFaultStatus(dm::DriveStatus::Enabled));
}
ZTEST_SUITE(motor_protocol, nullptr, nullptr, nullptr, nullptr, nullptr);
