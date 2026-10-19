#include <array>
#include <cerrno>
#include <cstdint>
#include <communication/interboard/interboard_codec.hpp>
#include <communication/interboard/interboard_link.hpp>
#include <communication/interboard/interboard_parser.hpp>
#include <communication/wire.hpp>
#include <zephyr/ztest.h>

namespace {
using namespace skywalker;
using namespace skywalker::communication;
using namespace skywalker::robotics;
std::array<std::uint8_t, kMaxFrame> bytes{};
int heartbeat(std::uint32_t seq=1, std::uint64_t boot=123) {
    BoardHeartbeat hb{};
    hb.role = BoardRole::GimbalController;
    hb.sender_boot_id = boot;
    hb.safety_state = SafetyState::Ready;
    hb.ready = true;
    hb.sender_uptime_ms = 99;
    return InterBoardCodec::encodeHeartbeat(hb, seq, bytes.data(), bytes.size());
}
}
ZTEST(protocol_interboard, test_fragmented_heartbeat_frame_has_exact_metadata_and_payload) {
    using namespace skywalker::communication;
    InterBoardParser parser;
    const int n = heartbeat(77);
    zassert_equal(n, 34);
    zassert_equal(bytes[0], 0xa5);
    zassert_equal(bytes[1], 0x5a);
    zassert_equal(bytes[2], kInterBoardProtocolVersion);
    zassert_ok(parser.consume(bytes.data(), 11, 50));
    FrameMeta meta{};
    std::uint8_t payload[kMaxPayload]{};
    std::size_t size=0;
    zassert_false(parser.popFrame(meta, payload, sizeof(payload), size));
    zassert_ok(parser.consume(bytes.data()+11, n-11, 51));
    zassert_true(parser.popFrame(meta, payload, sizeof(payload), size));
    zassert_equal(size, 20u);
    zassert_equal(meta.frame_sequence, 77u);
    zassert_equal(meta.local_receive_ms, 51u);
    zassert_equal(meta.message_id, MessageId::Heartbeat);
    robotics::BoardHeartbeat decoded{};
    zassert_ok(InterBoardCodec::decodeHeartbeat(meta, payload, size, decoded));
    zassert_equal(decoded.sender_boot_id, 123u);
    zassert_true(decoded.ready);
    zassert_equal(decoded.stamp.timestamp_ms, 51u);
    zassert_false(parser.popFrame(meta, payload, sizeof(payload), size));
}

ZTEST(protocol_interboard, test_crc_version_and_length_recovery) {
    using namespace skywalker::communication;
    InterBoardParser parser;
    const int n = heartbeat();
    auto bad=bytes;
    bad[n-1] ^= 0x20;
    zassert_ok(parser.consume(bad.data(), n, 10));
    zassert_true(parser.stats().crc_errors > 0);
    bad=bytes;
    bad[2]++;
    zassert_ok(parser.consume(bad.data(), n, 11));
    zassert_true(parser.stats().version_errors > 0);
    bad=bytes;
    wire::storeLe16(bad.data()+6, kMaxPayload+1);
    zassert_ok(parser.consume(bad.data(), n, 12));
    zassert_true(parser.stats().length_errors > 0);
    zassert_ok(parser.consume(bytes.data(), n, 20));
    FrameMeta meta{};
    std::uint8_t payload[kMaxPayload]{};
    std::size_t size=0;
    zassert_true(parser.popFrame(meta, payload, sizeof(payload), size));
    zassert_equal(parser.stats().valid_frames, 1u);
}

ZTEST(protocol_interboard, test_queue_overflow_discards_oldest_and_reset_restarts_counters) {
    using namespace skywalker::communication;
    InterBoardParser parser;
    for (unsigned i=1; i<=6; ++i) {
        const int n=heartbeat(i);
        zassert_ok(parser.consume(bytes.data(), n, i));
    }
    zassert_equal(parser.stats().queue_overflows, 2u);
    FrameMeta meta{};
    std::uint8_t payload[kMaxPayload]{};
    std::size_t size=0;
    for (unsigned i=3; i<=6; ++i) {
        zassert_true(parser.popFrame(meta, payload, sizeof(payload), size));
        zassert_equal(meta.frame_sequence, i);
    }
    zassert_false(parser.popFrame(meta, payload, sizeof(payload), size));
    zassert_ok(parser.reset());
    zassert_equal(parser.stats().queue_overflows, 0u);
    zassert_equal(parser.stats().valid_frames, 0u);
}

ZTEST(protocol_interboard, test_link_rejects_control_without_session_and_replay) {
    using namespace skywalker;
    using namespace skywalker::communication;
    using namespace skywalker::robotics;
    InterBoardLink link(BoardRole::ChassisController);
    RemoteChassisControl control{};
    control.command.stamp={10, 8, true};
    control.command.mode=ChassisMode::BodyVelocity;
    control.command.vx_m_s=1.25f;
    control.global_action=SafetyAction::Active;
    control.receiver_boot_id=77;
    std::array<std::uint8_t, kMaxFrame> control_frame{};
    const int csize=InterBoardCodec::encodeChassisControl(control, 2, control_frame.data(), control_frame.size());
    zassert_true(csize>0);
    zassert_ok(link.processRxBytes(control_frame.data(), csize, 10));
    zassert_equal(link.rejectedFrames(), 1u);
    zassert_false(link.peerOnline(10));
    const int hsize=heartbeat(1);
    zassert_ok(link.processRxBytes(bytes.data(), hsize, 11));
    zassert_true(link.peerOnline(11, 100));
    zassert_ok(link.processRxBytes(control_frame.data(), csize, 12));
    RemoteChassisControl observed{};
    zassert_ok(link.latestChassisControl(observed));
    zassert_within(observed.command.vx_m_s, 1.25f, 0.000001f);
    zassert_equal(observed.command.stamp.sequence, 8u);
    zassert_ok(link.processRxBytes(control_frame.data(), csize, 13));
    zassert_equal(link.rejectedFrames(), 2u);
    zassert_equal(observed.stamp.timestamp_ms, 12u);
    zassert_false(link.peerOnline(112, 100));
    const int reboot=heartbeat(2, 456);
    zassert_ok(link.processRxBytes(bytes.data(), reboot, 113));
    zassert_equal(link.latestChassisControl(observed), -EAGAIN);
    zassert_true(link.peerOnline(113,100));
}

ZTEST(protocol_interboard, test_encoder_rejects_invalid_power_without_writing_output) {
    using namespace skywalker;
    using namespace skywalker::communication;
    robotics::ChassisConstraint value{};
    std::array<std::uint8_t, kMaxFrame> output{};
    output.fill(0x7d);
    value.power_limit_w=-1;
    zassert_equal(InterBoardCodec::encodeChassisConstraint(value, 1, output.data(), output.size()), -EINVAL);
    zassert_equal(output[0], 0x7d);
    value.power_limit_w=80;
    value.buffer_energy_j=12;
    const int n=InterBoardCodec::encodeChassisConstraint(value, 2, output.data(), output.size());
    zassert_equal(n, 34);
    zassert_equal(InterBoardCodec::encodeChassisConstraint(value, 2, output.data(), n-1), -ENOSPC);
}
ZTEST_SUITE(protocol_interboard, nullptr, nullptr, nullptr, nullptr, nullptr);
