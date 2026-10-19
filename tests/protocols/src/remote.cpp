#include <array>
#include <cerrno>
#include <cstdint>
#include <communication/remote/dr16_decoder.hpp>
#include <communication/remote/remote_service.hpp>
#include <communication/wire.hpp>
#include <zephyr/ztest.h>

namespace {
using namespace skywalker;
std::array<std::uint8_t, 18> frame(int ch0 = 1024, int ch1 = 1024, int ch2 = 1024, int ch3 = 1024) {
    std::array<std::uint8_t, 18> bytes{};
    const std::uint64_t packed = std::uint64_t(ch0) | (std::uint64_t(ch1) << 11) |
        (std::uint64_t(ch2) << 22) | (std::uint64_t(ch3) << 33) |
        (std::uint64_t(3) << 44) | (std::uint64_t(3) << 46);
    for (unsigned i = 0; i != 6; ++i)
        bytes[i] = static_cast<std::uint8_t>(packed >> (8 * i));
    communication::wire::storeLe16(bytes.data() + 16, 1024);
    return bytes;
}
}
ZTEST(protocol_remote, test_neutral_frame_and_channel_bounds) {
    using namespace skywalker;
    communication::Dr16Decoder decoder({});
    robotics::RemoteState out{};
    const auto neutral = frame();
    zassert_ok(decoder.decodeFrame(neutral.data(), neutral.size(), 100, out));
    zassert_true(out.online);
    zassert_equal(out.left_switch, robotics::RcSwitch::Middle);
    zassert_equal(out.right_switch, robotics::RcSwitch::Middle);
    zassert_equal(out.analog.left_y, 0);
    zassert_equal(out.analog.wheel, 0);
    zassert_equal(out.stamp.sequence, 1);
    const auto extremes = frame(364, 1684, 364, 1684);
    zassert_ok(decoder.decodeFrame(extremes.data(), extremes.size(), 101, out));
    zassert_equal(out.analog.right_x, -660);
    zassert_equal(out.analog.right_y, 660);
    zassert_equal(out.stamp.sequence, 2);
    zassert_equal(decoder.validFrameCount(), 2u);
}

ZTEST(protocol_remote, test_failed_decode_does_not_mutate_publication) {
    using namespace skywalker;
    communication::Dr16Decoder decoder({});
    auto good = frame();
    robotics::RemoteState out{};
    zassert_ok(decoder.decodeFrame(good.data(), good.size(), 7, out));
    const auto previous = out.stamp;
    good[5] &= static_cast<std::uint8_t>(~0x30u); // Invalid switch 0.
    zassert_equal(decoder.decodeFrame(good.data(), good.size(), 8, out), -EBADMSG);
    zassert_equal(out.stamp.sequence, previous.sequence);
    zassert_equal(decoder.decodeFrame(nullptr, good.size(), 8, out), -EINVAL);
    zassert_equal(decoder.decodeFrame(good.data(), good.size()-1, 8, out), -EINVAL);
    zassert_equal(decoder.invalidFrameCount(), 3u);
    zassert_ok(decoder.reset());
    zassert_equal(decoder.invalidFrameCount(), 0u);
    zassert_equal(decoder.validFrameCount(), 0u);
}

ZTEST(protocol_remote, test_stream_fragmentation_and_expiry_without_refreshing_stamp) {
    using namespace skywalker;
    communication::RemoteService service({}, {.offline_timeout_ms=100, .assembly_gap_ms=10});
    auto good = frame();
    robotics::RemoteState value{};
    zassert_equal(service.snapshot(100, value), -EAGAIN);
    zassert_ok(service.processBytes(good.data(), 7, 100));
    zassert_equal(service.snapshot(100, value), -EAGAIN);
    zassert_ok(service.processBytes(good.data()+7, good.size()-7, 101));
    zassert_ok(service.snapshot(101, value));
    zassert_equal(value.stamp.timestamp_ms, 101u);
    zassert_equal(value.stamp.sequence, 1u);
    zassert_ok(service.snapshot(201, value));
    zassert_equal(value.stamp.sequence, 1u);
    zassert_equal(service.snapshot(202, value), -ESTALE);
    zassert_false(value.online);
    zassert_equal(value.stamp.timestamp_ms, 101u);
    zassert_equal(service.processBytes(nullptr, 1, 202), -EINVAL);
}

ZTEST(protocol_remote, test_partial_frame_gap_does_not_publish_ghost_input) {
    using namespace skywalker;
    communication::RemoteService service({}, {.offline_timeout_ms=100, .assembly_gap_ms=5});
    const auto good = frame();
    robotics::RemoteState value{};
    zassert_ok(service.processBytes(good.data(), 12, 10));
    zassert_ok(service.processBytes(good.data()+12, 6, 20));
    zassert_equal(service.snapshot(20, value), -EAGAIN);
    service.discardPartial();
    zassert_ok(service.processBytes(good.data(), good.size(), 30));
    zassert_ok(service.snapshot(30, value));
    zassert_equal(value.stamp.sequence, 1u);
}

ZTEST_SUITE(protocol_remote, nullptr, nullptr, nullptr, nullptr, nullptr);
