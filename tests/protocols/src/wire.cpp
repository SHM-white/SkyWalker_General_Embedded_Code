#include <array>
#include <cstdint>
#include <communication/wire.hpp>
#include <core/byte_codec.hpp>
#include <robotics/messages/common.hpp>
#include <robotics/messages/interboard.hpp>
#include <zephyr/ztest.h>

using namespace skywalker;

ZTEST(protocol_wire, test_little_endian_known_vectors_and_roundtrip) {
    std::array<std::uint8_t, 18> bytes{};
    communication::wire::storeLe16(bytes.data(), 0x1234);
    communication::wire::storeLe32(bytes.data() + 2, 0x89abcdef);
    communication::wire::storeLe64(bytes.data() + 6, 0x0123456789abcdefULL);
    communication::wire::storeFloatLe(bytes.data() + 14, -2.5f);
    constexpr std::uint8_t expected[] = {0x34, 0x12, 0xef, 0xcd, 0xab, 0x89, 0xef, 0xcd, 0xab,
                                         0x89, 0x67, 0x45, 0x23, 0x01, 0x00, 0x00, 0x20, 0xc0};
    zassert_mem_equal(bytes.data(), expected, sizeof(expected));
    zassert_equal(communication::wire::loadLe16(bytes.data()), 0x1234);
    zassert_equal(communication::wire::loadLe32(bytes.data() + 2), 0x89abcdefu);
    zassert_equal(communication::wire::loadLe64(bytes.data() + 6), 0x0123456789abcdefULL);
    zassert_within(communication::wire::loadFloatLe(bytes.data() + 14), -2.5f, 0.000001f);
    zassert_equal(core::wire::u32(bytes.data() + 2), 0x89abcdefu);
    zassert_within(core::wire::f32(bytes.data() + 14), -2.5f, 0.000001f);
}

ZTEST(protocol_wire, test_sequence_wraparound_and_freshness_boundaries) {
    using namespace robotics;
    zassert_true(sequenceAfter(0, 0xffffffffu));
    zassert_true(sequenceAfter(0x10u, 0xfffffff0u));
    zassert_false(sequenceAfter(42, 42));
    zassert_false(sequenceAfter(0x80000000u, 0));
    zassert_false(sequenceAfter(0, 1));
    MessageStamp stamp{100, 4, true};
    zassert_true(isFresh(stamp, 150, 50));
    zassert_false(isFresh(stamp, 151, 50));
    zassert_false(isFresh(stamp, 99, 50));
    stamp.valid = false;
    zassert_false(isFresh(stamp, 100, 50));
}

ZTEST(protocol_wire, test_forwarded_age_never_renews_original_producer) {
    const robotics::MessageStamp stamp{100, 7, true};
    zassert_true(robotics::forwardedFresh(stamp, 30, 150, 100));
    zassert_true(robotics::forwardedFresh(stamp, 30, 170, 100));
    zassert_false(robotics::forwardedFresh(stamp, 30, 171, 100));
    zassert_false(robotics::forwardedFresh(stamp, 101, 101, 100));
    zassert_false(robotics::forwardedFresh(stamp, 0, 99, 100));
}

ZTEST_SUITE(protocol_wire, nullptr, nullptr, nullptr, nullptr, nullptr);
