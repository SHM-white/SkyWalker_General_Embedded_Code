#include <zephyr/ztest.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <core/byte_codec.hpp>
#include <core/measurement.hpp>

namespace wire = skywalker::core::wire;
namespace core = skywalker::core;

ZTEST(core_wire, test_decode_little_endian_reference_vectors) {
    constexpr std::array<std::uint8_t, 4> bytes{0x78, 0x56, 0x34, 0x12};
    zassert_equal(wire::u16(bytes.data()), 0x5678u);
    zassert_equal(wire::u32(bytes.data()), 0x12345678u);
    constexpr std::array<std::uint8_t, 4> one{0x00, 0x00, 0x80, 0x3f};
    zassert_within(wire::f32(one.data()), 1.0f, 0.0f);
}

ZTEST(core_wire, test_encode_reference_vectors_and_unaligned_offsets) {
    std::array<std::uint8_t, 12> bytes{};
    bytes.fill(0xa5);
    wire::put16(bytes.data() + 1, 0x1234);
    wire::put32(bytes.data() + 3, 0x89abcdefu);
    wire::putFloat(bytes.data() + 7, -2.5f);
    const std::array<std::uint8_t, 12> expected{
        0xa5, 0x34, 0x12, 0xef, 0xcd, 0xab, 0x89, 0x00, 0x00, 0x20, 0xc0, 0xa5
    };
    zassert_mem_equal(bytes.data(), expected.data(), bytes.size());
}

ZTEST(core_wire, test_float_bits_round_trip_including_signed_zero_and_specials) {
    constexpr std::array<std::uint32_t, 7> bit_patterns{
        0x00000000u, 0x80000000u, 0x3f800000u, 0xbf800000u,
        0x7f800000u, 0xff800000u, 0x7fc00001u
    };
    for (const auto bits : bit_patterns) {
        std::array<std::uint8_t, 4> raw{};
        wire::put32(raw.data(), bits);
        const float decoded = wire::f32(raw.data());
        std::array<std::uint8_t, 4> encoded{};
        wire::putFloat(encoded.data(), decoded);
        zassert_equal(wire::u32(encoded.data()), bits);
    }
}

ZTEST(core_stamp, test_freshness_inclusive_deadline_and_future_rejection) {
    const core::Stamp valid{1000, 42, true};
    zassert_true(core::fresh(valid, 1000, 100));
    zassert_true(core::fresh(valid, 1100, 100));
    zassert_false(core::fresh(valid, 1101, 100));
    zassert_false(core::fresh(valid, 999, 100));
    zassert_false(core::fresh(valid, 1000, 0));
    zassert_false(core::fresh({1000, 42, false}, 1000, 100));
}

ZTEST(core_stamp, test_large_timestamp_does_not_wrap_age_calculation) {
    const core::Stamp stamp{std::numeric_limits<core::TimeUs>::max() - 5, 1, true};
    zassert_true(core::fresh(stamp, std::numeric_limits<core::TimeUs>::max(), 5));
    zassert_false(core::fresh(stamp, 0, 5));
    zassert_false(core::fresh({1, 1, false}, 1, std::numeric_limits<core::TimeUs>::max()));
}

ZTEST(core_stamp, test_reference_identity_and_measurement_defaults) {
    constexpr core::OrientationReference a{1, 2};
    constexpr core::OrientationReference same{1, 2};
    constexpr core::OrientationReference new_epoch{1, 3};
    constexpr core::OrientationReference new_frame{2, 2};
    zassert_true(a == same);
    zassert_false(a == new_epoch);
    zassert_false(a == new_frame);
    const core::Measurement<core::Vec3> measurement{};
    zassert_false(measurement.stamp.valid);
    zassert_equal(measurement.stamp.sequence, 0);
    zassert_within(measurement.value.x, 0.0f, 0.0f);
}

ZTEST_SUITE(core_wire, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(core_stamp, nullptr, nullptr, nullptr, nullptr, nullptr);
