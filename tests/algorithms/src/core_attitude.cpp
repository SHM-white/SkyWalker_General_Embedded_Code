#include <benchmark.hpp>
#include <core/attitude.hpp>
#include <zephyr/ztest.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {
namespace core = skywalker::core;
constexpr float pi = 3.14159265358979323846f;
constexpr float half_sqrt = 0.7071067811865475244f;

void assertQuaternion(core::Quaternion actual, core::Quaternion expected, float tolerance = 2e-6f) {
    zassert_true(core::finite(actual));
    zassert_within(actual.w, expected.w, tolerance);
    zassert_within(actual.x, expected.x, tolerance);
    zassert_within(actual.y, expected.y, tolerance);
    zassert_within(actual.z, expected.z, tolerance);
}

void assertVector(core::Vec3 actual, core::Vec3 expected, float tolerance = 2e-6f) {
    zassert_true(core::finite(actual));
    zassert_within(actual.x, expected.x, tolerance);
    zassert_within(actual.y, expected.y, tolerance);
    zassert_within(actual.z, expected.z, tolerance);
}
}

ZTEST(core_attitude, test_finite_norm_and_normalization_contract) {
    zassert_true(core::finite(core::Vec3{3, 4, 12}));
    zassert_within(core::norm({3, 4, 12}), 13.0f, 1e-6f);
    zassert_false(core::finite(core::Vec3{0, std::numeric_limits<float>::quiet_NaN(), 0}));
    zassert_false(core::finite(core::Quaternion{1, 0, 0, std::numeric_limits<float>::infinity()}));
    core::Quaternion slight_error{1.02f, 0, 0, 0};
    zassert_true(core::normalize(slight_error));
    assertQuaternion(slight_error, {1, 0, 0, 0});
    const std::array<core::Quaternion, 5> corrupt{{
        {0, 0, 0, 0},
        {1e-6f, 0, 0, 0},
        {2, 0, 0, 0},
        {std::numeric_limits<float>::quiet_NaN(), 0, 0, 0},
        {1, std::numeric_limits<float>::infinity(), 0, 0},
    }};
    for (auto value : corrupt)
        zassert_false(core::normalize(value));
}

ZTEST(core_attitude, test_hamilton_product_and_conjugate_known_values) {
    const core::Quaternion x90{half_sqrt, half_sqrt, 0, 0};
    const core::Quaternion y90{half_sqrt, 0, half_sqrt, 0};
    assertQuaternion(core::multiply(x90, y90), {0.5f, 0.5f, 0.5f, 0.5f});
    assertQuaternion(core::multiply(y90, x90), {0.5f, 0.5f, 0.5f, -0.5f});
    assertQuaternion(core::conjugate({0.5f, 0.5f, -0.5f, 0.5f}), {0.5f, -0.5f, 0.5f, -0.5f});
    assertQuaternion(core::multiply({1, 0, 0, 0}, x90), x90);
}

ZTEST(core_attitude, test_axis_rotation_matches_right_handed_basis_vectors) {
    const core::Quaternion z90{half_sqrt, 0, 0, half_sqrt};
    assertVector(core::rotate(z90, {1, 0, 0}), {0, 1, 0});
    assertVector(core::rotate(z90, {0, 1, 0}), {-1, 0, 0});
    assertVector(core::rotate(z90, {0, 0, 1}), {0, 0, 1});
    assertVector(core::rotate({half_sqrt, half_sqrt, 0, 0}, {0, 1, 0}), {0, 0, 1});
    assertVector(core::rotate({half_sqrt, 0, half_sqrt, 0}, {0, 0, 1}), {1, 0, 0});
    assertVector(core::rotate({-half_sqrt, 0, 0, -half_sqrt}, {1, 0, 0}), {0, 1, 0});
}

ZTEST(core_attitude, test_rotation_matches_independent_zyx_matrix) {
    constexpr double roll = 0.4, pitch = -0.3, yaw = 0.7;
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const core::Vec3 input{1.2f, -0.7f, 2.3f};
    const core::Vec3 expected{
        float(cy * cp * double(input.x) + (cy * sp * sr - sy * cr) * double(input.y) +
              (cy * sp * cr + sy * sr) * double(input.z)),
        float(sy * cp * double(input.x) + (sy * sp * sr + cy * cr) * double(input.y) +
              (sy * sp * cr - cy * sr) * double(input.z)),
        float(-sp * double(input.x) + cp * sr * double(input.y) + cp * cr * double(input.z)),
    };
    const auto attitude = core::fromEuler({float(roll), float(pitch), float(yaw)});
    assertVector(core::rotate(attitude, input), expected, 3e-6f);
}

ZTEST(core_attitude, test_euler_conversion_known_attitudes_and_pitch_clamp) {
    assertQuaternion(core::fromEuler({0, 0, 0}), {1, 0, 0, 0});
    assertQuaternion(core::fromEuler({pi / 2, 0, 0}), {half_sqrt, half_sqrt, 0, 0});
    assertQuaternion(core::fromEuler({0, -pi / 2, 0}), {half_sqrt, 0, -half_sqrt, 0});
    assertQuaternion(core::fromEuler({0, 0, pi / 2}), {half_sqrt, 0, 0, half_sqrt});
    assertQuaternion(core::fromEuler({pi / 2, 0, pi / 2}), {0.5f, 0.5f, 0.5f, 0.5f});

    const auto angles = core::euler({0.5f, 0.5f, 0.5f, 0.5f});
    zassert_within(angles.roll, pi / 2, 2e-6f);
    zassert_within(angles.pitch, 0.0f, 2e-6f);
    zassert_within(angles.yaw, pi / 2, 2e-6f);
    const auto negative_yaw = core::euler({half_sqrt, 0, 0, -half_sqrt});
    zassert_within(negative_yaw.yaw, -pi / 2, 2e-6f);
    // Rounding can place sin(pitch) just above one at the singularity.
    const auto pole = core::euler({half_sqrt, 0, half_sqrt * 1.000001f, 0});
    zassert_true(std::isfinite(pole.pitch));
    zassert_within(pole.pitch, pi / 2, 2e-6f);
}

ZTEST(core_attitude, test_benchmark_rotate) {
    const core::Quaternion attitude{half_sqrt, 0, 0, half_sqrt};
    const std::array<core::Vec3, 3> inputs{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    const std::array<core::Vec3, 3> expected{{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}};
    core::Vec3 output{};
    std::uint32_t last_index = 0;
    skywalker::test::benchmark("attitude.rotate", 6000, [&](std::uint32_t index) {
        last_index = index % inputs.size();
        auto measured_attitude = attitude;
        auto measured_input = inputs[last_index];
        skywalker::test::doNotOptimize(measured_attitude);
        skywalker::test::doNotOptimize(measured_input);
        output = core::rotate(measured_attitude, measured_input);
        skywalker::test::doNotOptimize(output);
    });
    assertVector(output, expected[last_index]);
}

ZTEST_SUITE(core_attitude, nullptr, nullptr, nullptr, nullptr, nullptr);
