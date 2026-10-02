#include <benchmark.hpp>
#include <control/attitude_ekf.hpp>
#include <zephyr/ztest.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <limits>

namespace {
using Ekf = skywalker::control::QuaternionEkf;
using Quaternion = skywalker::core::Quaternion;
using Vec3 = skywalker::core::Vec3;
using TimeUs = skywalker::core::TimeUs;
constexpr double pi = 3.14159265358979323846;
constexpr float gravity = 9.80665f;
constexpr TimeUs sample_us = 2000;
constexpr double sample_s = 0.002;

// Independent double-precision ZYX attitude and gravity model. None of the
// expected values use core::fromEuler/rotate or EKF output as their oracle.
Quaternion referenceAttitude(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    return {float(cr * cp * cy + sr * sp * sy), float(sr * cp * cy - cr * sp * sy),
            float(cr * sp * cy + sr * cp * sy), float(cr * cp * sy - sr * sp * cy)};
}

Vec3 referenceGravity(double roll, double pitch) {
    return {float(-double(gravity) * std::sin(pitch)), float(double(gravity) * std::sin(roll) * std::cos(pitch)),
            float(double(gravity) * std::cos(roll) * std::cos(pitch))};
}

double angularError(Quaternion actual, Quaternion expected) {
    const double an = std::sqrt(double(actual.w) * double(actual.w) + double(actual.x) * double(actual.x) +
                                double(actual.y) * double(actual.y) + double(actual.z) * double(actual.z));
    const double en = std::sqrt(double(expected.w) * double(expected.w) + double(expected.x) * double(expected.x) +
                                double(expected.y) * double(expected.y) + double(expected.z) * double(expected.z));
    const double dot = (double(actual.w) * double(expected.w) + double(actual.x) * double(expected.x) +
                        double(actual.y) * double(expected.y) + double(actual.z) * double(expected.z)) /
                       (an * en);
    // q and -q describe the same rotation.
    return 2.0 * std::acos(std::clamp(std::fabs(dot), 0.0, 1.0));
}

void assertAttitude(Quaternion actual, Quaternion expected, double tolerance_rad) {
    zassert_true(std::isfinite(actual.w) && std::isfinite(actual.x) && std::isfinite(actual.y) &&
                     std::isfinite(actual.z),
                 "attitude must stay finite");
    const double n2 = double(actual.w) * double(actual.w) + double(actual.x) * double(actual.x) +
                      double(actual.y) * double(actual.y) + double(actual.z) * double(actual.z);
    zassert_within(n2, 1.0, 2e-5, "quaternion must stay normalized");
    const double error = angularError(actual, expected);
    zassert_true(error <= tolerance_rad, "angular error %g rad exceeds %g rad", error, tolerance_rad);
}

Ekf::Config testConfig() {
    Ekf::Config config;
    config.initialization_samples = 16;
    return config;
}

TimeUs initialize(Ekf &filter, const Ekf::Config &config, Vec3 accel = {0, 0, gravity}, Vec3 gyro = {}) {
    zassert_ok(filter.init());
    TimeUs time = 0;
    for (unsigned i = 0; i < config.initialization_samples; ++i) {
        time += sample_us;
        const int expected = i + 1 == config.initialization_samples ? 0 : -EAGAIN;
        zassert_equal(filter.update({accel, gyro, time}), expected);
    }
    zassert_equal(filter.quality(), Ekf::Quality::Tracking);
    return time;
}

Vec3 add(Vec3 a, Vec3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
}

ZTEST(attitude_ekf, test_static_gravity_initializes_known_roll_and_pitch) {
    const std::array<std::array<double, 2>, 7> cases{{
        {0.0, 0.0}, {pi / 6, 0.0}, {-pi / 3, 0.0}, {0.0, pi / 4},
        {0.0, -pi / 3}, {0.55, -0.4}, {-0.7, 0.6},
    }};
    for (const auto &angles : cases) {
        const auto config = testConfig();
        Ekf filter(config);
        auto time = initialize(filter, config, referenceGravity(angles[0], angles[1]));
        const auto expected = referenceAttitude(angles[0], angles[1], 0);
        assertAttitude(filter.attitude(), expected, 1e-4);
        for (unsigned i = 0; i < 500; ++i) {
            time += sample_us;
            zassert_ok(filter.update({referenceGravity(angles[0], angles[1]), {}, time}));
            zassert_equal(filter.quality(), Ekf::Quality::Tracking);
        }
        assertAttitude(filter.attitude(), expected, 1e-3);
    }
}

ZTEST(attitude_ekf, test_constant_yaw_rate_over_multiple_turns) {
    const auto config = testConfig();
    Ekf filter(config);
    auto time = initialize(filter, config);
    constexpr double rate = 2.0;
    for (unsigned i = 1; i <= 4000; ++i) {
        time += sample_us;
        zassert_ok(filter.update({{0, 0, gravity}, {0, 0, float(rate)}, time}));
        if (i % 100 == 0)
            assertAttitude(filter.attitude(), referenceAttitude(0, 0, rate * i * sample_s), 0.01);
    }
    zassert_equal(filter.quality(), Ekf::Quality::Tracking);
}

ZTEST(attitude_ekf, test_yaw_rotation_with_fixed_roll_and_pitch) {
    const auto config = testConfig();
    Ekf filter(config);
    constexpr double roll = 0.4, pitch = -0.3, rate = 0.8;
    const auto accel = referenceGravity(roll, pitch);
    const Vec3 body_rate{float(-rate * std::sin(pitch)), float(rate * std::sin(roll) * std::cos(pitch)),
                         float(rate * std::cos(roll) * std::cos(pitch))};
    auto time = initialize(filter, config, accel);
    for (unsigned i = 1; i <= 2000; ++i) {
        time += sample_us;
        zassert_ok(filter.update({accel, body_rate, time}));
        if (i % 100 == 0)
            assertAttitude(filter.attitude(), referenceAttitude(roll, pitch, rate * i * sample_s), 0.015);
    }
}

ZTEST(attitude_ekf, test_instances_keep_attitude_bias_and_reset_independent) {
    const auto config = testConfig();
    Ekf first(config), second(config);
    const Vec3 bias_a{0.01f, -0.02f, 0.03f}, bias_b{-0.025f, 0.015f, -0.01f};
    const auto accel_b = referenceGravity(-0.5, 0.25);
    auto time_a = initialize(first, config, {0, 0, gravity}, bias_a);
    auto time_b = initialize(second, config, accel_b, bias_b);
    for (unsigned i = 1; i <= 500; ++i) {
        time_a += sample_us;
        time_b += sample_us;
        zassert_ok(first.update({{0, 0, gravity}, add(bias_a, {0, 0, 1}), time_a}));
        zassert_ok(second.update({accel_b, bias_b, time_b}));
    }
    assertAttitude(first.attitude(), referenceAttitude(0, 0, 1), 0.01);
    assertAttitude(second.attitude(), referenceAttitude(-0.5, 0.25, 0), 0.002);
    const auto other_generation = second.generation();
    first.reset();
    zassert_equal(first.quality(), Ekf::Quality::Initializing);
    for (unsigned i = 0; i < 100; ++i) {
        time_b += sample_us;
        zassert_ok(second.update({accel_b, bias_b, time_b}));
    }
    zassert_equal(second.generation(), other_generation);
    assertAttitude(second.attitude(), referenceAttitude(-0.5, 0.25, 0), 0.002);
}

ZTEST(attitude_ekf, test_initial_stationary_bias_is_removed_from_rotation) {
    const auto config = testConfig();
    const Vec3 bias{0.012f, -0.009f, 0.025f};
    Ekf filter(config);
    auto time = initialize(filter, config, {0, 0, gravity}, bias);
    for (unsigned i = 0; i < 1000; ++i) {
        time += sample_us;
        zassert_ok(filter.update({{0, 0, gravity}, bias, time}));
    }
    assertAttitude(filter.attitude(), referenceAttitude(0, 0, 0), 0.002);
    for (unsigned i = 1; i <= 1500; ++i) {
        time += sample_us;
        zassert_ok(filter.update({{0, 0, gravity}, add(bias, {0, 0, 0.7f}), time}));
    }
    assertAttitude(filter.attitude(), referenceAttitude(0, 0, 2.1), 0.01);
}

ZTEST(attitude_ekf, test_stationary_with_deterministic_sensor_noise) {
    const auto config = testConfig();
    const Vec3 bias{0.01f, -0.015f, 0.02f};
    constexpr double roll = 0.35, pitch = -0.45;
    const auto gravity_vector = referenceGravity(roll, pitch);
    Ekf filter(config);
    auto time = initialize(filter, config, gravity_vector, bias);
    double squared_error_sum = 0;
    for (unsigned i = 1; i <= 3000; ++i) {
        const double phase = i * 0.137;
        const Vec3 accel_noise{float(0.025 * std::sin(phase)), float(0.02 * std::cos(phase * 1.3)),
                                float(0.015 * std::sin(phase * 0.7))};
        const Vec3 gyro_noise{float(0.003 * std::sin(phase * 0.9)), float(0.002 * std::cos(phase * 1.1)),
                               float(0.002 * std::sin(phase * 1.7))};
        time += sample_us;
        zassert_ok(filter.update({add(gravity_vector, accel_noise), add(bias, gyro_noise), time}));
        zassert_equal(filter.quality(), Ekf::Quality::Tracking);
        const auto expected = referenceAttitude(roll, pitch, 0);
        assertAttitude(filter.attitude(), expected, 0.01); // < 0.6 degree maximum.
        const double error = angularError(filter.attitude(), expected);
        squared_error_sum += error * error;
    }
    zassert_true(std::sqrt(squared_error_sum / 3000) < 0.005, "stationary RMS error must be < 0.3 degree");
}

ZTEST(attitude_ekf, test_noisy_three_axis_trajectory_tracks_analytic_attitude) {
    auto config = testConfig();
    config.stationary_gyro_rad_s = 0.01f;
    const Vec3 bias{0.002f, -0.003f, 0.004f};
    Ekf filter(config);
    auto time = initialize(filter, config, {0, 0, gravity}, bias);
    double squared_error_sum = 0;
    for (unsigned i = 1; i <= 2500; ++i) {
        const double t = i * sample_s;
        const double roll = 0.25 * std::sin(0.7 * t), pitch = 0.18 * std::sin(0.5 * t), yaw = 0.45 * t;
        const double roll_rate = 0.175 * std::cos(0.7 * t), pitch_rate = 0.09 * std::cos(0.5 * t);
        const Vec3 rate{float(roll_rate - 0.45 * std::sin(pitch)),
                         float(pitch_rate * std::cos(roll) + 0.45 * std::sin(roll) * std::cos(pitch)),
                         float(-pitch_rate * std::sin(roll) + 0.45 * std::cos(roll) * std::cos(pitch))};
        const double phase = i * 0.173;
        const Vec3 accel_noise{float(0.02 * std::sin(phase)), float(0.025 * std::cos(phase * 1.3)),
                                float(0.015 * std::sin(phase * 0.7))};
        const Vec3 gyro_noise{float(0.002 * std::sin(phase * 0.9)), float(0.003 * std::cos(phase * 1.1)),
                               float(0.002 * std::sin(phase * 1.7))};
        time += sample_us;
        zassert_ok(filter.update({add(referenceGravity(roll, pitch), accel_noise), add(add(rate, bias), gyro_noise),
                                 time}));
        zassert_equal(filter.quality(), Ekf::Quality::Tracking);
        const auto expected = referenceAttitude(roll, pitch, yaw);
        assertAttitude(filter.attitude(), expected, 0.025); // < 1.5 degree through the whole motion.
        const double error = angularError(filter.attitude(), expected);
        squared_error_sum += error * error;
    }
    zassert_true(std::sqrt(squared_error_sum / 2500) < 0.015, "motion RMS error must be < 0.9 degree");
}

ZTEST(attitude_ekf, test_acceleration_magnitude_gate_preserves_prediction_and_recovers) {
    const auto config = testConfig();
    Ekf filter(config);
    auto time = initialize(filter, config);
    const auto generation = filter.generation();
    for (unsigned i = 1; i <= 100; ++i) {
        time += sample_us;
        zassert_ok(filter.update({{6, 0, gravity}, {0, 0, 1}, time}));
        zassert_equal(filter.quality(), Ekf::Quality::Degraded);
        assertAttitude(filter.attitude(), referenceAttitude(0, 0, i * sample_s), 0.002);
    }
    for (unsigned i = 0; i < 500; ++i) {
        time += sample_us;
        zassert_ok(filter.update({{0, 0, gravity}, {}, time}));
    }
    zassert_equal(filter.generation(), generation);
    zassert_equal(filter.quality(), Ekf::Quality::Tracking);
    assertAttitude(filter.attitude(), referenceAttitude(0, 0, 0.2), 0.025);
}

ZTEST(attitude_ekf, test_innovation_gate_rejects_direction_outlier_and_recovers) {
    auto config = testConfig();
    config.accel_tau_s = 0.00001f;
    config.innovation_gate = 0.05f;
    Ekf filter(config);
    auto time = initialize(filter, config);
    for (unsigned i = 0; i < 100; ++i) {
        time += sample_us;
        zassert_ok(filter.update({{0, 0, gravity}, {}, time}));
    }
    time += sample_us;
    // Its magnitude is exactly g, so only the innovation gate can reject it.
    zassert_ok(filter.update({{gravity, 0, 0}, {}, time}));
    zassert_equal(filter.quality(), Ekf::Quality::Degraded);
    assertAttitude(filter.attitude(), referenceAttitude(0, 0, 0), 0.001);
    for (unsigned i = 0; i < 20; ++i) {
        time += sample_us;
        zassert_ok(filter.update({{0, 0, gravity}, {}, time}));
    }
    zassert_equal(filter.quality(), Ekf::Quality::Tracking);
    assertAttitude(filter.attitude(), referenceAttitude(0, 0, 0), 0.002);
}

ZTEST(attitude_ekf, test_initialization_requires_consecutive_stationary_samples) {
    auto config = testConfig();
    config.initialization_samples = 5;
    Ekf filter(config);
    zassert_ok(filter.init());
    TimeUs time = 0;
    for (unsigned i = 0; i < 3; ++i) {
        time += sample_us;
        zassert_equal(filter.update({{0, 0, gravity}, {}, time}), -EAGAIN);
    }
    time += sample_us;
    zassert_equal(filter.update({{0, 0, gravity}, {0, 0, 1}, time}), -EAGAIN);
    for (unsigned i = 0; i < 5; ++i) {
        time += sample_us;
        zassert_equal(filter.update({referenceGravity(0.3, -0.2), {}, time}), i == 4 ? 0 : -EAGAIN);
    }
    assertAttitude(filter.attitude(), referenceAttitude(0.3, -0.2, 0), 1e-4);
}

ZTEST(attitude_ekf, test_time_bounds_stale_timestamps_and_gap_reset) {
    auto config = testConfig();
    config.dt_min_s = 0.001f;
    config.dt_max_s = 0.01f;
    Ekf filter(config);
    auto time = initialize(filter, config);
    const auto generation = filter.generation();
    zassert_equal(filter.update({{0, 0, gravity}, {}, time}), -ESTALE);
    zassert_equal(filter.update({{0, 0, gravity}, {}, time - 1}), -ESTALE);
    zassert_equal(filter.update({{0, 0, gravity}, {}, time + 500}), -EAGAIN);
    time += 1000; // Rejected short samples must not advance the accepted timestamp.
    zassert_ok(filter.update({{0, 0, gravity}, {}, time}));
    time += 10000;
    zassert_ok(filter.update({{0, 0, gravity}, {}, time}));
    zassert_equal(filter.generation(), generation);
    time += 10001;
    zassert_equal(filter.update({{0, 0, gravity}, {}, time}), -EAGAIN);
    zassert_equal(filter.generation(), generation + 1);
    zassert_equal(filter.quality(), Ekf::Quality::Initializing);
    assertAttitude(filter.attitude(), referenceAttitude(0, 0, 0), 1e-4);
    for (unsigned i = 0; i < config.initialization_samples; ++i) {
        time += sample_us;
        zassert_equal(filter.update({referenceGravity(-0.4, 0.1), {}, time}),
                      i + 1 == config.initialization_samples ? 0 : -EAGAIN);
    }
    assertAttitude(filter.attitude(), referenceAttitude(-0.4, 0.1, 0), 1e-4);
}

ZTEST(attitude_ekf, test_init_contract_and_invalid_configuration) {
    const auto good = testConfig();
    Ekf uninitialized(good);
    zassert_equal(uninitialized.generation(), 0);
    zassert_equal(uninitialized.update({{0, 0, gravity}, {}, sample_us}), -EACCES);
    zassert_ok(uninitialized.init());
    const auto generation = uninitialized.generation();
    zassert_true(generation > 0);
    zassert_equal(uninitialized.init(), -EALREADY);
    zassert_equal(uninitialized.generation(), generation);

    float Ekf::Config::*members[] = {
        &Ekf::Config::dt_min_s, &Ekf::Config::dt_max_s, &Ekf::Config::gravity_m_s2,
        &Ekf::Config::accel_gate_m_s2, &Ekf::Config::process_noise, &Ekf::Config::measurement_noise,
        &Ekf::Config::innovation_gate, &Ekf::Config::accel_tau_s, &Ekf::Config::stationary_gyro_rad_s,
        &Ekf::Config::bias_tau_s,
    };
    const float invalid[] = {0, -1, std::numeric_limits<float>::quiet_NaN(),
                              std::numeric_limits<float>::infinity()};
    for (auto member : members)
        for (float value : invalid) {
            auto config = good;
            config.*member = value;
            Ekf filter(config);
            zassert_equal(filter.init(), -EINVAL);
            zassert_equal(filter.generation(), 0);
        }
    auto reversed = good;
    reversed.dt_min_s = reversed.dt_max_s * 2;
    Ekf invalid_interval(reversed);
    zassert_equal(invalid_interval.init(), -EINVAL);
    auto too_few = good;
    too_few.initialization_samples = 1;
    Ekf invalid_samples(too_few);
    zassert_equal(invalid_samples.init(), -EINVAL);
}

ZTEST(attitude_ekf, test_invalid_samples_do_not_consume_time_and_range_error_resets) {
    const auto config = testConfig();
    Ekf filter(config);
    auto time = initialize(filter, config);
    const auto generation = filter.generation();
    const auto expected = referenceAttitude(0, 0, 0);
    time += sample_us;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    zassert_equal(filter.update({{nan, 0, gravity}, {}, time}), -EINVAL);
    zassert_equal(filter.update({{0, 0, gravity}, {0, infinity, 0}, time}), -EINVAL);
    zassert_equal(filter.generation(), generation);
    assertAttitude(filter.attitude(), expected, 1e-4);
    zassert_ok(filter.update({{0, 0, gravity}, {}, time}));
    time += sample_us;
    const float huge = std::numeric_limits<float>::max();
    zassert_equal(filter.update({{huge, huge, huge}, {}, time}), -ERANGE);
    zassert_equal(filter.generation(), generation + 1);
    zassert_equal(filter.quality(), Ekf::Quality::Initializing);
    assertAttitude(filter.attitude(), expected, 1e-4);
}

ZTEST(attitude_ekf, test_manual_reset_clears_previous_attitude_bias_and_history) {
    const auto config = testConfig();
    Ekf filter(config);
    auto time = initialize(filter, config, referenceGravity(0.6, -0.3), {0.01f, 0.02f, -0.03f});
    const auto generation = filter.generation();
    filter.reset();
    zassert_equal(filter.generation(), generation + 1);
    zassert_equal(filter.quality(), Ekf::Quality::Initializing);
    assertAttitude(filter.attitude(), referenceAttitude(0, 0, 0), 1e-4);
    // A fresh stream is allowed to start with an earlier timestamp after reset.
    time = 0;
    for (unsigned i = 0; i < config.initialization_samples; ++i) {
        time += sample_us;
        zassert_equal(filter.update({{0, 0, gravity}, {}, time}),
                      i + 1 == config.initialization_samples ? 0 : -EAGAIN);
    }
    for (unsigned i = 0; i < 1000; ++i) {
        time += sample_us;
        zassert_ok(filter.update({{0, 0, gravity}, {}, time}));
    }
    assertAttitude(filter.attitude(), referenceAttitude(0, 0, 0), 0.002);
}

ZTEST(attitude_ekf, test_benchmark_tracking_update) {
    const auto config = testConfig();
    Ekf filter(config);
    auto time = initialize(filter, config);
    int last_status = 0;
    unsigned failures = 0;
    Ekf::Input input{{0, 0, gravity}, {}, time};
    skywalker::test::benchmark("ekf.update.tracking", 4000, [&](std::uint32_t) {
        input.time_us += sample_us;
        last_status = filter.update(input);
        failures += last_status != 0 || filter.quality() != Ekf::Quality::Tracking;
    });
    zassert_ok(last_status);
    zassert_equal(failures, 0);
    assertAttitude(filter.attitude(), referenceAttitude(0, 0, 0), 0.002);
}

ZTEST(attitude_ekf, test_benchmark_degraded_prediction_update) {
    const auto config = testConfig();
    Ekf filter(config);
    auto time = initialize(filter, config);
    int last_status = 0;
    unsigned failures = 0, calls = 0;
    Ekf::Input input{{0, 0, 2 * gravity}, {0, 0, 0.8f}, time};
    skywalker::test::benchmark("ekf.update.degraded", 4000, [&](std::uint32_t) {
        input.time_us += sample_us;
        last_status = filter.update(input);
        failures += last_status != 0 || filter.quality() != Ekf::Quality::Degraded;
        ++calls;
    });
    zassert_ok(last_status);
    zassert_equal(failures, 0);
    // Include warmup calls in the expected trajectory, without timing trig.
    assertAttitude(filter.attitude(), referenceAttitude(0, 0, 0.8 * calls * sample_s), 0.01);
}

ZTEST_SUITE(attitude_ekf, nullptr, nullptr, nullptr, nullptr, nullptr);
