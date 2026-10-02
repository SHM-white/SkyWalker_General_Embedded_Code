#include <zephyr/ztest.h>
#include <benchmark.hpp>
#include <robotics/swerve/swerve_kinematics.hpp>
#include <robotics/chassis/chassis_power_limiter.hpp>
#include <cerrno>
#include <cmath>
#include <limits>

namespace {
using namespace skywalker::robotics;
constexpr float pi = 3.14159265358979323846f;
SwerveKinematics::Config square(float maximum = 10) {
    return {{{{0.5f, 0.5f}, {0.5f, -0.5f}, {-0.5f, 0.5f}, {-0.5f, -0.5f}}}, maximum, 0.01f};
}
void expect_vector(const ModuleTarget &target, float vx, float vy, float tolerance = 1e-5f) {
    zassert_true(std::isfinite(target.angle_rad) && std::isfinite(target.wheel_velocity_m_s));
    zassert_within(target.wheel_velocity_m_s * std::cos(target.angle_rad), vx, tolerance);
    zassert_within(target.wheel_velocity_m_s * std::sin(target.angle_rad), vy, tolerance);
}
}

ZTEST(swerve_kinematics, test_straight_and_sideways_translation) {
    SwerveKinematics solver(square());
    zassert_ok(solver.reset({}));
    ModuleTargets out;
    zassert_ok(solver.solve({.mode = ChassisMode::BodyVelocity, .vx_m_s = 3, .vy_m_s = 4}, out));
    for (const auto &target : out) {
        zassert_within(target.wheel_velocity_m_s, 5, 1e-6f);
        expect_vector(target, 3, 4);
    }
    zassert_ok(solver.solve({.mode = ChassisMode::BodyVelocity, .vy_m_s = -2}, out));
    for (const auto &target : out) {
        zassert_within(target.angle_rad, -pi / 2, 1e-6f);
        expect_vector(target, 0, -2);
    }
}

ZTEST(swerve_kinematics, test_rigid_body_rotation_and_spin_mode) {
    SwerveKinematics solver(square());
    zassert_ok(solver.reset({}));
    ModuleTargets out;
    zassert_ok(solver.solve({.mode = ChassisMode::Spin, .vx_m_s = 100, .vy_m_s = -100, .wz_rad_s = 2}, out));
    constexpr float expected_x[] = {-1, 1, -1, 1}, expected_y[] = {1, 1, -1, -1};
    for (unsigned i = 0; i < 4; ++i) {
        expect_vector(out[i], expected_x[i], expected_y[i]);
        zassert_within(out[i].wheel_velocity_m_s, std::sqrt(2.0f), 1e-6f);
    }
}

ZTEST(swerve_kinematics, test_mixed_twist_uniform_desaturation) {
    SwerveKinematics solver(square(2));
    zassert_ok(solver.reset({}));
    ModuleTargets out;
    zassert_ok(solver.solve({.mode = ChassisMode::BodyVelocity, .vx_m_s = 2, .vy_m_s = 1, .wz_rad_s = 2}, out));
    // Unscaled module vectors: (1,2), (3,2), (1,0), (3,0).
    // The fastest wheel is sqrt(13), so every vector scales by 2/sqrt(13).
    const float scale = 2 / std::sqrt(13.0f);
    constexpr float vx[] = {1, 3, 1, 3}, vy[] = {2, 2, 0, 0};
    for (unsigned i = 0; i < 4; ++i) {
        expect_vector(out[i], vx[i] * scale, vy[i] * scale);
        zassert_true(out[i].wheel_velocity_m_s <= 2.000001f);
    }
    zassert_within(out[1].wheel_velocity_m_s, 2, 1e-6f);
}

ZTEST(swerve_kinematics, test_stopped_wheels_keep_last_angle) {
    SwerveKinematics solver(square());
    ModuleTargets initial{};
    for (unsigned i = 0; i < 4; ++i) {
        initial[i].angle_rad = 0.2f * (i + 1);
    }
    zassert_ok(solver.reset(initial));
    ModuleTargets out;
    zassert_ok(solver.solve({.mode = ChassisMode::BodyVelocity, .vx_m_s = 0.005f}, out));
    for (unsigned i = 0; i < 4; ++i) {
        zassert_equal(out[i].wheel_velocity_m_s, 0);
        zassert_equal(out[i].angle_rad, initial[i].angle_rad);
    }
    zassert_ok(solver.solve({.mode = ChassisMode::BodyVelocity, .vy_m_s = 1}, out));
    zassert_ok(solver.solve({.mode = ChassisMode::Disabled, .vx_m_s = 3, .wz_rad_s = 5}, out));
    for (const auto &target : out) {
        zassert_equal(target.wheel_velocity_m_s, 0);
        zassert_within(target.angle_rad, pi / 2, 1e-6f);
    }
}

ZTEST(swerve_kinematics, test_invalid_input_does_not_change_output_or_history) {
    SwerveKinematics solver(square());
    ModuleTargets out{};
    out[0] = {0.7f, 9};
    zassert_equal(solver.solve({}, out), -EACCES);
    zassert_equal(out[0].wheel_velocity_m_s, 9);
    auto invalid = square(0);
    SwerveKinematics bad(invalid);
    zassert_equal(bad.validate(), -EINVAL);
    zassert_equal(bad.reset({}), -EINVAL);
    ModuleTargets initial{};
    initial[0].angle_rad = 0.3f;
    zassert_ok(solver.reset(initial));
    zassert_equal(solver.solve({.mode = ChassisMode::BodyVelocity,
                                .vx_m_s = std::numeric_limits<float>::quiet_NaN()}, out), -EINVAL);
    zassert_equal(out[0].wheel_velocity_m_s, 9);
    zassert_ok(solver.solve({}, out));
    zassert_equal(out[0].angle_rad, 0.3f);
}

ZTEST(swerve_kinematics, test_benchmark_solve) {
    SwerveKinematics solver(square(2));
    zassert_ok(solver.reset({}));
    const ChassisCommand input{.mode = ChassisMode::BodyVelocity, .vx_m_s = 2, .vy_m_s = 1, .wz_rad_s = 2};
    ModuleTargets out{};
    int failures = 0;
    skywalker::test::benchmark("swerve.solve.4_modules", 4096, [&](std::uint32_t) {
        failures += solver.solve(input, out) != 0;
    });
    zassert_equal(failures, 0);
    zassert_within(out[1].wheel_velocity_m_s, 2, 1e-6f);
}

ZTEST(chassis_power, test_recovery_limit_buffer_and_immediate_reduction) {
    ChassisPowerLimiter limiter({.recovery_per_s = 2, .buffer_reserve_j = 10});
    zassert_ok(limiter.reset());
    ChassisPowerDecision out;
    const ChassisPowerInput nominal{.measured_power_w = 50, .power_limit_w = 100, .buffer_energy_j = 10};
    for (unsigned i = 1; i <= 5; ++i) {
        zassert_ok(limiter.step(nominal, 0.1f, out));
        zassert_within(out.effort_scale, i * 0.2f, 1e-6f);
    }
    zassert_ok(limiter.step({200, 100, 10}, 0.1f, out));
    zassert_within(out.effort_scale, 0.5f, 1e-6f);
    zassert_ok(limiter.step({200, 100, 5}, 0.1f, out));
    zassert_within(out.effort_scale, 0.25f, 1e-6f);
    zassert_ok(limiter.step(nominal, 0.1f, out));
    zassert_within(out.effort_scale, 0.45f, 1e-6f);
    zassert_ok(limiter.step({50, 0, 10}, 0.1f, out));
    zassert_equal(out.effort_scale, 0);
    zassert_ok(limiter.step(nominal, 0.1f, out));
    zassert_within(out.effort_scale, 0.2f, 1e-6f);
    zassert_ok(limiter.step({50, 100, 0}, 0.1f, out));
    zassert_equal(out.effort_scale, 0);
}

ZTEST(chassis_power, test_invalid_input_is_transactional) {
    ChassisPowerLimiter limiter;
    zassert_ok(limiter.reset());
    ChassisPowerDecision out{.effort_scale = 9};
    zassert_equal(limiter.step({-1, 100, 10}, 0.1f, out), -EINVAL);
    zassert_equal(limiter.step({1, 100, 10}, 0, out), -EINVAL);
    zassert_equal(limiter.step({1, 100, 10}, 0.101f, out), -EINVAL);
    zassert_equal(limiter.step({1, 100, std::numeric_limits<float>::infinity()}, 0.1f, out), -EINVAL);
    zassert_equal(out.effort_scale, 9);
    zassert_ok(limiter.step({50, 100, 10}, 0.1f, out));
    zassert_within(out.effort_scale, 0.05f, 1e-6f);
}

ZTEST(chassis_power, test_benchmark_step) {
    ChassisPowerLimiter limiter;
    zassert_ok(limiter.reset());
    const ChassisPowerInput input{50, 100, 10};
    ChassisPowerDecision out;
    int failures = 0;
    skywalker::test::benchmark("chassis_power.step", 4096, [&](std::uint32_t) {
        failures += limiter.step(input, 0.01f, out) != 0;
    });
    zassert_equal(failures, 0);
    zassert_equal(out.effort_scale, 1);
}

ZTEST_SUITE(swerve_kinematics, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(chassis_power, nullptr, nullptr, nullptr, nullptr, nullptr);
