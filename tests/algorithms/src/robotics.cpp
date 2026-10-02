#include <zephyr/ztest.h>
#include <benchmark.hpp>
#include <robotics/swerve/swerve_kinematics.hpp>
#include <robotics/swerve/swerve_chassis.hpp>
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
control_pid_config modulePid(float gain) {
    control_pid_config config{};
    config.kp = gain;
    config.integral_min = -100;
    config.integral_max = 100;
    config.output_min = -100;
    config.output_max = 100;
    config.dt_min_s = 0.001f;
    config.dt_max_s = 0.02f;
    return config;
}
SwerveModule::Config moduleConfig(float radius = 0.1f) {
    SwerveModule::Config config{};
    config.wheel_radius_m = radius;
    config.drive.regulator.feedback = modulePid(2);
    config.drive.reference_slew = {100000, 100000};
    config.drive.requested_velocity_abs_max_rad_s = 100;
    config.drive.effort_abs_max = 100;
    config.steer.position = modulePid(3);
    config.steer.velocity = config.drive;
    return config;
}
SwerveChassis::Config chassisConfig() {
    SwerveChassis::Config config{};
    config.kinematics = square();
    constexpr float radii[] = {0.1f, 0.2f, 0.05f, 0.15f};
    for (unsigned i = 0; i < 4; ++i) config.modules[i] = moduleConfig(radii[i]);
    return config;
}
void expect_module_equal(const ModuleOutput &actual, const ModuleOutput &expected) {
    zassert_within(actual.optimized_angle_rad, expected.optimized_angle_rad, 1e-5f);
    zassert_within(actual.optimized_wheel_velocity_m_s, expected.optimized_wheel_velocity_m_s, 1e-5f);
    zassert_within(actual.steer_continuous_target_rad, expected.steer_continuous_target_rad, 1e-5f);
    zassert_within(actual.drive_target_rad_s, expected.drive_target_rad_s, 1e-5f);
    zassert_within(actual.steer_effort, expected.steer_effort, 1e-5f);
    zassert_within(actual.drive_effort, expected.drive_effort, 1e-5f);
}
void expect_chassis_equal(const ChassisOutput &actual, const ChassisOutput &expected) {
    for (unsigned i = 0; i < 4; ++i) {
        zassert_within(actual.target[i].angle_rad, expected.target[i].angle_rad, 1e-5f);
        zassert_within(actual.target[i].wheel_velocity_m_s, expected.target[i].wheel_velocity_m_s, 1e-5f);
        expect_module_equal(actual.module[i], expected.module[i]);
    }
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

ZTEST(swerve_module, test_output_axis_units_and_continuous_steering) {
    SwerveModule module(moduleConfig());
    const ModuleFeedback feedback{0.2f, 8 * pi + 0.2f, 1, 5};
    zassert_ok(module.validate());
    zassert_ok(module.reset(feedback));
    ModuleOutput output{};
    zassert_ok(module.step({0.5f, 2}, feedback, 0.005f, output));
    zassert_within(output.optimized_angle_rad, 0.5f, 1e-6f);
    zassert_within(output.optimized_wheel_velocity_m_s, 2, 1e-6f);
    zassert_within(output.steer_continuous_target_rad, 8 * pi + 0.5f, 1e-5f);
    // The wheel target is output-axis rad/s: 2 m/s / 0.1 m = 20 rad/s.
    // P-only drive effort = 2 * (20 - 5); steer effort = 2 * (3 * 0.3 - 1).
    zassert_within(output.drive_target_rad_s, 20, 1e-5f);
    zassert_within(output.drive_effort, 30, 1e-5f);
    zassert_within(output.steer_effort, -0.2f, 2e-5f);
}

ZTEST(swerve_module, test_flipped_wheel_preserves_velocity_vector_and_wraps) {
    SwerveModule module(moduleConfig());
    ModuleFeedback feedback{0, 4 * pi, 0, 0};
    zassert_ok(module.reset(feedback));
    ModuleOutput output{};
    zassert_ok(module.step({3 * pi / 4, 2}, feedback, 0.005f, output));
    zassert_within(output.optimized_angle_rad, -pi / 4, 1e-6f);
    zassert_within(output.optimized_wheel_velocity_m_s, -2, 1e-6f);
    zassert_within(output.steer_continuous_target_rad, 4 * pi - pi / 4, 2e-6f);
    zassert_within(output.drive_target_rad_s, -20, 1e-5f);
    expect_vector({output.optimized_angle_rad, output.optimized_wheel_velocity_m_s},
                  -std::sqrt(2.0f), std::sqrt(2.0f));

    feedback = {pi - 0.1f, 7 * pi - 0.1f, 0, 0};
    zassert_ok(module.reset(feedback));
    zassert_ok(module.step({-pi + 0.1f, 1}, feedback, 0.005f, output));
    zassert_within(output.steer_continuous_target_rad, feedback.steer_continuous_rad + 0.2f, 1e-5f);
    zassert_within(output.optimized_wheel_velocity_m_s, 1, 1e-6f);
    zassert_within(output.drive_target_rad_s, 10, 1e-5f);
}

ZTEST(swerve_module, test_rejected_step_preserves_output_and_controller_history) {
    SwerveModule module(moduleConfig());
    const ModuleFeedback feedback{};
    ModuleOutput output{};
    output.drive_effort = 73;
    zassert_equal(module.step({0.2f, 1}, feedback, 0.005f, output), -EACCES);
    zassert_equal(output.drive_effort, 73);
    zassert_ok(module.reset(feedback));
    zassert_ok(module.step({0.2f, 1}, feedback, 0.005f, output));
    auto baseline = module;
    const auto before = output;
    zassert_equal(module.step({0.4f, 20}, feedback, 0.005f, output), -ERANGE);
    expect_module_equal(output, before);
    zassert_equal(module.step({0.4f, 1}, feedback, 0.03f, output), -ERANGE);
    expect_module_equal(output, before);
    ModuleOutput expected{};
    zassert_ok(baseline.step({0.6f, 2}, feedback, 0.005f, expected));
    zassert_ok(module.step({0.6f, 2}, feedback, 0.005f, output));
    expect_module_equal(output, expected);
}

ZTEST(swerve_chassis, test_four_modules_map_mixed_twist_to_different_radii) {
    const auto config = chassisConfig();
    SwerveChassis chassis(config);
    ChassisFeedback feedback{};
    for (unsigned i = 0; i < 4; ++i) {
        feedback.module[i].steer_continuous_rad = 2 * pi * i;
        feedback.module[i].drive_velocity_rad_s = i + 1;
    }
    zassert_ok(chassis.validate());
    zassert_ok(chassis.reset(feedback));
    ChassisOutput output{};
    zassert_ok(chassis.step({.mode = ChassisMode::BodyVelocity, .vx_m_s = 2, .vy_m_s = 1, .wz_rad_s = 2},
                           feedback, 0.005f, output));
    // FL, FR, RL, RR: rigid-body vectors (1,2), (3,2), (1,0), (3,0).
    constexpr float vx[] = {1, 3, 1, 3}, vy[] = {2, 2, 0, 0};
    for (unsigned i = 0; i < 4; ++i) {
        expect_vector(output.target[i], vx[i], vy[i]);
        const auto &module = output.module[i];
        const float expected_rad_s = std::hypot(vx[i], vy[i]) / config.modules[i].wheel_radius_m;
        zassert_within(module.drive_target_rad_s, expected_rad_s, 1e-5f);
        zassert_within(module.drive_effort, 2 * (expected_rad_s - (i + 1)), 1e-5f);
        zassert_within(module.steer_continuous_target_rad, 2 * pi * i + std::atan2(vy[i], vx[i]), 1e-5f);
        expect_vector({module.optimized_angle_rad, module.optimized_wheel_velocity_m_s}, vx[i], vy[i]);
    }
    const auto moving = output;
    zassert_ok(chassis.step({}, feedback, 0.005f, output));
    for (unsigned i = 0; i < 4; ++i) {
        zassert_equal(output.target[i].wheel_velocity_m_s, 0);
        zassert_equal(output.module[i].drive_target_rad_s, 0);
        zassert_within(output.target[i].angle_rad, moving.target[i].angle_rad, 1e-6f);
    }
}

ZTEST(swerve_chassis, test_rotation_retains_four_vector_directions_after_flipping) {
    const auto config = chassisConfig();
    SwerveChassis chassis(config);
    const ChassisFeedback feedback{};
    zassert_ok(chassis.reset(feedback));
    ChassisOutput output{};
    zassert_ok(chassis.step({.mode = ChassisMode::Spin, .vx_m_s = 100, .vy_m_s = -100, .wz_rad_s = 2},
                           feedback, 0.005f, output));
    constexpr float vx[] = {-1, 1, -1, 1}, vy[] = {1, 1, -1, -1};
    constexpr float sign[] = {-1, 1, -1, 1};
    for (unsigned i = 0; i < 4; ++i) {
        const auto &module = output.module[i];
        expect_vector({module.optimized_angle_rad, module.optimized_wheel_velocity_m_s}, vx[i], vy[i]);
        zassert_within(module.drive_target_rad_s,
                       sign[i] * std::sqrt(2.0f) / config.modules[i].wheel_radius_m, 1e-5f);
        zassert_true(std::fabs(module.steer_continuous_target_rad) <= pi / 2);
    }
}

ZTEST(swerve_chassis, test_last_module_failure_is_transactional_for_entire_chassis) {
    auto config = chassisConfig();
    config.modules[3].drive.requested_velocity_abs_max_rad_s = 2;
    SwerveChassis chassis(config);
    const ChassisFeedback feedback{};
    zassert_ok(chassis.reset(feedback));
    auto baseline = chassis;
    ChassisOutput output{};
    output.target[0] = {0.7f, 8};
    output.module[3].drive_effort = 73;
    const auto before = output;
    // The first three modules accept 1 m/s. RR rejects 1/0.15 rad/s > 2.
    zassert_equal(chassis.step({.mode = ChassisMode::BodyVelocity, .vx_m_s = 1},
                               feedback, 0.005f, output), -ERANGE);
    expect_chassis_equal(output, before);
    auto invalid_feedback = feedback;
    invalid_feedback.module[3].steer_absolute_rad = std::numeric_limits<float>::quiet_NaN();
    zassert_equal(chassis.reset(invalid_feedback), -EINVAL);
    ChassisOutput expected{};
    const ChassisCommand next{.mode = ChassisMode::BodyVelocity, .vy_m_s = 0.1f};
    zassert_ok(baseline.step(next, feedback, 0.005f, expected));
    zassert_ok(chassis.step(next, feedback, 0.005f, output));
    expect_chassis_equal(output, expected);
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
ZTEST_SUITE(swerve_module, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(swerve_chassis, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(chassis_power, nullptr, nullptr, nullptr, nullptr, nullptr);
