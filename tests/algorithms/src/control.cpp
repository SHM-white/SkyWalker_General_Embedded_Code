#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include <zephyr/ztest.h>

#include <benchmark.hpp>
#include <control/angle.h>
#include <control/feedforward.h>
#include <control/feedforward_pid.h>
#include <control/motor_position.h>
#include <control/motor_velocity.h>
#include <control/pid.h>
#include <control/slew_rate_limiter.h>

namespace {
constexpr float pi = 3.14159265358979323846f;
constexpr float tolerance = 1.0e-5f;
constexpr std::uint32_t benchmark_iterations = 4000;

control_pid_config pidConfig() {
    control_pid_config config{};
    config.integral_min = -100.0f;
    config.integral_max = 100.0f;
    config.output_min = -100.0f;
    config.output_max = 100.0f;
    config.dt_min_s = 0.001f;
    config.dt_max_s = 1.0f;
    return config;
}

control_feedforward_config feedforwardConfig() {
    control_feedforward_config config{};
    config.k_bias = 1.0f;
    config.k_static = 2.0f;
    config.k_velocity = 3.0f;
    config.k_acceleration = 4.0f;
    config.k_gravity = 5.0f;
    config.velocity_epsilon = 0.1f;
    config.acceleration_epsilon = 0.1f;
    config.gravity_model = CONTROL_GRAVITY_NONE;
    return config;
}

control_motor_velocity_config velocityConfig() {
    control_motor_velocity_config config{};
    config.regulator.feedback = pidConfig();
    config.regulator.feedback.kp = 2.0f;
    config.reference_slew = {1000.0f, 1000.0f};
    config.requested_velocity_abs_max_rad_s = 20.0f;
    config.effort_abs_max = 100.0f;
    return config;
}

control_motor_position_config positionConfig() {
    control_motor_position_config config{};
    config.position = pidConfig();
    config.position.kp = 3.0f;
    config.position.ki = 4.0f;
    config.position.output_min = -10.0f;
    config.position.output_max = 10.0f;
    config.velocity = velocityConfig();
    return config;
}

template <typename T> std::array<unsigned char, sizeof(T)> bytesOf(const T &value) {
    std::array<unsigned char, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(T));
    return bytes;
}
}

ZTEST(control_pid, test_known_proportional_integral_derivative_terms) {
    auto config = pidConfig();
    config.kp = 2.0f;
    config.ki = 3.0f;
    config.kd = 4.0f;
    control_pid_state state{};
    control_pid_result result{};
    control_pid_input input{3.0f, 1.0f, 0.1f, false};
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.p, 4.0f, tolerance);
    zassert_within(result.i, 0.6f, tolerance);
    zassert_within(result.d, 0.0f, tolerance);
    zassert_within(result.output, 4.6f, tolerance);
    zassert_false(result.saturated);

    input.setpoint = 4.0f;
    input.measurement = 1.5f;
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.error, 2.5f, tolerance);
    zassert_within(result.p, 5.0f, tolerance);
    zassert_within(result.i, 1.35f, tolerance);
    zassert_within(result.d, -20.0f, tolerance);
    zassert_within(result.output, -13.65f, tolerance);
}

ZTEST(control_pid, test_derivative_follows_measurement_without_setpoint_kick) {
    auto config = pidConfig();
    config.kd = 2.0f;
    control_pid_state state{};
    control_pid_result result{};
    zassert_ok(control_pid_reset(&state, 0.0f));
    control_pid_input input{10.0f, 0.0f, 0.1f, false};
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.d, 0.0f, tolerance);
    input.setpoint = -10.0f;
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.d, 0.0f, tolerance);
    input.measurement = 1.0f;
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.d, -20.0f, tolerance);
}

ZTEST(control_pid, test_first_order_derivative_filter_step_response) {
    auto config = pidConfig();
    config.kd = 2.0f;
    config.derivative_tau_s = 0.1f;
    control_pid_state state{};
    control_pid_result result{};
    zassert_ok(control_pid_reset(&state, 0.0f));
    const control_pid_input input{0.0f, 1.0f, 0.1f, false};
    // A 10 unit/s impulse with dt = tau has half its amplitude at each step.
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.d, -10.0f, tolerance);
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.d, -5.0f, tolerance);
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.d, -2.5f, tolerance);
}

ZTEST(control_pid, test_deadband_boundary_and_integral_limits) {
    auto config = pidConfig();
    config.kp = 2.0f;
    config.ki = 4.0f;
    config.deadband = 0.5f;
    config.integral_min = -1.0f;
    config.integral_max = 1.0f;
    control_pid_state state{};
    control_pid_result result{};
    control_pid_input input{0.5f, 0.0f, 0.1f, false};
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.error, 0.5f, tolerance);
    zassert_within(result.effective_error, 0.0f, tolerance);
    zassert_within(result.output, 0.0f, tolerance);
    input.setpoint = 2.0f;
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.i, 0.8f, tolerance);
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.i, 1.0f, tolerance);
    input.setpoint = -2.0f;
    for (int i = 0; i < 3; ++i) {
        zassert_ok(control_pid_step(&state, &config, &input, &result));
    }
    zassert_within(result.i, -1.0f, tolerance);
}

ZTEST(control_pid, test_asymmetric_saturation_prevents_integrator_windup) {
    auto config = pidConfig();
    config.kp = 2.0f;
    config.ki = 1.0f;
    config.output_min = -2.0f;
    config.output_max = 3.0f;
    control_pid_state state{};
    control_pid_result result{};
    control_pid_input input{2.0f, 0.0f, 0.1f, false};
    for (int i = 0; i < 20; ++i) {
        zassert_ok(control_pid_step(&state, &config, &input, &result));
    }
    zassert_true(result.saturated);
    zassert_within(result.output, 3.0f, tolerance);
    zassert_within(result.i, 0.0f, tolerance);
    input.setpoint = -2.0f;
    for (int i = 0; i < 20; ++i) {
        zassert_ok(control_pid_step(&state, &config, &input, &result));
    }
    zassert_within(result.output, -2.0f, tolerance);
    zassert_within(result.i, 0.0f, tolerance);
}

ZTEST(control_pid, test_integrator_can_unwind_while_output_is_saturated) {
    auto config = pidConfig();
    config.ki = 1.0f;
    config.output_min = -1.0f;
    config.output_max = 1.0f;
    control_pid_state state{};
    control_pid_result result{};
    zassert_ok(control_pid_reset(&state, 0.0f));
    state.integral_output = 2.0f;
    const control_pid_input input{-1.0f, 0.0f, 0.1f, false};
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_true(result.saturated);
    zassert_within(result.output, 1.0f, tolerance);
    zassert_within(result.i, 1.9f, tolerance);
}

ZTEST(control_pid, test_freeze_integrator_and_reset) {
    auto config = pidConfig();
    config.ki = 2.0f;
    control_pid_state state{};
    control_pid_result result{};
    control_pid_input input{1.0f, 0.0f, 0.1f, false};
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.i, 0.2f, tolerance);
    input.freeze_integrator = true;
    for (int i = 0; i < 10; ++i) {
        zassert_ok(control_pid_step(&state, &config, &input, &result));
    }
    zassert_within(result.i, 0.2f, tolerance);
    input.freeze_integrator = false;
    input.setpoint = -1.0f;
    zassert_ok(control_pid_step(&state, &config, &input, &result));
    zassert_within(result.i, 0.0f, tolerance);
    zassert_ok(control_pid_reset(&state, 7.0f));
    zassert_true(state.initialized);
    zassert_within(state.integral_output, 0.0f, tolerance);
    zassert_within(state.previous_measurement, 7.0f, tolerance);
    zassert_within(state.filtered_measurement_rate, 0.0f, tolerance);
}

ZTEST(control_pid, test_invalid_dt_and_nonfinite_input_preserve_sample) {
    auto config = pidConfig();
    config.kp = 1.0f;
    control_pid_state state{};
    control_pid_result result{};
    zassert_ok(control_pid_reset(&state, 2.0f));
    result.output = 123.0f;
    const auto state_before = bytesOf(state);
    const auto result_before = bytesOf(result);
    control_pid_input input{3.0f, 2.0f, 0.0f, false};
    zassert_equal(control_pid_step(&state, &config, &input, &result), -ERANGE);
    zassert_mem_equal(&state, state_before.data(), sizeof(state));
    zassert_mem_equal(&result, result_before.data(), sizeof(result));
    input.dt_s = 0.1f;
    input.measurement = std::numeric_limits<float>::quiet_NaN();
    zassert_equal(control_pid_step(&state, &config, &input, &result), -EINVAL);
    zassert_mem_equal(&state, state_before.data(), sizeof(state));
    zassert_mem_equal(&result, result_before.data(), sizeof(result));
}

ZTEST(control_pid, test_benchmark_step) {
    auto config = pidConfig();
    config.kp = 2.0f;
    config.ki = 0.1f;
    config.kd = 0.05f;
    config.derivative_tau_s = 0.01f;
    control_pid_state state{};
    control_pid_result result{};
    zassert_ok(control_pid_reset(&state, 0.0f));
    int errors = 0;
    float checksum = 0.0f;
    skywalker::test::benchmark("pid_step", benchmark_iterations, [&](std::uint32_t index) {
        const control_pid_input input{1.0f, (index & 1u) ? 0.25f : -0.25f, 0.001f, false};
        errors |= control_pid_step(&state, &config, &input, &result);
        checksum += result.output;
    });
    zassert_ok(errors);
    zassert_true(std::isfinite(checksum));
    zassert_true(std::fabs(result.output) <= 100.0f);
}

ZTEST(control_feedforward, test_bias_static_velocity_acceleration_and_direction_priority) {
    const auto config = feedforwardConfig();
    float output = 0.0f;
    control_feedforward_reference reference{0.0f, 2.0f, 3.0f};
    zassert_ok(control_feedforward_calculate(&config, &reference, &output));
    zassert_within(output, 21.0f, tolerance);
    reference.velocity_ref = -2.0f;
    zassert_ok(control_feedforward_calculate(&config, &reference, &output));
    zassert_within(output, 5.0f, tolerance);
    reference.velocity_ref = 0.0f;
    reference.acceleration_ref = -3.0f;
    zassert_ok(control_feedforward_calculate(&config, &reference, &output));
    zassert_within(output, -13.0f, tolerance);
    reference.velocity_ref = 0.1f;
    reference.acceleration_ref = 0.1f;
    zassert_ok(control_feedforward_calculate(&config, &reference, &output));
    zassert_within(output, 1.7f, tolerance);
    reference = {};
    zassert_ok(control_feedforward_calculate(&config, &reference, &output));
    zassert_within(output, 1.0f, tolerance);
}

ZTEST(control_feedforward, test_sine_and_cosine_gravity_at_cardinal_angles) {
    auto config = feedforwardConfig();
    config.gravity_model = CONTROL_GRAVITY_SIN;
    control_feedforward_reference reference{pi / 2.0f, 0.0f, 0.0f};
    float output = 0.0f;
    zassert_ok(control_feedforward_calculate(&config, &reference, &output));
    zassert_within(output, 6.0f, tolerance);
    reference.position_ref_rad = -pi / 2.0f;
    zassert_ok(control_feedforward_calculate(&config, &reference, &output));
    zassert_within(output, -4.0f, tolerance);
    config.gravity_model = CONTROL_GRAVITY_COS;
    reference.position_ref_rad = 0.0f;
    zassert_ok(control_feedforward_calculate(&config, &reference, &output));
    zassert_within(output, 6.0f, tolerance);
    reference.position_ref_rad = pi;
    zassert_ok(control_feedforward_calculate(&config, &reference, &output));
    zassert_within(output, -4.0f, tolerance);
    reference.position_ref_rad = std::numeric_limits<float>::infinity();
    output = 123.0f;
    zassert_equal(control_feedforward_calculate(&config, &reference, &output), -EINVAL);
    zassert_within(output, 123.0f, tolerance);
}

ZTEST(control_feedforward, test_benchmark_calculate) {
    auto config = feedforwardConfig();
    config.gravity_model = CONTROL_GRAVITY_SIN;
    float output = 0.0f;
    float checksum = 0.0f;
    int errors = 0;
    skywalker::test::benchmark("feedforward_calculate", benchmark_iterations, [&](std::uint32_t index) {
        const control_feedforward_reference reference{pi / 2.0f, (index & 1u) ? 2.0f : -2.0f, 3.0f};
        errors |= control_feedforward_calculate(&config, &reference, &output);
        checksum += output;
    });
    zassert_ok(errors);
    zassert_true(std::isfinite(checksum));
    zassert_true(output == 26.0f || output == 10.0f);
}

ZTEST(control_feedforward_pid, test_feedback_and_feedforward_sum) {
    control_feedforward_pid_config config{};
    config.feedback = pidConfig();
    config.feedback.kp = 1.5f;
    config.feedback.ki = 1.0f;
    config.feedforward = feedforwardConfig();
    control_feedforward_pid_state state{};
    control_feedforward_pid_result result{};
    const control_feedforward_pid_input input{{3.0f, 1.0f, 0.1f, false}, {0.0f, 2.0f, 3.0f}};
    zassert_ok(control_feedforward_pid_step(&state, &config, &input, &result));
    zassert_within(result.feedback.p, 3.0f, tolerance);
    zassert_within(result.feedback.i, 0.2f, tolerance);
    zassert_within(result.feedback.feedback_unsaturated, 3.2f, tolerance);
    zassert_within(result.feedforward, 21.0f, tolerance);
    zassert_within(result.feedback.total_unsaturated, 24.2f, tolerance);
    zassert_within(result.output, 24.2f, tolerance);
}

ZTEST(control_feedforward_pid, test_combined_saturation_freezes_and_then_unwinds_integrator) {
    control_feedforward_pid_config config{};
    config.feedback = pidConfig();
    config.feedback.kp = 2.0f;
    config.feedback.ki = 1.0f;
    config.feedback.output_min = -5.0f;
    config.feedback.output_max = 5.0f;
    config.feedforward.k_bias = 4.0f;
    control_feedforward_pid_state state{};
    control_feedforward_pid_result result{};
    control_feedforward_pid_input input{{1.0f, 0.0f, 0.1f, false}, {}};
    for (int i = 0; i < 20; ++i) {
        zassert_ok(control_feedforward_pid_step(&state, &config, &input, &result));
    }
    zassert_true(result.feedback.saturated);
    zassert_within(result.feedback.i, 0.0f, tolerance);
    zassert_within(result.feedback.total_unsaturated, 6.0f, tolerance);
    zassert_within(result.output, 5.0f, tolerance);
    input.feedback.setpoint = -1.0f;
    zassert_ok(control_feedforward_pid_step(&state, &config, &input, &result));
    zassert_false(result.feedback.saturated);
    zassert_within(result.feedback.i, -0.1f, tolerance);
    zassert_within(result.output, 1.9f, tolerance);
}

ZTEST(control_feedforward_pid, test_benchmark_step) {
    control_feedforward_pid_config config{};
    config.feedback = pidConfig();
    config.feedback.kp = 2.0f;
    config.feedforward = feedforwardConfig();
    control_feedforward_pid_state state{};
    control_feedforward_pid_result result{};
    zassert_ok(control_feedforward_pid_reset(&state, 0.0f));
    int errors = 0;
    float checksum = 0.0f;
    skywalker::test::benchmark("feedforward_pid_step", benchmark_iterations, [&](std::uint32_t index) {
        const control_feedforward_pid_input input{{1.0f, (index & 1u) ? 0.25f : -0.25f, 0.001f, false},
                                                  {0.0f, 2.0f, 3.0f}};
        errors |= control_feedforward_pid_step(&state, &config, &input, &result);
        checksum += result.output;
    });
    zassert_ok(errors);
    zassert_true(std::isfinite(checksum));
    zassert_true(std::fabs(result.output) <= 100.0f);
}

ZTEST(control_slew_rate, test_asymmetric_rise_fall_and_no_overshoot) {
    const control_slew_rate_config config{2.0f, 4.0f};
    control_slew_rate_state state{};
    zassert_ok(control_slew_rate_reset(&state, 1.0f));
    float value = 0.0f;
    float rate = 0.0f;
    zassert_ok(control_slew_rate_step(&state, &config, 10.0f, 0.25f, &value, &rate));
    zassert_within(value, 1.5f, tolerance);
    zassert_within(rate, 2.0f, tolerance);
    zassert_ok(control_slew_rate_step(&state, &config, -10.0f, 0.25f, &value, &rate));
    zassert_within(value, 0.5f, tolerance);
    zassert_within(rate, -4.0f, tolerance);
    zassert_ok(control_slew_rate_step(&state, &config, 0.75f, 0.25f, &value, &rate));
    zassert_within(value, 0.75f, tolerance);
    zassert_within(rate, 1.0f, tolerance);
    zassert_ok(control_slew_rate_step(&state, &config, 0.75f, 0.25f, &value, &rate));
    zassert_within(rate, 0.0f, tolerance);
}

ZTEST(control_slew_rate, test_zero_rate_holds_and_invalid_dt_preserves_state) {
    const control_slew_rate_config config{};
    control_slew_rate_state state{};
    float value = 123.0f;
    float rate = 456.0f;
    zassert_equal(control_slew_rate_step(&state, &config, 10.0f, 0.1f, &value, &rate), -EACCES);
    zassert_ok(control_slew_rate_reset(&state, 2.0f));
    zassert_ok(control_slew_rate_step(&state, &config, 10.0f, 0.1f, &value, &rate));
    zassert_within(value, 2.0f, tolerance);
    zassert_within(rate, 0.0f, tolerance);
    const auto before = bytesOf(state);
    value = 123.0f;
    rate = 456.0f;
    zassert_equal(control_slew_rate_step(&state, &config, -10.0f, 0.0f, &value, &rate), -ERANGE);
    zassert_mem_equal(&state, before.data(), sizeof(state));
    zassert_within(value, 123.0f, tolerance);
    zassert_within(rate, 456.0f, tolerance);
}

ZTEST(control_slew_rate, test_benchmark_step) {
    const control_slew_rate_config config{2.0f, 4.0f};
    control_slew_rate_state state{};
    zassert_ok(control_slew_rate_reset(&state, 0.0f));
    float value = 0.0f;
    float rate = 0.0f;
    float checksum = 0.0f;
    int errors = 0;
    skywalker::test::benchmark("slew_rate_step", benchmark_iterations, [&](std::uint32_t index) {
        errors |= control_slew_rate_step(&state, &config, (index & 1u) ? 1.0f : -1.0f, 0.001f, &value, &rate);
        checksum += value;
    });
    zassert_ok(errors);
    zassert_true(std::isfinite(checksum));
    zassert_true(rate >= -4.001f && rate <= 2.001f);
}

ZTEST(control_angle, test_unwrap_crosses_boundary_in_both_directions) {
    control_angle_unwrapper state{};
    float continuous = 0.0f;
    zassert_ok(control_angle_unwrap_reset(&state, pi - 0.1f));
    zassert_ok(control_angle_unwrap_step(&state, -pi + 0.1f, &continuous));
    zassert_within(continuous, pi + 0.1f, tolerance);
    zassert_ok(control_angle_unwrap_step(&state, pi - 0.1f, &continuous));
    zassert_within(continuous, pi - 0.1f, tolerance);
}

ZTEST(control_angle, test_unwrap_tracks_multiple_revolutions) {
    constexpr float forward[] = {pi / 2.0f, pi, -pi / 2.0f, 0.0f, pi / 2.0f, pi, -pi / 2.0f, 0.0f};
    constexpr float backward[] = {-pi / 2.0f, -pi, pi / 2.0f, 0.0f, -pi / 2.0f, -pi, pi / 2.0f, 0.0f};
    control_angle_unwrapper state{};
    float continuous = 0.0f;
    zassert_ok(control_angle_unwrap_reset(&state, 0.0f));
    for (std::size_t i = 0; i < std::size(forward); ++i) {
        zassert_ok(control_angle_unwrap_step(&state, forward[i], &continuous));
        zassert_within(continuous, (i + 1) * pi / 2.0f, 2.0e-5f);
    }
    zassert_ok(control_angle_unwrap_reset(&state, 0.0f));
    for (std::size_t i = 0; i < std::size(backward); ++i) {
        zassert_ok(control_angle_unwrap_step(&state, backward[i], &continuous));
        zassert_within(continuous, -(static_cast<float>(i) + 1.0f) * pi / 2.0f, 2.0e-5f);
    }
}

ZTEST(control_angle, test_shortest_error_and_canonical_half_turn) {
    const float degree = pi / 180.0f;
    float error = 0.0f;
    zassert_ok(control_shortest_angle_error(degree, 359.0f * degree, &error));
    zassert_within(error, 2.0f * degree, tolerance);
    zassert_ok(control_shortest_angle_error(359.0f * degree, degree, &error));
    zassert_within(error, -2.0f * degree, tolerance);
    zassert_ok(control_shortest_angle_error(pi, 0.0f, &error));
    zassert_within(error, -pi, tolerance);
    zassert_ok(control_shortest_angle_error(-pi, 0.0f, &error));
    zassert_within(error, -pi, tolerance);
    error = 123.0f;
    zassert_equal(control_shortest_angle_error(std::numeric_limits<float>::quiet_NaN(), 0.0f, &error), -EINVAL);
    zassert_within(error, 123.0f, tolerance);
}

ZTEST(control_angle, test_nearest_target_preserves_revolution_count) {
    const float degree = pi / 180.0f;
    float target = 0.0f;
    zassert_ok(control_angle_nearest_continuous_target(10.0f * degree, 350.0f * degree, 710.0f * degree, &target));
    zassert_within(target, 730.0f * degree, 2.0e-5f);
    zassert_ok(control_angle_nearest_continuous_target(350.0f * degree, 10.0f * degree, -710.0f * degree, &target));
    zassert_within(target, -730.0f * degree, 2.0e-5f);
}

ZTEST(control_angle, test_benchmark_unwrap) {
    control_angle_unwrapper state{};
    zassert_ok(control_angle_unwrap_reset(&state, 0.0f));
    float continuous = 0.0f;
    float checksum = 0.0f;
    int errors = 0;
    skywalker::test::benchmark("angle_unwrap_step", benchmark_iterations, [&](std::uint32_t index) {
        errors |= control_angle_unwrap_step(&state, static_cast<float>(index) * 0.001f, &continuous);
        checksum += continuous;
    });
    zassert_ok(errors);
    zassert_true(std::isfinite(checksum));
}

ZTEST(control_angle, test_benchmark_shortest_error) {
    float error = 0.0f;
    float checksum = 0.0f;
    int errors = 0;
    skywalker::test::benchmark("shortest_angle_error", benchmark_iterations, [&](std::uint32_t index) {
        errors |= control_shortest_angle_error(static_cast<float>(index) * 0.1f, 0.25f, &error);
        checksum += error;
    });
    zassert_ok(errors);
    zassert_true(std::isfinite(checksum));
    zassert_true(error >= -pi && error < pi);
}

ZTEST(control_angle, test_benchmark_nearest_target) {
    float target = 0.0f;
    float checksum = 0.0f;
    int errors = 0;
    skywalker::test::benchmark("angle_nearest_continuous_target", benchmark_iterations, [&](std::uint32_t index) {
        errors |= control_angle_nearest_continuous_target(static_cast<float>(index) * 0.1f, 0.25f, 20.0f, &target);
        checksum += target;
    });
    zassert_ok(errors);
    zassert_true(std::isfinite(checksum));
    zassert_true(std::fabs(target - 20.0f) <= pi);
}

ZTEST(control_motor_velocity, test_known_proportional_velocity_output) {
    const auto config = velocityConfig();
    control_motor_velocity_state state{};
    control_motor_velocity_output output{};
    zassert_ok(control_motor_velocity_reset(&state, 1.0f, 0.0f));
    const control_motor_velocity_input input{4.0f, 1.0f, 0.0f, 0.1f, false};
    zassert_ok(control_motor_velocity_step(&state, &config, &input, &output));
    zassert_within(output.velocity_reference_rad_s, 4.0f, tolerance);
    zassert_within(output.acceleration_reference_rad_s2, 40.0f, tolerance);
    zassert_within(output.filtered_velocity_rad_s, 1.0f, tolerance);
    zassert_within(output.velocity_error_rad_s, 3.0f, tolerance);
    zassert_within(output.effort_command, 6.0f, tolerance);
}

ZTEST(control_motor_velocity, test_velocity_acceleration_feedforward_added_to_feedback) {
    auto config = velocityConfig();
    config.regulator.feedforward.k_bias = 1.0f;
    config.regulator.feedforward.k_velocity = 2.0f;
    config.regulator.feedforward.k_acceleration = 0.5f;
    control_motor_velocity_state state{};
    control_motor_velocity_output output{};
    zassert_ok(control_motor_velocity_reset(&state, 1.0f, 0.0f));
    const control_motor_velocity_input input{4.0f, 1.0f, 0.0f, 0.1f, false};
    zassert_ok(control_motor_velocity_step(&state, &config, &input, &output));
    zassert_within(output.regulator.feedforward, 29.0f, tolerance);
    zassert_within(output.regulator.feedback.feedback_unsaturated, 6.0f, tolerance);
    zassert_within(output.effort_command, 35.0f, tolerance);
}

ZTEST(control_motor_velocity, test_measurement_filter_and_soft_deadband_response) {
    auto config = velocityConfig();
    config.measurement_filter_tau_s = 0.1f;
    config.soft_deadband_rad_s = 0.25f;
    control_motor_velocity_state state{};
    control_motor_velocity_output output{};
    zassert_ok(control_motor_velocity_reset(&state, 0.0f, 0.0f));
    const control_motor_velocity_input input{2.0f, 2.0f, 0.0f, 0.1f, false};
    zassert_ok(control_motor_velocity_step(&state, &config, &input, &output));
    zassert_within(output.filtered_velocity_rad_s, 1.0f, tolerance);
    zassert_within(output.effort_command, 1.5f, tolerance);
    zassert_ok(control_motor_velocity_step(&state, &config, &input, &output));
    zassert_within(output.filtered_velocity_rad_s, 1.5f, tolerance);
    zassert_within(output.effort_command, 0.5f, tolerance);
    zassert_ok(control_motor_velocity_step(&state, &config, &input, &output));
    zassert_within(output.filtered_velocity_rad_s, 1.75f, tolerance);
    zassert_within(output.effort_command, 0.0f, tolerance);
}

ZTEST(control_motor_velocity, test_reference_slew_and_actuator_effort_limits) {
    auto config = velocityConfig();
    config.reference_slew = {2.0f, 4.0f};
    config.effort_abs_max = 0.5f;
    control_motor_velocity_state state{};
    control_motor_velocity_output output{};
    zassert_ok(control_motor_velocity_reset(&state, 0.0f, 0.0f));
    control_motor_velocity_input input{10.0f, 0.0f, 0.0f, 0.25f, false};
    zassert_ok(control_motor_velocity_step(&state, &config, &input, &output));
    zassert_within(output.velocity_reference_rad_s, 0.5f, tolerance);
    zassert_within(output.acceleration_reference_rad_s2, 2.0f, tolerance);
    zassert_within(output.regulator.output, 1.0f, tolerance);
    zassert_within(output.effort_command, 0.5f, tolerance);
    input.requested_velocity_rad_s = -10.0f;
    zassert_ok(control_motor_velocity_step(&state, &config, &input, &output));
    zassert_within(output.velocity_reference_rad_s, -0.5f, tolerance);
    zassert_within(output.acceleration_reference_rad_s2, -4.0f, tolerance);
    zassert_within(output.effort_command, -0.5f, tolerance);
}

ZTEST(control_motor_velocity, test_rejected_request_preserves_state_and_output) {
    const auto config = velocityConfig();
    control_motor_velocity_state state{};
    control_motor_velocity_output output{};
    output.effort_command = 123.0f;
    control_motor_velocity_input input{1.0f, 0.0f, 0.0f, 0.1f, false};
    zassert_equal(control_motor_velocity_step(&state, &config, &input, &output), -EACCES);
    zassert_ok(control_motor_velocity_reset(&state, 0.0f, 0.0f));
    const auto state_before = bytesOf(state);
    const auto output_before = bytesOf(output);
    input.requested_velocity_rad_s = 21.0f;
    zassert_equal(control_motor_velocity_step(&state, &config, &input, &output), -ERANGE);
    zassert_mem_equal(&state, state_before.data(), sizeof(state));
    zassert_mem_equal(&output, output_before.data(), sizeof(output));
    input.requested_velocity_rad_s = 1.0f;
    input.measured_velocity_rad_s = std::numeric_limits<float>::quiet_NaN();
    zassert_equal(control_motor_velocity_step(&state, &config, &input, &output), -EINVAL);
    zassert_mem_equal(&state, state_before.data(), sizeof(state));
    zassert_mem_equal(&output, output_before.data(), sizeof(output));
}

ZTEST(control_motor_velocity, test_benchmark_step) {
    auto config = velocityConfig();
    config.measurement_filter_tau_s = 0.01f;
    control_motor_velocity_state state{};
    control_motor_velocity_output output{};
    zassert_ok(control_motor_velocity_reset(&state, 0.0f, 0.0f));
    float checksum = 0.0f;
    int errors = 0;
    skywalker::test::benchmark("motor_velocity_step", benchmark_iterations, [&](std::uint32_t index) {
        const control_motor_velocity_input input{(index & 1u) ? 2.0f : -2.0f, 0.1f, 0.0f, 0.001f, false};
        errors |= control_motor_velocity_step(&state, &config, &input, &output);
        checksum += output.effort_command;
    });
    zassert_ok(errors);
    zassert_true(std::isfinite(checksum));
    zassert_true(std::fabs(output.effort_command) <= config.effort_abs_max);
}

ZTEST(control_motor_position, test_known_cascaded_position_velocity_output_and_frozen_outer_integral) {
    const auto config = positionConfig();
    control_motor_position_state state{};
    control_motor_position_output output{};
    zassert_ok(control_motor_position_reset(&state, &config, 0.5f, 1.0f));
    const control_motor_position_input input{2.0f, 0.5f, 1.0f, 0.1f, 0.0f, false};
    for (int i = 0; i < 30; ++i) {
        zassert_ok(control_motor_position_step(&state, &config, &input, &output));
    }
    zassert_within(output.position.error, 1.5f, tolerance);
    zassert_within(output.position.output, 4.5f, tolerance);
    zassert_within(output.position.i, 0.0f, tolerance);
    zassert_within(output.velocity.velocity_reference_rad_s, 4.5f, tolerance);
    zassert_within(output.velocity.velocity_error_rad_s, 3.5f, tolerance);
    zassert_within(output.effort_command, 7.0f, tolerance);
}

ZTEST(control_motor_position, test_physical_position_reference_drives_gravity_after_rebase) {
    auto config = positionConfig();
    config.velocity.regulator.feedforward.gravity_model = CONTROL_GRAVITY_SIN;
    config.velocity.regulator.feedforward.k_gravity = 5.0f;
    control_motor_position_state state{};
    control_motor_position_output output{};
    zassert_ok(control_motor_position_reset(&state, &config, 100.0f, 0.0f));
    control_motor_position_input input{100.0f, 100.0f, 0.0f, 0.1f, pi / 2.0f, true};
    zassert_ok(control_motor_position_step(&state, &config, &input, &output));
    zassert_within(output.position.output, 0.0f, tolerance);
    zassert_within(output.velocity.regulator.feedforward, 5.0f, tolerance);
    zassert_within(output.effort_command, 5.0f, tolerance);
    input.position_reference_rad = 0.0f;
    zassert_ok(control_motor_position_step(&state, &config, &input, &output));
    zassert_within(output.effort_command, 0.0f, tolerance);
}

ZTEST(control_motor_position, test_inner_failure_rolls_back_outer_state_and_output) {
    auto config = positionConfig();
    config.velocity.regulator.feedback.dt_max_s = 0.05f;
    control_motor_position_state state{};
    control_motor_position_output output{};
    zassert_ok(control_motor_position_reset(&state, &config, 0.0f, 0.0f));
    output.effort_command = 123.0f;
    const auto state_before = bytesOf(state);
    const auto output_before = bytesOf(output);
    const control_motor_position_input input{1.0f, 0.0f, 0.0f, 0.1f, 0.0f, false};
    // This dt passes the outer PID and fails only after the inner slew/filter step.
    zassert_equal(control_motor_position_step(&state, &config, &input, &output), -ERANGE);
    zassert_mem_equal(&state, state_before.data(), sizeof(state));
    zassert_mem_equal(&output, output_before.data(), sizeof(output));
}

ZTEST(control_motor_position, test_benchmark_step) {
    const auto config = positionConfig();
    control_motor_position_state state{};
    control_motor_position_output output{};
    zassert_ok(control_motor_position_reset(&state, &config, 0.0f, 0.0f));
    float checksum = 0.0f;
    int errors = 0;
    skywalker::test::benchmark("motor_position_step", benchmark_iterations, [&](std::uint32_t index) {
        const control_motor_position_input input{(index & 1u) ? 1.0f : -1.0f, 0.1f, 0.0f, 0.001f, 0.0f, false};
        errors |= control_motor_position_step(&state, &config, &input, &output);
        checksum += output.effort_command;
    });
    zassert_ok(errors);
    zassert_true(std::isfinite(checksum));
    zassert_true(std::fabs(output.effort_command) <= config.velocity.effort_abs_max);
}

ZTEST_SUITE(control_pid, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(control_feedforward, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(control_feedforward_pid, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(control_slew_rate, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(control_angle, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(control_motor_velocity, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(control_motor_position, nullptr, nullptr, nullptr, nullptr, nullptr);
