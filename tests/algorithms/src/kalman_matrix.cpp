#include <zephyr/ztest.h>
#include <benchmark.hpp>
#include <control/kalman_filter.h>
#include <cerrno>
#include <cmath>
#include <cstring>

namespace {
template <unsigned N, unsigned M> struct Filter {
    float f[N * N]{}, h[N * M]{}, r[M * M]{}, x[N]{}, p[N * N]{}, q[N * N]{}, k[N * M]{};
    KalmanFilter value{};
    KalmanBuffers buffers() {
        return {{f, N * N}, {h, N * M}, {r, M * M}, {x, N}, {p, N * N}, {q, N * N}, {k, N * M}};
    }
    void init() {
        auto storage = buffers();
        zassert_ok(KalmanFilter_Init(&value, N, M, &storage));
    }
};
void expect_values(const float *actual, const float *expected, unsigned count, float tolerance = 1e-5f) {
    for (unsigned i = 0; i < count; ++i) {
        zassert_true(std::isfinite(actual[i]), "nonfinite element %u", i);
        zassert_within(actual[i], expected[i], tolerance, "element %u", i);
    }
}
}

ZTEST(matrix_math, test_rectangular_helpers) {
    float data[6] = {1, 2, 3, 4, 5, 6};
    Matrix matrix;
    Matrix_Init(&matrix, 2, 3, data);
    Matrix_SetDiag(&matrix, 2.5f);
    const float diagonal[] = {2.5f, 0, 0, 0, 2.5f, 0};
    expect_values(data, diagonal, 6);
    Matrix_Zero(&matrix);
    const float zero[6]{};
    expect_values(data, zero, 6);
}

ZTEST(matrix_math, test_rectangular_product_and_transpose) {
    float left[] = {1, 2, 3, 4, 5, 6}, right[] = {7, 8, 9, 10, 11, 12};
    float result[4]{}, transpose[6]{};
    Matrix a, b, c, at;
    Matrix_Init(&a, 2, 3, left);
    Matrix_Init(&b, 3, 2, right);
    Matrix_Init(&c, 2, 2, result);
    Matrix_Init(&at, 3, 2, transpose);
    zassert_equal(Matrix_Multiply(&a, &b, &c), ARM_MATH_SUCCESS);
    const float product[] = {58, 64, 139, 154};
    expect_values(result, product, 4);
    zassert_equal(Matrix_Transpose(&a, &at), ARM_MATH_SUCCESS);
    const float transposed[] = {1, 4, 2, 5, 3, 6};
    expect_values(transpose, transposed, 6);
}

ZTEST(matrix_math, test_addition_subtraction_and_alias) {
    float left[] = {1, -2, 3, -4}, right[] = {-2, 3, 0.5f, 4}, result[4]{};
    Matrix a, b, out;
    Matrix_Init(&a, 2, 2, left);
    Matrix_Init(&b, 2, 2, right);
    Matrix_Init(&out, 2, 2, result);
    zassert_equal(Matrix_Add(&a, &b, &out), ARM_MATH_SUCCESS);
    const float sum[] = {-1, 1, 3.5f, 0};
    expect_values(result, sum, 4);
    zassert_equal(Matrix_Subtract(&out, &b, &out), ARM_MATH_SUCCESS);
    expect_values(result, left, 4);
}

ZTEST(matrix_math, test_inverse_known_solution_and_identity) {
    const float original[] = {3, 0, 2, 2, 0, -2, 0, 1, 1};
    float working[9], inverse[9]{}, identity[9]{};
    std::memcpy(working, original, sizeof(working));
    Matrix a, ai, product;
    Matrix_Init(&a, 3, 3, working);
    Matrix_Init(&ai, 3, 3, inverse);
    Matrix_Init(&product, 3, 3, identity);
    zassert_equal(Matrix_Inverse(&a, &ai), ARM_MATH_SUCCESS);
    const float expected[] = {0.2f, 0.2f, 0, -0.2f, 0.3f, 1, 0.2f, -0.3f, 0};
    expect_values(inverse, expected, 9);
    // CMSIS inversion can modify the source; restore before A * inverse(A).
    std::memcpy(working, original, sizeof(working));
    zassert_equal(Matrix_Multiply(&a, &ai, &product), ARM_MATH_SUCCESS);
    const float unit[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    expect_values(identity, unit, 9);
}

ZTEST(matrix_math, test_singular_and_dimension_errors) {
    float singular[] = {1, 2, 2, 4}, result[4]{}, bad_data[6]{};
    Matrix a, out, bad;
    Matrix_Init(&a, 2, 2, singular);
    Matrix_Init(&out, 2, 2, result);
    Matrix_Init(&bad, 3, 2, bad_data);
    zassert_equal(Matrix_Inverse(&a, &out), ARM_MATH_SINGULAR);
    zassert_equal(Matrix_Multiply(&a, &bad, &out), ARM_MATH_SIZE_MISMATCH);
    zassert_equal(Matrix_Add(&a, &bad, &out), ARM_MATH_SIZE_MISMATCH);
    zassert_equal(Matrix_Transpose(&a, &bad), ARM_MATH_SIZE_MISMATCH);
}

ZTEST(matrix_math, test_benchmark_product_and_inverse) {
    float a_data[] = {4, 1, 0, 0, 1, 4, 1, 0, 0, 1, 4, 1, 0, 0, 1, 4};
    float b_data[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, product[16]{};
    Matrix a, b, out;
    Matrix_Init(&a, 4, 4, a_data);
    Matrix_Init(&b, 4, 4, b_data);
    Matrix_Init(&out, 4, 4, product);
    int failures = 0;
    skywalker::test::benchmark("matrix.multiply.4x4", 4096,
                               [&](std::uint32_t) { failures += Matrix_Multiply(&a, &b, &out) != ARM_MATH_SUCCESS; });
    zassert_equal(failures, 0);
    expect_values(product, a_data, 16);
    const float input[] = {4, 7, 2, 6};
    float scratch[4], inverse[4]{};
    Matrix_Init(&a, 2, 2, scratch);
    Matrix_Init(&out, 2, 2, inverse);
    skywalker::test::benchmark("matrix.inverse.2x2_with_input_copy", 4096, [&](std::uint32_t) {
        std::memcpy(scratch, input, sizeof(scratch));
        failures += Matrix_Inverse(&a, &out) != ARM_MATH_SUCCESS;
    });
    zassert_equal(failures, 0);
    const float expected[] = {0.6f, -0.7f, -0.2f, 0.4f};
    expect_values(inverse, expected, 4);
}

ZTEST(linear_kalman, test_default_model_rectangular_storage) {
    Filter<3, 2> filter;
    filter.init();
    zassert_equal(filter.value.F.pData, filter.f);
    zassert_equal(filter.value.X.pData, filter.x);
    zassert_equal(filter.value.H.numRows, 2);
    zassert_equal(filter.value.H.numCols, 3);
    zassert_equal(filter.value.K.numRows, 3);
    zassert_equal(filter.value.K.numCols, 2);
    const float identity[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const float observation[] = {1, 0, 0, 0, 1, 0};
    expect_values(filter.f, identity, 9);
    expect_values(filter.h, observation, 6);
    for (unsigned i = 0; i < 9; ++i) {
        zassert_within(filter.p[i], identity[i] * 1000, 1e-6f);
        zassert_within(filter.q[i], identity[i] * 0.001f, 1e-6f);
    }
    const float noise[] = {1, 0, 0, 1}, zero3[3]{}, zero6[6]{};
    expect_values(filter.r, noise, 4);
    expect_values(filter.x, zero3, 3);
    expect_values(filter.k, zero6, 6);

    Filter<2, 3> more_measurements;
    more_measurements.init();
    const float tall_observation[] = {1, 0, 0, 1, 0, 0};
    expect_values(more_measurements.h, tall_observation, 6);
}

ZTEST(linear_kalman, test_rejects_invalid_storage_without_mutation) {
    Filter<2, 1> filter;
    filter.init();
    filter.x[0] = 17;
    filter.f[1] = 2;
    const auto before = filter.value;
    auto storage = filter.buffers();
    zassert_equal(KalmanFilter_Init(nullptr, 2, 1, &storage), -EINVAL);
    zassert_equal(KalmanFilter_Init(&filter.value, 0, 1, &storage), -EINVAL);
    zassert_equal(KalmanFilter_Init(&filter.value, 2, 0, &storage), -EINVAL);
    zassert_equal(KalmanFilter_Init(&filter.value, 2, 1, nullptr), -EINVAL);
    zassert_equal(KalmanFilter_Init(&filter.value, 65535, 65535, &storage), -EOVERFLOW);
    KalmanBuffer *buffers[] = {&storage.F, &storage.H, &storage.R, &storage.X, &storage.P, &storage.Q, &storage.K};
    for (auto buffer : buffers) {
        const auto saved = *buffer;
        buffer->data = nullptr;
        zassert_equal(KalmanFilter_Init(&filter.value, 2, 1, &storage), -EINVAL);
        *buffer = saved;
        buffer->capacity = saved.capacity - 1;
        zassert_equal(KalmanFilter_Init(&filter.value, 2, 1, &storage), -ENOSPC);
        *buffer = saved;
    }
    zassert_mem_equal(&filter.value, &before, sizeof(before));
    zassert_equal(filter.x[0], 17);
    zassert_equal(filter.f[1], 2);
}

ZTEST(linear_kalman, test_scalar_closed_form) {
    Filter<1, 1> filter;
    filter.init();
    filter.p[0] = 4;
    filter.q[0] = 1;
    filter.r[0] = 3;
    float observation = 2;
    Matrix z;
    Matrix_Init(&z, 1, 1, &observation);
    KalmanFilter_Predict(&filter.value);
    zassert_equal(filter.p[0], 5);
    KalmanFilter_Correct(&filter.value, &z);
    zassert_within(filter.k[0], 5.0f / 8, 1e-6f);
    zassert_within(filter.x[0], 5.0f / 4, 1e-6f);
    zassert_within(filter.p[0], 15.0f / 8, 1e-6f);
}

ZTEST(linear_kalman, test_constant_velocity_known_prediction_and_correction) {
    Filter<2, 1> filter;
    filter.init();
    filter.f[1] = 0.5f;
    filter.x[0] = 1;
    filter.x[1] = 2;
    const float covariance[] = {4, 1, 1, 3};
    std::memcpy(filter.p, covariance, sizeof(covariance));
    filter.q[0] = 0.1f;
    filter.q[3] = 0.2f;
    filter.r[0] = 0.5f;
    KalmanFilter_Predict(&filter.value);
    const float predicted_x[] = {2, 2}, predicted_p[] = {5.85f, 2.5f, 2.5f, 3.2f};
    expect_values(filter.x, predicted_x, 2);
    expect_values(filter.p, predicted_p, 4);
    float observation = 3;
    Matrix z;
    Matrix_Init(&z, 1, 1, &observation);
    KalmanFilter_Correct(&filter.value, &z);
    const float gain[] = {117.0f / 127, 50.0f / 127};
    const float corrected_x[] = {371.0f / 127, 304.0f / 127};
    const float corrected_p[] = {117.0f / 254, 25.0f / 127, 25.0f / 127, 1407.0f / 635};
    expect_values(filter.k, gain, 2);
    expect_values(filter.x, corrected_x, 2);
    expect_values(filter.p, corrected_p, 4);
}

ZTEST(linear_kalman, test_coupled_two_channel_observation) {
    Filter<2, 2> filter;
    filter.init();
    filter.h[1] = 1;
    filter.x[0] = 1;
    filter.x[1] = -1;
    filter.p[0] = 2;
    filter.p[3] = 3;
    Matrix_Zero(&filter.value.Q);
    filter.r[3] = 2;
    float observation[] = {2, 0};
    Matrix z;
    Matrix_Init(&z, 2, 1, observation);
    KalmanFilter_Predict(&filter.value);
    KalmanFilter_Correct(&filter.value, &z);
    // S = [6 3; 3 5], det(S) = 21. These are exact rational solutions.
    const float gain[] = {10.0f / 21, -6.0f / 21, 6.0f / 21, 9.0f / 21};
    const float state[] = {5.0f / 3, 0};
    const float covariance[] = {22.0f / 21, -4.0f / 7, -4.0f / 7, 6.0f / 7};
    expect_values(filter.k, gain, 4);
    expect_values(filter.x, state, 2);
    expect_values(filter.p, covariance, 4);
}

ZTEST(linear_kalman, test_noisy_constant_reduces_rmse_and_matches_double_reference) {
    Filter<1, 1> filter;
    filter.init();
    filter.p[0] = 1;
    filter.q[0] = 0.001f;
    filter.r[0] = 0.25f;
    double reference_x = 0, reference_p = 1, raw_error = 0, estimate_error = 0;
    constexpr float truth = 3;
    const float noise[] = {-0.6f, 0.4f, -0.2f, 0.7f, -0.3f};
    float observation = 0;
    Matrix z;
    Matrix_Init(&z, 1, 1, &observation);
    for (unsigned i = 0; i < 1000; ++i) {
        observation = truth + noise[i % 5];
        reference_p += 0.001;
        const double gain = reference_p / (reference_p + 0.25);
        reference_x += gain * (double(observation) - reference_x);
        reference_p *= 1 - gain;
        KalmanFilter_Predict(&filter.value);
        KalmanFilter_Correct(&filter.value, &z);
        zassert_within(double(filter.x[0]), reference_x, 2e-5);
        zassert_within(double(filter.p[0]), reference_p, 2e-6);
        zassert_true(filter.p[0] >= 0);
        if (i >= 100) {
            const double raw_residual = double(observation) - double(truth);
            const double estimate_residual = double(filter.x[0]) - double(truth);
            raw_error += raw_residual * raw_residual;
            estimate_error += estimate_residual * estimate_residual;
        }
    }
    zassert_true(std::sqrt(estimate_error / raw_error) < 0.2, "filtered RMSE must be below 20%% of measurement RMSE");
}

ZTEST(linear_kalman, test_instances_and_reinitialization_are_independent) {
    Filter<1, 1> first, second;
    first.init();
    second.init();
    first.x[0] = 10;
    second.x[0] = -5;
    first.f[0] = 2;
    KalmanFilter_Predict(&first.value);
    zassert_equal(first.x[0], 20);
    zassert_equal(second.x[0], -5);
    zassert_equal(second.p[0], 1000);
    first.init();
    zassert_equal(first.x[0], 0);
    zassert_equal(first.f[0], 1);
    zassert_equal(second.x[0], -5);
}

ZTEST(linear_kalman, test_benchmark_predict_correct) {
    Filter<2, 1> filter;
    filter.init();
    filter.f[1] = 0.001f;
    filter.r[0] = 0.1f;
    float observation = 0;
    Matrix z;
    Matrix_Init(&z, 1, 1, &observation);
    skywalker::test::benchmark("kalman.predict_correct.n2_m1", 4096, [&](std::uint32_t) {
        KalmanFilter_Predict(&filter.value);
        KalmanFilter_Correct(&filter.value, &z);
    });
    zassert_equal(filter.x[0], 0);
    zassert_equal(filter.x[1], 0);
    for (float v : filter.p) {
        zassert_true(std::isfinite(v));
    }
    zassert_true(filter.p[0] > 0 && filter.p[3] > 0);
    zassert_within(filter.p[1], filter.p[2], 1e-5f);
}

ZTEST_SUITE(matrix_math, nullptr, nullptr, nullptr, nullptr, nullptr);
ZTEST_SUITE(linear_kalman, nullptr, nullptr, nullptr, nullptr, nullptr);
