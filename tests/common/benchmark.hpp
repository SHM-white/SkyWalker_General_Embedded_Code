#pragma once

#include <zephyr/ztest.h>
#include <zephyr/timing/timing.h>
#include <cstdint>
#include <limits>

#if defined(CONFIG_BOARD_NATIVE_SIM)
extern "C" std::uint64_t skywalker_test_monotonic_ns(void);
#endif

namespace skywalker::test {
// Make inline-algorithm inputs unknown to the optimizer and retain each output.
// This is a compiler barrier: it emits no clock, logging, or assertion calls.
template <class T> void doNotOptimize(T &value) {
    asm volatile("" : "+m"(value) : : "memory");
}

// The callable must retain its output and must not print or assert while timed.
// min/max describe batch means, not the shortest/longest individual invocation.
template <class Operation> void benchmark(const char *name, std::uint32_t iterations, Operation operation) {
    constexpr std::uint32_t warmup = 64, batches = 16;
    zassert_true(iterations >= batches, "benchmark needs at least 16 operations");
#if !defined(CONFIG_BOARD_NATIVE_SIM)
    timing_init();
    timing_start();
#endif
    for (std::uint32_t i = 0; i < warmup; ++i) {
        operation(i);
    }
    std::uint64_t total_ns = 0;
    std::uint64_t minimum_ns = std::numeric_limits<std::uint64_t>::max(), maximum_ns = 0;
    std::uint32_t index = warmup;
    for (std::uint32_t batch = 0; batch < batches; ++batch) {
        const auto count = iterations / batches + (batch < iterations % batches ? 1u : 0u);
#if defined(CONFIG_BOARD_NATIVE_SIM)
        const auto start = skywalker_test_monotonic_ns();
#else
        auto start = timing_counter_get();
#endif
        asm volatile("" ::: "memory");
        for (std::uint32_t i = 0; i < count; ++i) {
            operation(index++);
        }
        asm volatile("" ::: "memory");
#if defined(CONFIG_BOARD_NATIVE_SIM)
        const auto end = skywalker_test_monotonic_ns();
        zassert_true(start != UINT64_MAX && end != UINT64_MAX && end >= start, "host monotonic clock failed");
        const auto elapsed_ns = end - start;
#else
        auto end = timing_counter_get();
        const auto elapsed_ns = timing_cycles_to_ns(timing_cycles_get(&start, &end));
#endif
        total_ns += elapsed_ns;
        const auto mean_ns = elapsed_ns / count;
        if (mean_ns < minimum_ns) {
            minimum_ns = mean_ns;
        }
        if (mean_ns > maximum_ns) {
            maximum_ns = mean_ns;
        }
    }
#if !defined(CONFIG_BOARD_NATIVE_SIM)
    timing_stop();
#endif
    zassert_true(total_ns > 0, "timing clock cannot resolve this workload");
    TC_PRINT("BENCH %s: iterations=%u total_ns=%llu avg_ns=%llu batch_min_ns=%llu batch_max_ns=%llu clock=%s\n", name,
             static_cast<unsigned>(iterations), static_cast<unsigned long long>(total_ns),
             static_cast<unsigned long long>(total_ns / iterations), static_cast<unsigned long long>(minimum_ns),
             static_cast<unsigned long long>(maximum_ns),
#if defined(CONFIG_BOARD_NATIVE_SIM)
             "host_monotonic"
#else
             "zephyr_timing"
#endif
    );
}
} // namespace skywalker::test
