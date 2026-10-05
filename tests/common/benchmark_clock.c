#define _POSIX_C_SOURCE 200809L
#include <stdint.h>
#include <time.h>

/* Compiled by the native runner against host headers, outside Zephyr's libc. */
uint64_t skywalker_test_monotonic_ns(void) {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        return UINT64_MAX;
    }
    return (uint64_t)now.tv_sec * UINT64_C(1000000000) + (uint64_t)now.tv_nsec;
}
