/*
 * Copyright (c) 2025 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include <lib/vofa/vofa.h>

#define VOFA_SEND_PERIOD_MS 10
#define SINE_PERIOD_MS 1000
#define SINE_AMPLITUDE 1.0f
#define TWO_PI 6.28318530718f

int main(void) {
    const struct device *vofa_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
    if (!device_is_ready(vofa_uart)) {
        printk("VOFA UART device not ready\n");
        return -ENODEV;
    }

    Vofa vofa = {0};
    int err = vofa_init(&vofa, vofa_uart);
    if (err != 0) {
        printk("VOFA initialization failed: %d\n", err);
        return err;
    }

    printk("Hello, Skywalker!\n");
    printk("Board: %s\n", CONFIG_BOARD);

    const int64_t start_ms = k_uptime_get();
    while (1) {
        /* Keep the phase bounded to avoid float precision loss over time. */
        const int64_t phase_ms = (k_uptime_get() - start_ms) % SINE_PERIOD_MS;
        const float phase = TWO_PI * (float)phase_ms / (float)SINE_PERIOD_MS;
        const float channels[1] = {SINE_AMPLITUDE * sinf(phase)};
        vofa_send(&vofa, channels, 1);
        k_msleep(VOFA_SEND_PERIOD_MS);
    }

    return 0;
}
