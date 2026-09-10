/*
 * Copyright (c) 2025 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include <lib/vofa/vofa.h>

int main(void) {
	const struct device *vofa_uart = DEVICE_DT_GET(DT_NODELABEL(usart1));
	if (!device_is_ready(vofa_uart)) {
		printk("VOFA UART device not ready\n");
		return -ENODEV;
	}

	Vofa vofa = {0};
	vofa_init(&vofa, vofa_uart);

	printk("Hello, Skywalker!\n");
	printk("Board: %s\n", CONFIG_BOARD);

	while (1) {
		/* JustFloat channel: uptime_ms. */
		const float channels[1] = {(float)k_uptime_get_32()};
		vofa_send(&vofa, channels, 1);
		k_msleep(5000);
	}

	return 0;
}
