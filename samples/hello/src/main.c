/*
 * Copyright (c) 2025 RobotPilots-SZU
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	printk("Hello, Skywalker!\n");
	printk("Board: %s\n", CONFIG_BOARD);

	while (1) {
		printk("Tick...\n");
		k_msleep(5000);
	}

	return 0;
}
