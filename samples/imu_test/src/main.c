#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "drivers/imu/imu.h"
#include "lib/vofa/vofa.h"

int main(void) {
	const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(imu));
	const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));
	if (!device_is_ready(imu_dev)) {
		printk("IMU device not ready\n");
		return -1;
	}
	if (!device_is_ready(uart_dev)) {
		printk("UART device not ready\n");
		return -1;
	}

	Vofa vofa;
	vofa_init(&vofa, uart_dev);

	imu_data *data = imu_dev->data;

	uint32_t now = k_uptime_get_32();
	uint32_t t_last = now, t_heat = now, t_send = now;

	while (1) {
		now = k_uptime_get_32();
		float dt = (now - t_last) / 1000.0f;
		t_last = now;

		imu_fetch(imu_dev);
		imu_estimate(imu_dev, dt);

		if (now - t_heat >= 100) {
			imu_heat_control(imu_dev, 50.0f, (now - t_heat) / 1000.0f);
			t_heat = now;
		}

		if (now - t_send >= 10) {
			float out[3] = { data->angle[0], data->angle[1], data->angle[2] };
			vofa_send(&vofa, out, 3);
			t_send = now;
		}

		k_usleep(1000);
	}
	return 0;
}
