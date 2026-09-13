# 06 IMU 与姿态解算

实现位置：`drivers/imu/imu.c`、`include/drivers/imu/imu.h`。IMU 通过设备树把加速度计、陀螺仪、加热 PWM 和滤波器组合成一个 Zephyr device。

## 1. 设备树结构

```dts
imu0: imu {
    compatible = "skywalker,imu";
    accel-dev = <&bmi08x_accel>;
    gyro-dev = <&bmi08x_gyro>;
    heat-dev = <&heat_pwm>;
    filter-dev = <&ekf_filter>;
    estimator = "ekf";
    heat-kp = "6000000";
    heat-ki = "0";
    heat-kd = "0.02";
    heat-integral-max = "10000000";
    heat-output-max = "20000000";
    heat-deadband = "0";
    heat-derivative-tau-s = "0.1";
    heat-dt-min-s = "0.001";
    heat-dt-max-s = "0.2";
    heat-feedforward-ns = "6750000";
};
```

`heat-*` binding 是 string，驱动在编译期转换为 float；不要写成 `<...>`。IMU 驱动固定使用 PWM 通道 4、周期 20 ms（50 Hz），输出单位是 ns，`heat-output-max` 不能超过周期。

## 2. API

```c
void imu_fetch(const struct device *dev);
void imu_estimate(const struct device *dev, float dt_s);
int imu_heat_control(const struct device *dev, float target_temp_c, float dt_s);
```

`imu_fetch()` 读取 accel/gyro/temp；`imu_estimate()` 调用 estimator 的 predict → correct → get_angle；`imu_heat_control()` 用复合 PID 计算 PWM 脉宽，发生非法输入/输出时先把 PWM 置零。

运行时数据 `imu_data` 包含：

- `accel[3]`：m/s²。
- `gyro[3]`：rad/s。
- `temp`：°C。
- `angle[3]`：roll/pitch/yaw，单位 rad。

## 3. estimator 扩展

驱动通过 `estimator` 字符串查找 `imu_filter_api`。当前实现只有 `"ekf"`。扩展新算法时需要提供 `init/predict/correct/get_angle`，并在 `imu_get_api()` 注册；应用侧不需要改变 `imu_fetch()` / `imu_estimate()` 调用。

## 4. 当前 EKF

当前实现使用 4 维四元数状态和 3 维重力方向观测：

1. 陀螺积分进行预测，同时传播协方差。
2. 静止或低动态时估计陀螺零偏。
3. 加速度低通后归一化，用重力方向校正。
4. 使用卡方检验和自适应增益降低震动/冲击影响。
5. 输出 Euler 角；yaw 对外仍是 `[-π, π)` 包角。

EKF 内部维护连续 yaw 计数，但当前 `imu_data` 没有公开 `YawTotal`，需要连续 yaw 的应用应自行解包。

## 5. Kconfig 与运行顺序

```conf
CONFIG_SKYWALKER_DRIVER_IMU=y
CONFIG_SKYWALKER_DRIVER_KALMAN_FILTER=y
CONFIG_SKYWALKER_LIB_MATRIX=y
CONFIG_SKYWALKER_LIB_VOFA=y       # 仅在需要曲线时
CONFIG_UART_ASYNC_API=y
```

初始化时驱动会检查四个 phandle 设备是否 ready，并校验温控参数。应用应在循环中先 fetch，再 estimate；温控建议低频调用（约 10 Hz），姿态解算的 dt 使用真实采样周期。

## 6. 样例与风险

`samples/imu_test` 针对 `dm_mc02`，overlay 在 `samples/imu_test/boards/dm_mc02.overlay`，包含 BMI088、Kalman 和加热 PWM 的组合。

常见风险：

- 上电后 IMU 未静止，零偏和姿态难以收敛。
- `filter-dev` 维度不是 4/3，EKF 会越界或产生错误结果。
- 传感器 fetch 的错误不能被业务层忽略；首次运行要确认设备 ready 和数据更新时间。
- 加热器是持续功耗源，先以低目标温度验证 PWM 和温控限幅。
