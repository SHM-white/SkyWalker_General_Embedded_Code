# 06 IMU 与姿态解算

`drivers/imu/` 提供一个通用的 **IMU 设备**：它本身不做采样算法的硬编码，
而是通过设备树把「加速度计 + 陀螺仪 + 加热 PWM + 解算滤波器」组合起来，
并按 `estimator` 字符串选择解算实现。当前内置实现是**四元数扩展卡尔曼
滤波（EKF）**。

---

## 1. 设备树 binding（`skywalker,imu`）

| 属性 | 类型 | 说明 |
|---|---|---|
| `accel-dev` | phandle | 加速度计设备（BMI088 accel） |
| `gyro-dev` | phandle | 陀螺仪设备（BMI088 gyro） |
| `heat-dev` | phandle | PWM 加热设备节点 |
| `filter-dev` | phandle | 解算滤波器设备（`skywalker,kalman_filter`） |
| `estimator` | string | 解算方法，目前仅 `"ekf"` |
| `heat-kp` … `heat-dt-max-s` | string | 温控 PID 参数，编译期转 float |
| `heat-feedforward-ns` | string | 温控前馈偏置（PWM 脉宽，ns） |

> 温控参数在设备树里是**字符串**，由 `DT_STRING_UNQUOTED` 在编译期转成
> `float`。写 `heat-kp = "6000000";` 而不是 `<6000000>`。

### 示例（`samples/imu_test/boards/dm_mc02.overlay`）

```dts
/ {
    imu: imu {
        compatible = "skywalker,imu";
        accel-dev = <&bmi08x_accel>;
        gyro-dev = <&bmi08x_gyro>;
        heat-dev = <&pwm_heat>;
        filter-dev = <&ekf_filter>;
        estimator = "ekf";
        heat-kp = "6000000";
        heat-ki = "0";
        heat-kd = "0.02";
        heat-integral-max = "0";
        heat-output-max = "20000000";
        heat-deadband = "0";
        heat-derivative-tau-s = "0";
        heat-dt-min-s = "0.05";
        heat-dt-max-s = "0.20";
        heat-feedforward-ns = "6750000";
    };

    ekf_filter: ekf_filter {
        compatible = "skywalker,kalman_filter";
        state-dim = <4>;      /* 四元数 */
        measure-dim = <3>;    /* 归一化加速度 */
    };
};

&timers3 {
    status = "okay";
    st,prescaler = <99>;
    pwm_heat: pwm {
        status = "okay";
        pinctrl-names = "default";
        pinctrl-0 = <&tim3_ch4_pb1>;
    };
};
```

---

## 2. Kconfig

```conf
CONFIG_SKYWALKER_DRIVER_IMU=y            # select CMSIS_DSP_FASTMATH + SKYWALKER_LIB_CONTROL
CONFIG_SKYWALKER_DRIVER_KALMAN_FILTER=y  # EKF 依赖
CONFIG_SKYWALKER_LIB_MATRIX=y            # EKF 用 CMSIS-DSP 矩阵
```

设备初始化优先级：Kalman 滤波器 **50**，IMU **91**（确保滤波器先完成
矩阵缓冲区绑定与初始值设置）。

---

## 3. 对外 API（`include/drivers/imu/imu.h`）

```c
void imu_fetch(const device *dev);                       // 读 accel/gyro/temp
void imu_estimate(const device *dev, float dt);          // 姿态解算
int  imu_heat_control(const device *dev, float target_temp, float dt); // 温控，~10 Hz
```

内部数据结构：

```c
typedef struct {
    float accel[3];   // m/s², x/y/z
    float gyro[3];    // rad/s, x/y/z
    float temp;       // °C
    float angle[3];   // rad, roll/pitch/yaw
    control_feedforward_pid_state heat_controller_state;
} imu_data;
```

调用模式（参考 "fetch → estimate" 循环）：

```c
imu_fetch(dev);
imu_estimate(dev, dt);
// imu_data *data = dev->data; 读取 data->angle
if (++n % 20 == 0) {           // 约 10 Hz
    imu_heat_control(dev, 40.0f, dt * 20);
}
```

---

## 4. 解算器扩展机制

```c
struct imu_filter_api {
    void (*init)(const device *dev);
    void (*predict)(const device *dev, const float gyro[3], float dt, float angle[3]);
    void (*correct)(const device *dev, const float accel[3]);
    void (*get_angle)(const device *dev, float angle[3]);
};

typedef struct { const char *name; const struct imu_filter_api api; } imu_estimator;
```

`imu_estimators[]` 是注册表，`imu_get_api()` 按 `name` 匹配。新增算法：
实现四个函数 → 声明 `imu_estimator_xxx` → 在表里加一行 → 设备树把
`estimator` 改成对应名字。**不需要改 IMU 驱动本身。**

---

## 5. 内置 EKF 算法要点

状态：四元数 `q = [q0,q1,q2,q3]`（4 维）。

| 环节 | 做法 |
|---|---|
| 零偏估计 | 低动态（`|ω| < 0.2`、`||a|-9.8| < 0.35`）时对陀螺做 LPF（系数 0.9995） |
| 预测 | `F = I + 0.5·Ω(ω−bias)·dt`；`Q = 10·dt·I`；`KalmanFilter_Predict` |
| 观测 | 加速度先 LPF（系数 0.05，约 3 Hz），归一化为重力方向 |
| 抗扰动 | 卡方检验 `χ² = ||z − h(q)||²`；阈值 `1e-8`，连续异常 >50 次判定发散 |
| 自适应增益 | 接近阈值时缩放 `R`（`r_adj = R / scale`），抑制扰动 |
| 收敛标志 | `χ² < 0.5·阈值` 时置位；复位后重新缓慢收敛 |
| 输出 | 四元数 → Euler；yaw 做跨圈计数得到 `YawTotal` |

常量：`EKF_Q1 = 10`、`EKF_R = 1e6`、`EKF_CHI_THRESHOLD = 1e-8`。

**注意**：`imu_data.angle[2]` 输出的是 **wrap 到 `[-π, π)` 的 yaw**；
内部虽然维护了连续 `YawTotal`，但当前没有对外暴露。需要连续 yaw 时，
应在应用层自行解包。

数值实现用了 CMSIS-DSP：`arm_sqrt_f32`、`arm_atan2_f32`，以及一个快速
倒数平方根（`inv_sqrt`）。EKF 直接把 `filter_dev->data` 当作 `KalmanFilter` 使用，
因此 **`filter-dev` 必须是 `skywalker,kalman_filter` 设备**，且
`state-dim` / `measure-dim` 必须与算法一致（4 / 3）。

---

## 6. 恒温加热控制

- 通道：**PWM 通道 4**，周期 `PWM_MSEC(20)`（50 Hz）；PWM 通道号从 1 起，
  对应 STM32 的 TIM3_CH4。
- 控制器：`control_feedforward_pid`，输出单位 **ns**（PWM 脉宽）。
- 首次调用会 `reset`（用当前温度初始化）；参数非法或输出非有限值时，
  驱动会把脉宽置 0（安全侧）。
- `heat-output-max`（ns）不得超过 20 ms 周期；初始化会校验，
  越界返回 `-ERANGE`。

`imu_test` 的 PID 结构：`kp=6000000`、`kd=0.02`、前馈 `k_bias=6750000 ns`，
即约 6.75 ms 常开基础加热 + 比例项。实际值需按发热体与散热条件整定。

---

## 7. 样例

`samples/imu_test/`：`dm_mc02`，overlay 在 `samples/imu_test/boards/dm_mc02.overlay`。

```bash
west build -p -b dm_mc02/stm32h723xx -d build/imu_test samples/imu_test
west flash -d build/imu_test
```

`prj.conf` 关键项：

```conf
CONFIG_SKYWALKER_DRIVER_IMU=y
CONFIG_SKYWALKER_DRIVER_KALMAN_FILTER=y
CONFIG_SKYWALKER_LIB_MATRIX=y
CONFIG_SKYWALKER_LIB_VOFA=y
CONFIG_UART_ASYNC_API=y
CONFIG_NOCACHE_MEMORY=y
```

---

## 8. 常见坑

1. `heat-*` 参数必须是**字符串**，写成整型 `<...>` 会导致设备树/编译期
   转换错误。
2. EKF 的 `filter-dev` 尺寸写错（非 4/3）会在运行时越界；驱动不会自动
   校验维度，务必与算法匹配。
3. `sensor_sample_fetch` 的返回值在 `imu_fetch` 中被忽略；若传感器未就绪，
   数据可能保持旧值，初始化阶段应确认子设备 `device_is_ready`。
4. `YawTotal` 未输出，别指望 `angle[2]` 连续。
5. 加热 PWM 会持续耗电，台架调试注意电源与温升。

---

## 9. 相关文档

- [07 Kalman 与矩阵库](07-kalman-matrix.md)
- [08 纯 C 控制算法](08-control-algorithms.md)
- [10 样例索引](10-samples.md)
