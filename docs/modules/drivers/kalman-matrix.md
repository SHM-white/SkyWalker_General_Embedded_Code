# 07 Kalman 滤波与矩阵库

## 1. Kalman 设备

`skywalker,kalman_filter` 是一个固定尺寸、由设备数据区持有矩阵缓冲的 Zephyr device：

```dts
ekf_filter: filter {
    compatible = "skywalker,kalman_filter";
    state-dim = <4>;
    measure-dim = <3>;
};
```

结构体包含：

```text
F(n×n)  H(m×n)  R(m×m)
X(n×1)  P(n×n)  Q(n×n)  K(n×m)
```

`state-dim` / `measure-dim` 只负责分配尺寸，不知道上层算法语义；IMU EKF 必须是 n=4、m=3。

API：

```c
void KalmanFilter_Predict(KalmanFilter *kf);
void KalmanFilter_Correct(KalmanFilter *kf, const Matrix *z);
```

调用者负责让观测矩阵 `z` 的尺寸和 `pData` 正确。实现使用 CMSIS-DSP，并在预测/校正中使用按维度增长的局部临时缓冲，线程栈必须留有余量。

## 2. Matrix 封装

`include/lib/matrix/matrix.h` 将 `arm_matrix_instance_f32` 和 `arm_mat_*` 映射为 `Matrix`、`Matrix_Init`、`Matrix_Multiply`、`Matrix_Inverse` 等名字。它不负责分配内存；`Matrix_Init` 只绑定调用者提供的 float buffer。

启用：

```conf
CONFIG_SKYWALKER_LIB_MATRIX=y
```

该选项会选择 CMSIS-DSP 和矩阵组件。IMU 驱动也会依赖相关能力。

## 3. 矩阵调用示例

`Matrix` 只是 CMSIS-DSP 的结构体别名；数据缓冲区由调用者持有，不能把局部数组的地址留给更长寿的对象。下面展示一个 2×2 对角矩阵，不涉及 Kalman 设备内部的 4/3 状态：

```c
#include <lib/matrix/matrix.h>

float values[4] = {0};
Matrix gain;
Matrix_Init(&gain, 2, 2, values);
Matrix_SetDiag(&gain, 1.0f);  // values = {1, 0, 0, 1}
```

IMU 调用方应使用 [IMU API](../../06-drivers-imu.md#3-最小调用示例)；`KalmanFilter_Predict/Correct` 由 IMU estimator 组织，直接调用时需自行保证 F/H/Q/R、观测维度和缓冲区生命周期。

## 4. 可选 Flash 存储

```conf
CONFIG_SKYWALKER_LIB_MATRIX=y
CONFIG_SKYWALKER_LIB_MATRIX_STORAGE=y
```

存储选项会选择 FILE_SYSTEM、ZMS、FLASH_PAGE_LAYOUT 和 FLASH_MAP，`lib/matrix/matrix.c` 才会加入构建。它依赖板卡有可用的 flash driver 和 storage partition；不要仅在没有分区的板卡上打开。

## 5. 调试建议

- n/m 较大时检查主线程/工作线程栈，不要用默认的小栈运行复杂 EKF。
- 先打印矩阵尺寸和有限性，再查逆矩阵失败。
- 对 IMU 直接使用 4/3 的静态配置，不要让 overlay 和算法假设分离。
- 矩阵存储是可选能力，不是当前所有样例的必需依赖。
\n\n更多对象生命周期与完整调用顺序见[封装模块调用示例](../../../call-examples.md)。\n