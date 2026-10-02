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

`state-dim` / `measure-dim` 只负责分配尺寸，不知道上层算法语义；上面的 4/3 仅是通用线性 Kalman 的尺寸示例。当前 BMI088 姿态估计使用独立的 `control::QuaternionEkf`，不依赖这个设备树节点，也不调用下面的线性 Kalman API。

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

该选项会选择 CMSIS-DSP 和矩阵组件。通用 Kalman 设备还需开启 `CONFIG_SKYWALKER_DRIVER_KALMAN_FILTER=y`；当前 BMI088 的独立姿态 EKF 使用 `CONFIG_SKYWALKER_ATTITUDE_EKF=y`，自身持有固定尺寸缓冲，不要求开启矩阵库或通用 Kalman 驱动。

## 3. 矩阵调用示例

`Matrix` 只是 CMSIS-DSP 的结构体别名；数据缓冲区由调用者持有，不能把局部数组的地址留给更长寿的对象。下面展示一个 2×2 对角矩阵，不涉及设备树 Kalman 实例的状态：

```c
#include <lib/matrix/matrix.h>

float values[4] = {0};
Matrix gain;
Matrix_Init(&gain, 2, 2, values);
Matrix_SetDiag(&gain, 1.0f);  // values = {1, 0, 0, 1}
```

IMU 调用方应使用 [IMU API](../../06-drivers-imu.md#3-最小调用示例)，将独立的 `control::QuaternionEkf` 传给 `Bmi088Imu`。直接使用通用 `KalmanFilter_Predict/Correct` 时，调用方须自行保证 F/H/Q/R、观测维度、可逆性和缓冲区生命周期；这两个 `void` 接口不会把内部 CMSIS-DSP 运算错误返回给调用者。

## 4. 可选 Flash 存储

```conf
CONFIG_SKYWALKER_LIB_MATRIX=y
CONFIG_SKYWALKER_LIB_MATRIX_STORAGE=y
```

存储选项会选择 FILE_SYSTEM、ZMS、FLASH_PAGE_LAYOUT 和 FLASH_MAP，`lib/matrix/matrix.c` 才会加入构建。它依赖板卡有可用的 flash driver 和 storage partition；不要仅在没有分区的板卡上打开。

## 5. 调试建议

- n/m 较大时检查主线程/工作线程栈，不要用默认的小栈运行复杂 EKF。
- 先打印矩阵尺寸和有限性，再查逆矩阵失败。
- 对 BMI088 姿态估计使用 `QuaternionEkf::Config` 并检查初始化状态、时间戳和姿态质量；通用线性 Kalman 的 overlay 尺寸须与调用方矩阵一致。
- 矩阵存储是可选能力，不是当前所有样例的必需依赖。
\n\n更多对象生命周期与完整调用顺序见[封装模块调用示例](../../../call-examples.md)。\n
