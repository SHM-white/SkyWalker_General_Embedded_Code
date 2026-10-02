# 07 Kalman 滤波与矩阵库

## 1. 普通线性 Kalman 实例

通用线性 Kalman 位于 `lib/control/kalman_filter.c`，公开头文件是 `include/control/kalman_filter.h`。调用者持有独立状态和矩阵缓冲，通过显式初始化绑定；算法不创建 Zephyr device，不分配堆内存，也不需要设备树节点。

启用算法：

```conf
CONFIG_SKYWALKER_LIB_KALMAN_FILTER=y
```

该选项自动选择 CONTROL、MATRIX 及 CMSIS-DSP 矩阵组件，不启用可选 Flash 存储。旧 `skywalker,kalman_filter` binding、驱动头文件和 `CONFIG_SKYWALKER_DRIVER_KALMAN_FILTER` 已移除；使用旧入口的外部应用需要迁移。

API：

```c
int KalmanFilter_Init(KalmanFilter *kf, uint16_t state_dim,
                      uint16_t measure_dim, const KalmanBuffers *buffers);
void KalmanFilter_Predict(KalmanFilter *kf);
void KalmanFilter_Correct(KalmanFilter *kf, const Matrix *z);
```

`KalmanBuffers` 为七个矩阵分别提供 `KalmanBuffer { float *data; size_t capacity; }`，容量以 float 元素数计，不是字节数。尺寸要求如下：

| 矩阵 | 维度 | 最小容量 |
|---|---|---|
| F/P/Q | n×n | n² |
| H/K | m×n / n×m | nm |
| R | m×m | m² |
| X | n×1 | n |

七块数组须互不重叠，并覆盖实例的全部使用周期。描述符只在初始化调用时读取，滤波器只保留数据指针，不接管内存。独立实例各自持有一套数组；同一实例由所有者线程串行初始化、配置、计算和读取。

`Init` 返回 0 表示成功；空指针或零维度返回 `-EINVAL`，容量不足返回 `-ENOSPC`，维度乘积超过矩阵索引或字节数范围返回 `-EOVERFLOW`。参数失败时不改实例或缓冲；容量检查不能证明任意指针实际指向足够大的分配。

初始化设置 `F=I、P=1000I、Q=0.001I、R=I、X=0、K=0`，H 设置前 `min(n,m)` 个对角观测元素。成功后应按业务覆盖 F/H/Q/R 和初始 X/P。重复初始化会重新设置全部模型和状态，不能当作保留模型的 reset，调用前先停止计算。

下面是一维常量观测的完整 C 调用结构：

```c
#include <control/kalman_filter.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>

int smooth(float observation, float *estimate) {
    if (!estimate || !isfinite(observation))
        return -EINVAL;
    static KalmanFilter filter;
    static float f, h, r, x, p, q, gain;
    static const KalmanBuffers buffers = {
        .F = {&f, 1}, .H = {&h, 1}, .R = {&r, 1}, .X = {&x, 1},
        .P = {&p, 1}, .Q = {&q, 1}, .K = {&gain, 1},
    };
    static bool initialized;
    if (!initialized) {
        int rc = KalmanFilter_Init(&filter, 1, 1, &buffers);
        if (rc < 0)
            return rc;
        initialized = true;
    }
    Matrix z;
    Matrix_Init(&z, 1, 1, &observation);
    KalmanFilter_Predict(&filter);
    KalmanFilter_Correct(&filter, &z);
    *estimate = filter.X.pData[0];
    return 0;
}
```

此示例由一个线程串行调用，默认噪声只用于展示。预测/校正保留原有线性公式和 `void` 接口；不会向调用者传播内部 CMSIS-DSP 运算错误，包括求逆失败。调用方仍须保证模型、观测维度、有限数值与创新协方差可逆。实现仍使用随维度增长的临时栈数组，不宜在 ISR 使用，线程栈须留有余量。

BMI088 姿态估计使用独立的 `control::QuaternionEkf`，不调用这套线性接口。它由 `CONFIG_SKYWALKER_ATTITUDE_EKF` 控制，自身持有固定尺寸缓冲，不要求启用通用 Kalman 或 Matrix。线性滤波器的维度不能代替姿态 EKF 的非线性模型。

## 2. Matrix 封装

`include/lib/matrix/matrix.h` 将 `arm_matrix_instance_f32` 和 `arm_mat_*` 映射为 `Matrix`、`Matrix_Init`、`Matrix_Multiply`、`Matrix_Inverse` 等名字。`Matrix_Init` 只绑定调用者提供的 float buffer，不分配、不复制数据。

仅使用矩阵工具时启用：

```conf
CONFIG_SKYWALKER_LIB_MATRIX=y
```

下面是一个 2×2 对角矩阵：

```c
#include <lib/matrix/matrix.h>

float values[4] = {0};
Matrix gain;
Matrix_Init(&gain, 2, 2, values);
Matrix_SetDiag(&gain, 1.0f);
```

不能把局部数组的地址留给更长寿命的矩阵对象。矩阵 API 返回 `arm_status`，直接调用时应处理错误；不要假定所有运算都支持输入输出重叠。

## 3. 可选 Flash 存储

```conf
CONFIG_SKYWALKER_LIB_MATRIX=y
CONFIG_SKYWALKER_LIB_MATRIX_STORAGE=y
```

存储选项会选择 FILE_SYSTEM、ZMS、FLASH_PAGE_LAYOUT 和 FLASH_MAP，`lib/matrix/matrix.c` 才会加入构建。它依赖可用的 Flash 驱动和 storage partition，不是线性 Kalman 或姿态 EKF 的必需依赖。

## 4. 调试建议

- 初始化先处理返回值，再配置模型，最后开始计算；容量按元素数填写。
- n/m 较大时预留计算临时栈，持久数组静态分配不代表计算过程没有栈开销。
- 先确认矩阵尺寸和有限数值，再排查逆矩阵失败；当前线性 API 没有错误传播。
- BMI088 调用方按 [IMU API](imu.md) 使用独立 QuaternionEkf，遵守初始化、时间戳和姿态质量约定。

更多生命周期和调用顺序见[封装模块调用示例](../call-examples.md)。
