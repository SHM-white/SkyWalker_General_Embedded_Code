# 07 Kalman 滤波驱动与矩阵库

这两部分是一对：`drivers/kalman_filter/` 是 Zephyr 设备形态的通用卡尔曼
滤波器，`lib/matrix/` 是它对 CMSIS-DSP 矩阵运算的薄封装。IMU 的 EKF 就是
构建在二者之上的。

---

## 1. Kalman 滤波驱动

### 1.1 设备树 binding（`skywalker,kalman_filter`）

```dts
ekf_filter: ekf_filter {
    compatible = "skywalker,kalman_filter";
    state-dim = <4>;     /* 状态维度 n */
    measure-dim = <3>;   /* 观测维度 m */
};
```

`state-dim` / `measure-dim` 决定所有矩阵的尺寸，**必须由使用方保证与算法
一致**（IMU-EKF 要求 4/3），驱动不做语义校验。

### 1.2 数据结构

```c
typedef struct {
    Matrix F;       // n×n 状态转移
    Matrix H, R;    // m×n 观测矩阵、m×m 测量噪声
    Matrix X, P, Q; // n×1 状态、n×n 误差协方差、n×n 过程噪声
    Matrix K;       // n×m 卡尔曼增益
} KalmanFilter;
```

矩阵的 `pData` 由驱动设备在初始化时**在设备 data 内部分配**并绑定：

- 设备 data 布局 = `KalmanFilter` 结构体 + `F/H/R/X/P/Q/K` 的 float 缓冲区。
- 初始化时用指针偏移定位各 buffer，`Matrix_Init` 绑定尺寸。
- 初始值：`F = I`、`P = 1000·I`、`Q = 0.001·I`、`R = I`、
  `H = I` 的前 m 行、`X = 0`。
- `K` 为内部计算只读输出。

> EKF 正是通过 `(KalmanFilter *)dev->data` 直接操作这些矩阵，所以
> `filter-dev` 必须是本驱动，且尺寸要匹配。

### 1.3 API

```c
void KalmanFilter_Predict(KalmanFilter *kf);          // x=F·x, P=F·P·Fᵀ+Q
void KalmanFilter_Correct(KalmanFilter *kf, const Matrix *z); // m×1 观测
```

`Correct` 的完整公式：

$$
K = P H^T (H P H^T + R)^{-1},\quad
x = x + K(z - Hx),\quad
P = (I - K H)P
$$

实现细节：

- 为处理源/目标矩阵重叠，`Predict` / `Correct` 内部使用**局部中转缓冲**。
- 局部缓冲按维度用 **VLA**（`float FP_buf[n*n]` 等）分配在**栈**上。
  维度增大时栈占用按 $n^2$、$n m$ 增长，需相应调大调用线程/主栈。
- 编译选项为 `-O3 ... -ffreestanding -fno-builtin`。

### 1.4 Kconfig / 初始化

```conf
CONFIG_SKYWALKER_DRIVER_KALMAN_FILTER=y
```

设备初始化优先级 `50`（IMU 为 `91`），保证滤波器缓冲区与初始值先就绪。

---

## 2. 矩阵库（`lib/matrix`）

### 2.1 宏封装（`include/lib/matrix/matrix.h`）

```c
#define Matrix         arm_matrix_instance_f32
#define Matrix_Init    arm_mat_init_f32
#define Matrix_Add     arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse  arm_mat_inverse_f32

static inline void Matrix_Zero(Matrix *mat);
static inline void Matrix_SetDiag(Matrix *mat, float val);
```

即把 CMSIS-DSP 的 `arm_matrix_instance_f32` 与 `arm_mat_*` 换成更短的别名。
**矩阵库本身是 header-only**：`lib/matrix/matrix.c` 只在开启 flash 存储时
参与编译。

### 2.2 矩阵落 flash（可选）

```conf
CONFIG_SKYWALKER_LIB_MATRIX=y
CONFIG_SKYWALKER_LIB_MATRIX_STORAGE=y   # select FILE_SYSTEM/ZMS/FLASH_PAGE_LAYOUT/FLASH_MAP
```

```c
int  matrix_storage_save(uint32_t id, const float *data, uint16_t rows, uint16_t cols);
int  matrix_storage_get_size(uint32_t id, uint16_t *rows, uint16_t *cols);
int  matrix_storage_read(uint32_t id, Matrix *mat);   // 调用者先分配 pData 并设好 numRows/numCols
bool matrix_storage_exists(uint32_t id);
int  matrix_storage_delete(uint32_t id);
```

适合把标定/整定后的协方差、观测矩阵按 ID 持久化，重启后读回。
需要板上有可用 flash 驱动与 `storage_partition`（MC02 已提供 256 KB
`storage` 分区）。

### 2.3 Kconfig

```conf
CONFIG_SKYWALKER_LIB_MATRIX=y            # select CMSIS_DSP + CMSIS_DSP_MATRIX
# 可选：
CONFIG_SKYWALKER_LIB_MATRIX_STORAGE=y
```

---

## 3. 使用示例

```c
#include "lib/matrix/matrix.h"
#include "drivers/kalman_filter/kalman_filter.h"

static const device *kf_dev = DEVICE_DT_GET(DT_NODELABEL(ekf_filter));
KalmanFilter *kf = (KalmanFilter *)kf_dev->data;

KalmanFilter_Predict(kf);

float z_buf[3] = { /* 观测值 */ };
Matrix z;
Matrix_Init(&z, 3, 1, z_buf);
KalmanFilter_Correct(kf, &z);
```

---

## 4. 常见坑

1. **维度必须一致**：`state-dim` / `measure-dim` 与算法假设不符会在运行时
   越界或算错，编译期不报错。
2. `Matrix_Init` 不会分配内存，`pData` 必须由调用者提供；驱动内部已为
   `KalmanFilter` 成员分配，自己创建的临时矩阵则要自备缓冲。
3. VLA 用栈，维度大或调用栈深时注意 `CONFIG_MAIN_STACK_SIZE`。
4. `Matrix_Inverse` 只适合小矩阵；当前实现完全依赖 CMSIS-DSP。
5. 矩阵存储是**可选**功能，未开 `SKYWALKER_LIB_MATRIX_STORAGE` 时
   `matrix.c` 不参与构建，相关符号不存在。

---

## 5. 相关文档

- [06 IMU 与 EKF](06-drivers-imu.md)
- [08 纯 C 控制算法](08-control-algorithms.md)
