# 姿态 EKF 与通用 Kalman / Matrix：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

明确两条独立路线：BMI088 四元数 EKF；CMSIS-DSP 线性 Kalman 和矩阵工具。

**接入状态：已有代码。** 独立QuaternionEkf已接BMI088输入与dual_imu样例；通用Kalman与Matrix为可选数学工具，不在当前BMI088姿态链中。整车仍需明确yaw外部参考、源选择和降级策略，EKF本身不提供绝对航向或双IMU融合。

## 职责与关联

QuaternionEkf维护每实例四元数、协方差、零偏和静止初始化。KalmanFilter提供通用线性预测/校正，调用者提供持久缓冲并显式Init，两者没有调用依赖。

输入 / 依赖：直接对接底层设备或算法。

消费者：[IMU：独立采集、姿态与加热](imu.md)

## 接口契约

### 1. QuaternionEkf::QuaternionEkf(const Config &c); int QuaternionEkf::init()

```cpp
QuaternionEkf::QuaternionEkf(const Config &c); int QuaternionEkf::init()
```

构造保存配置，init验证全部正数/范围并重置每实例状态。不依赖通用线性Kalman，也不动态分配内存。

| 参数 | 含义与边界 |
| --- | --- |
| `c` | dt范围、重力和加速度门限、过程/测量噪声、创新门限、滤波/零偏时间常数及静止初始化样本数。 |

**返回 / 输出：** init 0成功。

**线程 / 时序：** 单所有者；如果传给Bmi088Imu，则由该source.init初始化，不要先自行init。

**错误 / 边界：** -EINVAL：非有限/非正配置、dt范围逆序、初始化样本数<2；-EALREADY：已初始化配置。

### 2. int QuaternionEkf::update(const Input &input)

```cpp
int QuaternionEkf::update(const Input &input)
```

输入加速度与角速度，静止初始化后用陀螺仪预测、重力方向校正倾斜；强加速度时保留预测并标记Degraded。yaw无可观测绝对约束。

| 参数 | 含义与边界 |
| --- | --- |
| `input.accel_m_s2` | 同一传感器坐标的加速度，m/s²。 |
| `input.gyro_rad_s` | 角速度，rad/s。 |
| `input.time_us` | 两种输入对应的单调时间，us，必须严格递增。 |

**返回 / 输出：** 0=有有效输出；-EAGAIN=初始化中/采样间隔过短/间隔过长触发重置；读取quality区分Tracking与Degraded。

**线程 / 时序：** 算法所有者线程；没有内部锁，不从别的线程并发update/reset/read。Bmi088消费者应通过source快照读取。

**错误 / 边界：** -EACCES：未init；-EINVAL：非有限输入；-ESTALE：时间重复/倒退；-ERANGE：数值异常并重置。超过dt_max_s也会重置，generation递增。

### 3. void QuaternionEkf::reset(); core::Quaternion QuaternionEkf::attitude() const; Quality QuaternionEkf::quality() const; uint32_t QuaternionEkf::generation() const

```cpp
void QuaternionEkf::reset(); core::Quaternion QuaternionEkf::attitude() const; Quality QuaternionEkf::quality() const; uint32_t QuaternionEkf::generation() const
```

reset清除历史和零偏、进入Initializing、增加非零generation。attitude为q_WS，表示传感器坐标→局部重力对齐世界坐标的旋转。

**返回 / 输出：** 无返回码或值副本；Initializing时不得把默认四元数当有效姿态。

**线程 / 时序：** 仅所有者串行；source发布姿态时把estimator代次变化映射成OrientationReference.epoch。

**错误 / 边界：** reset不重新验证配置；Degraded仍有输出但加速度校正不足，需要消费者制定策略。

### 4. int KalmanFilter_Init(KalmanFilter *kf, uint16_t n, uint16_t m, const KalmanBuffers *buffers)

```cpp
int KalmanFilter_Init(KalmanFilter *kf, uint16_t n, uint16_t m, const KalmanBuffers *buffers)
```

校验维度和七块缓冲容量后绑定调用者内存，设置F=I、P=1000I、Q=0.001I、R=I、H对角观测、X/K=0；无设备树和堆分配。重复调用覆盖模型及状态。

| 参数 | 含义与边界 |
| --- | --- |
| `n / m` | 非零状态/观测维度。 |
| `buffers` | 七个KalmanBuffer的data与capacity，capacity以float元素计；数组互不重叠且覆盖使用周期。 |

**返回 / 输出：** 0成功；失败不改变实例和缓冲。描述符只在调用中读取。

**线程 / 时序：** 同一实例由所有者线程串行初始化、配置、计算和读取；不用局部短寿命数组。

**错误 / 边界：** -EINVAL：空指针/零维度；-ENOSPC：容量不足；-EOVERFLOW：矩阵索引或字节数范围溢出。

### 5. void KalmanFilter_Predict(KalmanFilter *kf)

```cpp
void KalmanFilter_Predict(KalmanFilter *kf)
```

通用线性预测：X=F·X，P=F·P·Fᵀ+Q。所有矩阵pData预先绑定，不由此函数分配持久内存。

| 参数 | 含义与边界 |
| --- | --- |
| `kf` | F/P/Q为n×n，X为n×1；有效非空缓冲。 |

**返回 / 输出：** void；内部CMSIS运算返回状态未向调用方传播。

**线程 / 时序：** 同一滤波实例单线程所有者；按n²增长的临时栈数组，不宜在ISR使用。

**错误 / 边界：** 不会检查空指针、全部维度/数值或返回矩阵运算错误；调用者必须保证合法性，不能据void调用完成判定数值有效。

### 6. void KalmanFilter_Correct(KalmanFilter *kf, const Matrix *z)

```cpp
void KalmanFilter_Correct(KalmanFilter *kf, const Matrix *z)
```

线性校正：由P/H/R算K，再更新X/P；这是线性矩阵步骤，不是自动四元数EKF。

| 参数 | 含义与边界 |
| --- | --- |
| `kf` | H=m×n，R=m×m，K=n×m，F/P/Q/X与predict一致。 |
| `z` | m×1观测矩阵，缓冲与维度正确；创新协方差须可逆。 |

**返回 / 输出：** void，结果写入kf->X/P/K。

**线程 / 时序：** 单所有者线程；多块n²/m²/nm临时栈缓冲。

**错误 / 边界：** 当前实现不传播Matrix_Inverse失败；不可逆/错误维度/非有限数可污染状态，调用方须建立模型并保证条件。

### 7. Matrix_Init(Matrix *matrix, uint16_t rows, uint16_t cols, float *buffer); void Matrix_Zero(Matrix *matrix); void Matrix_SetDiag(Matrix *matrix, float value)

```cpp
Matrix_Init(Matrix *matrix, uint16_t rows, uint16_t cols, float *buffer); void Matrix_Zero(Matrix *matrix); void Matrix_SetDiag(Matrix *matrix, float value)
```

Matrix是arm_matrix_instance_f32别名。Init绑定已有缓冲，Zero清零，SetDiag先清零再置对角元素；矩阵不拥有buffer。

| 参数 | 含义与边界 |
| --- | --- |
| `matrix` | 待初始化/写入矩阵。 |
| `rows / cols` | 行列，buffer至少rows×cols个float。 |
| `buffer` | 调用者持有、存活时间覆盖全部使用。 |
| `value` | 对角值。 |

**返回 / 输出：** void，修改结构体或缓冲。

**线程 / 时序：** 调用者管理内存与并发，不自动拷贝buffer。

**错误 / 边界：** 不提供空指针或容量检查；切勿把局部数组地址交给更长寿命对象。

### 8. Matrix_Add / Matrix_Subtract / Matrix_Multiply / Matrix_Transpose / Matrix_Inverse

```cpp
Matrix_Add / Matrix_Subtract / Matrix_Multiply / Matrix_Transpose / Matrix_Inverse
```

宏别名分别对应CMSIS-DSP arm_mat_add_f32/sub_f32/mult_f32/trans_f32/inverse_f32；通过输入矩阵指针与已初始化目标矩阵运算。

| 参数 | 含义与边界 |
| --- | --- |
| `输入 Matrix*` | 运算维度必须兼容。 |
| `目标 Matrix*` | 事先初始化正确尺寸与足量buffer；不要假设所有运算支持输入输出重叠。 |

**返回 / 输出：** arm_status；调用者应处理维度不匹配、奇异矩阵等返回。

**线程 / 时序：** 共享buffer访问需要调用者串行/加锁。

**错误 / 边界：** 独立矩阵API有返回状态，但通用Kalman封装当前未传播。

## 调用示例

### 独立EKF：静止初始化与时间戳驱动

```cpp
#include <control/attitude_ekf.hpp>
#include <cerrno>
using skywalker::control::QuaternionEkf;

int estimateExample() {
  QuaternionEkf estimator{QuaternionEkf::Config{}};
  int r = estimator.init();
  if (r < 0) return r;
  for (unsigned i=0; i<120; ++i) {
    // 演示输入；实机改成新鲜的同坐标传感器测量。
    const QuaternionEkf::Input in{
      .accel_m_s2={0,0,9.80665f}, .gyro_rad_s={0,0,0},
      .time_us=1000u + i*1250u};
    r = estimator.update(in);
    if (r < 0 && r != -EAGAIN) return r;
    if (!r && estimator.quality()==QuaternionEkf::Quality::Tracking) {
      const auto q = estimator.attitude();
      (void)q; // q_WS；主控若要机体姿态，还要应用安装旋转
    }
  }
  return r;
}
```

此例只说明数学接口，不驱动电机。Bmi088Imu内由source调用init/update并发布已经变换到机体的姿态；不要在应用线程再操作同一个estimator。

### 一维线性Kalman：静态缓冲与显式初始化

```cpp
#include <control/kalman_filter.h>
#include <cmath>
#include <limits>

float smoothExample(float measurement) {
  const float invalid = std::numeric_limits<float>::quiet_NaN();
  if (!std::isfinite(measurement)) return invalid;
  static float f, h, r, x, p, q, gain;
  static KalmanFilter kf{};
  static const KalmanBuffers buffers{
    .F={&f,1}, .H={&h,1}, .R={&r,1}, .X={&x,1},
    .P={&p,1}, .Q={&q,1}, .K={&gain,1}};
  static bool initialized=false;
  if (!initialized) {
    const int rc = KalmanFilter_Init(&kf,1,1,&buffers);
    if (rc < 0) return invalid;
    initialized=true;
  }
  Matrix z{};
  Matrix_Init(&z,1,1,&measurement);
  KalmanFilter_Predict(&kf);
  KalmanFilter_Correct(&kf,&z);
  return kf.X.pData[0];
}
```

示范平滑常量观测，Init默认R=1。此函数只能由一个线程串行调用；初始化失败或输入非有限返回NaN。开启CONFIG_SKYWALKER_LIB_KALMAN_FILTER，不需要设备树filter节点。void计算接口仍不传播内部矩阵错误。

## 调用顺序

1. 选定独立姿态EKF或通用线性模型，避免把二者串接成重复滤波。
2. QuaternionEkf：构造→init→稳定静止采样初始化→update→按quality使用；大时间缺口重置代次。
3. 通用Kalman：分配全部F/H/R/X/P/Q/K与观测缓冲→Init并处理错误→设置模型→Predict→Correct。
4. 重置姿态/参考系后，下游必须丢弃旧epoch的控制目标和历史。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `CONFIG_SKYWALKER_ATTITUDE_EKF` | 独立姿态算法，配合控制库构建；默认dt 0.2–20ms、初始化100个静止样本。 |
| `CONFIG_SKYWALKER_LIB_KALMAN_FILTER` | 开启普通线性Kalman算法，自动选择CONTROL、Matrix和CMSIS-DSP；与BMI088独立EKF分别开启。 |
| `QuaternionEkf::Config` | accel_gate_m_s2过滤明显非重力输入，innovation_gate控制测量校正，stationary_gyro_rad_s与initialization_samples控制静止初始化。 |
| `CONFIG_SKYWALKER_LIB_MATRIX_STORAGE` | 可选矩阵Flash存储，另需可用Flash/storage partition；不是姿态估计必需项。 |

- 没有磁力计/外部绝对观测的重力EKF不能提供绝对yaw；把yaw作为整车航向需要额外策略。
- 通用Kalman的void接口忽略内部矩阵错误，不能当作有错误恢复机制的通用安全封装。
- CMSIS Matrix_Init只绑定内存，不分配、不复制；实例缓冲不可共享给独立滤波器。
- 线性Kalman已移除设备树入口；维度由调用者模型决定，不能把4/3维度等同于姿态EKF。

## 正文与源码

- [Kalman与Matrix Markdown](../modules/drivers/kalman-matrix.md)

- [独立EKF API](../../include/control/attitude_ekf.hpp)
- [EKF实现](../../lib/control/attitude_ekf.cpp)
- [通用Kalman API](../../include/control/kalman_filter.h)
- [通用Kalman实现](../../lib/control/kalman_filter.c)
- [Matrix封装](../../include/lib/matrix/matrix.h)
- [数学选项](../../lib/Kconfig)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
