# IMU：独立采集、姿态与加热：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

BMI088 和达妙 RS485 共用带时间戳的快照，不同来源独立采集与独立诊断。

**接入状态：接入 / 配置未完成。** 板载与外置统一采集已实现，整车云台装配头部外置 IMU 与惯性适配；imu_mounting_confirmed 默认 false，姿态质量与参考仍须实测。

## 职责与关联

ImuSource 负责某种真实输入；ImuState 按字段发布测量与新鲜度；ImuReceiver 独占采集循环并可管理 ImuHeater。BMI088 可挂 QuaternionEkf 估计姿态；RS485 直接使用设备主动上报的四元数。

输入 / 依赖：[板级：MC02 / RoboMaster Type-C](boards.md)、[姿态 EKF 与通用 Kalman / Matrix](kalman.md)、[PID、前馈、斜坡与角度工具](pid.md)、[异步 UART 与 DMA](uart.md)

消费者：[头部惯性云台适配](inertial.md)

## 接口契约

### 1. virtual int ImuSource::init(); virtual int ImuSource::service(); virtual Snapshot ImuSource::snapshot() const

```cpp
virtual int ImuSource::init(); virtual int ImuSource::service(); virtual Snapshot ImuSource::snapshot() const
```

所有输入源的最小契约：init 初始化；service 驱动一次采集/解析；snapshot 读取副本。构造不执行I/O。

**返回 / 输出：** init：0成功；service：0有进展/-EAGAIN无新数据/其他负errno；snapshot：测量、状态、capabilities、fresh_mask和诊断。

**线程 / 时序：** 只有一个所有者调用init/service；任何线程可copy snapshot；已注册回调的对象与依赖须一直存活。

**错误 / 边界：** Running 不表示每个字段都新鲜；检查 fresh_mask 和 Measurement.stamp.valid。

### 2. Bmi088Imu::Bmi088Imu(const device *accel, const device *gyro, const Config &c, control::QuaternionEkf *estimator = nullptr); int Bmi088Imu::init(); int Bmi088Imu::service()

```cpp
Bmi088Imu::Bmi088Imu(const device *accel, const device *gyro, const Config &c, control::QuaternionEkf *estimator = nullptr); int Bmi088Imu::init(); int Bmi088Imu::service()
```

通过Zephyr传感器API读取加速度/角速度/温度；可选独立EKF生成姿态，sensor_to_body 将传感器轴旋转到机器人机体轴。

| 参数 | 含义与边界 |
| --- | --- |
| `accel / gyro` | 加速度和陀螺仪device，通常来自accel0/gyro0 alias。 |
| `c` | 采样周期默认1250us，温度周期10000us，最大输入时间偏差2000us，参考{id,epoch}、安装旋转与freshness。 |
| `estimator` | 可选长寿命 QuaternionEkf；交给该source独占，source.init会初始化它。 |

**返回 / 输出：** init 0成功；service 0已处理/-EAGAIN尚未到采样时间。

**线程 / 时序：** 所有者线程，会做SPI/sensor I/O；若ImuReceiver接管，其他线程不得直接再init/service或更新estimator。

**错误 / 边界：** -EALREADY：重复init；-EINVAL：周期/安装旋转不合法；-ENODEV：传感器不可用；-ENOTSUP：传入EKF但未编入；-EACCES：未初始化；-ESTALE：加速度/陀螺仪时间差超限；透传sensor错误。

### 3. DmImuRs485Source::DmImuRs485Source(const device *uart, communication::AsyncUart::DmaBuffers &dma, const Config &c); int DmImuRs485Source::init(); int DmImuRs485Source::service(); int DmImuRs485Source::resetReference()

```cpp
DmImuRs485Source::DmImuRs485Source(const device *uart, communication::AsyncUart::DmaBuffers &dma, const Config &c); int DmImuRs485Source::init(); int DmImuRs485Source::service(); int DmImuRs485Source::resetReference()
```

消费DM-IMU-L1主动串行帧，应用缩放和安装旋转；遇UART故障可内部重试。resetReference在已知设备复位/清零/标定后递增epoch并撤销旧姿态，不向设备发送配置命令。

| 参数 | 含义与边界 |
| --- | --- |
| `uart / dma` | 独占UART与长寿命DMA存储；H7使用__nocache。 |
| `c.protocol` | 设备id、半帧组装超时。 |
| `c.reference / sensor_to_body` | 独立参考身份与传感器→机体安装旋转。 |
| `c.device_quaternion_is_world_to_sensor` | 若固件上报世界→传感器，则先共轭成传感器→世界。 |
| `c.acceleration_scale / angular_velocity_scale` | 正数缩放系数，转换为m/s²和rad/s；取决于设备已保存输出单位。 |

**返回 / 输出：** init/service：0成功或负errno；resetReference：0完成本地重置。

**线程 / 时序：** 一个所有者调用；UART ISR只推进AsyncUart事件，不在ISR解析或算姿态。接收线程运行时不得外部直接resetReference。

**错误 / 边界：** -EINVAL：配置错误；-EALREADY：重复init；-EACCES：尚未初始化；-EAGAIN：无新帧/等待恢复；传输溢出丢掉半帧并记录transportGap；CRC/帧错误在diagnostics报告。

### 4. ImuReceiver::ImuReceiver(ImuSource &source, const Config &config, ImuHeater *heater = nullptr); int ImuReceiver::start()

```cpp
ImuReceiver::ImuReceiver(ImuSource &source, const Config &config, ImuHeater *heater = nullptr); int ImuReceiver::start()
```

启动一个源的专有工作线程，在线程内部异步init并持续service；可选加热器与此线程同所有者。

| 参数 | 含义与边界 |
| --- | --- |
| `source` | 生命周期覆盖工作线程的输入源。 |
| `config.poll_interval_us` | 轮询间隔，默认1000us；具体source决定真正采样频率。 |
| `config.priority` | 合法抢占优先级；默认CONFIG_SKYWALKER_IMU_RX_PRIORITY。 |
| `heater` | 可选长寿命加热器；由worker init/update管理。 |

**返回 / 输出：** 0=worker已启动，不等于source初始化成功或姿态可用。

**线程 / 时序：** 一次启动；无stop/restart或运行中销毁支持。不能再从别的线程调用source.init/service或heater.init/update/disable。

**错误 / 边界：** -EINVAL：轮询/优先级无效；-ENOTSUP：加热器未编入；-EALREADY：已经start。

### 5. Snapshot ImuReceiver::snapshot() const; ImuReceiver::Status ImuReceiver::status() const

```cpp
Snapshot ImuReceiver::snapshot() const; ImuReceiver::Status ImuReceiver::status() const
```

snapshot直接让source按读取时刻算freshness；status分别报告异步初始化、service以及heater错误/占空比。没有第二份样本缓存。

**返回 / 输出：** 独立副本；init_complete/init_error 判断初始化结果；service_error保留最近非-EAGAIN返回。

**线程 / 时序：** 允许多个读线程；snapshot与status彼此不是原子组合。

**错误 / 边界：** start成功后立即读到Uninitialized属于正常；业务要等姿态字段新鲜且quality达到要求。

### 6. int ImuState::init(uint32_t capabilities, core::OrientationReference reference); int ImuState::publish(const Update &update); Snapshot ImuState::snapshot() const

```cpp
int ImuState::init(uint32_t capabilities, core::OrientationReference reference); int ImuState::publish(const Update &update); Snapshot ImuState::snapshot() const
```

具体source共享的字段发布器，Update.updated_mask指示本次真正更新的字段；其余字段保留独立时间戳。

| 参数 | 含义与边界 |
| --- | --- |
| `capabilities` | Accel/Gyro/Orientation/Temperature能力位。 |
| `reference` | 参考系身份和重置epoch；身份不同不能直接混用姿态。 |
| `update` | 已转换为机体轴、SI单位的测量与更新位。 |

**返回 / 输出：** init/publish返回0或负errno；snapshot按Freshness逐字段计算fresh_mask。

**线程 / 时序：** 写者由source所有者串行；自旋锁保护多读者副本。

**错误 / 边界：** 无效/非有限更新不应视作新测量；diagnostics.invalid_updates报告被拒绝内容。

### 7. int DmImuParser::init(); int DmImuParser::consume(const uint8_t *bytes, size_t size, core::TimeUs now, DmImuSink &sink); void DmImuParser::discardPartial(); Statistics DmImuParser::statistics() const

```cpp
int DmImuParser::init(); int DmImuParser::consume(const uint8_t *bytes, size_t size, core::TimeUs now, DmImuSink &sink); void DmImuParser::discardPartial(); Statistics DmImuParser::statistics() const
```

主动帧字节组装、CRC与更新投递；consume(nullptr,0,now,sink)也可推进组装超时；传输缺口要discardPartial，避免拼接缺失帧。

| 参数 | 含义与边界 |
| --- | --- |
| `bytes / size` | 任意分片，size=0允许空指针。 |
| `now` | 当前单调时间us。 |
| `sink` | 同线程接收解码Update的对象。 |

**返回 / 输出：** consume返回成功接受帧数或负errno；statistics返回计数。

**线程 / 时序：** 所有方法仅解析器所有者线程，无内部并发保证。

**错误 / 边界：** CRC错误/无效帧/组装超时计数分别见Statistics；不执行配置写命令，不实现CAN。

### 8. int ImuHeater::init(); int ImuHeater::update(const core::Measurement<float> &temperature, core::TimeUs now); int ImuHeater::disable(); ImuHeater::Snapshot ImuHeater::snapshot() const

```cpp
int ImuHeater::init(); int ImuHeater::update(const core::Measurement<float> &temperature, core::TimeUs now); int ImuHeater::disable(); ImuHeater::Snapshot ImuHeater::snapshot() const
```

独立温控PID与PWM安全管理。init明确写0占空比；温度过期、超上限或无效时尝试关闭；snapshot分别保留原因和关闭操作错误。

| 参数 | 含义与边界 |
| --- | --- |
| `temperature` | 带有效性与时间戳的摄氏温度。 |
| `now` | 单调时间us；不允许倒退。 |
| `Config` | PWM设备/周期，目标默认50°C、最大65°C，温度超时200ms、控制周期100ms，输出PID限幅0–1。 |

**返回 / 输出：** 0成功；update -EAGAIN未到周期；其他负errno为错误。

**线程 / 时序：** 一个线程拥有init/update/disable；snapshot多读者。已附加ImuReceiver后全部控制由worker处理。

**错误 / 边界：** -EACCES：未init；-EALREADY：重复init；-EINVAL/-ENODEV：配置/设备错；-ESTALE：过期/时间倒退；-ERANGE：≥最大温度、低于-40或非有限；PWM关断失败会返回底层错误并保留disable_error。

## 调用示例

### 板载BMI088 + EKF：异步采集，业务只读快照

```cpp
#include <drivers/imu/bmi088_imu.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <core/attitude.hpp>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <cerrno>
namespace imu = skywalker::imu;

int main() {
  static skywalker::control::QuaternionEkf ekf{
      skywalker::control::QuaternionEkf::Config{}};
  static imu::Bmi088Imu source{
      DEVICE_DT_GET(DT_ALIAS(accel0)), DEVICE_DT_GET(DT_ALIAS(gyro0)),
      imu::Bmi088Imu::Config{.reference={1,1}, .sensor_to_body={}}, &ekf};
  static imu::ImuReceiver receiver{
      source, imu::ImuReceiver::Config{.poll_interval_us=500, .priority=5}};
  const int r = receiver.start();
  if (r < 0) return r;
  // start只启动线程；init与静止初始化由worker完成。
  for (;;) {
    const auto s = receiver.snapshot();
    const auto diagnostic = receiver.status();
    if (diagnostic.init_complete && diagnostic.init_error < 0)
      return diagnostic.init_error;
    if ((s.fresh_mask & imu::Orientation) &&
        s.sample.attitude_quality == imu::AttitudeQuality::Tracking) {
      const auto e = skywalker::core::euler(s.sample.orientation.value);
      printk("roll=%d pitch=%d mrad\n", int(e.roll*1000), int(e.pitch*1000));
    }
    k_msleep(20);
  }
}
```

安装旋转的单位四元数仅适用于传感器轴与机体轴一致；reference={1,1}是示例身份。启用IMU/BMI088/RECEIVER、ATTITUDE_EKF、SENSOR/BMI08X、FPU/FPU_SHARING以及CPP。初始化需稳定静止；加速度仅能校正倾斜，yaw会漂移。完整RS485+加热例见dual_imu样例。

## 调用顺序

1. 构造输入源、可选EKF和heater；UART DMA、所有对象长期存活。
2. 调用receiver.start；worker负责heater.init和source.init，应用通过status等异步结果。
3. 单一worker周期service，source将各字段独立发布；姿态初始化阶段不可直接控制云台。
4. 控制/遥测线程只snapshot并检查fresh_mask、参考身份/epoch和姿态质量。
5. 温度过期或错误由同一worker关闭heater；源失败与加热关闭失败分别诊断。
6. 设备复位或参考变化必须撤销旧姿态并更新epoch；没有通用自动双IMU融合/切换逻辑。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `CONFIG_SKYWALKER_IMU / BMI088 / RECEIVER` | 基础、板载传感器源、独立采集线程分别按需开启。 |
| `CONFIG_SKYWALKER_ATTITUDE_EKF` | 为BMI088生成姿态；不依赖线性Kalman或Matrix库。 |
| `CONFIG_SKYWALKER_IMU_DM_RS485` | 依赖UART_TRANSPORT并选择DM主动协议；设备单位、主动输出和速率需工具配置。 |
| `CONFIG_SKYWALKER_IMU_HEATER` | 需PWM，选择控制算法库；控制与source独立，但可由同一receiver持有。 |
| `Freshness` | 默认加速度/角速度/姿态20ms、温度1s；heater自己另外要求温度200ms内新鲜。 |
| `RX_STACK_SIZE / RX_PRIORITY` | 默认8192字节/优先级6；每个receiver独立线程，涉及浮点时按FPU配置分配寄存器上下文。 |

- DM CAN 类保留纯虚init/service/snapshot，不能直接实例化；没有CAN接收器或对应Kconfig。
- 两颗IMU不代表已融合。reference身份与epoch必须匹配消费者约定，不能随意相减不同零点的yaw。
- fresh_mask按字段变化；Running但Orientation不新鲜时不能使用上次四元数继续控制。
- ImuReceiver没有stop或析构协议，main栈上的短寿命对象不可交给它。
- H7的RS485 DMA缓冲需要__nocache和CONFIG_NOCACHE_MEMORY；避免CPU缓存看到旧字节。
- 加热停机要看disable_error；软件目标占空比0并不自动证明PWM外设写入成功。

## 正文与源码

- [IMU Markdown](../modules/drivers/imu.md)
- [双IMU台架](../../samples/imu/dual_imu/README.md)

- [统一测量类型](../../include/drivers/imu/imu_types.hpp)
- [输入源接口](../../include/drivers/imu/imu.hpp)
- [接收线程API](../../include/drivers/imu/imu_receiver.hpp)
- [BMI088输入](../../drivers/imu/bmi088_imu.cpp)
- [RS485输入](../../drivers/imu/dm_imu_rs485.cpp)
- [加热保护](../../drivers/imu/imu_heater.cpp)
- [双IMU样例](../../samples/imu/dual_imu/src/main.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
