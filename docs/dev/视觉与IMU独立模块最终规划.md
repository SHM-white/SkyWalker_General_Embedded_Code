# 视觉与 IMU 独立模块最终规划与实施记录

日期：2026-09-30。本文合并原视觉施工文档、落地评审和独立接口指南，保留最终设计依据及手册复核证据。

**本轮已按用户指令禁用古法编程模式并实施源码。** 独立接口不考虑旧 API 兼容；外置 IMU 实现 RS485 主动接收，CAN 仅保留抽象接口。按最新指令新增两个独立 sample；业务模块仍不依赖任何现有 application 或机器人角色。

## 本轮实施结果

| 内容 | 已实施位置/说明 |
|---|---|
| 公共测量与坐标 | include/core/measurement.hpp、clock.hpp、attitude.hpp、byte_codec.hpp |
| IMU 源与状态核心 | include/drivers/imu/imu.hpp、imu_types.hpp、imu_state.hpp；drivers/imu/imu_state.cpp |
| 板载采样、滤波、温控 | Bmi088Imu、QuaternionEkf、ImuHeater；无全局 EKF 状态和聚合 device |
| 外置 RS485 | DmImuParser + DmImuRs485Source，复用 AsyncUart，字段独立接收时间 |
| CAN | dm_imu_can.hpp 抽象接口/Config；没有实现或 Kconfig 开关 |
| 视觉 | VisionLink + AbProtocol + VisionReceiver，按用户随后提供的 RM2026-AutoAim 协议实现 |
| 视觉样例 | [UART7 接视觉，USART1 输出 VOFA](../../samples/communication/vision/README.md)；带独立 Python 发送器 |
| 双 IMU 样例 | [板载 BMI088 + 485-2 外置 IMU](../../samples/imu/dual_imu/README.md)；板载 EKF、FPU 与 50℃ 温控 |
| 旧入口迁移 | 移除 imu.h/imu.c、skywalker,imu binding 和旧配置项；imu_test 迁移新 API，无 wrapper |
| 正式说明 | [IMU 接口](../06-drivers-imu.md)、[视觉协议与接口](../18-vision.md)；实际声明以正式头文件为准 |

与初始规划相比，本轮已明确以下事项：

- 视觉线协议由用户选定为 AB：下行 29 字节、上行 43 字节，CRC 右移 8408；上行增加 mode 和带测量时间的 bullet_count，未采用新造 V1。
- AB 没有参考与有效位，内部仍保留这些语义；缺少必要反馈数据时拒绝编码，反馈参考必须与固定 command_reference 一致。视觉接收样例不填造姿态解锁上位机。
- 当前 BMI08x 的温度 channel_get 会独立读寄存器，可以独立检查读取成功，无须假定它只是前一次 fetch 的缓存。
- EKF 使用实例内固定数组和普通浮点数学，不依赖 Kalman device、CMSIS 矩阵注册或旧设备树节点。
- 样例明确采用 USART1 输出 VOFA；用户已将板级 telemetry-uart 恢复为 USART1。两个新样例的硬件绑定保留在各自配置中，不再给旧样例额外添加遥测 overlay。
- 下文保留规划时的接口示意与施工顺序，便于理解设计；新增的具体 codec、接收线程和反馈扩展详见正式接口文档。

主流程运行验收已通过，覆盖双 EKF 实例、DM 已知 CRC 报文、合成四元数、AB 流式接收/反馈、独立过期及模拟 PWM 关断。真实 UART/SPI、温控硬件、外置固件和安装方向尚未实测。构建产物与日志位于 /tmp，本次未烧录设备。

## 1. 采用的结构

```text
                   时间戳 / 向量 / 四元数 / 姿态参考标识
                              ↑         ↑
                        IMU 模块      视觉模块
                              │         │
                       ImuSnapshot    AimCommand / Feedback
                              │         │
                  ┌───────────┴────┐  VisionLink
                  │                │    │
              Bmi088Imu        外置 IMU  VisionProtocol
                  │                │    │
             采样 + 本地 EKF   设备解码  字节输入/输出

Heater 是独立可选组件。
未来组装者把 IMU 姿态值复制到 Feedback，两个模块不互相持有对象。
```

明确决策：

- BMI088 直接重构采样和 EKF，不再给旧 `imu_fetch/imu_estimate` 加 wrapper。
- 通用 IMU 接口只定义数据、读取、状态；不定义 base/head、云台、安全、融合或故障回退。
- 暂不增加 ImuService 和 AttitudeProvider 两层转发。多个实例由未来组装层命名和选择。
- 姿态以四元数为权威表示；Euler 由四元数推导，避免两个独立更新的姿态互相矛盾。
- 视觉线协议现已由用户选定为 AB；其具体布局、CRC、无有效位和固定参考的处理见正式视觉文档。
- 无兼容性要求允许改变内部 API；DM 等外部设备的真实帧格式仍须遵守。

## 2. 重构前的问题与拆分依据

| 当前代码 | 问题 | 新实现 |
|---|---|---|
| `include/drivers/imu/imu.h` | 暴露内部 data、滤波器和加热配置 | 用 imu_types.hpp、imu.hpp 替换，不提供旧函数转发 |
| `drivers/imu/imu.c:128` | void 采样接口忽略 sensor 错误 | 检查每次 fetch/get；成功字段才更新时间和数据 |
| `drivers/imu/imu.c:242` | EKF 运行状态是文件级 static | 每实例持有四元数、协方差、零偏、历史和暂存区 |
| `drivers/imu/imu.c:164` | 估计器依赖 device config/data | 普通算法对象输入测量、输出四元数 |
| `drivers/imu/imu.c:189` | PWM 通道、周期与 IMU 强绑定 | 可选 ImuHeater，显式接收 PWM 配置 |
| `dts/bindings/imu/skywalker,imu.yaml` | 强制依赖 heater/filter 聚合节点 | C++ 对象通过构造参数组合底层设备 |

保留 Zephyr BMI08x sensor 驱动作为采样工具，复用已验证的算法思路，不保留阻碍新结构的 API。

每个底层 sensor 的 fetch/get 由一个采样所有者串行调用，读快照者不触发采样。[Zephyr 官方 Fetch and Get 文档](https://docs.zephyrproject.org/latest/hardware/peripherals/sensor/fetch_and_get.html)说明 fetch 会阻塞线程，无锁多上下文访问不受保证；本项目锁定版本的本地文档也有相同约束。

## 3. 共同值类型和时间语义

新增 `include/core/measurement.hpp`，不引用现有 `robotics/messages/common.hpp`，避免引入 motor lifecycle。

```cpp
#pragma once
#include <cstdint>

namespace skywalker::core {
using TimeUs = std::uint64_t;
struct Vec3 { float x = 0, y = 0, z = 0; };
struct Quaternion { float w = 1, x = 0, y = 0, z = 0; };

struct Stamp {
    TimeUs time_us = 0;
    std::uint64_t sequence = 0;
    bool valid = false;
};
template<class T> struct Measurement {
    T value{};
    Stamp stamp{};
};
struct OrientationReference {
    std::uint32_t frame_id = 0;
    std::uint32_t epoch = 0;
};
inline bool fresh(const Stamp& s, TimeUs now, TimeUs limit) {
    return s.valid && limit != 0 && now >= s.time_us &&
           now - s.time_us <= limit;
}
}
```

约定：

1. 所有 time_us 使用 MCU 同一单调时钟域；0 不代表无效，valid 才决定有没有已确认的数据。
2. 本地 sequence 只随新测量/新命令递增；读取、无数据轮询和失败重试不递增。
3. 没有硬件采样时间时用获取完成或完整报文接收时间，不用处理积压队列时的 now。
4. 微秒单位不等于微秒精度。现有 AsyncUart 的 timestamp_ms 若乘 1000 接入，仍只有毫秒粒度；提高精度需另改 transport。
5. frame_id 由实例配置指定且非 0，不表示 base/head。epoch 在姿态初始化、已知重启、重新校准后改变且非 0。
6. reference 改变时旧姿态失效。epoch 的跨 MCU 重启唯一性属于将来的会话协议，不声称这里已经解决。
7. 相同微秒时间可以接收两个不同事件，用 sequence 区分；不能用“数值没有变化”判断没有新采样。倒退时间不可覆盖较新字段。

共同数据头不包含设备、UART、CAN、内核锁、控制命令；平台时钟调用放在实现文件。

## 4. IMU 数据与公开接口

新增 `include/drivers/imu/imu_types.hpp`：

```cpp
#pragma once
#include <core/measurement.hpp>

namespace skywalker::imu {
enum Field : std::uint32_t {
    Accel = 1u << 0, Gyro = 1u << 1,
    Orientation = 1u << 2, Temperature = 1u << 3
};
enum class State : std::uint8_t { Uninitialized, Running, Fault };
enum class AttitudeQuality : std::uint8_t {
    Unavailable, Unknown, Initializing, Tracking, Degraded
};
struct Freshness {
    core::TimeUs accel_us = 20000, gyro_us = 20000;
    core::TimeUs orientation_us = 20000, temperature_us = 1000000;
};
struct Sample {
    core::Measurement<core::Vec3> accel_m_s2{};
    core::Measurement<core::Vec3> gyro_rad_s{};
    core::Measurement<core::Quaternion> orientation{};
    core::Measurement<float> temperature_c{};
    core::OrientationReference reference{};
    AttitudeQuality attitude_quality = AttitudeQuality::Unavailable;
};
struct Diagnostics {
    int last_error = 0;
    std::uint32_t io_errors = 0, invalid_updates = 0, transport_gaps = 0;
};
struct Snapshot {
    Sample sample{};
    State state = State::Uninitialized;
    std::uint32_t capabilities = 0, fresh_mask = 0;
    Diagnostics diagnostics{};
};
}
```

三个概念不能混成一个 alive：

| 字段 | 含义 |
|---|---|
| capabilities | 当前实现/配置能输出哪些数据，例如无 EKF 的源不声明 Orientation |
| measurement.stamp.valid | 保留的这份数据曾通过校验；默认单位四元数不自动成为有效姿态 |
| fresh_mask | 当前读取时哪些字段尚未过期；新 accel 不能续期旧 gyro 或 quat |

timeout 默认值只是初值；各支持字段须配置为大于 0。Running 表示已初始化，不等于数据仍然新鲜。

新增 `include/drivers/imu/imu.hpp`：

```cpp
#pragma once
#include <drivers/imu/imu_types.hpp>

namespace skywalker::imu {
class ImuSource {
public:
    virtual ~ImuSource() = default;
    ImuSource(const ImuSource&) = delete;
    ImuSource& operator=(const ImuSource&) = delete;
    virtual int init() = 0;
    virtual int service() = 0;
    virtual Snapshot snapshot() const = 0;
protected:
    ImuSource() = default;
};
}
```

采用普通 C++ 对象，不增加第二套 Zephyr device API。底层设备由构造参数提供；对象可静态分配，不要求 heap、字符串工厂或注册表。

| 接口 | 契约 |
|---|---|
| 构造 | 保存依赖/配置，不做 I/O、不创建线程；设备和外部依赖必须活得更久 |
| init | 0 初始化成功，但不保证已经有姿态；-EINVAL 配置错误；-ENODEV 设备不可用；-EALREADY 已成功初始化；其他错误透传 |
| service | 单所有者线程推进一轮采样或有界队列处理；不 sleep 等下一帧。0 本轮完成，可能只更新部分字段；-EAGAIN 无新数据/未到采样期；其他负值表示错误 |
| snapshot | 普通读线程可并发调用，返回一致副本，无 I/O、不推进滤波；复制后取当前时间生成 fresh_mask |

service 中的 SPI 操作可能阻塞底层传输，不将它称为硬实时无阻塞函数。不同源可由不同线程按各自周期调度。

失败字段保留最后正确值和时间，错误计入 diagnostics；已知参考丢失或硬复位清除相应 valid。Fault 时 fresh_mask=0。注册了 transport 回调的对象必须持续存活，不能随意析构重建。init 的部分失败由后端记录并通过 service 推进恢复，不重复注册回调；不可恢复错误进入 Fault。未开始初始化的源调用 service 返回 -EACCES。恢复为 Running 前清楚区分资源已注册与接收已就绪；没有新测量时仍不能置 valid。

### 4.1 必须完成具体状态核心

新增内部 `drivers/imu/imu_state.hpp/.cpp`，由具体源组合持有，不只写一套纯虚声明：

- 输入 Update 带 updated_mask，仅列本次实际成功更新的字段；其余字段保持原值。
- 先在锁外校验数值、时间、四元数范数及变换结果。一个 Update 校验失败则整次不提交；backend 可分次提交互不依赖的成功字段。
- 发布 orientation 时同时发布对应 reference 与 quality。
- 锁内只复制状态/诊断；不能做采样、解码、矩阵运算、日志或分配。
- snapshot 在短临界区复制，离开锁后取 now 计算各字段 freshness，即使采样线程卡住也能过期。
- 可使用 k_spinlock 实现短复制；公共 API 都按线程上下文调用，中断只送原始数据到有界队列。

单次温度失败不阻止已经成功的 gyro 发布，但绝不能把 temperature 的时间一起更新。底层 fetch 失败后即使 channel_get 能读到缓存，也不能把该缓存当作新测量。

## 5. BMI088、EKF、温控的实现边界

`Bmi088Imu final : public ImuSource` 接收 accel/gyro 的 const device 指针、采样周期、安装变换、freshness 和可选估计器引用。它拥有采样与发布状态；不要求存在 heater。

| 组件 | 工作 |
|---|---|
| Bmi088Imu | 获取、检查和标记原始数据；调用估计器；发布快照 |
| QuaternionEkf | 输入校准后的原始测量和时间；每实例估计四元数与质量 |
| ImuHeater | 接收有效温度、目标温度与 PWM 配置；独立输出加热功率 |

外置智能 IMU 直接采用其设备输出，不再送进本地 EKF；其内部恒温也不模拟成 MCU PWM。

`drivers/imu/bmi088_imu.cpp` 的 service 流程：

```text
检查初始化和下次采样期限；未到期返回 -EAGAIN
更新 next_due，落后时不无界补采样
fetch(accel) 成功才 get(accel_xyz)
fetch(gyro)  成功才 get(gyro_xyz)
按独立温度周期 get 温度，并检查返回值（锁定 BMI08x 驱动会直接读温度寄存器）
各成功字段检查 finite，标记获取完成时间并发布
本轮 accel/gyro 均成功获得新输入且时间差不超过 max_input_skew_us：
    用新输入推进估计器一次
    输出合法才发布 orientation
有读取错误则报告实际错误，已成功字段可以保留发布结果
```

读取缓存不一定是新硬件样本。首版按配置输出频率限速轮询；要严格新样本节奏时使用 DRDY/硬件序号。分别读取的 accel/gyro 不宣称硬件同步。

### 5.1 EKF 直接提取为算法对象

建议新增 `include/control/attitude_ekf.hpp`、`lib/control/attitude_ekf.cpp`。对外提供 reset、update(raw_input)、attitude，单所有者调用；算法无 device 指针、无 DT、无锁。

- 四元数、协方差、零偏、滤波历史、上一采样时间、所有矩阵存储都属于实例。
- 矩阵尺寸按当前算法固定，不再要求 skywalker,kalman_filter 节点提供内存。
- 参考旧算法公式，重写错误路径；不能直接搬全局 static、void 错误语义、通过 long 指针类型别名实现的倒平方根。
- 矩阵失败、非有限值、近零四元数返回错误，不发布坏结果。初始化完成前保持 Initializing。
- dt 超出配置范围时重新初始化/报告错误，不跨长时间掉线积分旧 gyro。
- update 返回 0 表示新姿态可发布；-EAGAIN 初始化中；-EINVAL/-ERANGE 输入/数值不合要求。
- 动态加速度导致重力修正被拒绝时，可以输出明确 Degraded 的 gyro 预测；不要声称该步获得了新的重力约束。

本轮固定一种 EKF 路线，不扩展为通用算法插件平台。旧 Kalman 设备即使被其他模块使用，也不必顺带重构整个模块；新估计器只需摆脱对它的 device 注册依赖。

### 5.2 温控是可选组件

ImuHeater 对外独立提供 init、update(温度测量, now)、disable，返回 0 或明确的配置/温度过期/PWM 错误。构造不触碰 PWM，init 不自动开始加热；update 只有在目标温度与测量均有效时输出。无有效温度时先尝试撤销 PWM，再报告错误，PWM 关闭失败必须在诊断中可见。

PWM 通道、周期和目标温度由明确配置提供，不把旧文件里的板级数值带入通用接口。温控故障和“能否读取 gyro”是两个状态。

## 6. 坐标和姿态约定一次定清

S 为传感器坐标，B 为配置指定的输出坐标，W 为实例当前姿态参考。采用右手系、Hamilton 四元数、wxyz。`q_WB` 将 B 中向量转到 W。

安装参数 `q_BS` 表示 S→B：

```text
accel_B = R(q_BS) × accel_S
gyro_B  = R(q_BS) × gyro_S
q_WB    = q_WS × conjugate(q_BS)
```

安装参数必须有限、非零且接近单位四元数；小误差可归一化，明显错误拒绝。测量四元数也先检查范数再归一化，可按与上次的点积选择 q/-q 保持表示连续。

本地 EKF 建议在 S 中计算 q_WS，最后变换一次；不要先旋转输入又重复旋转输出。Euler 采用 ZYX 分解、命名字段 roll/pitch/yaw、单位 rad：

```text
roll  = atan2(2(wx + yz), 1 - 2(x² + y²))
pitch = asin(clamp(2(wy - zx), -1, 1))
yaw   = atan2(2(wz + xy), 1 - 2(y² + z²))
```

pitch 接近 ±π/2 存在 Euler 奇异性，基础接口仍用四元数。旧 imu_ekf_get_angle 的输出槽与这套约定需要重新核对；无兼容性要求下直接规范，不保留旧槽映射。

六轴 IMU 的 W 可以是局部初始化参考，不默认为地理绝对航向。已知复位/归零改变 epoch；若设备协议无法报告远端重启，须明确“参考变化无法完全自动检测”的限制。

## 7. DM-IMU-L1 手册复核与外置源落地

### 7.1 基础功能能否支持

**结论：可以支持加速度、角速度、姿态四元数的基础读取，足以成为统一 ImuSource 的外置实现。** 这是设备能力与文档可实现性的结论，尚未进行目标实物接收验收。

依据：[达妙科技 DM-IMU-L1 六轴 IMU 模块使用说明书 V1.2](../datasheets/达妙科技DM-IMU-L1六轴IMU模块使用说明书V1.2.pdf)，封面日期 2025.12.05。下文 Pn 为手册印刷页码；PDF 有一页封面，所以 PDF 阅读器中的物理页码为 n+1。

| 功能 | 手册证据 | 最终接口处理 |
|---|---|---|
| 原始惯性测量 | P2—3：BMI088，加速度和角速度；典型量程 ±6G、±2000°/s | 发布 accel_m_s2、gyro_rad_s，逐字段时间戳 |
| 设备解算姿态 | P2、P5、P10：内部 EKF，欧拉角和四元数输出 | 以设备四元数为权威，不再送入本地 EKF |
| 通信与频率 | P2、P5—10：USB、RS485、CAN，支持 100—1000 Hz | 实际后端只实现选定总线；频率、波特率须与实物保存配置一致 |
| 安装坐标 | P4 有 X/roll、Y/pitch、Z/yaw 示意 | 作为初始核对依据；仍需验证正方向、四元数旋转方向及安装变换 |
| 自带恒温与校准 | P2、P6、P11—13 | 外置源不控制 MCU PWM；校准、归零和保存配置不在 init 中自动执行 |
| 温度 | P10 的 CAN 加速度报文含一个温度字节；主动串行四类帧不含温度 | CAN 温度编码尚需确认；首版不声明 Temperature 能力 |
| 绝对航向 | 六轴器件，P3 给出静置漂移数据 | 不承诺地理绝对 yaw，也不做双 IMU 自动融合 |
| 采样时刻/解算质量 | 公开数据帧未给硬件时间戳、采样序号、收敛质量 | 用本地接收时间和本地序号；合法姿态的 quality=Unknown |

P3 在 25℃ 一小时测试中列出的 yaw 漂移为 12.686°，不能把局部 yaw 当成长期稳定绝对航向；该数值也不是所有安装条件下的保证值。寄存器表出现“磁力计校准”一项，不足以证明本型号具备磁力计，仍按六轴处理。

四元数通过 CRC、有限值和范数检查，只能证明报文及数值可接受，不能推出内部 EKF 已收敛。没有姿态时用 Unavailable，有已接受但设备未报告质量的姿态时用 Unknown；freshness 继续独立判断。

### 7.2 RS485 主动输出的字节格式

P7 说明 RS485 主动输出沿用 P5 的 USB 数据格式。模块含 RS485 收发器，不把它当作 TTL UART 接到主控引脚。基础方案使用主动模式；应答模式的 A5…5A 协议另行实现，也不假定为 Modbus。

| 字段 | 三轴帧 | 四元数帧 |
|---|---|---|
| 帧头 | 偏移 0—1：55 AA | 同左 |
| 模块 ID | 偏移 2：1 字节 | 同左 |
| 类型 | 偏移 3：01 加速度、02 角速度、03 欧拉角 | 偏移 3：04 |
| 数据 | 偏移 4—15：三个小端 float32 | 偏移 4—19：w、x、y、z 四个小端 float32 |
| CRC | 偏移 16—17 | 偏移 20—21 |
| 帧尾 | 偏移 18：0A | 偏移 22：0A |
| 总长 | 19 字节 | 23 字节 |

一轮四类全开是 19+19+19+23=80 字节，**这是四个独立帧，不是固定的 80 字节大结构**。接收块、DMA 回调、类型到达次序都不能当作整组采样边界。float 内也可能出现 55、AA、0A，不能用换行切包。

解析器先找帧头，检查 ID 和类型，再按类型确定长度；长度齐全后检查帧尾和 CRC，再解码字段。不要把未对齐字节指针强转成 float 指针；使用按小端组装的 uint32 位模式和 memcpy，先确认目标支持 32 位 IEEE 754 float。所有数值必须 finite，四元数还需检查范数。

P5 对串行数据给出了 float 格式，但未在每个字段处完整标注单位。手册截图中的静止加速度约 9.7、欧拉角约 -169，以及 CAN 量程都支持“加速度 m/s²、角速度 rad/s、欧拉角 deg”的初始解释；这是结合手册内容的推断。硬件验收必须用静置、已知角度和已知方向旋转确认，不能因字段是 float 就自动认定单位正确。公开接口始终输出第 6 节的单位和坐标。

### 7.3 CRC 已由手册实帧交叉核对

P20—21 的表采用 0x1021，初值 FFFF，更新式确实为 **左移 1 位**。本次从 P10 上位机截图中读出四条完整报文，分别按附录算法计算，全部匹配：

~~~text
55 AA 01 03 ED E3 2E BF 41 11 8F C0 82 1D 29 C3 6A 57 0A
55 AA 01 02 02 6D F1 3B EB E2 0E BD 5A 59 0F 3C 8F 26 0A
55 AA 01 01 07 F0 40 3F A0 43 C6 BD F1 A5 1B 41 87 6C 0A
55 AA 01 01 78 5F 47 3F 74 7C F4 BD 41 86 1B 41 E6 22 0A
~~~

| 类型 | 计算得到的 CRC | 帧中的低字节、高字节 | 改为常见左移 8 位的结果 |
|---|---|---|---|
| 03 欧拉角 | 576A | 6A 57 | 429B，不匹配 |
| 02 角速度 | 268F | 8F 26 | B330，不匹配 |
| 01 加速度 | 6C87 | 87 6C | D392，不匹配 |
| 01 加速度 | 22E6 | E6 22 | 90BF，不匹配 |

这四条 19 字节帧支持：从帧头开始计算前 16 字节，排除 CRC 和帧尾；CRC 低字节先发；初值 FFFF，无末尾异或，沿用手册的左移 1 位更新。**原评审中“可能是附录移位笔误”的疑点，在这四条手册实帧上已排除，不能擅自替换成常见 CRC-CCITT 例程。**

建议在 drivers/imu/dm_imu_protocol.cpp 内独立实现并命名为 DM 专用 CRC，不链接 referee.cpp。以下代码展示确切计算语义；table 可在编译期生成，不必每次建表：

~~~cpp
// 先生成与手册一致的 256 项表。
for (unsigned i = 0; i < 256; ++i) {
    std::uint16_t c = static_cast<std::uint16_t>(i << 8);
    for (unsigned bit = 0; bit < 8; ++bit) {
        const bool high = (c & 0x8000u) != 0;
        c = static_cast<std::uint16_t>(
            (static_cast<std::uint32_t>(c) << 1) ^
            (high ? 0x1021u : 0u));
    }
    table[i] = c;
}

// 计算一帧从头到数据末尾的 CRC；不含 CRC 自身和 0A。
std::uint16_t crc = 0xffffu;
for (std::size_t i = 0; i < covered_size; ++i) {
    const auto index = static_cast<std::uint8_t>((crc >> 8) ^ bytes[i]);
    crc = static_cast<std::uint16_t>(
        (static_cast<std::uint32_t>(crc) << 1) ^ table[index]);
}
~~~

四元数 23 字节帧拟沿用同一算法，覆盖前 20 字节、CRC 位于 20—21，但本次未取得可读的完整四元数实帧，必须在目标固件抓包后确认。上面四条截图帧是实现依据，不代表已验证所有固件版本；不在运行中“尝试多种 CRC、哪个通过就接受”。

### 7.4 CAN 的解码规则与手册冲突

P8—10 给出标准 CAN 帧、DLC=8。配置请求使用 CAN_ID，应答使用 MST_ID；主动输出接收 ID 应与实际配置和抓包一致，不能写死为其他设备的 ID。接收过滤器先检查总线、ID、标准数据帧和 DLC，再识别报文类型。

P10 的主动数据首字节为类型：

| 类型 | 剩余数据 | 归一化规则 |
|---|---|---|
| 01 | data[1] 温度字节，data[2..7] 加速度 XYZ | 三个小端 uint16，映射区间 ±235.2 |
| 02 | data[1] 保留，data[2..7] 角速度 XYZ | 三个小端 uint16，映射区间 ±34.88 |
| 03 | data[1] 保留，data[2..7] pitch、yaw、roll | pitch ±90°，yaw/roll ±180°；注意顺序 |
| 04 | data[1..7] 共 56 位四元数 wxyz | 每分量 14 位，映射到 [-1,1] |

P15—16 的映射形式是：

~~~text
physical = raw / (2^bits - 1) × (max - min) + min
~~~

这里 raw 是无符号整数，不是 int16_t 补码。±235.2 是 CAN 编码映射区间，不能拿它替换 P3 的实际传感器量程；±34.88 与 ±2000°/s 对应，按 rad/s 接入仍须用实测确认。欧拉角转换 deg→rad，且只用于诊断对照，不和四元数交替更新权威姿态。

P10 位布局给出的四元数提取为：

~~~cpp
const std::uint16_t w = (std::uint16_t(d[1]) << 6) | (d[2] >> 2);
const std::uint16_t x = ((std::uint16_t(d[2]) & 0x03u) << 12) |
                        (std::uint16_t(d[3]) << 4) | (d[4] >> 4);
const std::uint16_t y = ((std::uint16_t(d[4]) & 0x0fu) << 10) |
                        (std::uint16_t(d[5]) << 2) | (d[6] >> 6);
const std::uint16_t z = ((std::uint16_t(d[6]) & 0x3fu) << 8) | d[7];
~~~

P18 附录 W 项使用 (d[2] & 0xF8) >> 2，与 P10 位布局不一致：W 所需六位对应 0xFC。比如 d[2]=04 时低六位部分应得到 1，0xF8 会错误得到 0。上面的实现依据位布局推导；合成位边界只能检查实现，正式验收还要真实四元数帧和姿态方向对照，不能把 CAN 载荷整体按“小端四个整数”读取。

还必须保留两项配置限制：

1. P9 注明 RID=0F 的“输出数据选择”在对应固件中不可修改。P6 的 USB 快捷输出开关不能直接证明 CAN 可以关闭某几类数据；CAN 带宽按实际输出的四类规划。四类全开 1000 Hz 时为 4000 帧/秒，还要计入仲裁、位填充和其他节点。
2. P8 正文给出的配置请求是 ID=CAN_ID、DLC=8、内容含 CC/RID/读写/DD/数据；P17 附录旧请求却用 ID=6FF、DLC=4、内容为 CAN_ID 两字节/RID/CC。两者不能拼在一起使用。首版接收器依赖厂家上位机预配置并保存；如后续需要 MCU 写寄存器，先按目标固件确认实际事务格式、应答与保存行为。

CAN 控制应答和类型 01—04 的测量帧分开处理。未知类型、控制应答和格式不符的帧不能刷新测量时间。温度字节的比例、符号语义未充分说明，先不解码成有效摄氏温度。

### 7.5 推荐接入路径与带宽

接口核心完成后，**优先选择 RS485 主动接收作为第一个具体外置后端**，前提是实物连接允许使用 RS485。它复用现有 AsyncUart，字段为 float、主动帧已有上述 CRC 证据。若已确定接独立 CAN，则只实现 CAN 后端；不要首轮同时铺两套总线。最终选型仍以实际接线为准。

建议联调起点为四类全开、500 Hz、RS485 1 Mbit/s。这是工程起始配置，不是手册默认配置。先在厂家上位机设置主动模式、接口、ID、速率及波特率，保存后重新上电核对；更改到 RS485 输出前保留厂家工具恢复配置的方法。

8N1 每字节按 10 bit 估算：

| 全部四类输出频率 | 载荷字节/秒 | 串行线路占用 | 1 Mbit/s 占比 |
|---|---|---|---|
| 100 Hz | 8000 | 80 kbit/s | 8% |
| 500 Hz | 40000 | 400 kbit/s | 40% |
| 1000 Hz | 80000 | 800 kbit/s | 80% |

1000 Hz 在 921600 baud 约占 86.8%，余量有限；需要该速率时可从手册列出的 1.5/2 Mbit/s 等档位选择，并核对主控误差、线缆和实际丢包。不要仅因平均带宽够就忽略接收队列容量与线程调度。RS485 半双工还要计入主动输出之外的发送和转向时间，首版无需边接收边配置。

### 7.6 后端与公共状态核心的连接

~~~text
带原始接收时间的 CAN 帧 / UART 字节块
 → 设备协议：ID、类型、长度、CRC、字节序
 → 单位和坐标归一化、数值检查
 → 本次实际成功字段的 Update
 → 共用 ImuState
 → Snapshot
~~~

通用 ImuSource 不增加 readCan/readRs485，也不为 SPI/CAN/UART 发明统一传输虚基类。

- DM 协议解码器不直接触碰 UART/CAN，不执行加热和应用动作。UART 解析器维护有界半包缓存；CAN 解码器按帧输入。
- 具体 DmImuRs485Source 或 DmImuCanSource 实现 init/service/snapshot，并组合公共 ImuState。构造依赖和线程约定遵循第 4 节。
- UART init 建立接收；CAN init 注册正确过滤器。init 成功只代表接收就绪，不能直接置姿态有效或发送归零命令。
- UART 服务线程有界排空接收队列；CAN 回调只复制帧、原始时间及必要元数据到有界队列，service 在线程中解码。队列溢出计入 transport_gaps。
- 一次有效 accel 帧只发布 Accel；gyro、quat 同理。完整帧时间取最后一段数据的原始接收时间，不取稍后解码时间。分帧不能伪装为硬件同步采样。
- UART 字节缺口丢弃半包并重新找帧；CAN 丢帧保留上次值的原时间。两者都不能因为 transport 恢复而给旧值重新盖章。
- CRC/帧错误累计诊断并继续扫描后续有效输入；service 若本轮无可发布数据返回 -EAGAIN，底层故障返回实际错误。坏帧不使已经提交的正确字段回滚。
- 需要联合使用姿态和角速度时，消费者除各自 freshness 外还检查时间差。参考被明确重置时更新 epoch 并清除旧姿态。
- 手册没有可可靠检测一切远端重启的会话字段。链路中断可以由策略判旧数据失效，但不能据此声称能发现所有设备静默归零；受控重新初始化时明确开启新 epoch。

### 7.7 设备验收仍需确认的内容

已从手册核对：基础能力、串行长度与字段、四条 19 字节帧 CRC、CAN 量化区间与位布局、输出选择限制及配置报文冲突。

尚需目标硬件确认：固件版本和保存配置、实际接入总线/ID/波特率、23 字节四元数 CRC、输出单位、四元数旋转方向、安装方向、CAN 四元数实帧；如启用温度或 MCU 寄存器写入，还需相应协议确认。上述事项不会阻塞公共接口和状态核心的实现，但未完成时不能声称外置源已经实机验收。

## 8. 视觉数据接口

新增 `include/communication/vision/vision_types.hpp`，只依赖共同值类型：

```cpp
#pragma once
#include <core/measurement.hpp>

namespace skywalker::communication::vision {
struct AxisTarget {
    float angle_rad = 0, rate_rad_s = 0, acceleration_rad_s2 = 0;
};
struct AimCommand {
    bool control_requested = false, fire_requested = false;
    core::OrientationReference reference{};
    AxisTarget yaw{}, pitch{};
};
struct Feedback {
    core::OrientationReference reference{};
    core::Measurement<core::Quaternion> orientation{};
    core::Measurement<core::Vec3> gyro_rad_s{};
    core::Measurement<float> bullet_speed_m_s{};
};
struct Snapshot {
    core::Measurement<AimCommand> aim{};
    bool aim_fresh = false;
};
}
```

约定：

- yaw/pitch 是 reference 中的绝对朝向角，采用第 6 节的 Euler 定义；rate/acceleration 为这些角轨迹的导数。
- Feedback 的 gyro 是 B 中的角速度向量，任意姿态下不等于 Euler 角导数。
- active 目标要求有效 reference、有限数值；fire_requested 要求 control_requested。
- 停止请求可以不带有效目标。接受后清除不使用的目标和射击请求，并作为合法新消息发布，覆盖旧 active 命令。
- 未知弹速 valid=false；姿态和 gyro 保留原始时间，不因复制/发包重新盖章。
- 视觉模块表达请求，不解释成电机模式、不选择机械轴、不执行射击。
- 如果新协议最终使用相对误差，应显式更改类型/语义；不能在同一字段中偷偷混用绝对角和偏差角。

### 8.1 协议边界与 VisionLink

`include/communication/vision/vision_protocol.hpp` 包含 types 和 cstddef：

```cpp
namespace skywalker::communication::vision {
class CommandSink {
public:
    virtual ~CommandSink() = default;
    virtual int accept(const AimCommand&, core::TimeUs frame_rx_us) = 0;
};
class VisionProtocol {
public:
    virtual ~VisionProtocol() = default;
    virtual void reset() = 0;
    virtual void discardPartial() = 0;
    virtual int consume(const std::uint8_t*, std::size_t,
                        core::TimeUs rx_us, CommandSink&) = 0;
    virtual int encodeFeedback(const Feedback&, core::TimeUs encode_us,
                               std::uint8_t* dst, std::size_t capacity) = 0;
};
}
```

`include/communication/vision/vision_link.hpp` 包含上述 protocol 头：

```cpp
namespace skywalker::communication::vision {
class VisionLink final : private CommandSink {
public:
    struct Config {
        core::TimeUs aim_timeout_us = 100000;
        core::TimeUs orientation_timeout_us = 20000, gyro_timeout_us = 20000;
        core::TimeUs bullet_speed_timeout_us = 1000000;
    };
    VisionLink(VisionProtocol&, const Config&);
    VisionLink(const VisionLink&) = delete;
    VisionLink& operator=(const VisionLink&) = delete;
    int init();
    int processRxBytes(const std::uint8_t*, std::size_t, core::TimeUs rx_us);
    void discardPartial();
    int setFeedback(const Feedback&);
    int encodeFeedback(std::uint8_t* dst, std::size_t capacity);
    Snapshot snapshot() const;
private:
    int accept(const AimCommand&, core::TimeUs frame_rx_us) override;
    // 实施时补配置、协议引用、值存储、统计和短复制锁。
};
}
```

link 不出现 UART 节点、IMU 指针或机器人对象，不创建线程、不 sleep。protocol 对象必须比 link 存活更久。

| 方法 | 语义 |
|---|---|
| init | 校验 timeout>0、清空快照、reset 协议；0 成功，-EINVAL 配置错误，重复初始化 -EALREADY |
| processRxBytes | 单所有者；返回接受的命令数；nullptr+正长度 -EINVAL，未初始化 -EACCES；坏 CRC 等计数后继续扫描 |
| processRxBytes(nullptr,0,now) | 周期推进半包超时，不刷新已接受命令 |
| discardPartial | 丢未完成字节，保留 latest 的原始时间，用于 overflow/字节缺口 |
| accept | 内部校验并发布；0 成功，非法值 -EINVAL，倒退时间 -ESTALE；成功才增加本地 sequence |
| setFeedback | 一个生产者发布完整副本，可与 RX 线程不同；只校验 valid 字段；成功 0，错误 -EINVAL 且旧值不变；未初始化 -EACCES |
| encodeFeedback | 所有者复制反馈后取 now，编码副本中清除过期 valid；返回字节数，容量不足 -EMSGSIZE，参数错误 -EINVAL，未初始化 -EACCES；失败输出不发送 |
| snapshot | 线程安全复制，复制后取 now 计算 aim_fresh；无数据返回默认无效值，无 I/O |

全部 protocol 调用由同一线程串行执行。setFeedback/snapshot 的短锁只交换值，不在锁内调用 protocol。统计也必须通过副本发布，不能向其他线程暴露 parser 内部引用。

一次 consume 可发布多条完整命令，最终快照保存最后一条。fire_requested 是电平请求；latest 快照不能承诺不丢离散“收到一次开一发”事件。失联只使 aim_fresh=false，不擅自变成某个执行器 Hold/Disable 指令。

实际产品协议已按用户给出的 AB 源码实现；它没有有效位和 reference/session 字段，因此采用本地固定参考并拒绝无有效测量的上行。正式 Feedback 已扩展 mode 和 bullet_count。

### 8.2 保留原施工文档中的传输与解析约束

视觉字节协议确定后，增加独立 transport 端点，把收发结果交给 VisionLink。端点只负责 I/O 调度，不承担 IMU 选择、目标仲裁和机器人动作。

| 可复用的当前实现 | 使用约束 |
|---|---|
| include/communication/async_uart.hpp、lib/communication/async_uart.cpp | 一个线程拥有 init/service/read/send；一个 UART 只由一个对象注册回调；对象及 DMA 存储寿命覆盖回调 |
| AsyncUart::DmaBuffers | 每实例独占、静态分配、按现有要求放入 __nocache；不能放在线程栈或与其他实例共用 |
| AsyncUart::RxChunk | 最多 64 字节、timestamp_ms 是接收时间；队列只有 8 个 chunk，调度必须留出余量 |
| AsyncUart::read | -EAGAIN 表示空；-EOVERFLOW 必须 discardPartial，再从后续字节重新同步 |
| AsyncUart::send | 当前上限 256 字节；-EAGAIN 时跳过本轮或下次发最新反馈，不能无限积压旧姿态 |
| 已有流式协议的处理思路 | 复用有界缓冲、半包超时、重同步和统计；不沿用未经确认的帧结构及 CRC 参数 |

初始化时检查每一步的返回值。AsyncUart 初次启动 RX 失败后，回调可能已注册且对象已初始化；保留对象并按当前 service 重试约定恢复，不能反复创建临时对象注册同一个 UART。设备未就绪、DMA 缺失、发包失败均进入可读取诊断。

解析器必须做到：非法长度在等待整帧前拒绝；缓存容量有上限；错误 CRC/帧尾后向前丢一个字节继续找头，不能把候选帧中潜在的有效头全部跳过；拆包、粘包与噪声混入可恢复。不同具体协议可按其明确格式优化，但不能默认“一次 read 等于一帧”。半包超时由 processRxBytes(nullptr,0,now) 推进；命令超时由 snapshot 取当前时间独立判断。

视觉 CRC 只有在新协议明确参数与已知帧后才能定型。即便最后与现有 referee CRC 相同，也应提取到独立算法单元，不能让关闭 CONFIG_SKYWALKER_REFEREE 后的视觉模块产生链接错误，更不能与第 7 节 DM CRC 混用。

硬件配置不属于公共接口，但真实端点验收前必须满足：

- 明确实际物理接口、电平、引脚、波特率和收发方向；不从现有应用抄取 UART alias。
- 当前 MC02 的 USART2/3 有 RS485 硬件 DE 配置，但尚需配置异步 RX/TX 所需 DMA；UART7 同样不能只改 status/current-speed 就使用异步收发。仅启用接收的端点核对 RX DMA，使用 AsyncUart TX 时还需 TX DMA。
- 按板级和锁定 Zephyr 源码核对 DMA request/channel、SPI DMA 与实际启用设备的冲突；device_is_ready() 不等于异步收发已可用。
- 优先采用已支持的硬件 DE；只在具体硬件确需 GPIO 方向控制时增加该实现。
- 按最终帧长、频率和串口格式算带宽。全双工分 TX/RX 方向计算，半双工合并方向与转向开销。

相关现状可只读参考 boards/damiao/dm_mc02/dm_mc02.dts 和 docs/16-uart-dma-nocache.md。它们是底层能力依据，不把任何现有 sample/application 作为本轮集成入口。

## 9. 两模块如何组合

以下是未来任意组装者的值复制示意，不加入现有应用，不放进 VisionLink：

```cpp
const auto s = source.snapshot();
vision::Feedback f{};
f.reference = s.sample.reference;
f.orientation = s.sample.orientation;
f.gyro_rad_s = s.sample.gyro_rad_s;
f.orientation.stamp.valid &= (s.fresh_mask & imu::Orientation) != 0;
f.gyro_rad_s.stamp.valid &= (s.fresh_mask & imu::Gyro) != 0;
// 未知弹速保持无效，保留测量时间和 sequence。
const int ret = link.setFeedback(f);
```

名称由实际使用方解析到对应 namespace。姿态选择、机械角色、参考变化、时间对齐属于后续组装职责；两个模块公共头不互相包含。

## 10. 文件施工顺序

先连续完成所选阶段的功能，再做一轮整体检查。表中的自检点为静态核对，不要求逐函数增加测试；新文件及其构建入口应一起补齐。

| 步骤 | 文件 | 修改与自检 |
|---|---|---|
| 1 | include/core/measurement.hpp；独立姿态数学头/实现 | 值类型、fresh、旋转语义；不依赖机器人头 |
| 2 | include/drivers/imu/imu_types.hpp、imu.hpp；drivers/imu/imu_state.hpp/.cpp | 实际实现更新/快照；分字段过期、多实例无共享状态 |
| 3 | include/communication/vision/vision_types.hpp、vision_protocol.hpp、vision_link.hpp；lib/communication/vision_link.cpp | 接口、校验、状态、超时、协议委托全部落地；第一阶段至此形成完整核心 |
| 4 | include/control/attitude_ekf.hpp；lib/control/attitude_ekf.cpp | 估计器摆脱 DT/device/static；矩阵存储属于实例；错误与初始化处理 |
| 5 | include/drivers/imu/bmi088_imu.hpp；drivers/imu/bmi088_imu.cpp | 直接 sensor 采样，注入设备；可不配 EKF 而只输出原始测量 |
| 6 | include/drivers/imu/imu_heater.hpp；drivers/imu/imu_heater.cpp | 独立可选温控；初始化不加热，过期温度撤销 PWM |
| 7 | drivers/imu/dm_imu_protocol.hpp/.cpp | 选定总线的实际解码，不接设备；单位、坐标、CRC/位布局校验；见第 7 节 |
| 8 | include/drivers/imu/dm_imu_rs485.hpp；drivers/imu/dm_imu_rs485.cpp，或对应 CAN 文件 | 只实现所选后端；组合协议、ImuState 和传输；有界队列、接收原时间及掉线处理 |
| 9 | drivers/Kconfig、drivers/CMakeLists.txt、drivers/imu/CMakeLists.txt、lib/Kconfig、lib/CMakeLists.txt、lib/control/CMakeLists.txt、lib/communication/CMakeLists.txt | 各模块独立选择，新源文件加入正确条件编译入口 |
| 10 | 旧 include/drivers/imu/imu.h、drivers/imu/imu.c、skywalker,imu binding | 新板载实现完成后移除旧聚合实现及构建入口，不加 wrapper；记录旧入口迁移影响 |

第一阶段不依赖第 4—8 步已经完成，也不由温控或外置总线选型阻塞。删除旧聚合节点不等于删除 BMI08x 底层 sensor 驱动，也不要求顺带删除被其他模块使用的 Kalman 设备。

最新实施范围新增两个 samples，且将旧 imu_test 迁移新 API 以满足仓库构建门禁；不在现有 application 中集成。表内列出的旧聚合实现已移除，没有保留兼容层。

建议 Kconfig 职责：

- SKYWALKER_IMU：依赖 CPP，编译接口与状态核心，不强制 SENSOR/CAN/UART/PWM。
- SKYWALKER_IMU_BMI088：依赖 IMU、SENSOR 与实际 BMI08x 驱动。
- SKYWALKER_ATTITUDE_EKF：依赖 CPP 及实际使用的矩阵/DSP；不依赖 Kalman device 注册。若文件放在 lib/control，须保证该目录随此配置被编译，且不会重复添加。
- SKYWALKER_IMU_HEATER：可选 PWM 和温控算法；不是读取 gyro 的前提。
- SKYWALKER_IMU_DM_RS485 或 SKYWALKER_IMU_DM_CAN：实际后端完成后才增加对应开关，前者要求 UART_TRANSPORT，后者要求 CAN。
- SKYWALKER_VISION：依赖 CPP 与 SKYWALKER_LIB_COMMUNICATION；核心不要求 UART，独立 UART 端点才要求 UART_TRANSPORT。

上述是拟新增名字，旧 SKYWALKER_DRIVER_IMU 不映射到新名字。当前通信库的 REMOTE_DR16、REFEREE、INTERBOARD 有 default y；独立验收配置显式关闭无关项，并验证没有隐藏符号依赖。未实现的 backend 不提供看似可用的空开关；共同数学实现只保留一个编译入口。

本轮已新增 AbProtocol 和 VisionReceiver，并按用户选定的协议实现和验证；它们与 VisionLink 核心分开编译。

## 11. 独立验收与上电顺序

### 11.1 功能完成后的一轮检查

准备新的模块级 fixture（可放在 /tmp），链接新模块，不包含现有应用头。整体验证覆盖：

1. 两个 IMU 状态实例输入不同数据，时间、姿态和诊断互不干扰。
2. 只继续更新 accel 时旧 gyro/quat 仍过期；温度错误不刷新其他字段；合法外置姿态质量为 Unknown。
3. 单轴旋转、安装变换、范数错误、倒退时间和参考重置；默认单位四元数不算有效姿态。
4. 合成视觉 active 目标后接收停止请求，停止消息能覆盖旧 active；停止来包后 snapshot 独立判断过期。
5. IMU 值复制进入 Feedback 后仍保留测量时间；编码清除过期 valid；受控协议覆盖拆包、粘包、坏输入、容量不足及 overflow 后恢复。

选择实现 DM 后端时，将第 7.3 节已知帧加入同一模块流程并对照期望 CRC；追加目标设备取得的 23 字节四元数帧。错误 CRC 不得发布测量，串行坏帧后仍能找到下一帧。若选择 CAN，追加真实 DLC/ID/类型过滤、映射与四元数方向对照；合成位边界不能替代实帧验证。

这里只需要覆盖主流程，不为每个移位、分支单独搭测试工程。受控视觉协议和合成 IMU 更新只能证明接口核心，不能冒充真实设备接入。

初始规划曾建议如下临时 fixture。现已提供 tests/vision_imu 及两个真实 sample；本轮实际执行的验证见文首及正式样例说明：

~~~bash
west build -b native_sim/native/64 /tmp/skywalker-vision-imu-fixture -d /tmp/skywalker-vision-imu-build
/tmp/skywalker-vision-imu-build/zephyr/zephyr.exe

west build -b dm_mc02/stm32h723xx /tmp/skywalker-vision-imu-hardware-fixture -d /tmp/skywalker-vision-imu-hardware-build
~~~

第二个 fixture 自己提供硬件节点、DMA 和配置，用于 BMI088、ARM DSP、PWM 及选定 DM 后端的真实组合。host 核心通过不能替代这些平台依赖的构建与运行验收。检查实际生成的配置确保未拉入现有机器人应用或无关设备。

### 11.2 外置 IMU 上电验收

1. 使用厂家上位机读取型号/固件和当前配置，设置所选总线的主动模式、ID、波特率和频率，保存并重新上电核对。手册的配置命令有设置模式要求；本阶段无需主控发送写寄存器命令。
2. 断电检查供电和物理接线。P3 给出的输入范围为 5—28V；RS485 接对应收发器接口、共地并核对 A/B，CAN 核对位速率和端接。DMA/DE 由独立 fixture 配置，不连接电机。
3. 主控先只接收并统计类型、长度、CRC/帧错误、队列缺口；保留原始十六进制报文和接收时间。数量与输出频率应相符，不能只看“有字节”就判成功。
4. 静止检查加速度模长接近当地重力、角速度接近零、四元数范数接近 1；绕每个轴按已知正方向转动约 90°，核对单位、姿态变化及第 6 节安装变换。静止与单次旋转只能验证基础行为，不能量化全部动态性能。
5. 在输入回放中只停止四元数、继续 accel/gyro，验证姿态独立过期；再实物断开、重连，检查旧值没有重新盖章，恢复后只有新报文对应字段重新有效。对受控重启/校准更新 reference.epoch。
6. 记录实际可持续频率、丢包数和最长接收间隔，再确定 freshness 与调度周期。默认 20ms 是起始值，不是经过硬件测定的限值。

板载 BMI088 另按“raw 采样→本地 EKF→可选温控”的顺序上电；温控单独使能并验证温度失效撤销 PWM，不能用外置模块内部加热替代这一验收。

### 11.3 常见故障定位

| 现象 | 优先检查 |
|---|---|
| 设备 ready，但 AsyncUart init 返回 -ENODEV | UART 对应的 RX DMA 是否实际配置；发送路径另查 TX DMA |
| 能收到字节，CRC 全错 | 主动/应答模式是否一致、长度和 ID、CRC 是否误改成左移 8 位、是否把四类合成一帧 |
| 高速时偶发不完整帧 | transport_gaps、8 个 chunk 队列、线程服务预算、DMA cache 属性、真实线速与线缆 |
| 四元数数值有限但方向错误 | wxyz 顺序、设备四元数方向、安装旋转、CAN W 位提取；不能靠归一化修复坐标错误 |
| 设备断开后姿态一直 fresh | 是否把处理时刻当接收时刻、是否用 accel 更新 quat 时间、snapshot 是否使用当前时钟 |
| CAN 能读数据但配置请求无回应 | 正文与附录请求格式冲突、目标固件、CAN_ID/MST_ID；回到厂家工具核对，不循环尝试写入 |
| 关闭裁判或旧 IMU 后链接失败 | 是否仍从 referee.cpp 引 CRC、遗漏新的 CMake 条件入口或残留旧聚合符号 |

## 12. 完成判据与剩余输入

- [ ] 公共头独立，不引用电机、robotics command 或应用配置。
- [ ] 实现校验、发布、快照和过期核心，而非只有接口声明。
- [ ] BMI088 采样、EKF、温控解耦，运行状态均属于实例。
- [ ] 内外置源遵循共同数据语义，字段分别计时，质量未知不伪装为已收敛。
- [ ] DM 选定后端按目标固件和真实帧验收；19 字节 CRC 已知证据、23 字节待核验边界明确。
- [ ] 视觉以值接收姿态，不固定 IMU 类型或 base/head 角色。
- [ ] 无旧 API 兼容层，无 Auto、安全仲裁、执行器逻辑混入模块。
- [ ] 区分接口核心、真实视觉线协议和硬件后端三种完成状态。

视觉帧已确认并实现，外置已选 RS485-2；剩余待实物确认项为 DM 目标固件、四元数完整实帧、单位与安装方向，以及真实温控、SPI/UART 链路和可持续采样频率。公共接口和状态核心已完成，不能把构建及模拟验收等同于实机验收。


