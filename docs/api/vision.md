# 视觉链路与 AB 协议：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

接收独立的 yaw/pitch 目标请求，提供带参考系与有效期的值；反馈由应用复制姿态等实测数据。

**接入状态：已有代码。** AB 接收已接入整车可选观察/执行阶段；自动执行要求头部参考匹配。弹速/弹数来源与显式视觉会话待补，反馈 TX 默认关闭。

## 职责与关联

AbProtocol 定义 29 字节下行/43 字节上行线协议；VisionLink 校验参考、时间与反馈；VisionReceiver 驱动独占 UART。模块不持有 IMU，不仲裁机器人命令，不操作电机和发射机构。

输入 / 依赖：[异步 UART 与 DMA](uart.md)

消费者：[来源适配与手动映射](command-sources.md)

## 接口契约

### 1. VisionReceiver(const device *uart, AsyncUart::DmaBuffers &dma, VisionProtocol &protocol, const Config &config); int VisionReceiver::start()

```cpp
VisionReceiver(const device *uart, AsyncUart::DmaBuffers &dma, VisionProtocol &protocol, const Config &config); int VisionReceiver::start()
```

绑定 UART、DMA 与长生命周期 codec；start 初始化 Link 并启动独立 worker。反馈周期默认 0，仅收包。

| 参数 | 含义与边界 |
| --- | --- |
| `uart` | 专属视觉串口 |
| `dma` | 专属静态 __nocache 缓冲 |
| `protocol` | AbProtocol 或其他实现 VisionProtocol 的对象，必须持续存活 |
| `config` | Link 的超时配置与 feedback_period_us |

**返回 / 输出：** start：0 表示 worker 创建；UART 就绪由 Snapshot::state 与 uart_error 表示。

**线程 / 时序：** 线程上下文，启动一次，无停止/销毁后重建 API；交给 VisionSource 时由 manager 启动。

**错误 / 边界：** -EALREADY：已启动；其他值为 Link/codec 的配置错误。线程创建成功也可能随后 InitFailed 并重试 UART。

### 2. VisionReceiver::Snapshot VisionReceiver::snapshot() const

```cpp
VisionReceiver::Snapshot VisionReceiver::snapshot() const
```

读取命令、协议统计和 RX/TX 诊断；在读者侧按当前时间重新计算 aim_fresh。

**返回 / 输出：** 值副本：link.aim、link.aim_fresh、state、uart_error、dropped、resets、tx_frames、tx_skipped、tx_error。

**线程 / 时序：** 多个线程可读，内部短锁；读取不产生新命令、不刷新时间。

**错误 / 边界：** 必须同时检查 aim_fresh、aim.stamp.valid 与 aim.value.control_requested；合法 stop 可能是新鲜消息，但不能控制电机。

### 3. int VisionReceiver::setFeedback(const Feedback &feedback); int VisionLink::setFeedback(const Feedback &feedback)

```cpp
int VisionReceiver::setFeedback(const Feedback &feedback); int VisionLink::setFeedback(const Feedback &feedback)
```

复制应用提供的反馈，保留姿态、gyro、弹速、计数各自的原始时间；不填造任何缺失测量。

| 参数 | 含义与边界 |
| --- | --- |
| `feedback` | mode、reference、Hamilton wxyz 四元数（B→W）、body-frame gyro(rad/s)、弹速(m/s)、累计计数，均带各自 stamp |

**返回 / 输出：** 0：保存副本；负 errno：拒绝。

**线程 / 时序：** Link 初始化后其他线程可设置；实际编码与 UART 发送由唯一 worker 进行。

**错误 / 边界：** -EACCES：Link 尚未初始化；-EINVAL：未知 mode、未来时间、非法参考/非有限值、无效四元数或负弹速。

### 4. int VisionLink::init(); int VisionLink::processRxBytes(const std::uint8_t *bytes, std::size_t size, core::TimeUs rx_us); void VisionLink::discardPartial()

```cpp
int VisionLink::init(); int VisionLink::processRxBytes(const std::uint8_t *bytes, std::size_t size, core::TimeUs rx_us); void VisionLink::discardPartial()
```

不使用 Receiver 时的纯链路入口；init 校验配置并清状态，processRxBytes 流式解析，零长度推进半帧超时，断流时 discardPartial。

| 参数 | 含义与边界 |
| --- | --- |
| `bytes / size` | 原始字节流，nullptr 只允许 size=0 |
| `rx_us` | 接收时间 μs；AsyncUart timestamp_ms 乘 1000 |

**返回 / 输出：** 0 或负 errno；命令更新为新的 Measurement<AimCommand> 本地序号。

**线程 / 时序：** processRxBytes/discardPartial/encodeFeedback 只允许一个 owner；snapshot/setFeedback 可以由其他线程调用。

**错误 / 边界：** -EALREADY：重复 init；-EACCES：未 init；-EINVAL：配置、空指针、未来时间或目标非法；-ESTALE：接收时间回退。

### 5. int VisionLink::encodeFeedback(std::uint8_t *out, std::size_t capacity)

```cpp
int VisionLink::encodeFeedback(std::uint8_t *out, std::size_t capacity)
```

在当前时刻判断各测量有效期和姿态/gyro 时间差，再交给 codec 编码；AB 输出完整 43 字节。

| 参数 | 含义与边界 |
| --- | --- |
| `out` | 输出字节数组 |
| `capacity` | AB 至少 43 字节 |

**返回 / 输出：** 正数：编码字节数；负 errno：此次不能发送。

**线程 / 时序：** 唯一协议 owner 编码；worker 忙时跳过本周期，下一周期取最新反馈。

**错误 / 边界：** -EACCES：未初始化；-EINVAL：空输出或值非法；-EMSGSIZE：容量不足；-ENODATA：必要字段无效/过期；-ESTALE：参考不一致或姿态/gyro 不同步；-ERANGE：ZYX pitch 接近 ±π/2。

### 6. AbProtocol(const Config &config); int AbProtocol::validateConfig() const; int AbProtocol::consume(const std::uint8_t *bytes, std::size_t size, core::TimeUs rx_us, CommandSink &sink)

```cpp
AbProtocol(const Config &config); int AbProtocol::validateConfig() const; int AbProtocol::consume(const std::uint8_t *bytes, std::size_t size, core::TimeUs rx_us, CommandSink &sink)
```

AB 下行 mode=0 停止、1 控制、2 控制并请求开火；yaw/pitch 角度 rad、速度 rad/s、加速度 rad/s²，float32 小端，帧经 CRC 校验后调用 sink。

| 参数 | 含义与边界 |
| --- | --- |
| `config.command_reference` | 固定本地约定 {frame_id, epoch}，默认 {1,1}；AB 线上没有此字段 |
| `config.assembly_timeout_us` | 半帧装配超时，默认 20000 μs |
| `sink` | 完整目标请求的接受者，VisionLink 内部实现 |

**返回 / 输出：** 0 或负 errno；statistics() 给出 frames/crc_errors/invalid_frames/assembly_timeouts。

**线程 / 时序：** 单 owner 的有状态流式 codec；statistics 不跨线程直接读。

**错误 / 边界：** 未知 mode/非有限 active 目标/坏 CRC 会拒绝；AB 没有会话号，不能把本地序号当远端采样序号。

## 调用示例

### 独立接收视觉请求并正确识别停止

```cpp
#include <communication/vision/vision_receiver.hpp>
#include <communication/vision/ab_protocol.hpp>
#include "board_config.hpp"
namespace vision = skywalker::communication::vision;
static skywalker::communication::AsyncUart::DmaBuffers dma __nocache;
static vision::AbProtocol protocol({.command_reference = {1, 1}});
static vision::VisionReceiver receiver(bench::vision_uart, dma, protocol, {});

int main() {
    const int ret = receiver.start();
    if (ret < 0) return ret;
    for (;;) {
        const auto frame = receiver.snapshot();
        const bool requested = frame.link.aim_fresh &&
                               frame.link.aim.value.control_requested;
        if (requested) {
            const float yaw_rad = frame.link.aim.value.yaw.angle_rad;
            // 目标先进入 VisionSource/CommandArbiter；这里不驱动电机。
            (void)yaw_rad;
        }
        k_sleep(K_MSEC(10));
    }
}
```

bench::vision_uart 来自 samples/communication/vision/src/board_config.hpp。这是 RX-only 示例；上行需设 feedback_period_us>0 并持续 setFeedback 传真实数据。

### 应用把实测反馈送给 receiver

```cpp
// 参数必须是保存原始时间的实测值；由应用/IMU owner 提供。
int publishVisionFeedback(vision::VisionReceiver &receiver,
                          const vision::Feedback &measurements) {
    return receiver.setFeedback(measurements);
}
// measurements.orientation 与 gyro_rad_s 的 stamp 不改成发送时刻。
// 无效/过期字段保留无效状态，AB 编码器会拒绝不完整反馈。
```

正式架构需由应用桥接 IMU → vision::Feedback，接收器本身不拥有 IMU。

## 调用顺序

1. 静态构造 codec、DMA 和 Receiver，并统一应用/视觉的参考系约定。
2. start 初始化 Link 后启动 UART worker；通过快照区分串口状态和请求新鲜度。
3. VisionSource 把 Measurement 原样送仲裁，不更改时间；mode=0 清空旧目标。
4. 需要回传时，应用复制 IMU 和发射机构实测值，设 feedback_period_us 并调用 setFeedback。
5. 执行层仍检查本地位置参考与授权；fire_requested 只是电平请求。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `CONFIG_SKYWALKER_VISION / VISION_AB / VISION_RECEIVER` | 分别启用独立 Link、AB codec、UART worker；与 REFEREE 可独立编译。 |
| `VisionLink::Config` | aim=100000 μs；orientation/gyro=20000 μs；bullet_speed/count=1000000 μs；max_feedback_skew=20000 μs。 |
| `VisionReceiver::Config::feedback_period_us` | 默认 0，只接收；正值启用周期回传，TX 忙时跳过。 |
| `CONFIG_SKYWALKER_VISION_RX_STACK_SIZE / PRIORITY` | 默认栈 4096 字节、优先级 6；AB 参考样例使用 115200、8N1。 |

- aim_fresh 只说明消息年龄；停止帧也可以 fresh，必须同时检查 control_requested。
- AB 线上不携带 frame_id、epoch、valid bits 或远端 sequence；参考一致性是双方明确约定。
- body-frame gyro.z/gyro.y 在任意姿态下不等于 Euler yaw_vel/pitch_vel；codec 负责转换。
- 没有反馈测量时不能发送伪造弹速或姿态；默认 feedback_period_us=0。
- 连续快照可能读到同一 fire_requested，不能把每次读取当成一次开火事件。

## 正文与源码

- [AB 字节布局与坐标约定](../modules/communication/vision.md)
- [视觉台架样例](../../samples/communication/vision/README.md)
- [视觉/IMU 流程场景](../../tests/vision_imu/README.md)

- [VisionReceiver](../../include/communication/vision/vision_receiver.hpp)
- [VisionLink](../../include/communication/vision/vision_link.hpp)
- [AB codec](../../include/communication/vision/ab_protocol.hpp)
- [目标与反馈值类型](../../include/communication/vision/vision_types.hpp)
- [链路校验实现](../../lib/communication/vision_link.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
