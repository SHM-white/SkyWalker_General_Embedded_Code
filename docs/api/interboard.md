# 双主控板间通信：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

统一 v4 字节契约，七类消息、boot/生产序号与原年龄，UART/RS485/CAN 三后端。

**接入状态：已有代码。** v4 已实现，旧版本直接拒绝；恢复授权 generation 删除。真实两板应使用同版固件，整车默认 USART1 UART。

## 职责与关联

Transport 管物理批次；Endpoint 唯一通信线程推进收发，其他线程复制输入/状态，不根据电机 ready 决定目标发送。

输入 / 依赖：[异步 UART 与 DMA](uart.md)、[板级：MC02 / RoboMaster Type-C](boards.md)

消费者：[双主控应用与执行器](application.md)、[舵轮底盘与功率缩放](chassis.md)、[大 Yaw 回中与独立速度环](big-yaw.md)

## 接口契约

### 1. ConfiguredInterBoardTransport(const Config &, AsyncUart::DmaBuffers *dma = nullptr)

```cpp
ConfiguredInterBoardTransport(const Config &, AsyncUart::DmaBuffers *dma = nullptr)
```

一次选择 UART/RS485/CAN；对象和所选设备独占，纯 CAN 不需要 DMA。

**返回 / 输出：** 构造不启动，后续 service/poll 推进。

**线程 / 时序：** 唯一通信线程使用，static 存活。

**错误 / 边界：** 后端未编译/电气配置不支持 -ENOTSUP。

### 2. InterBoardEndpoint(InterBoardTransport &, const Config &); void poll(uint64_t now_ms)

```cpp
InterBoardEndpoint(InterBoardTransport &, const Config &); void poll(uint64_t now_ms)
```

唯一通信 owner 约 1 ms 推进；命令/心跳/status 默认 100 ms，TX 默认 60 ms。

**返回 / 输出：** void；服务、解析和在线诊断由 snapshot 返回。

**线程 / 时序：** poll 仅一个通信线程，其他 API 交换短锁值副本。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

### 3. void submit(const ChassisCommand &); void setReferee(const RefereeState &); void setStatus(const RunStatus &); void submitBigYaw(const BigYawRequest &); void setBigYawFeedback(const BigYawFeedback &)

```cpp
void submit(const ChassisCommand &); void setReferee(const RefereeState &); void setStatus(const RunStatus &); void submitBigYaw(const BigYawRequest &); void setBigYawFeedback(const BigYawFeedback &)
```

复制原生产值与年龄；反复提交/重发同 producer 不续期，状态 stamp 由执行 owner 生产。

**返回 / 输出：** void；发送机会由 poll 安排，接受不代表远端执行。

**线程 / 时序：** 应用线程可发布值副本，不能修改源 stamp。

**错误 / 边界：** 目标不会等待 ready/armed；真实 boot/link、原输入和数值仍需有效。

### 4. int submitOperatorControl(const OperatorControl &); Snapshot snapshot() const

```cpp
int submitOperatorControl(const OperatorControl &); Snapshot snapshot() const
```

独立操作管理传 run_allowed/estop/clear event 与 boot/原年龄；snapshot 含七类消息、online/error/transport/parser_stats。

**返回 / 输出：** submit 0=复制；snapshot 为值副本。

**线程 / 时序：** 线程上下文，短锁交换。

**错误 / 边界：** 非云台 producer -EACCES，矛盾运行/停止/清除请求 -EINVAL；重复事件不改绑新 boot。

### 5. int InterBoardTransport::service(uint64_t); int read(RxChunk &); int send(const uint8_t *, size_t, uint32_t timeout_ms = 60)

```cpp
int InterBoardTransport::service(uint64_t); int read(RxChunk &); int send(const uint8_t *, size_t, uint32_t timeout_ms = 60)
```

批次完整复制，背压不积压运动目标；原始接收时刻随 chunk 保留。

**返回 / 输出：** 0 成功；无数据/背压 -EAGAIN，断流 -EOVERFLOW。

**线程 / 时序：** 唯一通信 owner，不在电机执行 ISR 调用。

**错误 / 边界：** 非法 -EINVAL，过大 -EMSGSIZE，未就绪 -EACCES；溢出丢半帧。

## 调用示例

### 独立通信与生产者

```cpp
// 执行线程：RunStatus.stamp 来自本次实际 update。
endpoint.setStatus(status);
endpoint.submitBigYaw(fresh_request);
// 通信线程：
endpoint.poll(k_uptime_get());
const auto rx = endpoint.snapshot();
// rx.online 只代表心跳，原输入和状态年龄分别检查。
```

应用周期片段，构造与实物配置以链接源码为准。

## 调用顺序

1. 两端同版 v4，选择同后端并绑定独占资源。
2. 静态构造 Transport/Endpoint 与 DMA，持续 poll。
3. 执行/管理 owner 发布保留源年龄的目标与状态。
4. 按真实 boot/link 更新上下文；Motor 恢复不改变输入身份。
5. 状态停止生产后撤销旧 ready/armed，通信心跳可继续。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `kInterBoardProtocolVersion` | 4；最大 payload 128、frame 142 B，CRC16、小端；七类消息。 |
| `状态年龄` | RunStatus/BigYawFeedback 原始生产年龄；反复发送不续期。 |

- v4 没有 resume_generation；不要用旧恢复教程。
- frame sequence 是发送次数，不是 producer sequence。
- RS485 主动 IMU 协议不是板间轮询封装。
- 底盘电机占满 CAN1/2/3，默认板间 UART；不能共享控制器给 CAN 后端。

## 正文与源码

- [communication.md](../modules/communication/communication.md)
- [interboard-transports.md](../modules/communication/interboard-transports.md)

- [interboard_protocol.hpp](../../include/communication/interboard/interboard_protocol.hpp)
- [interboard_endpoint.hpp](../../include/communication/interboard/interboard_endpoint.hpp)
- [interboard.hpp](../../include/robotics/messages/interboard.hpp)
- [interboard_endpoint.cpp](../../lib/communication/interboard_endpoint.cpp)
- [interboard_codec.cpp](../../lib/communication/interboard_codec.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
