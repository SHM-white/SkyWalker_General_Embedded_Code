# 异步 UART 与 DMA：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

把 UART 回调产生的字节复制进有界队列，业务线程按接收时刻解析；同一设备只有一个所有者。

**接入状态：已有代码。** 已用于遥控、裁判、视觉和板间 UART/RS485；实际设备、DMA 通道与 nocache 区域仍由板级配置确定。

## 职责与关联

AsyncUart 负责双 RX 缓冲、整批 TX 复制、断流代次与重试。它不识别 DR16、裁判、视觉或板间消息，也不判断业务在线。

输入 / 依赖：[板级：MC02 / RoboMaster Type-C](boards.md)

消费者：[IMU：独立采集、姿态与加热](imu.md)、[DR16 遥控输入](remote.md)、[裁判许可与功率预算](referee.md)、[双主控板间通信](interboard.md)、[视觉链路与 AB 协议](vision.md)

## 接口契约

### 1. AsyncUart(const device *uart, DmaBuffers &buffers)

```cpp
AsyncUart(const device *uart, DmaBuffers &buffers)
```

绑定独占 UART 与外部 DMA 存储；构造不注册回调。DmaBuffers 含两个 128 字节 RX 缓冲和一个 256 字节 TX 缓冲，均按 32 字节对齐。

| 参数 | 含义与边界 |
| --- | --- |
| `uart` | Zephyr UART 设备，支持 async API，且没有被 console、VOFA 或其他接收器占用 |
| `buffers` | 实例专属、静态存活的 __nocache DmaBuffers；只有缓冲区放 nocache |

**返回 / 输出：** 构造对象；不可复制。

**线程 / 时序：** 构造后由一个通信线程拥有 init/service/read/send；对象和缓冲区持续存活至重启。

**错误 / 边界：** 构造不检查设备；实际错误由 init 返回。

### 2. int AsyncUart::init()

```cpp
int AsyncUart::init()
```

注册 UART 回调并开启 RX；只成功注册一次。回调只复制字节、维护缓冲与完成状态。

**返回 / 输出：** 0：RX 启动成功；负 errno：初始化或驱动错误。

**线程 / 时序：** 唯一通信线程调用；不是 ISR 接口。

**错误 / 边界：** -ENODEV：空设备或设备未就绪；-EALREADY：回调已注册；其他值来自 uart_callback_set/uart_rx_enable。回调已注册但 RX 启动失败后，继续 service，不再次 init。

### 3. int AsyncUart::service(std::uint64_t now_ms)

```cpp
int AsyncUart::service(std::uint64_t now_ms)
```

持续推进接收重启与发送超时中止。即使没有字节或发生错误也要周期调用；RX 重试间隔为 100 ms。

| 参数 | 含义与边界 |
| --- | --- |
| `now_ms` | 本机单调时间，单位 ms，例如 k_uptime_get() |

**返回 / 输出：** 0：当前 RX 可服务；-EAGAIN：等待重试；负值：错误。

**线程 / 时序：** 唯一通信线程调用，不能阻塞其他实时控制线程。

**错误 / 边界：** -EACCES：尚未 init；其他值来自重新开启 RX。TX busy 仅在完成/中止回调到达后释放。

### 4. int AsyncUart::read(RxChunk &out)

```cpp
int AsyncUart::read(RxChunk &out)
```

非阻塞复制最多 64 字节及原始接收时间；队列深度为 8。断流或溢出时先通知调用者，淘汰旧代次字节。

| 参数 | 含义与边界 |
| --- | --- |
| `out` | 成功时写入 bytes、size、timestamp_ms 和 generation |

**返回 / 输出：** 0：取到一个块；-EAGAIN：队列空；-EOVERFLOW：接收代次变化。

**线程 / 时序：** 与 service 同一个通信线程；按块原始 timestamp_ms 交给解析器。

**错误 / 边界：** -EACCES：未初始化。-EOVERFLOW 后必须丢弃上层 parser 半帧；不能把溢出前后字节拼成一帧。

### 5. int AsyncUart::send(const std::uint8_t *bytes, std::size_t size, std::uint32_t timeout_ms = 20)

```cpp
int AsyncUart::send(const std::uint8_t *bytes, std::size_t size, std::uint32_t timeout_ms = 20)
```

先复制整个批次到实例 TX 缓冲，再启动异步发送；一次只允许一个批次。返回成功后调用者可以复用原始 bytes。

| 参数 | 含义与边界 |
| --- | --- |
| `bytes` | 有效字节数组 |
| `size` | 1～256 字节 |
| `timeout_ms` | 整批发送期限，1～1000 ms，默认 20 ms |

**返回 / 输出：** 0：驱动接受发送；不保证远端接收或执行。

**线程 / 时序：** 唯一通信线程调用；不要在 busy 时积压运动命令。

**错误 / 边界：** -EINVAL：未初始化、空指针、长度或期限非法；-EAGAIN：TX 忙；其他负值来自 UART 驱动。

### 6. bool AsyncUart::txBusy() const; int AsyncUart::txError() const; atomic_val_t AsyncUart::droppedChunks() const

```cpp
bool AsyncUart::txBusy() const; int AsyncUart::txError() const; atomic_val_t AsyncUart::droppedChunks() const
```

读取发送占用、最近发送结果与累计丢块数，供通信诊断。

**返回 / 输出：** busy 为真表示 DMA 仍占有 TX；txError 在 busy 变 false 后稳定；droppedChunks 返回累计计数。

**线程 / 时序：** 原子读；I/O 的生命周期仍由唯一通信线程推进。

**错误 / 边界：** 发送中止时 txError=-ECANCELED；初始化/发送成功不等于业务在线。

## 调用示例

### 线程中读取 DR16 字节并正确处理断流

```cpp
#include <communication/async_uart.hpp>
#include <communication/remote/remote_service.hpp>
#include "board_config.hpp"
using namespace skywalker::communication;

static AsyncUart::DmaBuffers dma __nocache;
static AsyncUart uart(board_config::remote_uart, dma);
static RemoteService parser({}, {});

// 此函数只由一个常驻通信线程调用。
void communicationTask() {
    int ret = uart.init();
    for (;;) {
        const auto now = static_cast<std::uint64_t>(k_uptime_get());
        ret = uart.service(now);
        if (ret == -EACCES) ret = uart.init();
        if (ret == 0) {
            AsyncUart::RxChunk chunk{};
            for (unsigned budget = 0; budget < 8; ++budget) {
                const int rr = uart.read(chunk);
                if (rr == -EOVERFLOW) { parser.discardPartial(); continue; }
                if (rr != 0) break;
                parser.processBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
            }
        }
        parser.processBytes(nullptr, 0, now);
        k_sleep(K_MSEC(1));
    }
}
```

board_config::remote_uart 可参考 DR16 样例。上车通常直接使用 RemoteReceiver，不额外创建第二个 AsyncUart。错误日志与跨线程快照可按应用补齐。

## 调用顺序

1. 静态分配设备所有者及实例专属 __nocache DMA 缓冲。
2. 通信线程 init 一次，然后持续 service → read → parser；send 与这些操作由同一个线程调用。
3. 遇到 -EOVERFLOW 立即 discardPartial；即使没数据，也用零长度输入推进解析超时。
4. 不销毁、不移动对象；没有 stop 或热替换设备接口。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `CONFIG_SKYWALKER_LIB_COMMUNICATION / CONFIG_SKYWALKER_UART_TRANSPORT` | 启用通信库及 UART async transport；后者选择 UART_ASYNC_API。 |
| `设备树与 DMA` | 配置 UART 引脚、波特率、独占 DMA 通道和 nocache 区域；遥控、视觉和板间串口各自独立。 |
| `时间单位` | AsyncUart 时间是 ms；视觉 core::TimeUs 是 μs，适配时乘 1000。 |

- 同一 UART 不能同时交给 AsyncUart 与 VOFA/console/shell。
- 复制字节和快照时保留原始接收时间；读取时间不能冒充采样时间。
- DCACHE 开启时需要 CONFIG_NOCACHE_MEMORY；不能仅靠 alignas 解决 DMA 缓存一致性。
- TX 超时发起 abort 后仍须等待回调；不要复写还被 DMA 占用的缓冲。

## 正文与源码

- [UART DMA 配置](../guides/uart-dma.md)
- [通信模块](../modules/communication/communication.md)

- [AsyncUart 公共接口](../../include/communication/async_uart.hpp)
- [UART/DMA 实现](../../lib/communication/async_uart.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
