# DR16 遥控输入：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

从 18 字节 DR16 帧发布摇杆、开关、鼠标和键盘快照；脱机时保持数据但撤销 online。

**接入状态：已有代码。** DR16 receiver、独立样例和命令来源适配器已实现；sentry_gimbal 已接入 RemoteSource，硬件接线门禁仍需配置。

## 职责与关联

Dr16Decoder 解码单帧，RemoteService 负责流式对齐和超时，RemoteReceiver 拥有 UART worker。遥控输入如何变成运动模式由命令层决定。

输入 / 依赖：[异步 UART 与 DMA](uart.md)

消费者：[来源适配与手动映射](command-sources.md)

## 接口契约

### 1. RemoteReceiver(const device *uart, AsyncUart::DmaBuffers &dma, const Config &config)

```cpp
RemoteReceiver(const device *uart, AsyncUart::DmaBuffers &dma, const Config &config)
```

创建独占 UART 的遥控接收器，构造不启动 I/O。Config 包含 decoder 与 remote 两组配置。

| 参数 | 含义与边界 |
| --- | --- |
| `uart` | DR16 所接 UART 设备 |
| `dma` | 专属、静态 __nocache 缓冲 |
| `config` | 通道合法范围、中心死区、帧间间隔与离线超时 |

**返回 / 输出：** 不可复制的接收器。

**线程 / 时序：** 接收器及 DMA 需持续存活至重启，包括 start 出错以后。

**错误 / 边界：** 配置由 start 校验，设备初始化结果由 Snapshot 报告。

### 2. int RemoteReceiver::start()

```cpp
int RemoteReceiver::start()
```

创建后台线程。0 只表示调度成功，不能据此认定遥控器已在线。

**返回 / 输出：** 0：worker 已调度；负 errno：启动失败。

**线程 / 时序：** 线程上下文，只启动一次；若交给 RemoteSource，则由 CommandManager 启动，不要预先手动 start。

**错误 / 边界：** -EINVAL：通道范围/死区/超时配置非法；-EALREADY：重复启动。UART 错误异步记录在 snapshot.state/uart_error。

### 3. int RemoteReceiver::snapshot(Snapshot &out)

```cpp
int RemoteReceiver::snapshot(Snapshot &out)
```

复制完整遥控与诊断信息，并按当前时刻重新判定 online。读者需要保留自己的已初始化 out。

| 参数 | 含义与边界 |
| --- | --- |
| `out` | 读者独有的 Snapshot{}；包含 remote、state、rx_chunks、resets、dropped、uart_error |

**返回 / 输出：** 0：复制成功，但可能离线；-EAGAIN：暂时取不到锁，保留原值并仍使旧 online 过期。

**线程 / 时序：** 线程上下文，允许多个读者；不是消费队列。

**错误 / 边界：** 不得把 -EAGAIN 当作一条新帧；检查 out.remote.online 和原始 stamp，不只检查返回值。

### 4. int RemoteService::processBytes(const std::uint8_t *bytes, std::size_t size, std::uint64_t now_ms); int RemoteService::snapshot(std::uint64_t now_ms, robotics::RemoteState &out) const

```cpp
int RemoteService::processBytes(const std::uint8_t *bytes, std::size_t size, std::uint64_t now_ms); int RemoteService::snapshot(std::uint64_t now_ms, robotics::RemoteState &out) const
```

自行管理通信线程时使用：流式累积 18 字节，坏帧滑动一个字节重同步；snapshot 复制最近合法帧并按 timeout 判断在线。

| 参数 | 含义与边界 |
| --- | --- |
| `bytes / size` | 原始字节；nullptr 与 size=0 用于推进半帧超时 |
| `now_ms` | 处理字节时传原始接收时间；读快照时传当前单调 ms |
| `out` | 遥控状态输出 |

**返回 / 输出：** processBytes：0 或 -EINVAL；snapshot：0 在线、-EAGAIN 尚无合法帧、-ESTALE 已过期（仍复制 out）。

**线程 / 时序：** 同一个 owner 线程调用；跨线程前另行发布值副本。

**错误 / 边界：** 非空长度配空指针返回 -EINVAL；UART 溢出后调用 discardPartial()。

### 5. int Dr16Decoder::decodeFrame(const std::uint8_t *bytes, std::size_t size, std::uint64_t timestamp_ms, robotics::RemoteState &out); int Dr16Decoder::reset()

```cpp
int Dr16Decoder::decodeFrame(const std::uint8_t *bytes, std::size_t size, std::uint64_t timestamp_ms, robotics::RemoteState &out); int Dr16Decoder::reset()
```

解码固定 18 字节帧，通道减去中心并应用死区，生成单调递增本地序号；reset 清零统计与序号。

| 参数 | 含义与边界 |
| --- | --- |
| `bytes / size` | 完整 DR16 帧，size 必须等于 18 |
| `timestamp_ms` | 接收时刻 ms |
| `out` | 成功时写入 RemoteState，鼠标按有符号 16 位解析 |

**返回 / 输出：** 0：合法；负 errno：拒绝且不替换 out。

**线程 / 时序：** 解码器单线程所有；不直接接受任意 UART 分块。

**错误 / 边界：** -EINVAL：参数或配置非法；-EBADMSG：通道、开关或鼠标按键字段非法。

## 调用示例

### 独立启动并读取遥控快照

```cpp
#include <communication/remote/remote_receiver.hpp>
#include "board_config.hpp"
using namespace skywalker::communication;
static AsyncUart::DmaBuffers dma __nocache;
static RemoteReceiver remote(board_config::remote_uart, dma, {});

int main() {
    const int ret = remote.start();
    if (ret < 0) return ret;
    RemoteReceiver::Snapshot frame{}; // 每个读者自己保留一份
    for (;;) {
        const int copied = remote.snapshot(frame);
        if (copied == 0 && frame.remote.online) {
            const auto right_x = frame.remote.analog.right_x;
            // right_x 是中心化原始值；运动映射交给命令层。
            (void)right_x;
        }
        k_sleep(K_MSEC(10));
    }
}
```

使用 samples/communication/dr16 的 board_config 和 UART 配置。正式命令服务通过 RemoteSource 读取，不直接在控制线程解析 DR16。

## 调用顺序

1. 准备 UART、DMA 与 RemoteReceiver；或把 receiver 引用交给 RemoteSource。
2. start 创建 worker，观察 State::Running 与 remote.online，二者分别代表串口服务和有效输入。
3. 后台每约 1 ms 处理有界字节预算；执行线程只读快照。
4. 断流时撤销 online；重连后使用新帧和新的执行授权，不凭遗留摇杆值重新使能。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `Dr16Decoder::Config` | 默认 center=1024、min=364、max=1684、center_deadband=10；decode_wheel 默认 false。 |
| `RemoteService::Config` | 默认 offline_timeout_ms=100、assembly_gap_ms=10；全部使用 ms。 |
| `CONFIG_SKYWALKER_REMOTE_DR16 / CONFIG_SKYWALKER_REMOTE_RECEIVER` | 分别启用纯解码服务与后台接收器；接收器依赖 UART_TRANSPORT。 |
| `CONFIG_SKYWALKER_REMOTE_RX_STACK_SIZE / PRIORITY` | 默认栈 3072 字节、线程优先级 6；UART 参数按接收机和板级配置提供。 |

- bytes 16～17 默认保留，decode_wheel=false；只有明确确认接收机提供第五通道才打开。
- RemoteState 的摇杆是原始中心化值，不能直接当 m/s 或 rad/s。
- 读取快照不消费数据、不延长有效期；0 不等于在线。
- RemoteReceiver 不支持 stop、重复 start 或运行中析构。

## 正文与源码

- [DR16 接线与样例](../../samples/communication/dr16/README.md)
- [命令来源服务](../modules/robotics/command-service.md)

- [接收器接口](../../include/communication/remote/remote_receiver.hpp)
- [流式服务](../../include/communication/remote/remote_service.hpp)
- [单帧解码](../../include/communication/remote/dr16_decoder.hpp)
- [消息字段](../../include/robotics/messages/remote.hpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
