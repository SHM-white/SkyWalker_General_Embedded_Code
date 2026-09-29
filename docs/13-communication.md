# 13 通信层：UART、DR16、裁判与板间协议

实现位置：`lib/communication/`、`include/communication/`，消息类型位于 `include/robotics/messages/`。

| 模块 | 上游 | 下游 | 示例 |
|---|---|---|---|
| `AsyncUart` | Zephyr UART / DMA 回调 | 带原始时间戳的字节块 | [板间样例](../samples/communication/interboard/README.md) |
| `RemoteReceiver` | `AsyncUart` + DR16 解码 | 可复制的遥控快照 | [DR16 样例](../samples/communication/dr16/README.md) |
| `RefereeService` | 裁判字节流 | 权限、功率与在线状态 | [裁判样例](../samples/communication/referee/README.md) |
| `InterBoardLink` | 板间帧 | 心跳、控制、约束和反馈快照 | [双主控应用](15-applications.md) |

业务层消费快照并检查时间戳；通信回调本身不做机器人决策。遥控到电机、板间到本地安全的完整流程见 [模块联动](module-integration.md)。

## 1. `AsyncUart`

`AsyncUart` 是所有异步串口协议的传输边界：

- UART callback 管理两块 128 字节 RX DMA buffer。
- RX 数据被复制成最多 64 字节的 `RxChunk`，放入固定 8 项消息队列。
- TX 使用 256 字节固定缓冲；发送忙时 `send()` 返回 `-EAGAIN`。
- `service(now_ms)` 负责重新启用被禁用的 RX，并在发送超时后 abort。
- `read()` 返回 `-EOVERFLOW` 表示队列丢失、RX 停止或代际变化。

约定是一个通信线程拥有 `init/service/read/send`；callback 不执行 parser 和控制逻辑。读取出 `-EOVERFLOW` 后必须调用对应 parser 的 `discardPartial()`。

## 2. DR16

`Dr16Decoder` 解码固定 18 字节帧：四个摇杆通道、拨杆、鼠标、鼠标键和键盘 bitmask。默认按仓库 DT7/DR16 手册将最后两字节视为保留字段，不参与通道范围校验。配置默认值：

- channel center=1024，合法范围 364–1684。
- center deadband=10。
- decode_wheel=false；只有确认接收机把最后两字节用作滚轮通道时才显式设为 true，关闭时 `analog.wheel=0`。
- frame assembly gap=10 ms。
- offline timeout=100 ms。

`RemoteService::snapshot()` 返回 `RemoteState` 值拷贝；没有有效帧返回 `-EAGAIN`，已有快照但超时返回 `-ESTALE`，并将 `online=false`。

MC02 使用 UART5，C 板使用 USART3，均由板级 `remote-uart` 提供 100000 baud、8E1 和 RX DMA；实际遥控器链路还要确认电平反相和接线。

[DR16 示例](../samples/communication/dr16/README.md) 使用 `RemoteReceiver` 接收，独立 VOFA 线程每 100 ms 输出 16 通道，包括在线标志、摇杆、拨杆、鼠标、键盘、帧序号和接收块数；console 显示初始化与错误变化。该示例关闭解码死区，其余接入方保留默认死区 10。

### `RemoteReceiver`：可复用接收线程

正式接口在 [remote_receiver.hpp](../include/communication/remote/remote_receiver.hpp)，实现在 [remote_receiver.cpp](../lib/communication/remote_receiver.cpp)。它组合 `AsyncUart`、`RemoteService` 和公共 `Latest<Snapshot>`，内部线程每轮最多读取 8 个 chunk，休眠 1 ms；发生 `-EOVERFLOW` 时丢弃半帧。解析使用 chunk 的原始时间戳，发布快照不会刷新有效帧时间或序号。

`dr16`、`command_safety`、`gimbal_control` 三个 sample 和 `applications/sentry_gimbal` 均使用此模块。业务线程只读取快照，拨杆映射、急停、命令和电机使能由各调用方决定。需要自行调度解析的程序仍可单独使用 RemoteService。

| 接口/字段 | 语义 |
| --- | --- |
| `start()` | 0 表示接收线程已安排启动；配置非法返回 `-EINVAL`，重复启动返回 `-EALREADY`；不代表 UART 已初始化成功 |
| `snapshot(out)` | 0 表示已复制快照；锁竞争返回 `-EAGAIN`，保留调用方旧值，但仍检查其在线有效期；本接口不返回 `-ESTALE` |
| `state` | `NotStarted`、`Starting`、`Running`、`InitFailed`；Running 不等于遥控在线 |
| `remote.online` | 按有效帧时间戳和 `offline_timeout_ms` 判定；读取时再次检查，接收线程停滞也不会使旧值永久在线 |
| `uart_error` | UART 初始化或运行错误；运行 service 成功时清零，重试等待 `-EAGAIN` 不覆盖此前错误 |
| `rx_chunks` / `resets` / `dropped` | 成功读取块数 / 接收连续性重置次数 / UART 队列丢块数；都不是有效帧计数 |

所有公开调用仅在线程上下文进行。多个消费者分别保留自己的零初始化 Snapshot；不要跨线程直接访问 RemoteService。UART 初始化失败后模块每 100 ms 发布离线失败状态，不重新调用 init；修复配置后重启。运行中的 RX 恢复仍由 AsyncUart.service 处理。

启用配置：

```conf
CONFIG_CPP=y
CONFIG_SERIAL=y
CONFIG_SKYWALKER_LIB_COMMUNICATION=y
CONFIG_SKYWALKER_UART_TRANSPORT=y
CONFIG_SKYWALKER_REMOTE_DR16=y
CONFIG_SKYWALKER_REMOTE_RECEIVER=y
```

UART_TRANSPORT 会选择 UART_ASYNC_API。可选 `CONFIG_SKYWALKER_REMOTE_RX_STACK_SIZE` 默认 3072，`CONFIG_SKYWALKER_REMOTE_RX_PRIORITY` 默认 6；优先级必须小于 `CONFIG_NUM_PREEMPT_PRIORITIES`。

最小入口示例（设备树已提供 remote-uart）：

```cpp
#include <communication/remote/remote_receiver.hpp>
using namespace skywalker::communication;

static AsyncUart::DmaBuffers dma_buffers __nocache; // 不加初始化器
static RemoteReceiver receiver(DEVICE_DT_GET(DT_ALIAS(remote_uart)), dma_buffers, {});

int main() {
    const int ret = receiver.start();
    if (ret < 0)
        return ret;
    RemoteReceiver::Snapshot input{};
    for (;;) {
        receiver.snapshot(input); // 争锁时保留上一副本并检查超时
        if (input.remote.online) {
            // 将 input.remote 交给业务层；保留其原始 stamp。
        }
        k_sleep(K_MSEC(10));
    }
}
```

实例和每实例独占的 DMA 缓冲区必须静态存活到系统结束，初始化失败也一样；不支持 stop、销毁后重建或重复 start。同一个 UART 不能再交给另一接收模块或 console/VOFA。只有 DmaBuffers 放在 nocache 区域，Receiver 本体仍在普通内存。详见 [DMA 说明](16-uart-dma-nocache.md)。

## 3. 裁判系统

`RefereeParser` 不从 payload 长度猜协议版本，必须明确构造 `RefereeVersion`。当前实现 profile 是 `Rm2026V1_3`，识别：

- `0x0201`：机器人 ID、云台/底盘/发射机构权限和底盘功率上限。
- `0x0202`：buffer energy 等功率状态。

解析前校验帧头、CRC8、长度和 CRC16。保留字段不会被伪装成实测功率。`RefereeService::snapshot(now, out)` 会按 timeout 把状态标记为 online/offline。

## 4. 板间帧格式

`InterBoardCodec` 产生完整帧，最大 payload 128 字节、最大 frame 142 字节：

```text
0..1    0xA5 0x5A
2       protocol version = 1
3       sender role: GimbalController=1 / ChassisController=2
4..5    MessageId little-endian
6..7    payload length little-endian
8..11   frame sequence little-endian
12..    payload
end     CRC16-CCITT little-endian
```

当前消息 ID：

| ID | 发送角色 | 内容 |
|---:|---|---|
| `0x0001` | 任一 | Heartbeat |
| `0x0101` | Gimbal | ChassisControl |
| `0x0102` | Gimbal | ChassisConstraint |
| `0x0103` | Chassis | ChassisFeedback |
| `0x0104` | Chassis | ChassisFault，已预留 |
| `0x0201` | Chassis | GimbalFeedback，已预留 |
| `0x0301` | 任一 | SystemEvent，已预留 |

当前 `InterBoardLink` 实际解码并缓存 Heartbeat、ChassisControl、ChassisConstraint、ChassisFeedback 四类消息。

## 5. boot / sequence / generation

- `frame_sequence` 用于拒绝重复和乱序帧。
- `sender_boot_id` 用于识别远端重启；检测到变化时清空控制、约束和反馈快照。
- `resume_generation` 表示一次新的本地恢复上下文；它阻止断线前的旧命令跨恢复边界继续生效。
- `peerOnline()` 默认要求最近 200 ms 内收到 heartbeat。
- `forwardedFresh()` 同时检查消息年龄和转发链路年龄，避免“消息本身新但转发已经过期”。

## 6. 推荐接入模板

```cpp
static communication::AsyncUart::DmaBuffers dma_buffers __nocache;
static communication::AsyncUart uart(board_config::interboard_uart, dma_buffers);
communication::InterBoardLink link(BoardRole::GimbalController);
uart.init();
for (;;) {
    auto now = k_uptime_get();
    uart.service(now);
    AsyncUart::RxChunk chunk{};
    while (uart.read(chunk) == 0)
        link.processRxBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);
    link.processRxBytes(nullptr, 0, now); // 触发半帧超时检查
    k_sleep(K_MSEC(1));
}
```

实际应用应把 `latest*()` 的值拷贝到线程安全快照，再由安全/控制线程消费。不要把 parser 内部引用跨线程保存。
