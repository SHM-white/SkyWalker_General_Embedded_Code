# 13 通信层：UART、DR16、裁判与板间协议

实现位置：`lib/communication/`、`include/communication/`，消息类型位于 `include/robotics/messages/`。

## 1. `AsyncUart`

`AsyncUart` 是所有异步串口协议的传输边界：

- UART callback 管理两块 128 字节 RX DMA buffer。
- RX 数据被复制成最多 64 字节的 `RxChunk`，放入固定 8 项消息队列。
- TX 使用 256 字节固定缓冲；发送忙时 `send()` 返回 `-EAGAIN`。
- `service(now_ms)` 负责重新启用被禁用的 RX，并在发送超时后 abort。
- `read()` 返回 `-EOVERFLOW` 表示队列丢失、RX 停止或代际变化。

约定是一个通信线程拥有 `init/service/read/send`；callback 不执行 parser 和控制逻辑。读取出 `-EOVERFLOW` 后必须调用对应 parser 的 `discardPartial()`。

## 2. DR16

`Dr16Decoder` 解码固定 18 字节帧：四个摇杆、拨轮、左右拨杆、鼠标、鼠标键和键盘 bitmask。默认配置来自 `RemoteService`：

- channel center=1024，合法范围 364–1684。
- center deadband=10。
- frame assembly gap=10 ms。
- offline timeout=100 ms。

`RemoteService::snapshot()` 返回 `RemoteState` 值拷贝；没有有效帧返回 `-EAGAIN`，已有快照但超时返回 `-ESTALE`，并将 `online=false`。

MC02 样例使用 UART5 的 100000 baud、8E1；实际遥控器链路还要确认电平反相和 DMA 引脚。

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
static communication::AsyncUart uart(board_config::interboard_uart);
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
