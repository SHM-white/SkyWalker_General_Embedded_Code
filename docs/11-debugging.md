# 11 调试与观测

## 1. VOFA+ JustFloat

`lib/vofa` 的发送帧是：

```text
[little-endian float0] ... [little-endian floatN-1] [00 00 80 7F]
```

末尾 `0x7F800000` 是同步标记。`vofa_send()` 使用异步 UART；发送忙时丢弃当前帧，不会覆盖 DMA 正在使用的缓冲。启用：

```conf
CONFIG_SKYWALKER_LIB_VOFA=y
CONFIG_UART_ASYNC_API=y
```

样例通道通常包括 target、measurement、error、reference、effort、温度和状态。使用 VOFA 看连续曲线，日志只打印状态切换和错误。

库还支持 `key=value\n` 接收命令，但当前样例主要使用发送观测；如果接入 RX，回调中只更新参数快照，不要直接运行闭环或阻塞。

## 2. 日志与构建信息

```conf
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_ASSERT=y
CONFIG_MAIN_STACK_SIZE=8192
```

每次构建后先看：

- `build/<name>/zephyr/.config`：最终 Kconfig。
- `build/<name>/zephyr/zephyr.dts`：合并后的设备树。
- `build/<name>/zephyr/runners.yaml`：默认 runner 和 probe 参数。
- `build/<name>/zephyr/zephyr.map`：链接、Flash 和 RAM 占用。

不要只看 `prj.conf`；board defconfig、Kconfig select 和 overlay 可能改变最终结果。

## 3. 异步 UART 调试

`AsyncUart` 的 callback 只把 RX 数据切成固定 chunk 放入队列。业务线程必须循环：

```cpp
uart.service(now);
while (uart.read(chunk) == 0)
    parser.process(chunk.bytes, chunk.size, chunk.timestamp_ms);
```

`read()` 返回 `-EOVERFLOW` 时，表示 RX buffer/队列发生丢失或代际变化；必须调用 parser 的 `discardPartial()`，然后等待新帧。继续拼接旧半帧会污染协议状态。

## 4. 电机实时调试

软件闭环使用真实 dt。断点暂停会导致：

- 反馈 timestamp 超时。
- PID dt 超过 `dt_max_s`。
- command cache 超时。
- runtime 自动 suspend/fault。

首次调试建议用日志/VOFA 而不是断点；要停机检查时先通过键盘/遥控发送 disable，或者直接切断动力电源。MC02 的部分样例打开 `CONFIG_STM32_ENABLE_DEBUG_SLEEP_STOP=y` 只是帮助睡眠时保持调试连接，不会修复控制时序。

## 5. 观测点

| 现象 | 优先观测 |
|---|---|
| 电机无动作 | `device state`、BusState、反馈 timestamp、arm/flush 返回值 |
| 位置方向错误 | raw encoder、`absolute_position_rad`、减速比、motor direction |
| 控制发抖 | dt、filtered velocity、PID error、effort、限幅状态 |
| 反复恢复 | `MotorStatus.error`、`last_recovery_error`、recovery_attempts、CAN state |
| 双主控不在线 | AsyncUart dropped chunks、parser CRC/sequence、peer boot_id/generation |
| 安全一直 Disable | active reason、权限 timestamp、远端 heartbeat/feedback age |

## 6. 现场记录

每次上机建议记录 board、commit、overlay、motor firmware、CAN ID、限幅、零点和日志中的错误码。这样才能区分代码变化、硬件接线和标定参数变化。
