# 11 调试与观测

## 1. VOFA+ JustFloat

`lib/vofa` 的发送帧是：

```text
[little-endian float0] ... [little-endian floatN-1] [00 00 80 7F]
```

末尾 `0x7F800000` 是同步标记。USART 和 USB CDC ACM 共用 IRQ/FIFO 收发接口：`vofa_send()` 将完整帧复制进实例自己的队列并立即返回，驱动回调负责分段发送。USART 的 VOFA 路径不再使用 UART Async/DMA API。启用：

```conf
CONFIG_SKYWALKER_LIB_VOFA=y
```

库自动选择 `CONFIG_UART_INTERRUPT_DRIVEN=y`。其他设备如果需要异步 UART，仍可启用 `CONFIG_UART_ASYNC_API`，但不能对 VOFA 所占用的同一设备再注册 Async 回调。

### 初始化时切换设备

```c
static Vofa vofa;

/* 二选一；同一个实例只初始化一次。 */
int err = vofa_init(&vofa, DEVICE_DT_GET(DT_NODELABEL(usart1)));
/* int err = vofa_init(&vofa, DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0))); */
if (err != 0) {
    /* 报告初始化错误，不能继续使用这个 VOFA 实例。 */
    return err;
}

const float channels[] = {1.0f, 2.0f};
err = vofa_send(&vofa, channels, 2);
```

两个设备使用完全相同的发送调用。也可像 `samples/hello` 一样使用 `DT_ALIAS(telemetry_uart)`，通过设备树选择目标。

- 每帧 1–16 个 float，每个实例可排队 4 帧；驱动还有自己的发送缓冲。
- 返回 0 表示已复制完整帧，可以立即复用输入数组；不表示主机已收到。
- 队列满返回 `-ENOBUFS`，拒绝整帧；输入无效返回 `-EINVAL`，实例未成功初始化返回 `-ENODEV`。
- FIFO 驱动返回负错误时停止该实例 TX，后续发送返回该错误；需要排查配置并重新启动，不支持原地重初始化。
- `vofa_send()` 可从多个线程或普通 ISR 调用，不等待主机打开端口。未连接时缓冲可能保留旧帧，缓冲满后新帧被拒绝；断线不保证数据交付。
- `Vofa` 实例必须保持地址不变且一直有效，不能复制、热切换设备，或让局部变量在回调仍有效时退出作用域。
- 初始化/接收配置应在启动线程中完成，并检查返回值。原有忽略返回值的发送语句仍可编译，但无法统计丢帧或报告错误。

### DM MC02 的 USB VOFA 示例

MC02 的 USB D−/D+ 为 PA11/PA12。板级设备树已启用 `usbotg_hs`，继承 HSI48 和片内 FS PHY，使用全速模式；USB 栈按应用配置启用。

从仓库根目录运行（已激活工作区 Zephyr 环境）：

```bash
# hello 默认通过 USB CDC ACM 输出 VOFA，无需额外配置参数。
# 首次迁移用 -p always 清理旧 overlay / EXTRA_CONF_FILE 缓存。
west build -p always -b dm_mc02/stm32h723xx \
  -d samples/hello/build/dm_mc02/stm32h723xx samples/hello

west flash -d samples/hello/build/dm_mc02/stm32h723xx
```

USB 配置已并入默认 `prj.conf`，设备选择在自动加载的 `app.overlay` 中；原来的 `usb-vofa.conf` / `usb-vofa.overlay` 已移除。若 IDE 曾显式添加这两个文件，需要移除对应参数。`prj.conf` 使用新 USB Device 栈的单 CDC ACM 启动辅助代码，不需要手动调用 `usb_enable()` 或 `usbd_enable()`。MC02 的 Console/Shell 继续指向 USART10，不要叠加 `cdc-acm-console` snippet。

要将 hello 的 VOFA 切回 USART1，只需把 `app.overlay` 中 `telemetry-uart` 的目标改为 `&usart1`，`main.c` 无需修改；保留 USB 配置时设备仍会枚举，但 VOFA 数据会发往 USART1。

电脑端在 VOFA+ 选择新出现的 COMx 或 `/dev/ttyACM*`、JustFloat 协议，hello 每隔约 10 ms 发送一个正弦波通道，周期 1 秒、幅值 1，数值范围为 −1 到 +1。可在 `samples/hello/src/main.c` 调整 `VOFA_SEND_PERIOD_MS`、`SINE_PERIOD_MS` 和 `SINE_AMPLITUDE`。USB 的串口波特率设置不改变 USB 总线速率。无需等待 DTR 才启动业务。

移植到其他应用时添加同样的 CDC ACM 节点和 USB 配置，再将初始化设备或 `telemetry-uart` 指向它；仅换设备指针不能代替 USB 栈初始化。USB VOFA 实例需独占，不能混入 Console/Shell 文字，也不能同时注册其他串口接收回调。

样例通道通常包括 target、measurement、error、reference、effort、温度和状态。使用 VOFA 看连续曲线，日志只打印状态切换和错误。

### 接收命令

初始化成功后调用一次 `vofa_set_handler(&vofa, buffer, sizeof(buffer), handler)`，它会自动启用接收。buffer 至少 2 字节，包含字符串结束符空间，且生命周期必须覆盖设备使用期间。

库按字节流拼接 `key=value\n`，支持命令跨多个 USART/USB 回调及 CRLF；超长或含 NUL 的行丢弃到下一次换行。数值保留原有十进制解析规则，不支持科学计数法。handler 的 key 指针仅在回调期间有效。

handler 在设备回调中运行：USART 可能在 ISR，当前 CDC ACM 在工作队列中。回调只更新参数快照或非阻塞投递消息，不要睡眠或直接运行闭环。RX 配置不支持并发设置或运行时替换。

从旧版迁移：移除手动注册 `vofa_uart_cb` 和调用 `uart_rx_enable()` 的代码；旧 Async 回调已移除，收发均由库管理。

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
