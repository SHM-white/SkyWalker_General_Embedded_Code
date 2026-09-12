# 11 调试与上位机

本文覆盖两条调试链路：**VOFA+ 上位机绘图**（`lib/vofa`）与 **日志/断点**，
以及实时控制场景下必须注意的时序陷阱。

---

## 1. VOFA+ JustFloat 协议

### 1.1 帧格式

每帧 = N 个小端 IEEE-754 float + 4 字节帧尾 `0x7F800000`（`+inf`）：

```text
[float0][float1]...[floatN-1][00 00 80 7F]
```

上位机按通道顺序解析，帧尾用于同步。单次最多 **16 个 float**
（`VOFA_MAX_FLOATS`）。

### 1.2 API（`include/lib/vofa/vofa.h`）

```c
typedef struct {
    const struct device *uart;
    uint8_t *rx_buf; size_t rx_buf_size;
    vofa_cmd_handler on_cmd;
} Vofa;

void vofa_init(Vofa *vofa, const struct device *uart);
void vofa_send(Vofa *vofa, const float *data, uint8_t num);
void vofa_set_handler(Vofa *vofa, uint8_t *rx_buf, size_t rx_buf_size, vofa_cmd_handler on_cmd);
void vofa_uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

typedef void (*vofa_cmd_handler)(const char *key, float val);
```

- `vofa_init()` 设置 UART 设备并注册 `vofa_uart_cb`。
- `vofa_send()` 用**一次异步 DMA 发送**，函数立即返回；内部静态
  `__nocache` 缓冲，上一帧未发完时**丢弃当前帧**（`tx_busy` 原子标志），
  不会覆盖 DMA 正在读取的缓冲区。
- 串口需开 `CONFIG_UART_ASYNC_API=y`。

### 1.3 发送示例

```c
const float channels[4] = { target, measured, error, effort };
vofa_send(&vofa, channels, 4);
```

`hello` 用 1 通道；`dm_mit_position_control` 用 10 通道（目标、实际、位置误差、
速度参考、速度、速度误差、effort、力矩、驱动温度、电机温度）。

### 1.4 命令行在线调参（RX）

`lib/vofa` 支持 `key=value\n` 文本行命令：`vofa_uart_cb` 扫描 DMA 缓冲中的
换行，把 `=` 左侧作为 key、右侧用内置 `parse_float` 转 float，回调
`vofa_cmd_handler`。

启用方式：

```c
static uint8_t rx_buf[64];
vofa_set_handler(&vofa, rx_buf, sizeof(rx_buf), onCmd);
uart_rx_enable(uart, rx_buf, sizeof(rx_buf), SYS_FOREVER_US);
```

`UART_RX_BUF_RELEASED` / `UART_RX_DISABLED` 时回调会自动重新使能。

> 当前**没有样例接入 RX 路径**，只有库支持；接入时注意回调运行在 UART 回调
> 上下文，不要在回调里直接跑重控制逻辑，应只更新参数快照。

---

## 2. 日志

- 打开：`CONFIG_LOG=y`，级别 `CONFIG_LOG_DEFAULT_LEVEL`（3 = INF）。
- 模块：`LOG_MODULE_REGISTER(name, LOG_LEVEL_INF);`
- 默认 UART 后端（MC02 `dm_mc02_defconfig` 里 `CONFIG_LOG_BACKEND_UART=y`）。
- `rm_typec` 默认开了 `CONFIG_USE_SEGGER_RTT`，可用 RTT Viewer 看日志。

注意：日志本身有开销，高频控制路径里不要每周期打印；用 VOFA 看曲线，
日志只在状态切换/错误时打点。

---

## 3. 实时调试的时序陷阱

软件闭环对**真实 dt** 敏感，而调试器停在断点上会直接破坏时序：

- 样例的 `dt_max_s` 通常设为 20 ms（5 ms 控制周期）。**断点暂停超过 20 ms
  会让 `update()` 返回 `-ERANGE` 并触发停机**。
- 调试 MC02 的软件闭环样例时，开启 `CONFIG_STM32_ENABLE_DEBUG_SLEEP_STOP=y`
  可让线程睡眠期间保持调试连接（否则可能掉线）。
- **首次运行建议不放断点**，先看日志/VOFA 确认正常，再在有把握的时刻单步。
- 断点命中后寄存器/PC 可能不可信（尤其发生 stacking error 时），
  不要仅凭 PC 判断崩溃点。

相关仓库内指南：`Zephyr调试跳入底层函数排查指南.md`。

---

## 4. 常用构建调试开关

```conf
CONFIG_DEBUG_OPTIMIZATIONS=y      # -Og，便于单步
CONFIG_DEBUG_THREAD_INFO=y
CONFIG_THREAD_RUNTIME_STATS=y     # MC02 默认开，状态灯显示 CPU 负载
CONFIG_ASSERT=y
CONFIG_LOG=y
CONFIG_MAIN_STACK_SIZE=8192       # C++ / 长打印
```

`west build` 时也可通过 Zephyr IDE 的
「Zephyr IDE: Build Active」任务附加
`-DCONFIG_DEBUG_OPTIMIZATIONS=y -DCONFIG_DEBUG_THREAD_INFO=y`。

---

## 5. 观察点建议

| 现象 | 看什么 |
|---|---|
| 电机无反应 | 设备 `State`（Offline/Ready/Fault）、反馈 `timestamp_ms` |
| 反馈 ID 不对 | `dji::describe()` / `dm::describe()` 的 `feedback_id` / `control_id` |
| 控制发抖 | VOFA 的 error、effort、dt 通道；确认 dt 在 `[dt_min, dt_max]` |
| 频繁停机 | `status().error` 与 `status().stop_error` |
| 位置漂移 | 坐标模式（`StartupRelative` vs `DriverContinuous`）、GM6020 零点 |
| IMU 不收敛 | 是否静止（零偏 LPF 条件）、EKF 卡方标志、加速度幅值 |

---

## 6. 相关文档

- [06 IMU 与 EKF](06-drivers-imu.md)
- [09 统一速度/位置封装](09-motor-wrapper.md)
- [12 故障排查](12-troubleshooting.md)
