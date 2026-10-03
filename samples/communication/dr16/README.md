# DR16 实机输入与 VOFA 回显

这是独立的上机微型项目。接收线程由正式 `RemoteReceiver` 模块管理，内部使用 `RemoteService` 解析 DR16，另一个线程通过 VOFA+ JustFloat 输出遥控状态。

## 接线

一块 MC02 或 RoboMaster C 板、DR16 接收机。板级 `remote-uart` 为 100000 baud、8E1、RX DMA：MC02 使用 UART5（PD2 RX），C 板使用 USART3（PC11 RX）。接收机 TX 接主控 RX，并共地；确认板卡 DBUS 通路的电平与反相。

默认 `app.overlay` 将 `telemetry-uart` 指向板载 USB CDC ACM。用数据线连接主控板的 USB 数据接口，在电脑上打开新出现的 `SkyWalker Telemetry` 串口；Horco CMSIS-DAP 的串口是调试器日志通路，不是这个遥测串口。板级 console 仍为 MC02 的 USART10 或 C 板的 USART1，可用来查看初始化和异常日志。

## 构建与刷写

```sh
west build -p always -b dm_mc02/stm32h723xx samples/communication/dr16 -d build/bench_dr16
west flash -d build/bench_dr16
```

C 板使用 `-b rm_typec/stm32f407xx` 和独立构建目录，无需添加 overlay。

本示例的 CMake 显式追加 `app.overlay`，确保 IDE 缓存的 overlay 选择不会遗漏 USB 遥测绑定；编译期同时要求 `telemetry-uart` 指向 CDC ACM。已有构建目录建议使用上述 `-p always` 完整重新配置，并从同一个目录刷写。

启动日志的 `Telemetry device=...` 应包含 `cdc_acm_uart0`。如果是 `serial@40011000`，说明运行的固件仍将遥测发往 MC02 的 USART1，而不是 USB。此时即使 USB 枚举成功、`queued` 持续增长，USB 串口也收不到遥测；应检查刷写目录和固件版本。USB 栈启动成功不代表应用选中了 USB 串口。

## 操作与 VOFA 通道

先在 console 确认 `DR16 receive and VOFA telemetry threads started` 和 `VOFA init=0`。前者表示接收线程已启动，不代表已收到有效遥控帧；后者只表示遥测 UART 绑定成功，不代表 USB 主机已经打开串口。遥控初始化失败时，接收模块持续发布离线和失败状态；VOFA 初始化失败时其线程停止。

在 VOFA+ 选择主控板的 USB 遥测串口、115200 baud、JustFloat 协议，沿用 hello 示例可用的连接设置。USB CDC 的波特率不决定物理 USB 传输速率。监视线程约每 100 ms 提交一帧 16 通道（68 字节），按下表顺序命名；JustFloat 帧不携带通道名称。普通文本串口助手不能把这种二进制数据直接显示成通道数值。

发送方式与 hello 一致，不以 DTR 为发送前提，DTR 只用于日志诊断。每秒 console 输出 `Telemetry queued=... send=...; DR16 state=... online=... seq=... rx_chunks=...`：`queued` 是累计入队成功帧数，不代表电脑已收到；`send` 是最近一次入队返回值；`state` 为 0=未启动、1=启动中、2=运行中、3=初始化失败。正常时 `queued` 约每秒增加 10；即使遥控器离线也应持续发送。

如果 USB 能连接但没有帧，保留这些周期日志及 `USB DTR=... query=...`。`send=-ENOBUFS` 表明应用发送队列已满，需要继续排查 CDC 消费路径；`queued` 持续增长但上位机接收字节数为 0 时，需要继续定位驱动到主机的传输。若接收字节数增长却没有曲线，检查 JustFloat 解析和通道选择。hello 每帧是 8 字节，本示例每帧是 68 字节，不能按 hello 的单通道固定帧长接收。

`usbd_ch9` 的 `80 06 00 06 00 00 0a 00` 是主机请求 Device Qualifier 描述符；控制器仅支持全速时，Zephyr 返回不支持，这是该请求的预期处理，不能单凭这条日志判定枚举失败。`Spurious resume event` 也不能单独证明遥测故障。多次启动横幅表示发生过多次启动，仅凭这些日志无法确定复位原因。

| 序号 | 建议名称 | 含义 |
|---:|---|---|
| 0 | online | 1=最近 100 ms 有有效帧，0=离线 |
| 1～4 | d0～d3 | 四个摇杆通道相对 1024 中心的值；加 1024 得到原始 CH0～CH3。示例关闭死区 |
| 5～6 | left、right | `RemoteState` 开关枚举：0=未知、1=上、2=中、3=下；与协议原始位值不同 |
| 7 | wheel | 第五通道相对中心的值；默认不解码，显示 0 |
| 8～10 | mouse_x、mouse_y、mouse_z | 鼠标位移 |
| 11～12 | mouse_left、mouse_right | 鼠标按键，0 或 1 |
| 13 | keys | 16 位键盘位图的数值；按整数或十六进制理解 |
| 14 | seq | 最近有效遥控帧的序号，没有新有效帧时保持不变 |
| 15 | rx_chunks | 应用累计读到的 UART 数据块数；块边界不等于帧边界 |

依次移动摇杆并拨动开关，核实通道与物理输入的对应关系。断开接收机约 100 ms 后 `online` 变为 0；其他通道保留最后一次有效帧的数据，不能当成新输入。若 `rx_chunks` 增长而 `seq` 不变，应排查串口格式、反相和协议；若 `rx_chunks` 不增长，应排查接线与 RX DMA。序号和累计计数转为 float 后，超过 16777216 不再保证逐一精确表示，适合短时台架观察。

VOFA 口只输出二进制 JustFloat 数据；console 输出初始化信息、每秒诊断状态以及 UART 连续性或 VOFA 队列错误。VOFA 发送返回 0 只表示帧已入队，不保证上位机收到。队列满时当前帧会被拒绝，程序在 console 报告错误，下一周期继续发送。

## 线程与解析边界

`RemoteReceiver` 内部线程独占 `AsyncUart` 和 `RemoteService`，每轮最多读取 8 个数据块，1 ms 休眠。`-EOVERFLOW` 会丢弃解析半帧，其余字节按原始接收时间戳解析。模块发布 `Snapshot`，包含遥控状态、接收块数、重同步次数、丢块数和 UART 错误。VOFA 线程每 100 ms 调用 `receiver.snapshot()`，模块在读取时重新检查超时；锁竞争时保留旧快照但仍使过期数据离线。VOFA 独占遥测 UART。

`receiver.start()` 返回 0 只表示线程已安排启动，UART 初始化结果见模块日志及快照的 `state/uart_error`。模块与独占的 `__nocache` DMA 缓冲区都采用静态存储；不支持停止、销毁或重复启动。

DR16 没有明确帧头与 CRC，`RemoteService` 内部只能用范围检查启发式重同步，无法保证排除所有噪声候选。

## 协议版本与滚轮

仓库 `docs/UTF-84.RoboMaster 机器人专用遥控器（接收机）用户手册.pdf` 第 6～7 页定义四个 11 位通道，最后两字节为保留字段。默认不解释尾字段。只有确认接收机使用滚轮扩展且尾字段中心、范围符合通道定义后，才在 `src/board_config.hpp` 的 `Dr16Decoder::Config` 中设置 `.decode_wheel = true`。启用后通道 7 为相对中心的值；不匹配的扩展可能导致候选帧被拒绝，`online=0`。

手册表格与附录对 S1/S2 顺序存在矛盾；`left` / `right` 沿用正式解码器的字段命名，上机时应核实它们对应的物理开关。保留当前 100000/8E1；不要照搬附录中的 8-N-1 注释。

## 自行配置

UART、DMA 和串口格式由各板 DTS 维护；`src/board_config.hpp` 通过 `DT_ALIAS(remote_uart)` 和 `DT_ALIAS(telemetry_uart)` 获取两个独立设备，并配置 DR16 解码中心、范围、死区和超时。`prj.conf` 同时启用异步 UART 接收与 VOFA 所需的中断 UART 发送，它们作用于不同设备。
