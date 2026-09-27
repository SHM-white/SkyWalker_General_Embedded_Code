# DR16 实机输入与 VOFA 回显

这是独立的上机微型项目。接收线程使用正式 `RemoteService` 解析 DR16，另一个线程通过 VOFA+ JustFloat 输出遥控状态。

## 接线

一块 MC02 或 RoboMaster C 板、DR16 接收机。板级 `remote-uart` 为 100000 baud、8E1、RX DMA：MC02 使用 UART5（PD2 RX），C 板使用 USART3（PC11 RX）。接收机 TX 接主控 RX，并共地；确认板卡 DBUS 通路的电平与反相。

VOFA 使用独立的板级 `telemetry-uart`，均为 115200 baud、8N1：MC02 为 USART1（PA9 TX），C 板为 USART6（PG14 TX）。USB 转串口模块的 RX 接对应 TX，并共地；确认模块的电平与板卡匹配。板级 console 仍为 MC02 的 USART10 或 C 板的 USART1，可用来查看初始化和异常日志。不要将 VOFA 与 DR16 或 console 接到同一个 UART 设备。

## 构建与刷写

```sh
west build -p always -b dm_mc02/stm32h723xx samples/communication/dr16 -d build/bench_dr16
west flash -d build/bench_dr16
```

C 板使用 `-b rm_typec/stm32f407xx` 和独立构建目录，无需添加 overlay。

## 操作与 VOFA 通道

先在 console 确认 `DR16 init=0` 和 `VOFA init=0`。某一端初始化失败时，其线程会停止，另一端继续运行；应检查对应设备、引脚和串口配置。

在 VOFA+ 选择连接遥测串口、115200 baud、JustFloat 协议。监视线程约每 100 ms 发送一帧 16 通道，按下表顺序命名；JustFloat 帧不携带通道名称。

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

VOFA 口只输出二进制 JustFloat 数据；console 只输出初始化信息以及 UART 连续性或 VOFA 队列错误。VOFA 发送返回 0 只表示帧已入队，不保证上位机收到。队列满时当前帧会被拒绝，程序在 console 报告错误，下一周期继续发送。

## 线程与解析边界

接收线程独占 `AsyncUart` 和 `RemoteService`。`AsyncUart::read()` 提供带时间戳的数据块；发生 `-EOVERFLOW` 时调用 `RemoteService::discardPartial()`；其他数据交给 `processBytes()`，通过 `snapshot()` 得到 `RemoteState`。接收线程每 1 ms 将遥控状态与接收诊断写入 `Latest<BenchSnapshot>`。VOFA 线程每 100 ms 读取快照，并根据原始时间戳再次判断是否在线；若本次读取因短暂锁竞争失败，沿用上一快照。VOFA 线程独占遥测 UART，不与接收线程共享 UART 回调。

DR16 没有明确帧头与 CRC，`RemoteService` 内部只能用范围检查启发式重同步，无法保证排除所有噪声候选。

## 协议版本与滚轮

仓库 `docs/UTF-84.RoboMaster 机器人专用遥控器（接收机）用户手册.pdf` 第 6～7 页定义四个 11 位通道，最后两字节为保留字段。默认不解释尾字段。只有确认接收机使用滚轮扩展且尾字段中心、范围符合通道定义后，才在 `src/board_config.hpp` 的 `Dr16Decoder::Config` 中设置 `.decode_wheel = true`。启用后通道 7 为相对中心的值；不匹配的扩展可能导致候选帧被拒绝，`online=0`。

手册表格与附录对 S1/S2 顺序存在矛盾；`left` / `right` 沿用正式解码器的字段命名，上机时应核实它们对应的物理开关。保留当前 100000/8E1；不要照搬附录中的 8-N-1 注释。

## 自行配置

UART、DMA 和串口格式由各板 DTS 维护；`src/board_config.hpp` 通过 `DT_ALIAS(remote_uart)` 和 `DT_ALIAS(telemetry_uart)` 获取两个独立设备，并配置 DR16 解码中心、范围、死区和超时。`prj.conf` 同时启用异步 UART 接收与 VOFA 所需的中断 UART 发送，它们作用于不同设备。
