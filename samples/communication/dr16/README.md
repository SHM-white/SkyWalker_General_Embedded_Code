# DR16 实机输入

这是独立的上机微型项目，只依赖正式模块库。

## 接线

一块 MC02 或 RoboMaster C 板、DR16 接收机。板级 `remote-uart` 统一提供 100000 baud、8E1 和 RX DMA：MC02 使用 UART5（PD2 RX），C 板使用 USART3（PC11 RX）。接收机 TX 接主控 RX，并共地；必须确认板卡 DBUS 通路的电平与反相。不配置 TX 引脚或 TX DMA。

控制台沿用板级定义：MC02 为 USART10，C 板为 USART1，均为 115200 baud。

## 构建与刷写

```sh
west build -p always -b dm_mc02/stm32h723xx samples/communication/dr16 -d build/bench_dr16
west flash -d build/bench_dr16
```

C 板使用 `-b rm_typec/stm32f407xx` 和独立构建目录，无需添加 overlay。

## 操作与预期现象

打开对应 console 的 115200 波特率串口，先确认 `DR16 init=0`。初始化失败会输出错误并停止示例，应先解决设备、DMA 或缓冲区问题。

示例每 100 ms 输出一次诊断文本，无需 VOFA。示意输出（数值仅为示例）：

```text
online=1 bytes=1296 chunks=72 valid=72 rejected_windows=0 gaps=0 dropped=0 service=0
CH0=1024 CH1=1024 CH2=1024 CH3=1024 d0=0 d1=0 d2=0 d3=0
SW_HIGH=3 SW_LOW=3 TAIL16=0 mouse=0/0/0 buttons=0/0 keys=0000
```

| 字段 | 含义 |
|---|---|
| `CH0`～`CH3` | 四个摇杆通道的原始值，规定范围 364～1684，中心约 1024 |
| `d0`～`d3` | 减去配置中心后的值，示例关闭死区以显示细微变化 |
| `SW_HIGH` / `SW_LOW` | 字节 5 的 bit6～7 / bit4～5，协议原值 1=上、2=下、3=中 |
| `TAIL16` | 字节 16～17 的小端原始值，默认按保留字段处理 |
| `mouse` / `buttons` / `keys` | 鼠标 X/Y/Z、左右键、16 位键盘位图 |
| `bytes` / `chunks` | 应用累计读到的字节数 / 数据块数，块边界不等于帧边界 |
| `valid` / `rejected_windows` | 接受的帧数 / 拒绝的 18 字节候选窗口数，后者不等于真实坏帧数 |
| `gaps` / `dropped` | 接收连续性错误通知数 / UART 队列丢弃的 chunk 数 |
| `service` | 当前记录的 UART 接收维护错误，0 表示无该错误 |

依次移动四个摇杆方向，确认哪个 CH 变化；再逐一拨动开关，核实物理位置与两个位域的映射。鼠标键盘需要遥控器对应的数据入口。

没有近期有效帧但收到过字节时，会额外输出最近一个原始 RX chunk 的十六进制内容，便于区分串口没有数据与解码拒绝。它不保证恰好是一帧。断开接收机后约 100 ms `online` 变为 0；如之前收到过有效帧，通道栏保留最后一次值，不能当成新数据。

该示例复用正式 `Dr16Decoder`，在入口中维护诊断用滑动窗口。DR16 没有明确帧头与 CRC，范围检查只能用于启发式重同步，无法保证排除所有噪声候选。

## 协议版本与滚轮

仓库 `docs/UTF-84.RoboMaster 机器人专用遥控器（接收机）用户手册.pdf` 第 6～7 页定义四个 11 位通道，最后两字节为保留字段。旧解码器强制把尾字段作为第五通道校验，尾字段为 0 时会拒绝整帧；现已改为默认不解释滚轮。

转动滚轮观察 `TAIL16`。只有确认接收机使用滚轮扩展且尾字段中心、范围符合通道定义后，才在对应 `Dr16Decoder::Config` 中设置 `.decode_wheel = true`。关闭时正式状态的 `analog.wheel` 为 0；依赖滚轮的应用需要显式配置扩展。示例始终保留尾字段原始输出。

手册表格与附录对 S1/S2 顺序存在矛盾，因此示例使用位域位置命名，不把二者预先解释为左右开关。保留当前 100000/8E1；不要照搬附录中的 8-N-1 注释。

## 自行配置

UART、DMA 和串口格式由各板 DTS 维护；`src/board_config.hpp` 通过 `DT_ALIAS(remote_uart)` 获取设备，并配置解码中心、范围、死区和超时。

当前通道输出版本已在两块板上编译通过；未在此环境刷写或连接真实外设验证。
