# 两板 UART / RS485 / CAN 联调

样例使用正式的 `ConfiguredInterBoardTransport + InterBoardEndpoint`，仅在初始化时选择后端。三种方式均有实际实现；默认 UART。此项目只交换业务消息，不连接电机。

## MC02 接线与角色

| 方式 | 接口 | 默认配置 |
| --- | --- | --- |
| UART | USART1：A PA9 → B PA10，B PA9 → A PA10，信号地连接 | 460800、8N1 |
| RS485 | 两板 RS485-2：A/B 对应连接，按实际线缆配置端接和信号参考 | USART2，460800、硬件 DE；云台协调端、底盘应答端 |
| CAN | 两板 CAN3：CANH/CANL 对应连接，信号参考与两端端接按硬件要求配置 | 经典 CAN 1 Mbps；云台 TX=0x600/RX=0x601，底盘相反 |

控制台使用 USART10，115200。RS485 收发器和 CAN 收发器已在 MC02 板上；MC02 CAN 收发器需要手册规定的供电。不要把 UART TX/RX 直接接到差分总线。第一次上机先断电接线，再给两板上电。

两块板选择相同通信方式，一块使用默认云台角色，另一块额外加载 `chassis.conf`。CAN3 由本样例独占，不与电机 CanBus 共用控制器。

## 构建

从 `skywalker_code` 根目录运行。UART 两端：

```sh
west build -b dm_mc02/stm32h723xx samples/communication/interboard -d build/interboard_uart_gimbal
west build -b dm_mc02/stm32h723xx samples/communication/interboard -d build/interboard_uart_chassis -- -DEXTRA_CONF_FILE=chassis.conf
```

RS485 两端：

```sh
west build -b dm_mc02/stm32h723xx samples/communication/interboard -d build/interboard_rs485_gimbal -- -DEXTRA_CONF_FILE=rs485.conf
west build -b dm_mc02/stm32h723xx samples/communication/interboard -d build/interboard_rs485_chassis -- '-DEXTRA_CONF_FILE=rs485.conf;chassis.conf'
```

CAN 两端：

```sh
west build -b dm_mc02/stm32h723xx samples/communication/interboard -d build/interboard_can_gimbal -- -DEXTRA_CONF_FILE=can.conf
west build -b dm_mc02/stm32h723xx samples/communication/interboard -d build/interboard_can_chassis -- '-DEXTRA_CONF_FILE=can.conf;chassis.conf'
```

刷写时使用对应目录，例如 `west flash -d build/interboard_rs485_gimbal`。根据连接的下载器明确选择要刷写的板子。

`prj.conf` 默认编译三种后端，配置片段只决定启动时选择哪个。UART 的设备树别名为 interboard-uart，RS485 为 interboard-rs485，CAN 为 interboard-can。USART2 新增 TX/RX DMA，使用通道 0/5，避开 SPI2、USART1 和 UART5 的板级分配。

## 业务调用与预期现象

三种方式都通过 `submit / setStatus / poll / snapshot` 交换数据。正常时日志 `peer_online=1`，底盘接收到递增命令序号，`cmd_fresh=1`。RS485 协调端上电先静默约一秒，之后开始轮询；CAN 每个批次包含多个分片，实际更新率受通信线程周期影响。

在云台控制台输入 `p` 只暂停新的命令生产，心跳继续；底盘最后一条命令超过 100 ms 后 `cmd_fresh=0`。再次输入 `p` 恢复生产。在底盘输入 `g` 改变恢复 generation，云台观察后绑定新的控制上下文。重启单板会生成新的 boot ID。

日志 `transport` 的 0/1/2 分别表示 UART/RS485/CAN，`error` 为负 errno。`-ENODEV` 优先定位设备/别名；RS485 的 `-ENOTSUP` 检查硬件 DE 和 8N1；CAN `-EBUSY` 检查控制器是否已被占用；超时检查两端模式、接线、供电、速率、CAN ID 和 poll 周期。Parser 的 CRC 计数仅针对内部 V1 帧，不包含下层 RS485/CAN 封装丢包。

完整接口、线上封装、缓冲限制和恢复行为见[板间传输文档](../../../docs/modules/communication/interboard-transports.md)。本次没有执行实际刷写或双板收发验证。
