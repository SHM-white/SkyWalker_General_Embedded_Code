# 板间通信演练

本例程只交换消息并模拟执行状态，不连接或驱动电机，因此保留自动演练，无需遥控器。串口只打印状态，没有终端指令入口。`ConfiguredInterBoardTransport` 在启动时选择 UART、RS485 或独立 CAN3。

双方必须使用当前唯一协议标识 `3`。没有能力协商、旧协议回退或可选契约版本；错误标识的帧直接拒收。底盘与大 Yaw 分别使用自己的恢复代次，重启绑定使用 boot ID，原始输入、执行状态和清故障事件分别保留实际年龄。

| 消息 | ID | 索引 | 载荷字节 |
| --- | --- | --- | --- |
| Heartbeat | `0x0001` | 0 | 24 |
| ChassisControl | `0x0101` | 1 | 36 |
| ChassisConstraint | `0x0102` | 2 | 20 |
| ChassisFeedback | `0x0103` | 3 | 32 |
| OperatorControl | `0x0301` | 4 | 36 |
| BigYawRequest | `0x0402` | 5 | 40 |
| BigYawFeedback | `0x0403` | 6 | 28 |

管理消息由云台角色产生；接收端要求心跳在线、双方 boot 匹配、原始输入年龄不超过 100ms。管理消息失效、撤销运行许可或请求急停时，轮底盘和大 Yaw 请求同时禁用。通信重发不更新原始输入时间；clear 事件按发送方 boot 与事件编号一次消费，重启后不得重绑定旧 clear。

从仓库根目录构建云台角色：

```sh
west build -b dm_mc02/stm32h723xx samples/communication/interboard -d build/interboard_gimbal
```

底盘角色使用 `-DEXTRA_CONF_FILE=chassis.conf`；两端必须选择同一种物理传输。UART、RS485、CAN 的设备、波特率、ID 与引脚以 `src/board_config.hpp` 和 overlay 为准。仅连接板间通信与供电，不连接执行器；UART 交叉 TX/RX 并共地，RS485 连接同名 A/B 并使用正确终端电阻，CAN 使用独立 CAN3 并正确终端匹配。

默认持续产生 100Hz 合成命令，执行状态和心跳独立发布。观察 `online`、`authority`、`cmd`、`ready`、`valid`、底盘 `gen` 与大 Yaw `localGen/peerGen`。命令生产停止时输入年龄增加，状态生产停止时 ready/valid 超时撤销，心跳继续工作。

诊断通过独立配置选择，正常固件无需控制台操作：

| `CONFIG_SAMPLE_DIAGNOSTIC_SCENARIO` | 演练 |
| --- | --- |
| 0 | 正常连续交换 |
| 1 | 暂停命令与管理输入生产 |
| 2 | 暂停执行生产 |
| 3 | 暂停状态发布 |
| 8 | 轮底盘恢复代次递增一次 |
| 9 | 大 Yaw 恢复代次递增一次 |
| 10 | 临时注入错误协议标识，CRC 仍正确 |

双方在线稳定 3 秒后触发一次；暂停与错误标识持续 1.5 秒，代次变化只发生一次。无自动重复。错误协议演练使用 `-DEXTRA_CONF_FILE=invalid_protocol.conf`，底盘角色组合为 `-DEXTRA_CONF_FILE="chassis.conf;invalid_protocol.conf"`。这会在样例传输装饰器中改变帧标识，正常 endpoint 始终只编码当前协议。

所有帧保持 14 字节封装开销，传输批次最多 240 字节。完整云台批次当前为 226 字节；调度器逐帧装箱，装不下的消息保持待发送，下轮按最新年龄重编码。发送成功只表示后端接收批次，不表示对端已复位或执行。
