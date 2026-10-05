# 大 Yaw 持续速度台架

使用中央 `chassis_can.hpp` 分配的独立 CAN2；硬件型号、MIT参数、方向、减速比和限幅来自 `include/robotics/vehicle/calibration.hpp`。完成接线与标定后启用 `connections_confirmed`。速度控制不要求绝对位置参考。

遥控使用MC02 remote-uart / UART5、100000 baud、8E1、RX DMA。双Down且全部通道归中500ms，再左Middle启动；左Down停止，左Up禁用。拨轮请求正负0.1rad/s。输入失联按100ms原始年龄停止，重复快照不会续期。

启动后持续提交最新速度；离线、协议握手或CAN恢复不撤销遥控意图。底层自动清错与重新使能，恢复后执行最新值，不需要手动清驱动故障。急停解除事件只影响用户急停。日志分别显示requested、实际run/wait、目标、实际反馈、控制与提交错误。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/big_yaw -d build/big_yaw
west build -b dm_mc02/stm32h723xx samples/robotics/big_yaw -d build/big_yaw_diagnostic -- -DEXTRA_CONF_FILE=diagnostic.conf
```

诊断支持1输入、2执行、3状态暂停。状态暂停保留生产时间，fresh显示状态年龄；它不反向控制目标。输入和执行暂停分别由命令源与底层命令有效期处理，恢复不要求电机重新授权。

操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)。初次验收给连续旋转接线留出空间；方向、停止延迟和断电恢复尚需现场记录。
