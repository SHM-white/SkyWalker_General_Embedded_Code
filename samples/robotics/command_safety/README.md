# 遥控命令保险观察台架

仅观察命令，不创建电机。使用摇杆、双三档开关和拨轮，不依赖终端指令或键鼠。

先左 Down、右 Down，双摇杆和拨轮归中保持 500 ms，再拨左 Middle 解锁。左 Down 立即撤销输出；左 Up 在本台架禁用。遥控断流 100 ms 后输出 Disabled，重连必须重新完成安全档解锁。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/command_safety -d build/command-insurance
```

`ArmedRemoteSource` 在真实 RemoteSource 与 CommandManager 之间应用统一授权；保留每份 RC 原始时间戳。后台服务按 10 ms 周期仲裁，观察循环读取非消费式快照。接收器只由服务启动一次，来源与 DMA 为静态生命周期。

第五通道拨轮解码已启用。接线与端口使用板级 remote-uart；统一操作见 [遥控操作](../common/REMOTE_CONTROL.md)。
