# 遥控命令保险观察台架

仅观察命令，不创建电机。Down 撤销全部输出，Middle 使用新鲜遥控帧恢复手动目标；Up 在本台架未启用。遥控断流后输出 Disabled，重新收帧后自动恢复，不需要复位键或模拟底盘心跳。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/command_safety -d build/command-insurance
```

main 仅注册 RemoteSource 并启动 CommandManager；后台服务按默认 10 ms 周期调用内部 CommandArbiter，观察循环读取非消费式快照。来源与服务保持静态生命周期，遥控接收器由服务启动一次。日志显示 online、操作模式、机构模式、原因、调用错误和生产者序号。配置与接线仍在 src/board_config.hpp 及 app.overlay。
