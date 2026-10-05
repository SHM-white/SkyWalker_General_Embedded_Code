# 单轴 Yaw 持续控制台架

GM6020 电流模式接 CAN1 / ID1，驱动限幅 0.5 A、减速比 1:1、编码器零点 0。型号、ID、零点、位置参考与 PID 在 `src/board_config.hpp`。Limited 拓扑需要可信机械坐标；连续角拓扑使用已校准绝对角。

遥控接收使用板级 remote-uart，MC02 UART5 / PD2、100000 baud、8E1、RX DMA，接收机与主控共地。第五通道拨轮启用，console 只打印遥测。

启动保持左右开关 Down、全部通道归中 500 ms，再将左开关拨到 Middle。左 Down 停止，左 Up 在本例禁用。输入失联超过 100 ms 停止，真实输入恢复后的操作遵循 [统一遥控操作](../common/REMOTE_CONTROL.md)。电机掉线不撤销运行意图，恢复供电后自动继续最新目标。

右开关 Down 时，右横杆控制正负 0.3 rad/s，回中进入 Hold；Middle 请求绝对 0 rad，Up 请求绝对 0.5 rad。运行时每周期持续调用 enable、yaw.update 和 CAN commit，不检查电机 ready/active；本轴反馈和参考暂时不足由控制封装等待。电机恢复只重置本轴控制历史。保留协议范围、机械目标范围和 PID 输出限幅；温度与实测超速不再在此层锁停。

遥控目标保留接收帧原始序号和时间。重复读取不会续期旧输入。日志分别显示 requested、实际状态、目标、输出有效性和控制/提交错误。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/yaw_gimbal -d build/yaw_gimbal
west build -b dm_mc02/stm32h723xx samples/robotics/yaw_gimbal -d build/yaw_gimbal_diagnostic -- -DEXTRA_CONF_FILE=diagnostic.conf
```

诊断支持 1 输入暂停、2 执行暂停，默认诊断配置选 1。启动意图持续 3 秒后暂停 1.5 秒；输入年龄与底层命令年龄自然增长，停止入口始终有效。恢复后无需因电机异常重新解锁。

初次验收支撑机构，先确认方向、零点与实际停止行为。当前没有本次重构的实板验收记录。
