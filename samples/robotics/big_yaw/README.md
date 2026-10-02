# 大 Yaw 遥控速度台架

大 Yaw 使用中央 `chassis_can.hpp` 分配的独立 CAN2。完成 `include/robotics/vehicle/calibration.hpp` 的接线确认、DM MIT 参数、减速比、方向和速度环标定后才设置 `connections_confirmed`；默认不输出。独立速度内环和恢复代次不依赖固定绝对机械零点。

遥控器使用板级 `remote-uart`，MC02 为 UART5/PD2，100000 baud、8E1、RX DMA；接收机与主控共地。启用第五通道拨轮，不使用遥控键鼠或终端输入。控制台和 VOFA 仍可用于观测。

上电保持左、右开关都在 Down，四个摇杆轴和拨轮归中至少 0.5 秒，再将左开关拨到 Middle 解锁。左 Down 随时停机，左 Up 在本台架不启用 Auto。失联超过 100 ms、执行器运行中故障或失能后，必须重新完成安全档归中与解锁。清除故障：先完成双 Down 归中 0.5 秒，保持左 Down，将右开关拨到 Up 且全部通道归中 1 秒；清除只发生一次，不会自动使能。

遥控目标保留接收帧的原始时间戳和序号。重复读取快照不会续期，也不需要手动以控制周期重复发送字母。硬件接线、校准和现有电流/力矩保护仍是运行前提。

拨轮连续控制正负 0.1 rad/s，归中请求零速度。正常命令及卸载台架权限直接保留 RC 接收帧时间，恢复代次由本地执行器维护。CAN 提交错误、控制故障或失去使能状态会撤回当前遥控授权，不会恢复旧速度。

普通构建 `CONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=0`，不提供运行时故障菜单。独立诊断构建：

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/big_yaw -d ../build/big_yaw_rc_diagnostic -- -DEXTRA_CONF_FILE=diagnostic.conf
```

`diagnostic.conf` 选择 1（暂停命令生产）。支持场景：1 输入暂停、2 执行暂停、3 状态发布暂停。也可为同一诊断构建设置 `-DCONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=N` 选择支持的编号。连续 Active 3 秒后仅触发一次，暂停持续 1.5 秒；实时遥控停机与清故障始终优先，诊断不伪造新的来源时间戳。停机后重新解锁；诊断计时器不会跟随被暂停的执行器停住。

状态暂停仅冻结日志使用的发布快照，其原时间戳保持不变；`fresh` 显示状态是否过期。执行和输入暂停分别走现有驱动命令超时、执行器输入超时路径。

普通构建：`west build -b dm_mc02/stm32h723xx samples/robotics/big_yaw -d ../build/big_yaw_rc`。连续旋转接线、限速/力矩、方向和停机延迟尚需台架记录。
