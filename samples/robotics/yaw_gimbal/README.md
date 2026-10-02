# 单轴 Yaw 遥控台架

GM6020 电流模式默认接 CAN1、ID1，电流限幅 0.5 A，减速比 1:1，编码器零点 0。型号、ID、零点、控制参考与 PID 在 `src/board_config.hpp`。上电前支撑机构并核对电流模式、零点、方向；Limited 拓扑必须有已校准的连续坐标与机械限位。

遥控器使用板级 `remote-uart`，MC02 为 UART5/PD2，100000 baud、8E1、RX DMA；接收机与主控共地。启用第五通道拨轮，不使用遥控键鼠或终端输入。控制台和 VOFA 仍可用于观测。

上电保持左、右开关都在 Down，四个摇杆轴和拨轮归中至少 0.5 秒，再将左开关拨到 Middle 解锁。左 Down 随时停机，左 Up 在本台架不启用 Auto。失联超过 100 ms、执行器运行中故障或失能后，必须重新完成安全档归中与解锁。清除故障：先完成双 Down 归中 0.5 秒，保持左 Down，将右开关拨到 Up 且全部通道归中 1 秒；清除只发生一次，不会自动使能。

遥控目标保留接收帧的原始时间戳和序号。重复读取快照不会续期，也不需要手动以控制周期重复发送字母。硬件接线、校准和现有电流/力矩保护仍是运行前提。

右开关 Down 时，右横杆控制正负 0.3 rad/s（向右为负），回中进入 Hold；右开关 Middle 请求绝对 0 rad，Up 请求绝对 0.5 rad。这两个绝对目标只在已解锁时生效。每次使能前 `yaw.reset()` 使用新鲜的失能测量建立 Hold 参考；随后仅在 Motor Active 时更新控制并提交 CAN。

普通构建 `CONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=0`，不提供运行时故障菜单。独立诊断构建：

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/yaw_gimbal -d ../build/yaw_gimbal_rc_diagnostic -- -DEXTRA_CONF_FILE=diagnostic.conf
```

`diagnostic.conf` 选择 1（暂停命令生产）。支持场景：1 输入暂停、2 执行暂停。也可为同一诊断构建设置 `-DCONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=N` 选择支持的编号。连续 Active 3 秒后仅触发一次，暂停持续 1.5 秒；实时遥控停机与清故障始终优先，诊断不伪造新的来源时间戳。停机后重新解锁；诊断计时器不会跟随被暂停的执行器停住。

普通构建：`west build -b dm_mc02/stm32h723xx samples/robotics/yaw_gimbal -d ../build/yaw_gimbal_rc`。未进行本轮实板验收；接收机拨轮编码与机械方向需实物确认。
