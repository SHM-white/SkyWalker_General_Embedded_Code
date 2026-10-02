# 底盘功率遥控台架

复用 `four_swerve` 的八电机双 CAN 拓扑，输入真实裁判底盘权限、功率上限、缓冲能量和测得功率。裁判 UART 取 `referee-uart`（本样例 USART1 PA10 RX / PA9 TX），显式使用 RM2026 V1.3；与 RC 专用 UART 分开。协议版本、机械连接、功率模型和控制标定需按实板完成，默认确认标志不放行电机。

遥控器使用板级 `remote-uart`，MC02 为 UART5/PD2，100000 baud、8E1、RX DMA；接收机与主控共地。启用第五通道拨轮，不使用遥控键鼠或终端输入。控制台和 VOFA 仍可用于观测。

上电保持左、右开关都在 Down，四个摇杆轴和拨轮归中至少 0.5 秒，再将左开关拨到 Middle 解锁。左 Down 随时停机，左 Up 在本台架不启用 Auto。失联超过 100 ms、执行器运行中故障或失能后，必须重新完成安全档归中与解锁。清除故障：先完成双 Down 归中 0.5 秒，保持左 Down，将右开关拨到 Up 且全部通道归中 1 秒；清除只发生一次，不会自动使能。

遥控目标保留接收帧的原始时间戳和序号。重复读取快照不会续期，也不需要手动以控制周期重复发送字母。硬件接线、校准和现有电流/力矩保护仍是运行前提。

左纵杆前后 ±0.1 m/s、左横杆左右 ±0.1 m/s（向右为负）、拨轮偏航 ±0.2 rad/s。底盘轨迹、翻轮、对齐门控、Coast/Hold、双 CAN 提交及 2 ms 绝对节拍与 `four_swerve` 一致。

`IPowerMeasurementSource` 可接真实 ADC/功率监测器；默认裁判测量保留原始生产时间。没有真实测量或裁判帧时不伪造有效权限、预算或功率。任一约束过期撤销输出，恢复仍需新鲜数据和重新遥控解锁。估计功率回退保持禁用；台架电流上限和功率控制标定门不受遥控改造影响。

普通构建 `CONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=0`，不提供运行时故障菜单。独立诊断构建：

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/chassis_power -d ../build/chassis_power_rc_diagnostic -- -DEXTRA_CONF_FILE=diagnostic.conf
```

`diagnostic.conf` 选择 1（暂停命令生产）。支持场景：1 输入暂停、2 执行暂停、3 状态发布暂停、6 裁判许可暂停、7 功率测量暂停。也可为同一诊断构建设置 `-DCONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=N` 选择支持的编号。连续 Active 3 秒后仅触发一次，暂停持续 1.5 秒；实时遥控停机与清故障始终优先，诊断不伪造新的来源时间戳。停机后重新解锁；诊断计时器不会跟随被暂停的执行器停住。

`measurement_diagnostic.conf` 选择场景 7：继续更新真实裁判权限、预算，只保留旧测量及其时间戳，观察过期停机。可用 `-DEXTRA_CONF_FILE=measurement_diagnostic.conf` 构建该版本。场景 6 仅冻结许可快照，其他裁判数据继续接收；场景 3 仅冻结发布给日志的执行状态。

普通构建：`west build -b dm_mc02/stm32h723xx samples/robotics/chassis_power -d ../build/chassis_power_rc`。独立功率传感器、模型、机械阈值和真实撤销时延尚待实板验收。
