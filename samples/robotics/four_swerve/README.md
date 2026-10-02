# 四舵轮遥控悬空台架

创建 FL/FR/RL/RR 四个 GM6020 舵向与四个 M3508 轮驱动，组成一个八电机故障组。CAN1 接舵向、CAN3 接轮驱动，各自 ID1..4；大 Yaw 不属于此组。中央 `calibration.hpp` 的接线确认默认关闭，参考零位 3060/2421/2383/3097、方向 -/+/-/+ 和减速比 3591/187 均需按实物确认，几何尺寸不得沿用未经测量的参考值。

遥控器使用板级 `remote-uart`，MC02 为 UART5/PD2，100000 baud、8E1、RX DMA；接收机与主控共地。启用第五通道拨轮，不使用遥控键鼠或终端输入。控制台和 VOFA 仍可用于观测。

上电保持左、右开关都在 Down，四个摇杆轴和拨轮归中至少 0.5 秒，再将左开关拨到 Middle 解锁。左 Down 随时停机，左 Up 在本台架不启用 Auto。失联超过 100 ms、执行器运行中故障或失能后，必须重新完成安全档归中与解锁。清除故障：先完成双 Down 归中 0.5 秒，保持左 Down，将右开关拨到 Up 且全部通道归中 1 秒；清除只发生一次，不会自动使能。

遥控目标保留接收帧的原始时间戳和序号。重复读取快照不会续期，也不需要手动以控制周期重复发送字母。硬件接线、校准和现有电流/力矩保护仍是运行前提。

左纵杆控制前后 ±0.1 m/s，左横杆控制左右 ±0.1 m/s（向右为负），拨轮控制偏航 ±0.2 rad/s。平移与旋转可叠加；归中请求零速度，保留 profile 的 Coast/Hold 策略。共享 95°/85°翻转滞回、4 rad/s 舵角限速、0.10/0.40 rad 对齐门控、cos 补偿与 0.01/0.02 m/s 低速滞回；所有非零模块对齐后才放行驱动。

执行器不持有 CAN。应用每周期暂存全部目标后对两条 CAN 各提交一次；故障撤销整个八电机组，反馈恢复后重建机械参考并等待恢复边界后的新 RC 帧。使用 2 ms 绝对节拍、迟到跳过时隙和 deferred logging，每 500 ms 输出四轮状态与电流。电流/功率保护和校准门保持有效。

普通构建 `CONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=0`，不提供运行时故障菜单。独立诊断构建：

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/four_swerve -d ../build/four_swerve_rc_diagnostic -- -DEXTRA_CONF_FILE=diagnostic.conf
```

`diagnostic.conf` 选择 1（暂停命令生产）。支持场景：1 输入暂停、2 执行暂停、3 状态发布暂停。也可为同一诊断构建设置 `-DCONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=N` 选择支持的编号。连续 Active 3 秒后仅触发一次，暂停持续 1.5 秒；实时遥控停机与清故障始终优先，诊断不伪造新的来源时间戳。停机后重新解锁；诊断计时器不会跟随被暂停的执行器停住。

普通构建：`west build -b dm_mc02/stm32h723xx samples/robotics/four_swerve -d ../build/four_swerve_rc`。方向、停机时延、CAN 故障恢复和机械性能仍需实板确认；本轮未执行实板操作。
