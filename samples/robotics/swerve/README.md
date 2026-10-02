# 单舵轮双 CAN 遥控台架

GM6020 舵向与 M3508 轮驱动使用 `board_config.hpp` / `chassis_can.hpp` 分配的两条物理 CAN，并属于同一故障组。机械参数取中央标定 FL 配置，控制参数复用 `../common/chassis_bench.hpp`。默认 `connections_confirmed=false` 阻止输出；先确认 ID、电流模式、舵向零点、方向、减速比、轮径、限流和机械安装。

遥控器使用板级 `remote-uart`，MC02 为 UART5/PD2，100000 baud、8E1、RX DMA；接收机与主控共地。启用第五通道拨轮，不使用遥控键鼠或终端输入。控制台和 VOFA 仍可用于观测。

上电保持左、右开关都在 Down，四个摇杆轴和拨轮归中至少 0.5 秒，再将左开关拨到 Middle 解锁。左 Down 随时停机，左 Up 在本台架不启用 Auto。失联超过 100 ms、执行器运行中故障或失能后，必须重新完成安全档归中与解锁。清除故障：先完成双 Down 归中 0.5 秒，保持左 Down，将右开关拨到 Up 且全部通道归中 1 秒；清除只发生一次，不会自动使能。

遥控目标保留接收帧的原始时间戳和序号。重复读取快照不会续期，也不需要手动以控制周期重复发送字母。硬件接线、校准和现有电流/力矩保护仍是运行前提。

左纵杆控制前后速度 ±0.1 m/s，左横杆控制左右速度 ±0.1 m/s（向右为负），拨轮控制偏航速度 ±0.2 rad/s。可同时给平移与旋转；归中使用既有零速度 Coast/Hold 配置。只把四轮运动学中的 FL 目标交给真实单模块，完整八电机使用 `four_swerve`。

恢复时重新采集绝对舵角与轮速反馈，重建连续参考，等待两轴的新反馈，再重置模块与运动学。`RecoveryGate` 保留原始 RC 年龄并拒绝恢复边界前的输入。反馈与暂态驱动故障的准备流程保留，但已运行机构失去授权后仍需操作员重新解锁；硬故障需显式清除。两条 CAN 在周期末各提交一次，任一成员/总线故障撤销同组。

普通构建 `CONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=0`，不提供运行时故障菜单。独立诊断构建：

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/swerve -d ../build/swerve_rc_diagnostic -- -DEXTRA_CONF_FILE=diagnostic.conf
```

`diagnostic.conf` 选择 1（暂停命令生产）。支持场景：1 输入暂停、2 执行暂停。也可为同一诊断构建设置 `-DCONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=N` 选择支持的编号。连续 Active 3 秒后仅触发一次，暂停持续 1.5 秒；实时遥控停机与清故障始终优先，诊断不伪造新的来源时间戳。停机后重新解锁；诊断计时器不会跟随被暂停的执行器停住。

普通构建：`west build -b dm_mc02/stm32h723xx samples/robotics/swerve -d ../build/swerve_rc`。控制继续使用绝对周期节拍与 deferred logging；日志记录来源年龄、恢复代次、等待原因、舵向限速目标、对齐、翻转、卸力和电流。未进行本轮实板方向、时延或性能验收。
