# 头部惯性云台兼容样例

更新日期：2026-10-25。本入口已迁移到共享实车配置、CAN头部IMU和共享诊断；仍保留 `CONFIG_INERTIAL_GIMBAL_LOCK_PITCH` 与额外conf的旧样例选择方式。实际逐模块上车推荐使用独立的 [头部IMU观察](../../bringup/gimbal_head_imu/README.md)、[惯性Yaw单轴](../../bringup/gimbal_inertial_yaw/README.md)、[惯性双轴](../../bringup/gimbal_inertial_pair/README.md)，以目录固定职责，见 [上车入口表](../../bringup/README.md)。

当前数据流为 `RemoteReceiver → CommandManager → InertialGimbalAdapter → GimbalExecutor`，真实姿态来自 `DmImuCanSource → ImuReceiver`。应用约每5ms推进，两条电机CAN分别提交；默认锁Pitch，先观察手动转动载体时的小Yaw补偿。默认锁Pitch仍会构造、注册两轴电机；需要仅启动Yaw电机时使用上面的独立Yaw工程。

## 配置与设备

本入口的CMake固定加载 `config/vehicle/installed/base.conf`、`gimbal.conf` 和本地 `prj.conf`；`app.overlay` 包含共享 `gimbal.overlay`。`src/board_config.hpp` 直接转接 [gimbal_profile.hpp](../../../include/robotics/vehicle/gimbal_profile.hpp)，不再引用另一个sample的board_config，也不维护另一份PID。ID、方向、零点、限位、参考和安装变换来自 [中央标定](../../../include/robotics/vehicle/calibration.hpp)。

| 资源 | 当前分配 |
| --- | --- |
| 小Yaw | CAN1，GM6020，ID7，电流模式 |
| Pitch | CAN2，J4310 MIT，ID1、Master0x00 |
| 遥控 | UART5 / remote-uart |
| 头部IMU | CAN3 / imu-can，DmImuCanSource |
| 诊断 | diagnostic-uart，当前共享候选UART7，115200，PE8 TX / PE7 RX |

头部配置使用共享 `headSensorConfig()`：主动上报、四元数、feedback_id=`0x11`、slave_id=1，安装变换和参考来自中央标定。当前不创建RS485 Source或 `head_uart`，不能再将CAN设备传给UART驱动。CAN3由头部IMU独占；本入口不启动板间链路。UART7的DAPLink实际接线尚需按现场确认，不能仅凭软件别名认定已接通。

中央 `connections_confirmed`、`imu_mounting_confirmed` 当前均为true；这是已有配置值，不代表本次已完成接线、IMU安装和机械验收。修改标定后应重新构建受影响的阶段和最终应用。

```sh
# 兼容样例：默认锁Pitch
west build -b dm_mc02/stm32h723xx samples/robotics/inertial_gimbal -d ../build/legacy/inertial_gimbal
# 兼容样例：释放Pitch，保留其原有构建方式
west build -b dm_mc02/stm32h723xx samples/robotics/inertial_gimbal -d ../build/legacy/inertial_gimbal_pitch -- -DEXTRA_CONF_FILE=pitch_free.conf
```

## 控制与操作

先完成单轴机械方向、零点、限位和头部IMU安装参考确认。双Down、摇杆和拨轮归中保持至少500ms，再拨左Middle启动；左Down或原始输入过期撤销运行授权。更完整的物理操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)。释放Pitch前支撑其重力负载。

惯性目标在电机恢复期间保留。适配器逐轴报告yaw_output_valid/pitch_output_valid，执行器分别推进有效轴；缺少本轴机械测量或头部数学参考时等待相应计算，恢复后重置本轴控制历史，不建立恢复授权代次。本入口的惯性控制策略与新的独立阶段策略分别配置，不能仅凭共用硬件参数认定输出上限和全部门控相同。

旧样例诊断场景支持1输入、2执行、3状态、4头部暂停，默认不注入。暂停读取保留原始测量时间；输入过期仍按原年龄处理。实际上车使用独立阶段入口，不需要为阶段选择叠加这些诊断宏。

## 共享日志与VOFA

只修改 [diagnostics_config.hpp](../../../include/robotics/vehicle/diagnostics_config.hpp) 的 `diagnostic_output`，重新编译、烧录后即可选择Log文本或Vofa JustFloat。两种输出都使用共享诊断UART，主机同一时刻只打开一个COM口程序；不依赖目标板USB，不再使用USART1作为本入口的VOFA出口。VOFA模式无文本混流，诊断队列满只丢观测数据，不阻塞控制。

JustFloat布局版本1共19通道，下表编号从0开始。本兼容样例的target和feedback物理量不同，应分别按各自单位读图。

| 通道 | 内容 | 单位/有效性 |
| --- | --- | --- |
| 0 | 布局版本 | 1 |
| 1 / 2 | 诊断序号低/高16位 | 合成u32 |
| 3 / 4 | 本板运行时间低/高16位 | ms |
| 5 | DiagnosticReason | 见共享diagnostics.hpp |
| 6 / 7 | valid / enabled | valid要求头部年龄<100ms且yaw_output_valid；enabled表示执行Active |
| 8 | 惯性适配后Yaw角速度命令 | rad/s |
| 9 | 头部yaw角 | rad；valid=false时为NaN |
| 10 | 头部姿态年龄 | ms；无有效时间戳为-1 |
| 11 | 本次控制计算耗时 | 微秒；此处不是相邻控制周期dt |
| 12 | effort | 本入口未填，保留0，不代表实测零电流 |
| 13 | 执行器WaitReason | 数值枚举 |
| 14 | 初始化或当前执行错误 | 错误码 |
| 15 / 16 | 诊断丢包数低/高16位 | 合成u32 |
| 17 | 头部pitch角 | rad，结合头部新鲜度读取 |
| 18 | pitch误差 | rad，结合适配器状态读取 |

日志包含应用身份、源码/硬件/标定/协议版本、初始化步骤及低频状态。初始化失败也保留诊断路径。本文同步当前实现，不新增方向、参考、停止延迟或恢复行为的实板通过结论。
