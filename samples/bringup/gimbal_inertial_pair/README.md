# gimbal_inertial_pair

本工程自行装配并运行头部 IMU 辅助的小 Yaw + Pitch 双轴惯性闭环；所有硬件参数来自 `include/robotics/vehicle/gimbal_profile.hpp` 和共享实车 `config/vehicle/installed/gimbal.overlay`，不依赖公共 `run()` 或角色、锁轴宏。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/gimbal_inertial_pair -d ../build/bringup/gimbal_inertial_pair
west flash -d ../build/bringup/gimbal_inertial_pair
```

UART5 接 DR16，CAN1 接小 Yaw，CAN2 接 Pitch（单 Yaw 入口不注册 CAN2 电机），CAN3 专供头部 IMU；诊断使用共享 `diagnostic-uart`（当前候选为 UART7 115200，DAPLink 实物接线仍需确认）。CAN3 IMU 为主动上报四元数模式，本阶段不接大 Yaw。

将双拨杆置 Down、摇杆和拨轮归中保持至少 500 ms，再把左拨杆推到 Middle，右拨杆保持 Down。右杆 X 请求 Yaw，右杆 Y 请求 Pitch；左拨杆 Down 或遥控过期立即撤销授权。上电不会自动动作。惯性适配器关节速度限制为 Yaw 1.0 rad/s、Pitch 0.8 rad/s。姿态和陀螺仪必须都在 20 ms 内；失效即撤销两轴命令。安装变换必须通过现场验证，现有 mounting 确认值不等于本次已实测。

仅修改 `include/robotics/vehicle/diagnostics_config.hpp` 中 `diagnostic_output` 后重建、烧录，即可选择日志或 VOFA。JustFloat 布局版本 1 共 19 通道：layout、seq低16位、seq高16位、uptime低16位、uptime高16位、reason、valid、enabled、target、feedback、age_ms、dt_us、effort、detail、error、dropped低16位、dropped高16位、extra0、extra1。`age_ms=UINT32_MAX` 表示缺少有效时间戳；始终结合 valid/原因码判断，不能将保留的历史反馈当作实时零速。target/feedback 是小 Yaw 关节角 rad；effort 为 Yaw 电流 A；age_ms 为头部姿态年龄；detail 为执行器 WaitReason；extra0/extra1 为头部 yaw/pitch rad。

原因码：0 Boot，1 Ready，2 InitFailed，3 RemoteStale，4 Disarmed，5 FeedbackStale，6 ImuStale，7 PeerStale，8 Configuration，9 Permission，10 CommandStale。初始化错误保留在日志事件与 error 通道，诊断队列拥塞只丢观测数据。

此处记录的是入口已实现；尚未声明编译通过、接线确认或实板动作通过。上车先支撑机构、使用小幅输入，记录撤销输入、相关反馈断流和恢复后的当前命令行为；Pitch 需要考虑重力支撑。确认阈值、日志位置和可退回固件由现场记录填写。
