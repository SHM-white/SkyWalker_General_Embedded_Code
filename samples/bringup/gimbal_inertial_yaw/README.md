# gimbal_inertial_yaw

本工程自行装配并运行头部 IMU 辅助的小 Yaw 单轴惯性保持；所有硬件参数来自 `include/robotics/vehicle/gimbal_profile.hpp` 和共享实车 `config/vehicle/installed/gimbal.overlay`，不依赖公共 `run()` 或角色、锁轴宏。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/gimbal_inertial_yaw -d ../build/bringup/gimbal_inertial_yaw
west flash -d ../build/bringup/gimbal_inertial_yaw
```

UART5 接 DR16，CAN1 接小 Yaw，CAN2 接 Pitch（单 Yaw 入口不注册 CAN2 电机），CAN3 专供头部 IMU；诊断使用共享 `diagnostic-uart`（当前候选为 UART7 115200，DAPLink 实物接线仍需确认）。本阶段仅构造、注册小 Yaw 电机与 CAN1 总线，Pitch 电机不会启动；CAN3 IMU 为主动上报四元数模式。

将双拨杆置 Down、摇杆和拨轮归中保持至少 500 ms，再把左拨杆推到 Middle，右拨杆保持 Down。右杆 X 请求 Yaw，右杆 Y 请求 Pitch；左拨杆 Down 或遥控过期立即撤销授权。上电不会自动动作。只控制 Yaw，生成的关节速度限制为 1.0 rad/s。IMU 的姿态和陀螺仪必须都在 20 ms 内；失效即关闭电机。适配器锁住 Pitch，未提供虚构 Pitch 反馈；适配器内部可能报告 Pitch 缺反馈，但只按独立的 yaw_output_valid 决定 Yaw 输出。

仅修改 `include/robotics/vehicle/diagnostics_config.hpp` 中 `diagnostic_output` 后重建、烧录，即可选择日志或 VOFA。JustFloat 布局版本 1 共 19 通道：layout、seq低16位、seq高16位、uptime低16位、uptime高16位、reason、valid、enabled、target、feedback、age_ms、dt_us、effort、detail、error、dropped低16位、dropped高16位、extra0、extra1。`age_ms=UINT32_MAX` 表示缺少有效时间戳；始终结合 valid/原因码判断，不能将保留的历史反馈当作实时零速。target/feedback 是小 Yaw 关节角 rad；effort 为 Yaw 电流 A；age_ms 为头部姿态年龄；detail 为惯性适配器 WaitReason；extra0 为头部 yaw rad，extra1 为 yaw 误差 rad。

原因码：0 Boot，1 Ready，2 InitFailed，3 RemoteStale，4 Disarmed，5 FeedbackStale，6 ImuStale，7 PeerStale，8 Configuration，9 Permission，10 CommandStale。初始化错误保留在日志事件与 error 通道，诊断队列拥塞只丢观测数据。

此处记录的是入口已实现；尚未声明编译通过、接线确认或实板动作通过。上车先支撑机构、使用小幅输入，记录撤销输入、相关反馈断流和恢复后的当前命令行为；Pitch 需要考虑重力支撑。确认阈值、日志位置和可退回固件由现场记录填写。
