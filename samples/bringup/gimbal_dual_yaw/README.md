# gimbal_dual_yaw

本工程自行装配并运行本地惯性双轴与远端大 Yaw 回中协同；所有硬件参数来自 `include/robotics/vehicle/gimbal_profile.hpp` 和共享实车 `config/vehicle/installed/gimbal.overlay`，不依赖公共 `run()` 或角色、锁轴宏。

```sh
west build -b dm_mc02/stm32h723xx samples/bringup/gimbal_dual_yaw -d ../build/bringup/gimbal_dual_yaw
west flash -d ../build/bringup/gimbal_dual_yaw
```

UART5 接 DR16，CAN1 接小 Yaw，CAN2 接 Pitch（单 Yaw 入口不注册 CAN2 电机），CAN3 专供头部 IMU；诊断使用共享 `diagnostic-uart`（当前候选为 UART7 115200，DAPLink 实物接线仍需确认）。USART10 460800 板间线连接独立 `chassis_big_yaw` 工程；不发送底盘平移目标，也不启动本地大 Yaw 电机。

将双拨杆置 Down、摇杆和拨轮归中保持至少 500 ms，再把左拨杆推到 Middle，右拨杆保持 Down。右杆 X 请求 Yaw，右杆 Y 请求 Pitch；左拨杆 Down 或遥控过期立即撤销授权。上电不会自动动作。本地惯性关节速度限制为 Yaw 1.0 rad/s、Pitch 0.8 rad/s；回中速度上限使用共享上车策略 `bringup::big_yaw_rate_rad_s`，与独立大 Yaw 接收入口一致；最终整车回中上限由独立整车策略保留。大 Yaw 线上方向保留 v4 当前适配的 -1 符号，仅真实反馈新鲜时发送非零回中目标；无实测反馈时最多发送已授权零速 FollowCenter 请求。对端掉线或重启撤销本地授权，需要重新归中解锁。

对参考底盘（`JJL0612/sentry_onlychassis`）另有两处差异，由 `reference_chassis_link_profile.hpp` 统一切换：其一，板载 **BMI088**（`accel0`/`gyro0` + `QuaternionEkf`）安装在大 Yaw 随动机构上，提供大 Yaw 世界系航向；小 Yaw 惯性控制仍由 CAN3 头部 DM IMU 完成。其二，参考底盘的陀螺补偿后端将 `mode=1` 的 `target_rate` 解释为**世界系角速度意图**，而非本仓库原生大 Yaw 关节相对速度，不能直接发送 `YawCenteringController` 的输出，也不能额外乘 `big_yaw_wire_direction`。

兼容模式现在采用 **2.5 s 滑动窗口的世界系目标**：`小Yaw世界角 = 板载BMI088世界航向 + 经方向和中心标定的小Yaw相对角`。对该角度进行 20 ms 采样的圆周平均，得到大 Yaw 需要追随的世界朝向，按 `0.8 × 最短角误差` 转成角速度意图，最高 **0.35 rad/s**，加速度限制 **0.6 rad/s²**，停止死区 **0.005 rad**。双 Yaw 共同转动时目标仍是世界系旧朝向；停止操控后历史窗口逐步更新，大 Yaw 最终追到小 Yaw 朝向，小 Yaw 回到机械中心。初次解锁窗口不足时使用已有的有效采样，不等待完整 2.5 s。

这是测试控制策略而非已经实车验收的参数。前提是板载 BMI088 确实装在**大 Yaw 随动体**上、编码器正方向与 BMI088 世界 Yaw 正方向一致，并测量 `yaw_center_rad` 对应小 Yaw 与大 Yaw 的真正物理 0°；当前标定中心还是限位中点候选，不能拿编码器零位或中点自动当作物理对正。方向错误会造成正反馈，应先悬空、低速验证。

参考底盘依然由其自身处理底盘陀螺仪角速度补偿。板间头部姿态扩展仍发送板载 BMI088 航向，另外补发零速 `BodyVelocity` 的 `ChassisControl`，只用于放行参考底盘的 `RobotBigYawControl()`。头部惯性无效、板载航向过期、关节反馈过期、对端实测速度无效或撤权时，非零跟随请求立即停止；对端未反馈前仍允许已授权零速使能请求。参考底盘的 BigYawFeedback 固定每 100 ms 上报一次，因此兼容模式接收过期门限使用 250 ms（与其电机内部 100 ms 实际反馈保护相互独立）；本仓库原生模式仍用 50 ms。原生底盘模式继续使用原有机械关节回中，暂不改变线上语义。

仅修改 `include/robotics/vehicle/diagnostics_config.hpp` 中 `diagnostic_output` 后重建、烧录，即可选择日志或 VOFA。JustFloat 布局版本 1 共 19 通道：layout、seq低16位、seq高16位、uptime低16位、uptime高16位、reason、valid、enabled、target、feedback、age_ms、dt_us、effort、detail、error、dropped低16位、dropped高16位、extra0、extra1。`age_ms=UINT32_MAX` 表示缺少有效时间戳；始终结合 valid/原因码判断，不能将保留的历史反馈当作实时零速。target/feedback 为线上大 Yaw 关节速度 rad/s；feedback 无效时为 NaN，valid=false；effort 为本地小 Yaw 电流 A；age_ms 为接收时携带的远端生产年龄，判断还叠加本地接收后年龄；detail 为原有机械回中 WaitReason；**参考底盘兼容模式**下 extra0 为滑动平均目标相对大 Yaw 世界航向的误差 rad，extra1 为平均世界目标航向 rad；**本仓库原生底盘模式**下 extra0 仍是关节中心误差 rad，extra1 为头部 yaw rad。

原因码：0 Boot，1 Ready，2 InitFailed，3 RemoteStale，4 Disarmed，5 FeedbackStale，6 ImuStale，7 PeerStale，8 Configuration，9 Permission，10 CommandStale。初始化错误保留在日志事件与 error 通道，诊断队列拥塞只丢观测数据。

此处记录的是入口已实现；尚未声明编译通过、接线确认或实板动作通过。上车先支撑机构、使用小幅输入，记录撤销输入、相关反馈断流和恢复后的当前命令行为；Pitch 需要考虑重力支撑。确认阈值、日志位置和可退回固件由现场记录填写。
