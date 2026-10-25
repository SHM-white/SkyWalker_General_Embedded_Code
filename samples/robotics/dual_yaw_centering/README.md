# 双板双 Yaw 兼容样例

更新日期：2026-10-25。本目录保留“同一main+角色宏/额外conf”的历史教学和兼容用途，尚未迁移到共享installed overlay与DAPLink日志/VOFA输出。实际逐模块上车使用 [云台双Yaw](../../bringup/gimbal_dual_yaw/README.md) 配 [底盘大Yaw](../../bringup/chassis_big_yaw/README.md)，先完成双方 [板间观察](../../bringup/README.md)。新旧入口的串口映射和线上方向适配应分别确认，不能混用默认连线。

本兼容样例默认云台角色，底盘角色控制独立大Yaw速度环。使用中央calibration的方向、减速比、限幅和小Yaw中心；电机输出仍受connections_confirmed控制。云台端目前仍通过 `../inertial_gimbal/src/board_config.hpp` 取得兼容配置，该头已转接共享 `gimbal_profile.hpp`；跨sample头引用尚保留，不是新的上车配置依赖方式。中央connections_confirmed与imu_mounting_confirmed当前为true，不能据此登记为实板已通过。

默认启用 `CONFIG_DUAL_YAW_ENCODER_ONLY=y`，不需要连接或启动头部IMU，也不要求imu_mounting_confirmed。右横杆控制Yaw速率；大Yaw跟随小Yaw偏离中心的误差，小Yaw用大Yaw实际速度反馈做反向补偿，松杆后逐渐回中。它仅用于固定底盘的编码器台架，不能抵消底盘自身转动，也不保证对地惯性朝向。方向、减速比和中心必须与机构一致。

编码器模式保留遥控解锁、输入时效、机械限位、电机反馈和板间通信保护。大Yaw速度反馈无效或超过50 ms时，回中请求速度清零；仅在反馈有效性变化时重置控制器，避免每轮清空周期时钟造成永久禁止使能。零速FollowCenter请求允许大Yaw先使能取得反馈，避免启动互等。恢复反馈后从零斜坡继续回中。小Yaw仍可使用遥控独立运动；Pitch默认锁定关节目标，不响应右纵杆，电机配置沿用原样。

双板USART10交叉TX/RX并共地，460800 baud、8N1；云台小Yaw/Pitch使用CAN1/CAN2，底盘大Yaw使用中央chassis_can分配的CAN3。DR16只接云台板。各板CAN编号互相独立。

| 用途 | 外设 | 接线 | 波特率 |
| --- | --- | --- | --- |
| 板间通信 | USART10 | 云台PE3 TX接底盘PE2 RX，云台PE2 RX接底盘PE3 TX，两板共地 | 460800，8N1 |
| 每块板的日志 | USART1 | PA9 TX接USB串口适配器RX，并共地 | 115200，8N1 |

本旧样例overlay将console和shell都指定到USART1，USART10独占板间协议；移除本样例的telemetry-uart别名，避免日志端口再被遥测占用。USART1原来的DMAMUX通道3/4转给USART10，TX/RX请求号分别为119/118；USART1日志使用轮询输出。配置只作用于本样例，两种板卡角色共用同一overlay。MC02板级别名usart2指向USART10，uart1则指向UART7，接线以外设名和引脚为准。

使用本兼容样例的两种角色时，双方都按上表重新编译、烧录并核对接线。其底盘板间端口也是USART10；新 `chassis_big_yaw` 的共享底盘基线则使用USART1板间、UART7诊断，不能将本页的两端USART10接法直接套用。当前头部IMU实现已改为CAN3，下文惯性构建不再需要RS485-2；overlay中遗留的RS485注释不代表当前驱动类型。本样例只有文本日志，不受共享diagnostic_output变量切换。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/dual_yaw_centering -d build/dual_yaw_gimbal
west build -b dm_mc02/stm32h723xx samples/robotics/dual_yaw_centering -d build/dual_yaw_chassis -- -DEXTRA_CONF_FILE=chassis.conf
```

启用本旧样例的头部惯性路径时，连接云台CAN3头部IMU，完成安装方向、坐标变换和参考确认，并核对中央imu_mounting_confirmed。当前Source为DmImuCanSource，采用共享headSensorConfig的主动上报四元数模式（feedback_id=0x11、slave_id=1），不创建RS485接收器；底盘角色的大Yaw在其自身CAN3，两板控制器编号互不冲突。云台构建时添加 `-- -DCONFIG_DUAL_YAW_ENCODER_ONLY=n` 恢复原惯性模式；该模式仍要求真实有效的头部IMU，不会在IMU掉线后自动降级。复用构建目录时，切回编码器模式需显式传入 `-DCONFIG_DUAL_YAW_ENCODER_ONLY=y`。

双方使用协议v4，完整字节、单位和时效合同见 [v4规范](../../../docs/protocols/interboard-v4.md)，消息没有resume_generation。本兼容样例直接使用自身回中方向逻辑，未套用新bringup工程的线上反号阶段策略；切换入口时应以实际配对方向为准。实际板启动身份由boot ID绑定，电机掉线不改变通信上下文。目标与原始source年龄在转发时累加，通信重发不续期；相同producer sequence不刷新接收命令时间。

启动后持续计算并发送大Yaw请求；编码器模式在缺少底盘速度反馈时发送零速请求。云台两轴和底盘大Yaw分别恢复；提交错误只记诊断，不能触发controls.withdraw或全机构复位。惯性模式由惯性适配器分别传递两轴output_valid。

物理操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)。OperatorControl承载真实运行意图、停止、急停和绑定boot的急停解除事件。输入过期或链路失联按输入有效期停止；电机掉电恢复自动继续最新目标。旧急停解除事件不重绑定到新的板启动身份。

诊断支持1输入、2执行、3状态暂停，普通构建不注入。日志显示source、requested、真实run/wait、测量和各CAN提交错误，不再显示恢复授权generation。两板应同批更新v4。

本文同步当前源码事实，不新增编译或实板通过结论。尚需实板记录回中方向、中心、斜坡、头部误差、停止延迟、输入断流和电机反复断电恢复。
