# 双板双 Yaw 持续回中

默认云台角色，底盘角色控制独立大Yaw速度环。使用中央calibration的方向、减速比、限幅和小Yaw中心；电机输出仍受connections_confirmed控制。

默认启用 `CONFIG_DUAL_YAW_ENCODER_ONLY=y`，不需要连接或启动头部IMU，也不要求imu_mounting_confirmed。右横杆控制Yaw速率；大Yaw跟随小Yaw偏离中心的误差，小Yaw用大Yaw实际速度反馈做反向补偿，松杆后逐渐回中。它仅用于固定底盘的编码器台架，不能抵消底盘自身转动，也不保证对地惯性朝向。方向、减速比和中心必须与机构一致。

编码器模式保留遥控解锁、输入时效、机械限位、电机反馈和板间通信保护。大Yaw速度反馈无效或超过50 ms时，回中请求速度清零；仅在反馈有效性变化时重置控制器，避免每轮清空周期时钟造成永久禁止使能。零速FollowCenter请求允许大Yaw先使能取得反馈，避免启动互等。恢复反馈后从零斜坡继续回中。小Yaw仍可使用遥控独立运动；Pitch默认锁定关节目标，不响应右纵杆，电机配置沿用原样。

双板USART10交叉TX/RX并共地，460800 baud、8N1；云台小Yaw/Pitch使用CAN1/CAN2，底盘大Yaw使用中央chassis_can分配的CAN3。DR16只接云台板。各板CAN编号互相独立。

| 用途 | 外设 | 接线 | 波特率 |
| --- | --- | --- | --- |
| 板间通信 | USART10 | 云台PE3 TX接底盘PE2 RX，云台PE2 RX接底盘PE3 TX，两板共地 | 460800，8N1 |
| 每块板的日志 | USART1 | PA9 TX接USB串口适配器RX，并共地 | 115200，8N1 |

样例overlay将console和shell都指定到USART1，USART10独占板间协议；移除本样例的telemetry-uart别名，避免日志端口再被遥测占用。USART1原来的DMAMUX通道3/4转给USART10，TX/RX请求号分别为119/118；USART1日志使用轮询输出。配置只作用于本样例，两种板卡角色共用同一overlay。MC02板级别名usart2指向USART10，uart1则指向UART7，接线以外设名和引脚为准。

更换串口后两块板都需要重新编译、烧录，并将板间连线移到PE3/PE2；日志适配器移到PA9和GND。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/dual_yaw_centering -d build/dual_yaw_gimbal
west build -b dm_mc02/stm32h723xx samples/robotics/dual_yaw_centering -d build/dual_yaw_chassis -- -DEXTRA_CONF_FILE=chassis.conf
```

接回头部IMU后，连接云台RS485-2，完成安装方向和坐标变换确认，再设置中央imu_mounting_confirmed。云台构建时添加 `-- -DCONFIG_DUAL_YAW_ENCODER_ONLY=n` 恢复原惯性模式；该模式仍要求真实有效的头部IMU，不会在IMU掉线后自动降级。复用构建目录时，切回编码器模式需显式传入 `-DCONFIG_DUAL_YAW_ENCODER_ONLY=y`。

双方使用唯一协议v4，消息没有resume_generation。实际板启动身份由boot ID绑定，电机掉线不改变通信上下文。目标与原始source年龄在转发时累加，通信重发不续期；相同producer sequence不刷新接收命令时间。

启动后持续计算并发送大Yaw请求；编码器模式在缺少底盘速度反馈时发送零速请求。云台两轴和底盘大Yaw分别恢复；提交错误只记诊断，不能触发controls.withdraw或全机构复位。惯性模式由惯性适配器分别传递两轴output_valid。

物理操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)。OperatorControl承载真实运行意图、停止、急停和绑定boot的急停解除事件。输入过期或链路失联按输入有效期停止；电机掉电恢复自动继续最新目标。旧急停解除事件不重绑定到新的板启动身份。

诊断支持1输入、2执行、3状态暂停，普通构建不注入。日志显示source、requested、真实run/wait、测量和各CAN提交错误，不再显示恢复授权generation。两板应同批更新v4。

尚需实板记录回中方向、中心、斜坡、头部误差、停止延迟、输入断流和电机反复断电恢复。
