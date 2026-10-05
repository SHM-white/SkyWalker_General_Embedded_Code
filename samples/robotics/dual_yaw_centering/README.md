# 双板双 Yaw 持续回中

默认云台角色，底盘角色控制独立大Yaw速度环。使用中央calibration的方向、减速比、限幅和小Yaw中心，输出确认默认关闭。回中算法需要有效关节角和头部参考；大Yaw自己的ready/armed/valid不作为目标产生条件。

双板USART1交叉TX/RX并共地；云台小Yaw/Pitch使用CAN1/CAN2，底盘大Yaw使用中央chassis_can分配的CAN2，头部IMU使用RS485-2。DR16只接云台板。各板CAN编号互相独立。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/dual_yaw_centering -d build/dual_yaw_gimbal
west build -b dm_mc02/stm32h723xx samples/robotics/dual_yaw_centering -d build/dual_yaw_chassis -- -DEXTRA_CONF_FILE=chassis.conf
```

双方使用唯一协议v4，消息没有resume_generation。实际板启动身份由boot ID绑定，电机掉线不改变通信上下文。目标与原始source年龄在转发时累加，通信重发不续期；相同producer sequence不刷新接收命令时间。

启动后持续计算并发送大Yaw请求，不等待底盘电机上线。云台两轴和底盘大Yaw分别恢复；提交错误只记诊断，不能触发controls.withdraw或全机构复位。惯性适配器分别传递两轴output_valid。

物理操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)。OperatorControl承载真实运行意图、停止、急停和绑定boot的急停解除事件。输入过期或链路失联按输入有效期停止；电机掉电恢复自动继续最新目标。旧急停解除事件不重绑定到新的板启动身份。

诊断支持1输入、2执行、3状态暂停，普通构建不注入。日志显示source、requested、真实run/wait、测量和各CAN提交错误，不再显示恢复授权generation。两板应同批更新v4。

尚需实板记录回中方向、中心、斜坡、头部误差、停止延迟、输入断流和电机反复断电恢复。
