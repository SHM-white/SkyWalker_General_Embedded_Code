# 双板双 Yaw 回中

云台角色默认构建，底盘角色只控制独立大 Yaw。输出确认默认关闭。先完成 `big_yaw`、`inertial_gimbal` 与中央 calibration 中的方向、减速比、限幅及独立小 Yaw 中心标定。只有头部保持有效、小云台实际 Active、底盘大 Yaw 反馈新鲜就绪，才开放回中。

USART1 双板交叉 TX/RX 并共地，console 只输出日志；云台小 Yaw/Pitch 使用 CAN1/CAN2，底盘独立大 Yaw 使用 CAN3，头部 IMU 使用 RS485-2。仅云台板连接 DR16。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/dual_yaw_centering -d build/dual_yaw_gimbal
west build -b dm_mc02/stm32h723xx samples/robotics/dual_yaw_centering -d build/dual_yaw_chassis -- -DEXTRA_CONF_FILE=chassis.conf
west build -b dm_mc02/stm32h723xx samples/robotics/dual_yaw_centering -d build/dual_yaw_diagnostic -- -DEXTRA_CONF_FILE=diagnostic.conf
```

双方只接受唯一当前协议标识 3，不协商旧协议或能力。大 Yaw 使用本次 boot、新鲜有效执行反馈和独立恢复代次建立上下文，轮控代次不能授权大 Yaw。原始输入与执行状态年龄在转发时累加，不因重传刷新。

物理操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)：安全档归中 500 ms 再左 Middle 解锁；左 Down 停机，Safe 手势单次复位，左 Up 禁用。独立 OperatorControl 将停机和绑定目标 boot 的清故障事件传到底盘，不随业务输入/状态/执行暂停停止。会话错误、过期或缺失的管理输入禁止底盘输出。

诊断构建支持输入、执行、状态暂停（1/2/3）。稳定运行 3 秒后暂停 1.5 秒，管理和通信继续；遥控 Safe 或失联取消演练。清除事件只消费一次，板卡重启后旧事件不能改绑新 boot。console 字母控制已移除。

TODO(实板)：记录回中方向、死区/迟滞、速度/斜坡、头部误差、停止延迟、断流、单板及双板重启，以及恢复后原始来源序号和独立代次。
