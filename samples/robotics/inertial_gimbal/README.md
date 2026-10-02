# 头部惯性云台实板入口

RemoteReceiver → RemoteSource → CommandManager → InertialGimbalAdapter → GimbalExecutor，头部姿态来自真实的 DmImuRs485Source / ImuReceiver。应用线程每 5 ms 执行一次，两条物理 CAN 分别统一提交；默认配置锁定 Pitch，仅验证载体手动旋转时的小 Yaw 补偿。

接线为 MC02 CAN1 小 Yaw GM6020、CAN2 Pitch DM4310、UART5 遥控、RS485-2 头部 IMU、USART1 VOFA。电机标定复用 `command_gimbal`，IMU 安装变换和确认开关使用 `include/robotics/vehicle/calibration.hpp`。连接与 IMU 确认默认关闭；TODO 标明尚需测量的变换、方向、限位、参数和质量策略。

构建：

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/inertial_gimbal -d build/inertial_gimbal
west build -b dm_mc02/stm32h723xx samples/robotics/inertial_gimbal -d build/inertial_gimbal_pitch -- -DEXTRA_CONF_FILE=pitch_free.conf
```

完成机械云台和 `dual_imu` 标定后，先锁 Pitch、低速手动转大 Yaw 载体，再使用 `pitch_free.conf`。VOFA 仅输出遥测；物理遥控使用统一的安全档解锁、停机与复位手势，诊断构建选择输入/执行/状态/头部暂停场景（1/2/3/4）；暂停读取头部快照保留原始生产时间，让适配器自然判定断流。遥控恢复、参考变化和执行器恢复代次变化都建立新会话，之后的新源输入才可再次使能。头部误差合格且机械执行器 Active 时才向后续回中逻辑报告稳定。

TODO(实板验收)：记录源序号、头部原始年龄、参考代次、两级恢复代次、角误差、实际机械反馈、周期超限、输出撤销延迟及断流恢复。尚未取得实板验收记录。

操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)。第五通道已启用，左 Up 在本例禁用。`diagnostic.conf` 默认选择输入暂停：稳定 Active 3 秒后暂停 1.5 秒，遥控管理始终继续运行。
