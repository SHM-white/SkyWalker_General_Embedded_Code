# 头部惯性云台持续控制入口

`RemoteReceiver → CommandManager → InertialGimbalAdapter → GimbalExecutor`，头部姿态来自真实的DmImuRs485Source / ImuReceiver。应用每5ms推进，物理CAN分别提交。默认锁Pitch，先观察手动转动载体时的小Yaw补偿。

MC02接线：CAN1小Yaw GM6020、CAN2 Pitch DM4310、UART5遥控、RS485-2头部IMU、USART1 VOFA。电机配置复用command_gimbal，IMU安装变换来自中央calibration。连接和IMU确认默认关闭。

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/inertial_gimbal -d build/inertial_gimbal
west build -b dm_mc02/stm32h723xx samples/robotics/inertial_gimbal -d build/inertial_gimbal_pitch -- -DEXTRA_CONF_FILE=pitch_free.conf
```

惯性目标在电机恢复期间保留。适配器逐轴报告yaw_output_valid/pitch_output_valid，执行器分别推进有效轴；Motor和CAN状态不作为全机构准入条件。缺少本轴机械测量或头部数学参考时只等待相应计算。输出恢复后本轴控制历史自动重置，不建立恢复授权代次。

VOFA16通道依次为头部Yaw/Pitch、两角误差、两目标角速度、yaw计算输出有效性、实际Active轴数、RunState、WaitReason、稳定标志、头部年龄、控制耗时、周期超限和两CAN错误。日志显示运行意图、活动/等待轴数及提交错误。

诊断支持1输入、2执行、3状态、4头部暂停。暂停读取仍保留原始测量时间；源过期会停止对应目标，正常设备恢复不要求新启动。头部稳定依据真实计算，不依赖Group全员Active。

先完成机械云台与dual_imu标定，再释放Pitch。物理操作见 [统一遥控操作](../common/REMOTE_CONTROL.md)。尚无本次方向、参考、停止延迟和断电恢复的实板记录。
