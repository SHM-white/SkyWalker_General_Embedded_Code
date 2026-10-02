# 双主控整车框架

`applications/sentry_gimbal`、`applications/sentry_chassis` 与 `samples/robotics/vehicle_integration` 共用板级运行时。独立样例先验证各机构，applications 再组合双板；接线确认默认关闭，机械参数与安装变换集中在 `include/robotics/vehicle/calibration.hpp`。

## 线程与设备归属

~~~text
云台板
  输入 worker → CommandManager → CommandSnapshot
  执行线程（5 ms）
    头部 IMU → InertialGimbalAdapter → GimbalExecutor
    小 Yaw 机械反馈 + 头部稳定状态 → YawCenteringController
    发射请求 + 实际权限/热量/云台状态 → ShooterExecutor
    所有机构暂存输出 → DJI CAN1 / Pitch CAN2 各提交一次
    执行状态与大 Yaw 请求 → 快照缓存
  通信线程 → InterBoardEndpoint V2 → UART1

底盘板
  通信线程 → InterBoardEndpoint V2 → 接收快照
  执行线程（5 ms）
    底盘命令 → ChassisExecutor → SwerveChassis → 八电机
    大 Yaw 请求 → BigYawExecutor → DM 速度内环
    CAN1 舵向 / CAN2 轮驱动 / CAN3 大 Yaw 各提交一次
    轮控与大 Yaw 独立状态 → 快照缓存
~~~

只有执行线程设置电机目标和提交 CAN。通信线程只传递快照、推进 endpoint；CommandManager 的消费者不会互相取走命令。云台组、摩擦组、拨盘、八电机轮控组和大 Yaw 分别管理故障与恢复；物理总线失效影响该总线全部成员。

## 云台板组合

默认手动阶段使用遥控来源；`VEHICLE_REFEREE` 接入真实裁判权限，`VEHICLE_VISION_OBSERVE` 只观察视觉目标，`VEHICLE_VISION_EXECUTE` 才参与执行。视觉与惯性控制消费头部 IMU 的实际参考会话。小 Yaw 的编码器角仍是机械关节角，惯性目标先经过适配层再进入机械执行器。

回中外环根据独立标定的小 Yaw 中心产生大 Yaw 角速度请求，并检查头部稳定、命令新鲜度与权限。请求携带底盘 boot ID、大 Yaw 独立恢复代次、生产序号和年龄。底盘轮控恢复不会授权大 Yaw；版本或能力不匹配时大 Yaw 保持禁用。

`VEHICLE_SHOOTING` 接入摩擦轮和拨盘。单发事件具有独立编号和生产时间；恢复、忙碌或条件不满足时的旧事件被丢弃。连发持续检查新鲜请求、摩擦轮就绪、真实热量、权限和云台状态。拨盘归位与热量测量的实际来源仍需接入，TODO 未完成时有载发射不能通过准备条件。

## 底盘板组合

`ChassisExecutor` 注入应用持有的 `SwerveHardware`；四舵向共用 CAN1，四轮驱动共用 CAN2。大 Yaw 使用独立 DM 电机与 CAN3，通过速度控制允许连续旋转，无需固定绝对机械零点。

`VEHICLE_POWER_BUDGET` 接入裁判预算，但预算不是实测功率。现有 V1 约束没有实测功率字段；整车框架预留 `IPowerMeasurementSource`，实际传感器与功率模型完成标定后才能开放完整功率控制。独立 `chassis_power` 样例提供测量与缩放观察入口。

## 配置与上机顺序

阶段配置、接线与构建命令见 `samples/robotics/vehicle_integration/README.md`。默认配置均保留中央接线门禁；解除前须填写电机 ID、方向、减速比、零点、机械限位、IMU 安装变换和低功率参数。

从 west workspace 根目录构建两应用：

~~~sh
west build -p -b dm_mc02/stm32h723xx -d build/sentry-gimbal skywalker_code/applications/sentry_gimbal
west build -p -b dm_mc02/stm32h723xx -d build/sentry-chassis skywalker_code/applications/sentry_chassis
~~~

先通过惯性云台与大 Yaw 单轴，再运行双 Yaw 回中；随后接入四舵轮、功率和发射。编译通过只说明框架可构建，实物性能与故障恢复按实施指南逐级记录。
