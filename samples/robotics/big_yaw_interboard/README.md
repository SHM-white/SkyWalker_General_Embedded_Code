# 无 IMU 的双板大 Yaw 遥控

云台板接收 DR16 拨轮，经 USART1 板间协议 v4 发送速度请求；底盘板使用现有 `BigYawExecutor` 驱动真实大 Yaw，并回传实际速度和执行状态。两端均不启动 IMU。云台板不驱动小 Yaw/Pitch，底盘板不启动四舵轮。此入口用于手动速度调试，不执行双 Yaw 自动回中或惯性稳定。

## 接线

两块 MC02 使用相同版本的本样例，分别构建下述角色。

| 连接 | 配置 |
| --- | --- |
| DR16 → 云台角色板 | remote-uart / UART5，100000 baud、8E1 |
| 云台 PA9 / USART1 TX → 底盘 PA10 / USART1 RX | 460800 baud |
| 底盘 PA9 / USART1 TX → 云台 PA10 / USART1 RX | 460800 baud，双板共地 |
| 大 Yaw → 底盘角色板 | `../common/chassis_can.hpp` 的 `big_yaw_can`，当前为 **CAN3**，1 Mbps |
| 各板日志 | USART10，115200 baud |

电机配置复用 `include/robotics/vehicle/calibration.hpp` 和 `big_yaw_profile.hpp`：当前为 DM J4310 MIT，ID 2、Master ID 0x12，方向、减速比、力矩限幅及速度 PID 沿用中央配置。`connections_confirmed` 控制底盘电机初始化，`imu_mounting_confirmed` 不参与本入口。底盘启动日志会打印实际 CAN 设备名、ID 和初始化错误；若之后修改中央 CAN 映射，也要同步 overlay 的控制器配置。

## 构建和烧录

在 `skywalker_code` 仓库根目录，使用已配置 Zephyr 的终端执行。先连接云台角色板并烧录：

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/big_yaw_interboard -d build/big_yaw_interboard_gimbal
west flash -d build/big_yaw_interboard_gimbal
```

再连接底盘角色板并烧录：

```sh
west build -b dm_mc02/stm32h723xx samples/robotics/big_yaw_interboard -d build/big_yaw_interboard_chassis -- -DEXTRA_CONF_FILE=chassis.conf
west flash -d build/big_yaw_interboard_chassis
```

不需要 `pitch_free.conf`、`power.conf` 或 IMU 配置。两块板均连接调试器时，应通过烧录器参数明确选择目标。

## 操作与预期日志

1. 确认电机 ID、MIT 参数、方向和实际 CAN 接线与中央配置一致，给大 Yaw 转动及线束留出空间。
2. 双板上电，双方日志应出现 `online=1`，底盘初始化 `setup=0`。DR16 只接云台板。
3. 遥控双拨杆 Down，摇杆和拨轮全部归中保持 500 ms，再拨左 Middle 解锁。
4. 拨轮控制正负 **0.1 rad/s**（约 5.7°/s），归中请求零速度；左 Down 停止，左 Up 禁用。右拨杆没有速度或发射功能。
5. 云台日志观察 `rc/run/target` 和 `peerActual/peerReady/peerValid`；底盘观察 `operator/run/target/actual/ready/valid/state/reason/error/commit`。`online=1` 只证明通信在线，`valid=0` 时速度数值不能作为真实反馈使用。

速度幅值在 `src/main.cpp` 的 `manual_rate_max_rad_s`，单位为输出关节 rad/s；电机侧继续受中央速度和力矩限幅约束。

## 停止、失联与恢复

沿用 100 ms 原始遥控输入有效期和板间输入有效期；重发不会刷新旧命令年龄。遥控失联、管理授权失效、板间请求过期或断线都会撤销大 Yaw 输出。通信线程独立运行，电机掉线或未就绪不阻止速度请求与心跳发送。

检测到板间断线或对端重启后，云台撤销本地解锁，需要重新双 Down 归中 500 ms 后左 Middle。单独电机/CAN 恢复不会要求重新遥控解锁，在输入和链路仍有效时执行最新目标。保留公共遥控清除事件手势：安全档归中建立基线后右 Up 保持 1 秒，再重新解锁；旧事件不绑定到重启后的新板会话。

本样例使用显式手动台架许可，不依赖裁判权限，也不向轮控发送运动命令。`FollowCenter` 仅复用当前协议的速度请求模式，本入口不调用回中算法。真实速度、方向、失联停止延迟与实机恢复仍需上板验证。
