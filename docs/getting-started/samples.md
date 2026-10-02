# 10 样例索引

`samples/` 中的每个目录都是独立 Zephyr 应用，拥有自己的 `CMakeLists.txt`、`prj.conf`、overlay 和入口。样例用于功能验证和台架观察，不等同于完整机器人固件。

## 1. 基础与算法

| 样例 | 作用 | 主要前提 |
|---|---|---|
| `samples/hello` | 最小启动、UART/VOFA 单通道 | 任一已支持板卡 |
| `samples/control` | PID、前馈、角度、斜坡等控制算法自检 | 任一已支持板卡 |
| `samples/imu_test` | 新接口的单板载 BMI088、EKF、温控、VOFA | MC02 板载传感器与 TIM3 CH4 加热 |
| [samples/imu/dual_imu](../../samples/imu/dual_imu/README.md) | 底盘/载体/头部安装位置观测，原始时间、质量、参考代次及 VOFA | MC02；云台配置另需 DM-IMU-L1 |

```bash
west build -p -b dm_mc02/stm32h723xx -d build/hello samples/hello
west build -p -b dm_mc02/stm32h723xx -d build/control samples/control
west build -p -b dm_mc02/stm32h723xx -d build/imu samples/imu_test
```

## 2. 通信样例

| 样例 | 作用 | 操作/观察 |
|---|---|---|
| `samples/communication/dr16` | RemoteReceiver 接收与 18 字节 DR16 解码 | MC02 UART5 接收，独立 VOFA UART 显示 16 通道；console 查看初始化与异常 |
| [samples/communication/vision](../../samples/communication/vision/README.md) | AB 视觉指令独立接收与 VOFA 回显 | MC02 UART7 115200 输入，USART1 输出 |
| `samples/communication/referee` | 裁判串口 CRC 和状态解析 | MC02 USART1，观察权限、功率、buffer 和统计量 |
| `samples/communication/interboard` | 两块板 UART 板间协议 | 一块默认 Gimbal role，另一块加 `chassis.conf` |

通信样例都只验证消息链路，不自动控制电机。样例级说明还在各自的 `README.md`。

```bash
west build -p -b dm_mc02/stm32h723xx -d build/dr16 samples/communication/dr16
west build -p -b dm_mc02/stm32h723xx -d build/referee samples/communication/referee
west build -p -b dm_mc02/stm32h723xx -d build/interboard-gimbal samples/communication/interboard
west build -p -b dm_mc02/stm32h723xx -d build/interboard-chassis \
  -DEXTRA_CONF_FILE=samples/communication/interboard/chassis.conf \
  samples/communication/interboard
```

两块板互连时使用 TX→RX、RX→TX 和共地；不要把两个板的同名 TX 直接并联。

## 3. DJI 电机

| 样例 | 作用 |
|---|---|
| `samples/motor/can_smoke` | 只接收 CAN 帧并输出观察信息，不发送命令 |
| `samples/motor/dji_unified` | GM6020 `Motor`/`CanBus` 低电流台架 |
| `samples/motor/dji_speed_control` | DJI `VelocityMotor` 速度闭环 |
| `samples/motor/dji_position_control` | DJI `PositionMotor` 位置-速度串级，推荐参考 |
| `samples/motor/m2006_speed_control` | M2006 + C610，含 36:1 减速比示例 |

```bash
west build -p -b dm_mc02/stm32h723xx -d build/can-smoke samples/motor/can_smoke
west build -p -b dm_mc02/stm32h723xx -d build/dji-speed samples/motor/dji_speed_control
west build -p -b rm_typec -d build/dji-position samples/motor/dji_position_control
```

上机前必须核对 `src/main.cpp` 中的 motor ID、CAN、零点、减速比和限流；GM6020 还必须确认 current loop。先悬空、低电流，再验证正负方向。

## 4. 达妙电机

| 样例 | 控制层 |
|---|---|
| `samples/motor/dm_mit_control` | 原生 MIT 力矩 |
| `samples/motor/dm_velocity_control` | 电机内置速度模式 |
| `samples/motor/dm_position_control` | 电机内置位置-速度模式 |
| `samples/motor/dm_mit_velocity_control` | MIT + SkyWalker 软件速度环 |
| `samples/motor/dm_mit_position_control` | MIT + SkyWalker 软件位置-速度环 |
| `samples/motor/recovery` | DJI 或 DM MIT 掉电/总线恢复台架 |
| `samples/motor/mixed_topology` | DJI 同帧独立组、DM 共用 Master ID、跨 CAN 联动组与独立组故障隔离 |

```bash
west build -p -b dm_mc02/stm32h723xx -d build/dm-mit \
  samples/motor/dm_mit_control
west build -p -b dm_mc02/stm32h723xx -d build/dm-position \
  samples/motor/dm_mit_position_control
```

DM 的 C++ 模式、Master ID、PMAX/VMAX/TMAX 必须和电机实际设置一致。掉电恢复样例只切电机动力，不切 MCU 电源；反馈恢复后还需再次按 `e` 显式使能。

## 5. 机器人算法台架

| 样例 | 作用 | 是否驱动电机 |
|---|---|---|
| `samples/robotics/command_safety` | RemoteSource → CommandManager 保险与恢复观察 | 否 |
| [samples/robotics/command_manager](../../samples/robotics/command_manager/README.md) | 遥控/视觉/裁判三源仲裁服务观测 | 否，仅消息观测 |
| [samples/robotics/execution_skeleton](../../samples/robotics/execution_skeleton/README.md) | 真实命令服务、独立消费者、生产停更与自动恢复 | 否，执行器模拟 |
| `samples/robotics/yaw_gimbal` | GM6020 Yaw，Hold/Rate/AbsoluteAngle | 是，单电机 |
| `samples/robotics/swerve` | 单物理舵轮：GM6020 舵向 + M3508 驱动 | 是，单模块 |
| `samples/robotics/gimbal_control` | DJI + DM 双轴 Group 云台台架 | 是，双轴联动，默认连接未配置 |
| [samples/robotics/command_gimbal](../../samples/robotics/command_gimbal/README.md) | CommandManager → 小 Yaw/Pitch 双轴机械执行器，统一总线提交 | 是，双 CAN，默认连接未配置 |
| [samples/robotics/inertial_gimbal](../../samples/robotics/inertial_gimbal/README.md) | 头部 IMU → 惯性适配 → 机械双轴，锁 Pitch/释放 Pitch 配置 | 是，安装确认默认关闭 |
| [samples/robotics/gimbal_shared_can](../../samples/robotics/gimbal_shared_can/README.md) | 小云台、双摩擦轮、拨盘共享 DJI CAN，独立故障组 | 是，无弹空载，默认连接未配置 |
| [samples/robotics/big_yaw](../../samples/robotics/big_yaw/README.md) | 大 Yaw 独立 MIT 软件速度环，连续旋转 | 是，默认连接未配置 |
| [samples/robotics/dual_yaw_centering](../../samples/robotics/dual_yaw_centering/README.md) | 双板头部惯性保持、中心外环与大 Yaw 内环 | 是，双角色，默认连接未配置 |
| [samples/robotics/four_swerve](../../samples/robotics/four_swerve/README.md) | 四舵四驱、完整 SwerveChassis、双 CAN 统一提交 | 是，默认连接未配置 |
| [samples/robotics/chassis_power](../../samples/robotics/chassis_power/README.md) | 裁判预算、真实功率测量合同和八电机执行器 | 是，完整功率标定默认关闭 |
| [samples/robotics/shooter_bench](../../samples/robotics/shooter_bench/README.md) | 摩擦就绪、拨盘分度、单发/连发、恢复丢弃和卡滞 | 是，默认空载；loaded 条件缺失时禁用 |
| [samples/robotics/vehicle_integration](../../samples/robotics/vehicle_integration/README.md) | 双板手动 → 视觉 → 发射阶段，复用机构执行器 | 是，默认连接未配置 |

典型构建：

```bash
west build -p -b dm_mc02/stm32h723xx -d build/command-safety \
  samples/robotics/command_safety
west build -p -b dm_mc02/stm32h723xx -d build/yaw \
  samples/robotics/yaw_gimbal
west build -p -b dm_mc02/stm32h723xx -d build/swerve \
  samples/robotics/swerve
```

`dr16`、`command_safety`、`gimbal_control` 统一使用 `RemoteReceiver`；后两者分别保留 10 ms 命令循环和 5 ms 双轴控制循环。DMA 声明和协议参数由各 sample 提供，接收循环由库管理。接入与返回值见 [13 通信](../modules/communication/communication.md)。

键盘操作和默认 GPIO/CAN 配置见各 sample README 及 `src/board_config.hpp`。

新增入口、配置变体、后续里程碑和实板验收记录见[逐级整车验证指南](../dev/项目优化与逐级整车验证样例实施指南.md)。上述索引描述代码入口，实板通过状态须以验收记录为准；单舵轮、视觉接收和历史设计方案不表示完成整车闭环。

## 6. applications 与 samples 的区别

| 路径 | 状态 |
|---|---|
| `samples/` | 独立小项目，目标是验证一个驱动/协议/算法 |
| `applications/sentry_chassis` | 复用整车运行入口：四舵轮与大 Yaw 独立执行和恢复，默认连接未配置 |
| `applications/sentry_gimbal` | 复用整车运行入口：惯性云台、回中、仲裁和发射约束，默认连接未配置 |

整机应用的配置和线程关系见 [15 应用骨架](../applications/dual-controller.md)；从遥控到执行器的调用顺序见 [模块联动](../applications/module-integration.md)。

中央标定入口为 `include/robotics/vehicle/calibration.hpp`。所有新入口的源码 TODO 指向硬件端口、模式、机械参数、IMU 安装、索引/热量来源和实测性能；框架入口存在不代表实板通过。`command_gimbal` 另有裁判权限、视觉观察和视觉执行三种配置，视觉执行必须经过头部惯性参考适配。
