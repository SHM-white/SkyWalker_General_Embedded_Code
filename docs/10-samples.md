# 10 样例索引

`samples/` 中的每个目录都是独立 Zephyr 应用，拥有自己的 `CMakeLists.txt`、`prj.conf`、overlay 和入口。样例用于功能验证和台架观察，不等同于完整机器人固件。

## 1. 基础与算法

| 样例 | 作用 | 主要前提 |
|---|---|---|
| `samples/hello` | 最小启动、UART/VOFA 单通道 | 任一已支持板卡 |
| `samples/control` | PID、前馈、角度、斜坡等控制算法自检 | 任一已支持板卡 |
| `samples/imu_test` | BMI088、EKF、恒温 PWM、VOFA | `dm_mc02`，需按 overlay 接好 IMU/加热 |

```bash
west build -p -b dm_mc02/stm32h723xx -d build/hello samples/hello
west build -p -b dm_mc02/stm32h723xx -d build/control samples/control
west build -p -b dm_mc02/stm32h723xx -d build/imu samples/imu_test
```

## 2. 通信样例

| 样例 | 作用 | 操作/观察 |
|---|---|---|
| `samples/communication/dr16` | 18 字节 DR16 解码 | MC02 UART5，日志显示通道、拨杆、鼠标、键盘和 offline |
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
| `samples/motor/dji_unified` | GM6020 原生通用 API / 低电流安全台架 |
| `samples/motor/dji_speed_control` | DJI `VelocityMotor` 速度闭环 |
| `samples/motor/dji_position_control` | DJI `PositionMotor` 位置-速度串级，推荐参考 |
| `samples/motor/m2006_speed_control` | M2006 + C610，含 36:1 减速比示例 |

```bash
west build -p -b dm_mc02/stm32h723xx -d build/can-smoke samples/motor/can_smoke
west build -p -b dm_mc02/stm32h723xx -d build/dji-speed samples/motor/dji_speed_control
west build -p -b rm_typec -d build/dji-position samples/motor/dji_position_control
```

上机前必须替换 overlay 中的 motor ID、CAN、零点、减速比和限流；GM6020 还必须确认 current loop。先悬空、低电流，再验证正负方向。

## 4. 达妙电机

| 样例 | 控制层 |
|---|---|
| `samples/motor/dm_mit_control` | 原生 MIT 力矩 |
| `samples/motor/dm_velocity_control` | 电机内置速度模式 |
| `samples/motor/dm_position_control` | 电机内置位置-速度模式 |
| `samples/motor/dm_mit_velocity_control` | MIT + SkyWalker 软件速度环 |
| `samples/motor/dm_mit_position_control` | MIT + SkyWalker 软件位置-速度环 |
| `samples/motor/recovery` | DJI 或 DM MIT 掉电/总线恢复台架 |

```bash
west build -p -b dm_mc02/stm32h723xx -d build/dm-mit \
  samples/motor/dm_mit_control
west build -p -b dm_mc02/stm32h723xx -d build/dm-position \
  samples/motor/dm_mit_position_control
```

DM 的 `control-mode`、`master-id`、PMAX/VMAX/TMAX 必须和电机实际设置一致。掉电恢复样例只切电机动力，不要切 MCU 电源，否则无法验证自动恢复。

## 5. 机器人算法台架

| 样例 | 作用 | 是否驱动电机 |
|---|---|---|
| `samples/robotics/command_safety` | DR16 → intent → global safety → command | 否，模拟底盘心跳/反馈 |
| `samples/robotics/yaw_gimbal` | GM6020 Yaw，Hold/Rate/AbsoluteAngle | 是，单电机 |
| `samples/robotics/swerve` | 单物理舵轮：GM6020 舵向 + M3508 驱动 | 是，单模块 |

典型构建：

```bash
west build -p -b dm_mc02/stm32h723xx -d build/command-safety \
  samples/robotics/command_safety
west build -p -b dm_mc02/stm32h723xx -d build/yaw \
  samples/robotics/yaw_gimbal
west build -p -b dm_mc02/stm32h723xx -d build/swerve \
  samples/robotics/swerve
```

键盘操作和默认 GPIO/CAN 配置见各 sample README 及 `src/board_config.hpp`。

## 6. applications 与 samples 的区别

| 路径 | 状态 |
|---|---|
| `samples/` | 独立小项目，目标是验证一个驱动/协议/算法 |
| `applications/sentry_chassis` | 四轮舵底盘双线程应用骨架，默认连接未配置 |
| `applications/sentry_gimbal` | 云台主控双主控应用骨架，默认连接未配置 |
| `application/` | 空历史占位，不是当前推荐入口 |

整机应用的配置和线程关系见 [15 应用骨架](15-applications.md)。
