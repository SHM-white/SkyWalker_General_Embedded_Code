# 02 架构与构建

## 1. 分层

```text
applications/、samples/
        │  业务编排、线程、设备树 alias、板级参数
        ▼
lib/robotics              命令、安全、舵轮、Yaw、功率限幅
lib/communication         UART、DR16、裁判、板间协议
lib/control               PID、前馈、运动控制内核、电机统一封装
        │
        ▼
drivers/                  Zephyr device、IMU、Kalman、DJI/DM CAN
        │
        ▼
Zephyr device API / CAN / UART / SPI / PWM / CMSIS-DSP / STM32 HAL
```

设计边界是：驱动负责设备状态和协议；`lib/control` 负责数值控制与电机生命周期；`lib/communication` 负责字节流到消息快照；`lib/robotics` 负责消息之间的策略和输出许可；应用负责线程、真实硬件绑定和最终安全决策。

## 2. 仓库布局

| 目录 | 责任 |
|---|---|
| `boards/` | `dm_mc02`、`rm_typec` 的 board metadata、DTS、defconfig、烧录器 |
| `dts/bindings/` | `skywalker,*`、`dji,*`、`dm,*` binding |
| `drivers/` | Zephyr 设备驱动；电机驱动为 C++，IMU/Kalman 为 C |
| `include/` | 公共 API；路径与命名空间对应实现层 |
| `lib/control/` | 无动态分配的 C 控制内核和 C++ 电机后端 |
| `lib/communication/` | 固定容量 UART 消费、协议编解码和快照服务 |
| `lib/robotics/` | 机器人决策、局部安全和运动学 |
| `samples/` | 可单独 west build 的验证项目，各自拥有配置和入口 |
| `applications/` | 双主控底盘/云台应用骨架，不是库的替代品 |
| `application/` | 当前为空的历史占位目录 |

## 3. west module 关系

`west.yml` 锁定 Zephyr revision，并只导入白名单依赖。`zephyr/module.yml` 将本仓库声明为 module，同时提供：

- `board_root: .`：Zephyr 能发现 `boards/`。
- `dts_root: .`：Zephyr 能发现 `dts/bindings/`。
- 根 `CMakeLists.txt`：将 `drivers/` 与 `lib/` 加入应用构建。
- 根 `Kconfig`：递归包含 `drivers/Kconfig` 和 `lib/Kconfig`。

因此应用只需打开 Kconfig，并在 overlay 中实例化节点；不需要手动把模块源文件复制进 sample。

## 4. Kconfig → CMake 门控

### 驱动

```text
SKYWALKER_DRIVER_KALMAN_FILTER ── drivers/kalman_filter
SKYWALKER_DRIVER_IMU            ── drivers/imu
SKYWALKER_DRIVER_MOTOR
  ├── SKYWALKER_MOTOR_DJI       ── drivers/motor/dji
  └── SKYWALKER_MOTOR_DM        ── drivers/motor/dm
```

### 库

```text
SKYWALKER_LIB_MATRIX            ── CMSIS-DSP 矩阵
SKYWALKER_LIB_VOFA              ── VOFA+ 协议
SKYWALKER_LIB_CONTROL           ── lib/control
SKYWALKER_LIB_MOTOR_CONTROL     ── 统一速度/位置电机封装
SKYWALKER_LIB_COMMUNICATION      ── lib/communication
SKYWALKER_LIB_ROBOTICS           ── lib/robotics
```

通信和机器人库再由更细的选项控制：`SKYWALKER_REMOTE_DR16`、`SKYWALKER_REFEREE`、`SKYWALKER_INTERBOARD`、`SKYWALKER_UART_TRANSPORT`、`SKYWALKER_ROBOTICS_COMMAND`、`SKYWALKER_ROBOTICS_SAFETY`、`SKYWALKER_ROBOTICS_SWERVE`、`SKYWALKER_ROBOTICS_GIMBAL`。

`SKYWALKER_LIB_MOTOR_CONTROL` 要求 C++、电机驱动和至少一族电机，并自动选择 `SKYWALKER_LIB_CONTROL`。配置修改后应在 `build/<name>/zephyr/.config` 中确认最终值。

## 5. 两条主要数据流

### 电机控制

```text
CAN feedback
  → motor device cache
  → MotorBackend::read
  → MotorRuntime freshness / safety / dt
  → C PID / feedforward kernel
  → MotorBackend::write
  → Bus::flush
```

原生 Bus 可以拥有同一 CAN 上的多个电机；统一 `MotorBackend` 目前按单电机独占 Bus 设计。多轴应用要在应用层创建共享 Bus 或使用已有的 `DjiChassisHardware`。

### 双主控消息

```text
UART async callback
  → AsyncUart 固定队列
  → 单一通信线程 read/service
  → InterBoardParser / RemoteService / RefereeParser
  → typed snapshot + timestamp/sequence
  → robotics safety / command manager
  → local actuator and/or InterBoardCodec
```

回调不运行控制算法；应用线程消费值拷贝。队列丢失或代际变化必须让解析器丢弃半帧，避免把跨越溢出的字节拼成合法命令。

## 6. 线程与并发约定

- `AsyncUart` 的 callback 只复制 RX chunk、维护 DMA buffer 和发送完成标志。
- `InterBoardLink`、`RefereeParser`、`RemoteService` 规定由一个通信线程拥有。
- 电机 Bus、backend 和 wrapper 默认由一个线程调用，不能从 ISR 调用，也不能让两个线程同时 flush。
- `applications/sentry_chassis` 将链路线程与底盘线程分开，通过 `Latest<T>` 交换值拷贝；`sentry_gimbal` 类似地拆分遥控、裁判、命令、板间和云台线程。
- C++ 对象应静态存活到应用结束，因为 CAN callback 和驱动的所有权在 stop 后仍可能存在。

## 7. 单位与状态

| 数据 | 单位 |
|---|---|
| 位置、角度 | rad |
| 速度 | rad/s |
| DJI effort | A |
| DM MIT effort | N·m |
| 时间 API | ms 或 s，按函数名/字段后缀区分 |
| 设备树电机限幅 | `current-limit-ma`、`torque-limit-millinewton-meter` |

不要把 `State`（电机设备 Offline/Ready/Fault）、`BusState`（总线生命周期）和 `ExecutionState`（统一封装 Waiting/Recovering/Ready/Active 等）混为一谈。

## 8. 常见设计误区

1. 只改 `prj.conf` 不改 overlay：不会生成电机设备。
2. 只改 overlay 不打开 Kconfig：驱动源文件可能根本没有加入构建。
3. 把 `drivers/pid`（如果外部工程提供设备型 PID）和 `lib/control/pid.h` 的纯算法 PID 当成同一个 API。
4. 用旧文档里的 `cpp_test`、`dm_common` 等已删除目录；当前电机样例路径以 `rg --files samples/motor` 为准。
5. 在 UART callback 里直接 arm 电机或执行重控制；这会把中断/回调时序和控制周期耦合起来。
