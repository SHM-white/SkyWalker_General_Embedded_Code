# 10 样例索引

`samples/` 下的每个目录都是独立可构建的 Zephyr 应用。除特别说明外，构建
命令都在 west 工作区根执行。

---

## 1. 总览

| 样例 | 语言 | 板卡 | 需要硬件 | 用途 |
|---|---|---|---|---|
| `hello/` | C | 任意 | 串口 | 最小工程 + VOFA 单通道（uptime） |
| `cpp_test/` | C | 任意 | 串口 | 最小工程冒烟（当前实际是 C 版 hello） |
| `control/` | C | 任意 | 可选 VOFA | `lib/control` 算法自检（断言输出） |
| `imu_test/` | C | `dm_mc02` | IMU + UART | IMU + EKF + 恒温加热 + VOFA |
| `motor/can_smoke/` | C++ | 任意有 CAN | CAN 总线 | 只收 CAN 帧并转发到 VOFA，不驱动电机 |
| `motor/dji_unified/` | C++ | 任意有 CAN | GM6020 | 开环电流老炼/安全测试 |
| `motor/dji_speed_control/` | C++ | `dm_mc02` | GM6020 | `VelocityMotor` 速度环 |
| `motor/dji_position_control/` | C++ | `dm_mc02`/`rm_typec` | GM6020 | `PositionMotor` 位置环（推荐参考） |
| `motor/m2006_speed_control/` | C++ | `dm_mc02` | M2006+C610 | M2006 速度环（减速比 36:1） |
| `motor/dm_mit_control/` | C++ | `dm_mc02` | DM-J4310 | MIT 开环力矩 |
| `motor/dm_velocity_control/` | C++ | `dm_mc02` | DM-J4310 | DM 原生速度模式 |
| `motor/dm_position_control/` | C++ | `dm_mc02` | DM-J4310 | DM 原生位置-速度模式 |
| `motor/dm_mit_velocity_control/` | C++ | `dm_mc02` | DM-J4310 | `VelocityMotor` + DM 后端 |
| `motor/dm_mit_position_control/` | C++ | `dm_mc02` | DM-J4310 | `PositionMotor` + DM 后端 |

`motor/dm_common/` 不是样例，而是 DM 样例共享的 **支持库**
（`dm_sample_support.hpp/.cpp`，含 `enableMotorPower` 等）。

---

## 2. 基础样例

### hello

最小可运行工程，同时演示 VOFA：在 `usart1` 上每 5 s 发一个 JustFloat 通道
（`k_uptime_get_32()`）。

```bash
west build -p -b dm_mc02/stm32h723xx -d build/hello samples/hello
```

### cpp_test

与 `hello` 同级的工具链冒烟工程（注意目录名虽是 `cpp_test`，源码是
`src/main.c`，`project(hello)`）。用于确认构建环境。

```bash
west build -p -b dm_mc02/stm32h723xx -d build/cpp_test samples/cpp_test
```

### control

`lib/control` 的自检：构造已知输入，断言复合前馈 PID、斜坡限幅、角度工具
的输出在容差内，并通过 VOFA 输出曲线。**不需要电机**。

```bash
west build -p -b dm_mc02/stm32h723xx -d build/control samples/control
```

`prj.conf`：`CONFIG_SKYWALKER_LIB_CONTROL=y`、`CONFIG_SKYWALKER_LIB_VOFA=y`、
`CONFIG_UART_ASYNC_API=y`。

### imu_test

IMU 全链路：BMI088 采样 → EKF 姿态 → 恒温加热 → VOFA 绘图。overlay 在
`samples/imu_test/boards/dm_mc02.overlay`（含 `skywalker,imu` 与
`skywalker,kalman_filter` 节点、TIM3_CH4 加热 PWM）。

```bash
west build -p -b dm_mc02/stm32h723xx -d build/imu_test samples/imu_test
```

详见 [06 IMU 与 EKF](06-drivers-imu.md)。

---

## 3. 电机样例

### can_smoke

只注册 CAN 接收回调，把 `can_id` 与 `dlc` 发到 VOFA，用于确认接线/波特率，
**不发送任何命令**。

```bash
west build -p -b dm_mc02/stm32h723xx -d build/can_smoke samples/motor/can_smoke
```

### DJI 系列

- `dji_unified/`：GM6020 开环电流老炼/安全测试，内置限速限温保护停机。
- `dji_speed_control/`：`VelocityMotor` 速度环，复合前馈 PID + 斜坡限幅，
  正弦速度目标，约 300 s。
- `dji_position_control/`：**推荐参考**。`PositionMotor` 位置-速度串级，
  默认固定零点最短路径；可选 begin 相对的 0/3/6/9 rad 序列；
  位置积分保持冻结。
- `m2006_speed_control/`：M2006 速度环，减速比 36:1，100 ms 后 5 rad/s。

```bash
west build -p -b dm_mc02/stm32h723xx -d build/dji_pos samples/motor/dji_position_control
west build -p -b rm_typec samples/motor/dji_position_control
```

每个样例自带 `app.overlay` 声明 `motor0` 与 CAN 参数，上电前务必核对
`motor-id`、`current-limit-ma`、GM6020 的 `encoder-zero-ticks`。

> `dji_speed_control/prj.conf.bak` 是遗留备份文件，不参与构建。

### 达妙系列

原生模式（使用电机内置闭环）：

- `dm_mit_control/`：模式 1 MIT，输出 0.5 N·m，最多 200 s，超 10 rad/s 保护。
- `dm_velocity_control/`：模式 3 速度模式，目标 0.5 rad/s。
- `dm_position_control/`：模式 2 位置-速度串级，相对保存零点在 0°/+90° 间切换。

软件闭环（复用统一封装）：

- `dm_mit_velocity_control/`：`VelocityMotor` + DM 后端，2 rad/s、±0.5 N·m。
- `dm_mit_position_control/`：`PositionMotor` + DM 后端，`DriverContinuous`
  坐标，每 6 s 正向 +90°。

```bash
west build -p -b dm_mc02/stm32h723xx -d build/dm_pos samples/motor/dm_mit_position_control
```

达妙样例统一约定：CAN1、1 Mbps、`motor-id=0x001`、`master-id=0x011`，
PMAX=12.5 rad、VMAX=30 rad/s、TMAX=10 N·m；**电机持久化模式必须与样例
一致**（在达妙调试助手中设置）。详细协议见 [05](05-drivers-motor-dm.md)，
示例说明见 `samples/motor/DM_J4310_EXAMPLES.md`。

---

## 4. 快速选择指引

- 刚拿到板子：`hello` → `control` → `imu_test`
- 查 CAN：`can_smoke`
- DJI 电机首测：`dji_position_control`（记得悬空 + 断电准备）
- 达妙电机首测：`dm_mit_control`（原生 MIT），确认后转 `dm_mit_*_control`
- 学统一封装：`dji_position_control` / `dm_mit_position_control` 是最完整的参考

---

## 5. 通用构建/烧录约定

- 样例目录内 `app.overlay` 自动生效；板卡专属 overlay 放
  `<sample>/boards/<board>.overlay`。
- 改过 overlay/`prj.conf` 后用 `-p` 重新 pristine 构建。
- `dm_mc02` 默认 runner 是 openocd；`rm_typec` 也是 openocd。
- C++ 样例建议 `CONFIG_MAIN_STACK_SIZE=8192`。

---

## 6. 相关文档

- [01 快速开始](01-getting-started.md)
- [04 DJI 电机驱动](04-drivers-motor-dji.md) / [05 达妙 DM 电机驱动](05-drivers-motor-dm.md)
- [06 IMU 与 EKF](06-drivers-imu.md) / [09 统一速度/位置封装](09-motor-wrapper.md)
