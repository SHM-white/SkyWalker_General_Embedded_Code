# 电机速度与位置闭环样例

`VelocityMotor` 和 `PositionMotor` 复用现有 C 控制算法，接收 `Motor&` 并输出电流或力矩暂存命令。`CanBus` 持有物理 CAN、接收路由和 I/O 线程；需要机械联动时由 `Group` 统一使能与停机。每条物理 CAN 只创建一个 `CanBus`。

## 最小调用顺序

```cpp
#include <control/velocity_motor.hpp>
#include <drivers/motor/can_bus.hpp>

static skywalker::motor::Motor drive{skywalker::motor::dji::gm6020({
    .id = 4, .current_limit_a = 0.8f, .encoder_zero_ticks = 0,
    .current_mode_confirmed = true, .timing = {20, 20, 20, 100},
})};
static skywalker::motor::CanBus bus{DEVICE_DT_GET(DT_NODELABEL(can1))};
static skywalker::control::VelocityMotor axis{drive, makeMotorConfig()};

int ret = bus.attach(drive);
if (ret == 0) ret = bus.start();
if (ret == 0) ret = axis.configure();
// 检查 ret；等待 drive.ready()，并确认人员、机构和电源条件。
if (ret == 0) ret = axis.reset(); // 使能前检查反馈、速度和温度。
if (ret == 0) ret = drive.enable();
// 等待 drive.active()；循环中的 dt_s 使用实际经过的秒数。
if (drive.active()) {
    ret = axis.update(target_rad_s, dt_s);
    if (ret == 0) ret = bus.commit().error;
    if (ret < 0) (void)drive.disable();
}
```

`enable()` 在进入使能流程前，会用最新反馈重验已绑定控制器的速度、温度、位置及参考要求；`reset()` 后实测条件变差时使能会被拒绝。`enable()` 返回成功只表示接受请求，`active()` 才表示使能完成。`axis.update()` 计算 PID 并暂存命令，`bus.commit()` 提交一份目标快照；CAN 发送异步完成，可用 `bus.status()` 和 `drive.snapshot()` 查看结果。控制器新使能代次的首次 `update()` 明确暂存零 effort，下一周期才开始正常 PID 计算。

同一电机只能绑定一个闭环控制器；配置完成后，业务不能对它混用直接 `setCurrent()`/`setTorque()`。若多个控制器共享 CAN，先逐个 `update()`，再对该总线 `commit()` 一次。`Group` 不代替 `commit()`，只决定联动许可和故障停机范围。

## 参数、坐标与诊断

`VelocityMotor::Config::loop` 和 `PositionMotor::Config::loop` 沿用现有 C 参数结构。`effort_unit` 必须指定 `Ampere` 或 `NewtonMeter`，且算法 effort 上限不能超过电机配置限幅。`MotorSafety` 限制实测速度和可选温度；DM 开启温度限制时同时检查转子和 MOS 温度。`configure()` 检查命令/反馈能力、单位和参数，但不使能电机。

位置目标的 `reference` 有三种：

| 模式 | 目标坐标 |
| --- | --- |
| `StartupRelative` | 显式 `reset()` 时的测量位置为零，跨使能代次保持；参考世代失效或未显式复位时按最新反馈重建 |
| `DriverContinuous` | 驱动的连续位置坐标；DM 默认在首帧归零，需要实物坐标时于禁用状态调用 `reseedPosition()` |
| `AbsoluteNearest` | 固定零点的单圈目标，控制器选最短路径；要求 `FeedbackAbsolutePosition`，当前 GM6020 1:1 配置可用 |

DJI 测量位置和速度已经换算为输出轴单位，不再额外除减速比。DM 连续展开假设固件在 ±PMAX 处回绕，相邻反馈间转动小于 PMAX。反馈中断后的位置参考可能失效；重新确认机械位置并在禁用状态重新 `reseedPosition()`，再请求使能。位置 PID 长时间运行时会在内部移动局部计算原点，业务目标仍使用所选参考坐标。

`telemetry()` 按值返回最近一次控制计算及对应的 `MotorSnapshot`、目标、真实 `dt_s`、输出 effort 和错误。它表示已暂存的控制结果，不表示设备已经执行。故障时旧目标失效；反馈恢复也不会自动使能。`clearFault()` 只处理可清故障，仍需新的业务授权和 `enable()`。`disable()` 立即撤销软件输出许可，安全帧由 I/O 线程异步发送，不代表机械已制动。

## 现有台架行为

| 示例 | 目标与原保护配置 |
| --- | --- |
| `dji_speed_control` | GM6020 ID4 正弦速度，软件 ±0.8 A，速度上限 75 rad/s，原 VOFA 12 通道 |
| `dji_position_control` | GM6020 ID4 固定零点 0/90/180/270°，软件 ±0.8 A，原 VOFA 14 通道 |
| `m2006_speed_control` | M2006 ID4、36:1，100 ms 后 5 rad/s，软件 ±10 A，原 VOFA 10 通道 |
| `dm_mit_velocity_control` | J4310 MIT ID1、Master 0x11，2 rad/s，软件 ±0.5 N·m，10 rad/s/60°C 保护 |
| `dm_mit_position_control` | 同一 MIT 配置，按原驱动原生起始角度每 6 s 正向增加 90°，软件 ±0.5 N·m，10 rad/s/60°C 保护 |

这些样例保留原 PID、运行时长、VOFA 通道、CAN ID 和限幅；电机配置在各自 `src/main.cpp`。DM 样例先启动 CAN，再打开 MC02 XT30_1，等待 1.5 s 后检查反馈。示例仍需在实际机构上回归验证，尤其电机模式、DM PMAX/VMAX/TMAX、编码器零点和转向。

构建示例：

```sh
west build -b dm_mc02/stm32h723xx samples/motor/dm_mit_position_control -d build/bench_dm_position
west build -b dm_mc02/stm32h723xx samples/motor/dji_speed_control -d build/bench_dji_speed
```
