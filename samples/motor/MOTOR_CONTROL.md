> 双主控框架已加入可恢复生命周期。新项目使用 configure/poll/resume/update，临时禁用用 suspend；下文 begin/stop 是保留的兼容用法。四个 wrapper samples 已常驻恢复，详见 [上机样例索引](../FRAMEWORK_SAMPLES.md) 和 [框架说明](../../双主控框架使用说明.md)。

# 统一速度与位置控制

`VelocityMotor` 和 `PositionMotor` 替代原来的 `VelocityController` / `PositionController`。
旧头文件已移除；纯计算调用仍可使用 `control_motor_velocity_*`、`control_motor_position_*` C API。
新封装不分配堆内存，支持当前 DJI 电流驱动与 DM MIT 力矩驱动。

## 最小调用方式

```cpp
#include <control/velocity_motor.hpp>
#include <control/dji_motor_backend.hpp>

// makeMotorConfig() 填入已有 PID 参数、EffortUnit::Ampere 和保护阈值。
static skywalker::control::DjiMotorBackend backend{DEVICE_DT_GET(DT_ALIAS(motor0))};
static skywalker::control::VelocityMotor motor{backend, makeMotorConfig()};

int ret = motor.begin();
if (ret < 0) return ret;
for (;;) {
    k_sleep(K_MSEC(5));
    ret = motor.update(targetVelocityRadS());
    if (ret < 0) return ret; // 内部已尽力撤销输出。
    const auto &data = motor.telemetry();
    // 在此读取 data.measurement.feedback / data.output，发送 VOFA。
}
```

达妙使用 `DmMotorBackend`、`EffortUnit::NewtonMeter`，控制参数和限幅的输出单位也必须是 N·m。
MC02 示例额外传入 `skywalker::samples::dm::enableMotorPower`，保持 CAN 初始化/挂接后才开启 XT30_1。
构造函数不访问硬件，`begin()` 会等待反馈并启动输出。业务层保留目标函数、睡眠和遥测展示。

Kconfig：

```conf
CONFIG_CPP=y
CONFIG_STD_CPP20=y
CONFIG_REQUIRES_FULL_LIBCPP=y
CONFIG_SKYWALKER_LIB_MOTOR_CONTROL=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
# 按设备启用 CONFIG_SKYWALKER_MOTOR_DJI 或 CONFIG_SKYWALKER_MOTOR_DM。
```

纯 C 控制库不依赖新 wrapper；新配置会自动选中 `SKYWALKER_LIB_CONTROL`。

## 参数和接口

`VelocityMotor::Config::loop` 是原来的 `control_motor_velocity_config`；
`PositionMotor::Config::loop` 是原来的 `control_motor_position_config`。
各示例集中用 `makeMotorConfig()` 填充，便于保留已有调参。

| 调用/字段 | 语义 |
| --- | --- |
| `begin()` | 校验参数、单位、能力和设备限幅；prepare → 首次安全反馈 → reset → arm；每对象只能尝试一次 |
| `VelocityMotor::update(float)` | 目标 rad/s；真实 dt、反馈保护、计算、暂存和发送全部在内部执行 |
| `PositionMotor::update(double)` | 目标 rad，含义由 reference 决定；在内部完成连续坐标/最短路径适配 |
| `stop()` | 撤销命令；DM 发送 Disable，DJI 发送组内零电流；不代表机械制动/断电 |
| `elapsedMs()` | 从 arm 完成起算的运行时间；不在 Running 时返回 0 |
| `telemetry()` | 最近成功输出及测量；begin 成功后有初始测量，首个 update 成功前 valid 为 false；失败/stop 后也为 false |
| `status()` | 运行状态、首次控制错误 error、最近撤销输出错误 stop_error |
| `backend.report()` / `stopReport()` | 分别保存正常操作与清理的品牌专属发送报告，便于定位原始错误 |
| `effort_unit` | 必须显式选 Ampere 或 NewtonMeter，不支持自动电流/力矩转换 |
| `safety.velocity_abs_max_rad_s` | 必需的正超速阈值，不能小于允许请求的速度上限 |
| `safety.temperature_max_c` | 0 表示未启用温度保护；正数要求电机具备温度能力，并检查有效测量；DM 同时检查 MOS 温度 |

控制器的输出限幅不得超过驱动设备树限幅；不静默裁剪错误配置。
dt 使用真实时间差，位置模式取两级允许范围交集。首个 update 之前同样需要睡眠。
控制线程暂停后恢复若超出 dt 范围，将触发停机；线程永远不恢复时，软件检查也无法主动执行。

## 位置坐标

- `StartupRelative`（默认）：begin 首次反馈处为 0，连续目标；+2π 表示正向一圈。
- `DriverContinuous`：使用后端连续驱动坐标。DM 初始化为保存零点相关的协议反馈并持续解包；DJI 沿用驱动首帧归零的累计输出轴位置。
- `AbsoluteNearest`：单圈固定零点目标，按最短路径解析；要求 `FeedbackAbsolutePosition`。当前仅满足传感器/传动比条件的 DJI 设备可用，DM 后端不伪造此能力。

连续模式中 `3π/2 → 0` 是回到原圈零点，`3π/2 → 2π` 才是继续正转。
DM 示例把时间生成的圈数留在目标函数中，保持每 6 s 正向增加 90° 的行为。
原生 DM 速度/位置模式继续使用原生接口和示例，不能用本软件闭环后端替代。

DM 连续位置仍假设固件在 ±PMAX 回绕，且相邻消费反馈之间的运动小于 PMAX；此条件需要实机确认。
新后端未改变通用驱动 `Feedback::position_rad`，适配后的连续值在 `measurement.position_rad`。
DJI 的位置/速度已经换算到输出轴，不要再除一次减速比。

为保持长时间运行的坐标精度，位置 PID 使用局部坐标，超出 128 rad 时同步平移测量历史；前馈仍接收驱动坐标中的目标。
C 位置输入增加可选 `position_reference_rad` / `has_position_reference`：默认 false 时沿用原行为。
位置积分仍按原 C 内核固定冻结，配置 ki 不会启用位置积分累积。

## 生命周期、错误与并发

对象和 backend 必须存活到应用结束，推荐如示例采用 static。stop 不注销驱动保存的 CAN 回调/所有权；不使用局部临时 backend。
不可复制或移动；一次 backend 只能绑定一个运行实例，第二个 begin 返回 `-EBUSY`。
一次 begin 失败后不自动重新初始化或恢复，重复 begin 返回 `-EALREADY`。

update 在未运行状态返回 `-EACCES`。非有限参数通常为 `-EINVAL`，单位不匹配也是 `-EINVAL`；越界/超周期/保护为 `-ERANGE`；缺能力为 `-ENOTSUP`；反馈失效为 `-ESTALE` / `-EHOSTDOWN`。
运行失败进入 Fault，不再提交控制输出，尽力 stop 并保留原始原因；读取 stop_error 判断撤销输出是否也失败。旧 telemetry 不作为有效新输出。
begin/更新/stop/遥测都由同一线程调用，禁止在 ISR 中执行。跨线程遥测应自行同步复制快照。

当前 backend 是**独占总线的单电机便利封装**。多个 DJI 轴不能分别创建 backend 发同一组帧；驱动也会拒绝第二个 DJI Bus 对同一 CAN 的所有权。
多轴扩展需要一个共享 Bus，所有轴先计算并暂存，再统一 flush。本实现没有隐式创建多轴调度器。

## 已迁移示例

| 示例 | 后端 / 类 | 保留的运行行为 |
| --- | --- | --- |
| dm_mit_velocity_control | DM / VelocityMotor | 2 rad/s，±0.5 N·m；10 rad/s 与 60°C 保护 |
| dm_mit_position_control | DM / PositionMotor | 保存零点相关的正向 90°/6 s 序列；±0.5 N·m、10 rad/s 与 60°C 保护 |
| dji_speed_control | DJI / VelocityMotor | 原正弦速度目标与 300 s 运行时长，±0.8 A |
| dji_position_control | DJI / PositionMotor | 默认固定零点最短路径；可选 begin 相对的 0/3/6/9 rad 序列，±0.8 A |
| m2006_speed_control | DJI / VelocityMotor | 100 ms 后 5 rad/s，原增益、原 ±10 A 及运行时长 |

DJI 示例现在均显式检查超速：速度示例阈值为允许请求上限的 1.5 倍（75 / 150 rad/s），位置示例为原先声明但注释掉检查的 45 rad/s。
DJI 温度保护保持未开启，M2006 不会因为缺少温度反馈而无法启动。
首次上电仍需核对参数、悬空输出并准备物理断电；示例限幅是当前配置，不代表适合所有负载。

## 构建

```sh
west build -b dm_mc02/stm32h723xx -d /tmp/skywalker-wrapper-dm-velocity samples/motor/dm_mit_velocity_control
west build -b dm_mc02/stm32h723xx -d /tmp/skywalker-wrapper-dm-position samples/motor/dm_mit_position_control
west build -b dm_mc02/stm32h723xx -d /tmp/skywalker-wrapper-dji-velocity samples/motor/dji_speed_control
west build -b dm_mc02/stm32h723xx -d /tmp/skywalker-wrapper-dji-position samples/motor/dji_position_control
west build -b dm_mc02/stm32h723xx -d /tmp/skywalker-wrapper-m2006 samples/motor/m2006_speed_control
```

构建不等于硬件验证。MC02 固件的正常入口需要真实 CAN 电机，本次迁移不自动烧录或驱动电机。
