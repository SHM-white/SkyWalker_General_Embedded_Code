# 09 统一速度/位置控制封装

`include/control/velocity_motor.hpp`、`position_motor.hpp` 把「读反馈 →
算控制 → 写 effort → 保护/时序」统一封装成 C++ 类，并通过 `MotorBackend`
抽象品牌差异。DJI（电流）与达妙（MIT 力矩）共用同一套上层代码。

> 旧的 `VelocityController` / `PositionController` 头文件已移除。
> 纯计算仍可用 [08 的 C API](08-control-algorithms.md)；
> 样例迁移说明见 `samples/motor/MOTOR_CONTROL.md`。

---

## 1. 组成

```text
应用
 │  VelocityMotor / PositionMotor      ← 业务看到的类
 ▼
MotorRuntime                          ← 生命周期、dt、保护、故障停机
 │  MotorBackend（虚接口）
 ├─ DjiMotorBackend  → dji::Bus → CAN（电流 A）
 └─ DmMotorBackend   → dm::Bus  → CAN（力矩 N·m）
```

### MotorBackend（`motor_backend.hpp`）

```cpp
class MotorBackend {
    virtual int describe(MotorInfo &info) = 0;   // 单位、能力、反馈超时
    virtual int prepare() = 0;                   // 初始化驱动/Bus
    virtual int read(MotorMeasurement &m) = 0;   // 读连续坐标反馈
    virtual int arm() = 0;                       // 使能输出
    virtual int write(float effort) = 0;         // 暂存命令
    virtual int flush() = 0;                     // 真正发送
    virtual int stop() = 0;                      // 撤销输出
};
```

约束：**单电机、独占总线、单线程**；对象必须在应用整个生命周期内存活
（static），因为 `stop()` 不注销驱动保存的 CAN 回调/所有权。不可复制/移动；
一个 backend 只能绑定一个运行实例，第二次 `begin()` 返回 `-EBUSY`。

`MotorSafety`：

```cpp
struct MotorSafety {
    float velocity_abs_max_rad_s; // 必需，正超速硬阈值
    float temperature_max_c;      // 0 = 关闭温度保护；>0 需要温度能力
};
```

### MotorRuntime

共享的执行机制：`prepare → 首次安全反馈 → reset → arm` 的启动，
每个周期的真实 dt 校验、反馈新鲜度检查、故障进入 `Fault` 后尽力 `stop()`。
状态：`Idle / Starting / Running / Stopped / Fault`。

---

## 2. 上层类

### VelocityMotor

```cpp
struct Config {
    control_motor_velocity_config loop;   // 见 08
    EffortUnit effort_unit;               // Ampere 或 NewtonMeter（必须显式）
    MotorSafety safety;
};
struct Telemetry {
    MotorMeasurement measurement;
    control_motor_velocity_output output;
    float target_rad_s, dt_s;
    bool valid;
};

VelocityMotor(MotorBackend &backend, const Config &config);
int begin();                              // 每对象只能成功一次
int update(float target_velocity_rad_s);  // 周期调用
int stop();
int64_t elapsedMs() const;
const Telemetry &telemetry() const;
const MotorStatus &status() const;
```

### PositionMotor

```cpp
enum class PositionReference {
    StartupRelative,  // begin 首次反馈处为 0，连续目标
    DriverContinuous, // 驱动连续坐标（DM 保存零点 / DJI 首帧零点）
    AbsoluteNearest,  // 固定零点单圈目标，最短路径（需 FeedbackAbsolutePosition）
};

struct Config {
    control_motor_position_config loop;
    EffortUnit effort_unit;
    MotorSafety safety;
    PositionReference reference = PositionReference::StartupRelative;
};

int update(double target_position_rad);
```

位置遥测里：`requested_position_rad`（你传入的）、
`target_position_rad`（解析到驱动坐标后的）、`position_rad`
（`StartupRelative` 时相对 begin）。

---

## 3. 坐标语义（位置模式）

- `StartupRelative`（默认）：`begin()` 首次有效反馈处为 0，连续坐标；
  `+2π` 表示正向一圈。
- `DriverContinuous`：直接使用后端连续驱动坐标。DM 使用协议反馈 + 保存零点
  解包；DJI 沿用驱动首帧归零的累计输出轴位置。
- `AbsoluteNearest`：单圈固定零点目标，按最短路径解析；要求
  `FeedbackAbsolutePosition`。当前**只有满足传动比条件的 DJI GM6020** 具备，
  DM 后端不会伪造此能力。

连续模式的语义：`3π/2 → 0` 是回到本圈零点，`3π/2 → 2π` 才是继续正转。
`dm_mit_position_control` 把时间生成的圈数保留在目标函数里，保持每 6 s
正向 +90°。

单位一致性：

- `EffortUnit::Ampere` → 配置里的 `output_min/max`、`effort_abs_max` 都是 A。
- `EffortUnit::NewtonMeter` → 同字段都是 N·m。
- **不支持自动电流/力矩换算**；单位不匹配返回 `-EINVAL`。

---

## 4. 调用模板

```cpp
#include <control/velocity_motor.hpp>
#include <control/dji_motor_backend.hpp>
#include <zephyr/kernel.h>

static skywalker::control::DjiMotorBackend backend{DEVICE_DT_GET(DT_ALIAS(motor0))};
static skywalker::control::VelocityMotor motor{backend, makeMotorConfig()};

int main() {
    int ret = motor.begin();
    if (ret < 0) return ret;

    for (;;) {
        k_sleep(K_MSEC(5));
        ret = motor.update(targetVelocityRadS());
        if (ret < 0) return ret;      // 内部已尽力撤销输出
        const auto &t = motor.telemetry();
        // 读取 t.measurement.feedback / t.output，发 VOFA
    }
}
```

达妙只把 backend 换成 `DmMotorBackend`、单位换成 `NewtonMeter`：

```cpp
static skywalker::control::DmMotorBackend backend{
    DEVICE_DT_GET(DT_ALIAS(motor0)), skywalker::samples::dm::enableMotorPower};
```

MC02 上传入 `enableMotorPower`，让 CAN 初始化/挂接完成后再开 XT30_1。

---

## 5. 生命周期与错误

| 返回 | 场景 |
|---|---|
| `-EINVAL` | 非有限参数、单位不匹配 |
| `-ERANGE` | 越界、dt 超出允许范围、保护触发 |
| `-ENOTSUP` | 缺少所需能力（如无温度却开温度保护） |
| `-EACCES` | 未运行状态调用 `update()` |
| `-EBUSY` / `-EALREADY` | backend 已被占用 / 重复 begin |
| `-ESTALE` / `-EHOSTDOWN` | 反馈失效 / 设备离线 |

规则：

- `begin()` 一次成功；失败后**不自动重试或恢复**，重复 begin 返回 `-EALREADY`。
- 运行期任何失败 → 进入 `Fault`，不再提交控制输出，尽力 `stop()`，
  原始原因保存在 `status().error`，撤销输出的错误在 `status().stop_error`。
- `update()` 之前必须先 `k_sleep`，让首个 dt 合理；线程长时间暂停后恢复
  若超出 dt 范围会触发停机。
- `telemetry().valid` 在首个 `update()` 成功前、以及失败/stop 后为 false，
  不要把旧遥测当成新输出。
- begin/update/stop/telemetry 都必须由**同一线程**调用；跨线程读取遥测要
  自行同步复制。
- `elapsedMs()` 从 arm 完成起算；不在 Running 时返回 0。

---

## 6. 多轴限制（重要）

当前 backend 是**独占总线的单电机便利封装**：

- 多个 DJI 轴**不能**各自建一个 backend 发同一命令帧——它们会互相覆盖
  同一组帧的不同 slot；DJI 驱动也会拒绝第二个 Bus 对同一 CAN 的所有权。
- DM 同一总线可挂多个电机（按 `master-id` 路由反馈），但上层仍是
  「一个 backend 一个电机」，需要自己做多轴调度。
- 真正多轴扩展需要一个**共享 Bus**：所有轴先计算并暂存，再统一 `flush()`。
  本实现没有隐式多轴调度器。

---

## 7. 已迁移样例对照

| 样例 | 后端 / 类 | 运行行为 |
|---|---|---|
| `dm_mit_velocity_control` | DM / VelocityMotor | 2 rad/s，±0.5 N·m；10 rad/s 与 60 °C 保护 |
| `dm_mit_position_control` | DM / PositionMotor | 连续坐标正向 90°/6 s 序列；±0.5 N·m、10 rad/s、60 °C |
| `dji_speed_control` | DJI / VelocityMotor | 正弦速度目标，±0.8 A，300 s |
| `dji_position_control` | DJI / PositionMotor | 固定零点最短路径；±0.8 A |
| `m2006_speed_control` | DJI / VelocityMotor | 5 rad/s，原 ±10 A 与运行时长 |

DJI 样例显式检查超速（速度示例为请求上限的 1.5 倍）；DJI 温度保护保持
关闭（M2006 无温度反馈）。首次上电仍需核对参数、悬空输出、准备物理断电。

---

## 8. Kconfig

```conf
CONFIG_CPP=y
CONFIG_STD_CPP20=y
CONFIG_REQUIRES_FULL_LIBCPP=y
CONFIG_SKYWALKER_LIB_MOTOR_CONTROL=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y     # 或 CONFIG_SKYWALKER_MOTOR_DM=y
```

开启 `SKYWALKER_LIB_MOTOR_CONTROL` 会自动 `select SKYWALKER_LIB_CONTROL`。

---

## 9. 相关文档

- [08 纯 C 控制算法](08-control-algorithms.md)
- [04 DJI 电机驱动](04-drivers-motor-dji.md) / [05 达妙 DM 电机驱动](05-drivers-motor-dm.md)
- 详细迁移说明：`samples/motor/MOTOR_CONTROL.md`
