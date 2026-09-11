# 移除 PID Device Driver，并将 IMU 恒温迁移到 `control/` PID 的施工指南

> 目标仓库：`SHM-white/SkyWalker_General_Embedded_Code`
>
> 目标分支：`dev`
>
> 本任务只处理一件事：**彻底移除 `drivers/pid` 这套把 PID 包装成 Zephyr device 的旧实现，并把 `imu_test` 所走的 IMU 恒温控制链迁移到 `lib/control` 的纯数值 PID。**
>
> 不在本任务中修改舵轮、Motor Controller、EKF 算法或其他无关模块。

---

# 0. 执行规则

开始前先读取：

```text
AGENTS.md
.agents/skills/ancient-programming/SKILL.md
```

仓库默认启用“古法编程模式”。

如果用户没有在当前任务明确写：

```text
本任务禁用古法编程模式
```

则只能阅读源码并输出施工说明，不能直接修改业务源码。

如果用户明确禁用，则按本文执行。

---

# 1. 为什么要删除 `drivers/pid`

当前仓库同时存在两套 PID：

```text
旧：
drivers/pid/
include/drivers/pid/
dts/bindings/pid/
```

以及：

```text
新：
include/control/pid.h
lib/control/pid.c
```

旧 PID 被包装成：

```text
Zephyr device
    ↓
devicetree node
    ↓
DEVICE_DT_DEFINE
    ↓
pid_dev->data / pid_dev->config
```

但 PID 本质是：

```text
config
+
state
+
input
↓
纯数值计算
↓
output
```

它不需要：

```text
device
devicetree device lifecycle
POST_KERNEL
device_is_ready()
```

所以 PID 不应该属于：

```text
drivers/
```

而应该只存在于：

```text
control/
```

最终目标：

```text
删除：
drivers/pid/
include/drivers/pid/
dts/bindings/pid/

保留：
include/control/pid.h
lib/control/pid.c
include/control/feedforward_pid.h
lib/control/feedforward_pid.c
```

---

# 2. 当前真实调用链

当前 `imu_test/main.c` 并没有直接调用 `pid_update()`。

真实链路是：

```text
samples/imu_test/src/main.c
        ↓
imu_heat_control()
        ↓
drivers/imu/imu.c
        ↓
pid_update()
        ↓
drivers/pid/pid.c
```

设备树则是：

```text
imu
 └─ pid-dev = <&temp_pid>

temp_pid
 └─ compatible = "skywalker,pid"
```

所以不能只删：

```text
drivers/pid/pid.c
```

必须同时清理：

```text
IMU config
IMU binding
imu_test overlay
Kconfig
CMake
prj.conf
旧 PID binding
旧 PID header
```

---

# 3. 开工前先做全仓库引用检查

执行：

```bash
git grep -n "drivers/pid"
git grep -n "pid_update"
git grep -n "skywalker,pid"
git grep -n "CONFIG_SKYWALKER_DRIVER_PID"
git grep -n "pid-dev"
git grep -n "pid_dev"
```

预期主要命中：

```text
drivers/pid/*
include/drivers/pid/*
drivers/imu/imu.c
include/drivers/imu/imu.h
drivers/CMakeLists.txt
drivers/Kconfig
dts/bindings/pid/skywalker,pid.yaml
dts/bindings/imu/skywalker,imu.yaml
samples/imu_test/boards/dm_mc02.overlay
samples/imu_test/prj.conf
```

如果发现其他业务模块仍依赖旧 PID：

```text
不要先删 driver 导致它们失效。
```

先将额外调用方同样迁移到：

```text
control_pid
或
control_feedforward_pid
```

再统一删除旧 driver。

---

# 4. IMU 恒温推荐使用哪一个 control PID

旧接口：

```c
float pid_update(
    pid_data *data,
    const pid_config *config,
    float setpoint,
    float measurement,
    float dt,
    float feedforward);
```

它本身把：

```text
PID feedback
+
feedforward
```

合在一起。

当前 IMU 恒温还显式使用：

```c
HEAT_OFFSET_NS = 6750000.0f
```

作为基础加热前馈。

因此最接近旧行为、同时仍完全属于 `control/` 层的方案是：

```text
control_feedforward_pid
```

而不是重新在 IMU 里手写：

```text
control_pid output + feedforward
```

因为：

```text
control_feedforward_pid
    ├─ 内部复用 control_pid
    ├─ additive output 参与最终限幅
    └─ additive output 参与 PID anti-windup 判断
```

最终依赖：

```text
IMU
 ↓
control_feedforward_pid
 ↓
control_pid
```

仍然满足：

> PID 不再是 driver，而是纯数值 control algorithm。

如果以后温控完全不需要前馈，可以再直接改成：

```text
control_pid
```

但本次迁移优先保证原有控制行为。

---

# 5. 第一阶段：删除 PID driver 的构建入口

先修改：

```text
drivers/CMakeLists.txt
```

删除：

```cmake
add_subdirectory_ifdef(CONFIG_SKYWALKER_DRIVER_PID pid)
```

修改：

```text
drivers/Kconfig
```

删除整个：

```kconfig
config SKYWALKER_DRIVER_PID
    bool "Skywalker PID Controller"
    help
        Enable the PID Controller driver.
```

不要给 `control/pid` 再创建一个：

```text
SKYWALKER_DRIVER_PID
```

新的 PID 已经属于：

```text
CONFIG_SKYWALKER_LIB_CONTROL
```

---

# 6. 第二阶段：让 IMU driver 明确依赖 control library

IMU 内部将使用：

```c
control_feedforward_pid_*
```

因此：

```text
SKYWALKER_DRIVER_IMU
```

应该自动带上：

```text
SKYWALKER_LIB_CONTROL
```

推荐修改 `drivers/Kconfig`：

```kconfig
config SKYWALKER_DRIVER_IMU
    bool "Skywalker IMU Driver"
    select CMSIS_DSP_FASTMATH
    select SKYWALKER_LIB_CONTROL
    help
        Enable the IMU attitude estimation driver.
        Select estimation method via DT property "estimator".
```

理由：

```text
这是 IMU driver 自己的内部依赖，
不应该要求每一个 application 手工记住打开 control library。
```

不要写成：

```text
depends on SKYWALKER_LIB_CONTROL
```

然后让用户自己补配置。

此处 `select` 更符合“内部必须依赖”的语义。

---

# 7. 第三阶段：移除 IMU 的 `pid_dev`

修改：

```text
include/drivers/imu/imu.h
```

删除：

```c
const struct device *pid_dev;
```

同时加入 control header：

```c
#include <control/feedforward_pid.h>
```

推荐让 IMU config 直接保存恒温控制配置：

```c
typedef struct {
    const struct device *accel_dev;
    const struct device *gyro_dev;
    const struct device *heat_dev;
    const struct device *filter_dev;

    control_feedforward_pid_config heat_controller;

    const char *estimator;
} imu_config;
```

运行状态放入 `imu_data`：

```c
typedef struct {
    float accel[3];
    float gyro[3];
    float temp;
    float angle[3];

    control_feedforward_pid_state heat_controller_state;
} imu_data;
```

这里非常重要：

```text
config = ROM，只保存参数
state  = RAM，只保存运行状态
```

不要把：

```text
control_pid_state
```

塞进 `imu_config`。

---

# 8. 第四阶段：把温控参数从独立 PID device 移到 IMU 配置

当前 overlay 是：

```dts
imu: imu {
    ...
    pid-dev = <&temp_pid>;
};

temp_pid: temp_pid {
    compatible = "skywalker,pid";
    k-p      = "6000000";
    k-i      = "0";
    k-d      = "0.02";
    i-max    = "0";
    out-max  = "20000000";
    deadband = "0";
};
```

这套结构应该删除。

## 8.1 修改 IMU binding

文件：

```text
dts/bindings/imu/skywalker,imu.yaml
```

删除：

```yaml
pid-dev:
  type: phandle
  required: true
```

增加温控参数。

推荐：

```yaml
heat-kp:
  type: string
  required: true

heat-ki:
  type: string
  required: true

heat-kd:
  type: string
  required: true

heat-integral-max:
  type: string
  required: true

heat-output-max:
  type: string
  required: true

heat-deadband:
  type: string
  required: true

heat-derivative-tau-s:
  type: string
  required: true

heat-dt-min-s:
  type: string
  required: true

heat-dt-max-s:
  type: string
  required: true

heat-feedforward-ns:
  type: string
  required: true
```

V1 继续使用 string，是因为当前工程已经用：

```text
DT_STRING_UNQUOTED()
```

解决 DTS 不方便直接表达 float 的问题。

本任务不要顺便设计新的 fixed-point DT 格式。

---

# 9. 第五阶段：在 `imu.c` 直接构造 control config

修改：

```text
drivers/imu/imu.c
```

删除：

```c
#include "drivers/pid/pid.h"
```

改成：

```c
#include <control/feedforward_pid.h>
```

`IMU_CONFIG_DEFINE()` 中删除：

```c
.pid_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, pid_dev)),
```

改为构造嵌套配置。

示意：

```c
.heat_controller = {
    .feedback = {
        .kp = (float)DT_STRING_UNQUOTED(
            DT_DRV_INST(inst), heat_kp),

        .ki = (float)DT_STRING_UNQUOTED(
            DT_DRV_INST(inst), heat_ki),

        .kd = (float)DT_STRING_UNQUOTED(
            DT_DRV_INST(inst), heat_kd),

        .derivative_tau_s =
            (float)DT_STRING_UNQUOTED(
                DT_DRV_INST(inst), heat_derivative_tau_s),

        .integral_min =
            -(float)DT_STRING_UNQUOTED(
                DT_DRV_INST(inst), heat_integral_max),

        .integral_max =
            (float)DT_STRING_UNQUOTED(
                DT_DRV_INST(inst), heat_integral_max),

        .output_min =
            0.0f,

        .output_max =
            (float)DT_STRING_UNQUOTED(
                DT_DRV_INST(inst), heat_output_max),

        .deadband =
            (float)DT_STRING_UNQUOTED(
                DT_DRV_INST(inst), heat_deadband),

        .dt_min_s =
            (float)DT_STRING_UNQUOTED(
                DT_DRV_INST(inst), heat_dt_min_s),

        .dt_max_s =
            (float)DT_STRING_UNQUOTED(
                DT_DRV_INST(inst), heat_dt_max_s),
    },

    .feedforward = {
        .k_bias =
            (float)DT_STRING_UNQUOTED(
                DT_DRV_INST(inst), heat_feedforward_ns),

        .k_static = 0.0f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
    },
},
```

## 9.1 一个重要修正：output_min

旧 PID 是：

```text
最终先 clamp 到 ±20 ms
然后负数再截成 0
```

加热器本身不能制冷。

新的控制器可以直接定义：

```text
output_min = 0
output_max = 20 ms
```

这样输出从控制算法开始就是合法 PWM 脉宽。

这是比旧实现更清楚的物理边界：

```text
heater command ∈ [0, PWM period]
```

不要继续保留没有物理意义的负加热输出。

## 9.2 integral limit

旧：

```text
i-max = 0
```

所以：

```text
integral_min = 0
integral_max = 0
```

即可。

如果未来启用 Ki，例如：

```text
i-max = 2e6
```

则：

```text
integral_min = -2e6
integral_max = +2e6
```

---

# 10. 第六阶段：IMU 初始化不再检查 PID device

`skywalker_imu_init()` 中删除：

```c
if (!device_is_ready(cfg->pid_dev)) {
    return -ENODEV;
}
```

因为 PID 已经不是 device。

改为验证数值配置：

```c
int ret =
    control_feedforward_pid_validate(
        &cfg->heat_controller);

if (ret < 0) {
    return ret;
}
```

同时初始化：

```c
data->heat_controller_state = (control_feedforward_pid_state){0};
```

这里暂时不要直接：

```c
control_feedforward_pid_reset(...)
```

因为 device init 阶段还没有可靠的真实温度 measurement。

第一次真正执行温控前，再使用当前温度 reset。

---

# 11. 第七阶段：重写 `imu_heat_control()`

当前函数：

```c
void imu_heat_control(...)
```

调用旧：

```c
pid_update(...)
```

建议本次顺便把返回值改成：

```c
int imu_heat_control(
    const struct device *dev,
    float target_temp,
    float dt);
```

原因：

```text
control PID 本身可能返回错误
pwm_set() 也可能返回错误
```

原来的 `void` 会吞掉所有错误。

---

## 11.1 第一次调用先 reset

第一次进入时：

```c
if (!data->heat_controller_state.feedback.initialized) {
    ret = control_feedforward_pid_reset(
        &data->heat_controller_state,
        data->temp);

    if (ret < 0) {
        return ret;
    }
}
```

这样 D 项基线是：

```text
当前真实温度
```

不会像旧 PID：

```text
last_error 初始为 0
```

那样在第一次控制时产生人为的巨大 D kick。

---

## 11.2 输入构造

温度 PID：

```c
control_feedforward_pid_input input = {
    .feedback = {
        .setpoint = target_temp,
        .measurement = data->temp,
        .dt_s = dt,
        .freeze_integrator = false,
    },

    .reference = {
        .position_ref_rad = 0.0f,
        .velocity_ref = 0.0f,
        .acceleration_ref = 0.0f,
    },
};
```

恒温前馈只使用：

```text
k_bias
```

因此 reference 三项均为 0。

执行：

```c
control_feedforward_pid_result result;

ret = control_feedforward_pid_step(
    &data->heat_controller_state,
    &cfg->heat_controller,
    &input,
    &result);

if (ret < 0) {
    return ret;
}
```

输出：

```c
float output = result.output;
```

此时由于：

```text
output_min = 0
output_max = PWM period
```

无需再手工：

```c
if (output < 0) output = 0;
```

只需要在转换成整数前验证：

```c
isfinite(output)
```

---

# 12. 第八阶段：PWM 输出错误也必须返回

当前：

```c
pwm_set(...);
```

返回值被忽略。

改为：

```c
return pwm_set(
    cfg->heat_dev,
    4,
    period,
    (uint32_t)output,
    PWM_POLARITY_NORMAL);
```

如果需要先检查 float → uint32：

```text
output >= 0
output <= period
isfinite(output)
```

再转换。

不要让非法 float 静默转换为整数。

---

# 13. 第九阶段：修改 `imu_test`

修改：

```text
samples/imu_test/src/main.c
```

原来：

```c
imu_heat_control(
    imu_dev,
    50.0f,
    (now - t_heat) / 1000.0f);
```

改成检查错误：

```c
int ret = imu_heat_control(
    imu_dev,
    50.0f,
    (now - t_heat) / 1000.0f);

if (ret < 0) {
    printk("IMU heat control failed: %d\n", ret);
    return ret;
}
```

这就是 `imu_test` 对新 control PID 链路的最终接入点。

`main.c` 不需要自己保存：

```text
PID config
PID state
```

因为恒温控制是 IMU 子系统内部能力。

但 PID 算法实现已经明确来自：

```text
lib/control
```

而不是：

```text
drivers/pid
```

最终链路：

```text
imu_test
    ↓
imu_heat_control()
    ↓
control_feedforward_pid_step()
    ↓
control_pid
    ↓
PWM
```

---

# 14. 第十阶段：修改 `imu_test` overlay

原：

```dts
imu: imu {
    ...
    pid-dev = <&temp_pid>;
};

temp_pid: temp_pid {
    compatible = "skywalker,pid";
    ...
};
```

改为：

```dts
imu: imu {
    compatible = "skywalker,imu";

    accel-dev = <&bmi08x_accel>;
    gyro-dev = <&bmi08x_gyro>;
    heat-dev = <&pwm_heat>;
    filter-dev = <&ekf_filter>;

    estimator = "ekf";

    heat-kp = "6000000";
    heat-ki = "0";
    heat-kd = "0.02";

    heat-integral-max = "0";
    heat-output-max = "20000000";
    heat-deadband = "0";

    heat-derivative-tau-s = "0";

    heat-dt-min-s = "0.05";
    heat-dt-max-s = "0.20";

    heat-feedforward-ns = "6750000";
};
```

这里：

```text
20,000,000 ns = 20 ms
```

刚好等于当前：

```c
PWM_MSEC(20)
```

所以 output max 不应超过 PWM period。

---

# 15. 第十一阶段：修改 `imu_test/prj.conf`

删除：

```text
CONFIG_SKYWALKER_DRIVER_PID=y
```

如果 `SKYWALKER_DRIVER_IMU` 已经：

```text
select SKYWALKER_LIB_CONTROL
```

则不必再手工写。

为了 sample 自说明，也可以显式保留：

```text
CONFIG_SKYWALKER_LIB_CONTROL=y
```

但推荐依赖由 driver 自己声明。

最终不应该再出现：

```text
CONFIG_SKYWALKER_DRIVER_PID
```

---

# 16. 第十二阶段：正式删除旧 PID 文件

确认所有引用迁移完成后，删除：

```text
drivers/pid/CMakeLists.txt
drivers/pid/pid.c

include/drivers/pid/pid.h

dts/bindings/pid/skywalker,pid.yaml
```

如果目录因此为空，则删除目录：

```text
drivers/pid/
include/drivers/pid/
dts/bindings/pid/
```

不要留下：

```text
deprecated wrapper
compatibility alias
旧 header 转发到新 header
```

这个项目还很轻量，没有必要为错误架构保留历史包袱。

---

# 17. 迁移后再次全仓库搜索

必须执行：

```bash
git grep -n "drivers/pid" || true
git grep -n "pid_update" || true
git grep -n "skywalker,pid" || true
git grep -n "CONFIG_SKYWALKER_DRIVER_PID" || true
git grep -n "pid-dev" || true
git grep -n "pid_dev" || true
```

期望：

```text
全部无结果
```

同时检查：

```bash
git grep -n "control_feedforward_pid" drivers/imu samples/imu_test
```

应该能看到新的温控调用链。

---

# 18. 新旧 PID 行为差异必须知道

迁移不是简单字段改名。

## 18.1 D 项

旧：

```text
D = Kd * d(error)/dt
```

新 `control_pid`：

```text
D = -Kd * d(measurement)/dt
```

当：

```text
target_temp = 常数 50°C
```

时：

```text
d(error)/dt
=
-d(measurement)/dt
```

因此稳定恒温过程中两者等价。

而且新实现避免 setpoint step 导致 derivative kick。

---

## 18.2 第一次 D

旧：

```text
last_error = 0
```

第一次：

```text
D = Kd * error / dt
```

可能很大。

新：

```text
reset(current_temperature)
```

第一次 measurement derivative 为 0。

这是预期改善，不需要模仿旧 bug。

---

## 18.3 积分抗饱和

旧 PID：

```text
积分直接累加
再 clamp I
```

新 PID：

```text
考虑输出饱和方向
决定是否提交积分
```

新行为更健壮。

当前温控：

```text
Ki = 0
```

所以本次迁移不会因为 anti-windup 改变当前调试效果。

---

## 18.4 deadband

旧实现：

```text
进入 deadband 后整个 output 直接变 0
```

新 control PID：

```text
只把 feedback error 视为 0
feedforward 仍然可以存在
```

对“恒温加热 + 基础加热前馈”来说，新语义更合理。

当前：

```text
deadband = 0
```

因此本次迁移不会明显受到这个差异影响。

---

# 19. 参数映射表

| 旧 PID | 新 control |
|---|---|
| `Kp` | `feedback.kp` |
| `Ki` | `feedback.ki` |
| `Kd` | `feedback.kd` |
| `max_i_out` | `feedback.integral_min/max` |
| `max_out` | `feedback.output_min/max` |
| `deadband` | `feedback.deadband` |
| `feedforward` 参数 | `feedforward.k_bias` |
| `last_error` | 不直接对应 |
| `p_out` | `result.feedback.p` |
| `i_out` | `result.feedback.i` |
| `d_out` | `result.feedback.d` |
| `output` | `result.output` |

新增必须配置：

```text
derivative_tau_s
dt_min_s
dt_max_s
```

当前迁移建议：

```text
derivative_tau_s = 0
dt_min_s = 0.05
dt_max_s = 0.20
```

因为当前 sample 大约每：

```text
100 ms
```

调用一次恒温控制。

---

# 20. 人工测试顺序

## 20.1 第一步：只验证编译

构建：

```bash
west build -p always -b dm_mc02 samples/imu_test
```

具体 board target 如果当前 workspace 名称不同，以现有项目实际命令为准。

必须确认不存在：

```text
undefined reference to pid_update
CONFIG_SKYWALKER_DRIVER_PID
skywalker,pid binding error
pid-dev property error
```

---

## 20.2 第二步：不上加热负载前检查初始化

确认：

```text
IMU device ready
accel ready
gyro ready
filter ready
heat PWM ready
```

PID 不再出现：

```text
PID device ready
```

因为它已经不是 device。

---

## 20.3 第三步：人工温控测试

上电后观察：

```text
环境温度 → 逐渐升温 → 接近 50°C
```

重点观察：

```text
[ ] 第一次控制没有异常 PWM 冲击
[ ] PWM 始终在 0~20 ms
[ ] 温度能继续升到目标附近
[ ] 没有明显比旧版本更严重的超调
[ ] IMU 姿态输出没有因为迁移受影响
```

本次只迁移架构。

不要一开始就重新调：

```text
Kp
Kd
feedforward
```

先使用原值：

```text
Kp = 6000000
Ki = 0
Kd = 0.02
feedforward = 6750000 ns
max = 20000000 ns
```

确认基本行为一致后，才人工微调。

---

# 21. 异常处理

`imu_heat_control()` 返回错误时：

```text
不要继续沿用上一拍加热命令。
```

更安全的策略是：

```text
设置 heater PWM = 0
然后向上返回错误
```

如果实现这个策略：

```text
PID step 失败
或
输入非法
```

则先尝试：

```text
pwm_set(... pulse = 0 ...)
```

再返回原始错误。

但是不要为了温控错误重置整个 EKF 状态。

---

# 22. 本任务禁止顺手做的事情

不要顺便：

```text
重写 EKF
重写 KalmanFilter driver
修改 BMI08x
改 VOFA
迁移 IMU 为 C++
修改 motor control
修改舵轮代码
重新设计 PWM driver
重新设计整个 IMU API
```

本任务唯一架构目标：

```text
PID 从 Device Driver 层彻底消失
           ↓
control/ 成为唯一 PID 算法实现
           ↓
IMU 恒温改为组合 control PID
```

---

# 23. 建议提交拆分

推荐两到三个提交：

```text
refactor(imu): migrate heater loop to control pid

refactor(build): remove legacy pid driver integration

chore(pid): delete legacy pid device driver
```

也可以合成一个：

```text
refactor(control): remove pid device driver and migrate imu heater
```

但不要和其他舵轮施工混在同一提交。

---

# 24. 最终验收清单

## 架构

```text
[ ] PID 不再属于 drivers/
[ ] PID 不再注册 Zephyr device
[ ] PID 不再有 devicetree compatible
[ ] control/ 是唯一 PID 数值实现
```

## 构建

```text
[ ] drivers/CMakeLists 无 PID driver
[ ] drivers/Kconfig 无 SKYWALKER_DRIVER_PID
[ ] imu_test/prj.conf 无 SKYWALKER_DRIVER_PID
[ ] IMU driver 自动依赖 SKYWALKER_LIB_CONTROL
```

## DTS

```text
[ ] skywalker,pid binding 已删除
[ ] temp_pid node 已删除
[ ] imu binding 无 pid-dev
[ ] heat PID 参数归入 imu node
```

## IMU

```text
[ ] imu_config 不再保存 pid_dev
[ ] imu_data 保存 control PID state
[ ] IMU init validate control config
[ ] 第一次温控用当前温度 reset
[ ] imu_heat_control 使用 control_feedforward_pid_step
[ ] pwm_set 错误能够向上传递
```

## 全仓库

```text
[ ] 无 drivers/pid 引用
[ ] 无 pid_update
[ ] 无 skywalker,pid
[ ] 无 CONFIG_SKYWALKER_DRIVER_PID
[ ] 无 pid-dev / pid_dev
```

## 人工验证

```text
[ ] imu_test 编译通过
[ ] 姿态解算仍正常
[ ] 加热 PWM 在合法范围
[ ] 温度可以稳定接近原 50°C 目标
[ ] 当前原始 PID 参数下效果与迁移前基本一致
```

---

# 25. 最终结构

施工完成后应该变成：

```text
drivers/
├── imu/
│   └── imu.c
├── kalman_filter/
└── motor/

include/
├── control/
│   ├── pid.h
│   ├── feedforward.h
│   └── feedforward_pid.h
│
└── drivers/
    └── imu/
        └── imu.h

lib/
└── control/
    ├── pid.c
    ├── feedforward.c
    └── feedforward_pid.c
```

不存在：

```text
drivers/pid/
include/drivers/pid/
dts/bindings/pid/
```

最终依赖关系：

```text
imu_test
   ↓
IMU
   ↓
control_feedforward_pid
   ↓
control_pid
   ↓
PWM heater
```

而不是：

```text
IMU
 ↓
PID Device Driver
 ↓
PID Algorithm
```

这就是本任务唯一需要守住的核心边界。
