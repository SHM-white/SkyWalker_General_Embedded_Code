# DJI 电机 PID 速度控制示例实施指南

> 适用仓库：当前 `skywalker_code`
>
> 示例平台：`dm_mc02/stm32h723xx`
>
> 示例对象：一台 M3508 + C620，CAN 电流模式
>
> 重要：本文给出应由你亲手创建的 sample；当前业务源码没有被本文自动修改。

> ## 2026-09-05 修订（当前仓库代码状态）
>
> `samples/motor/dji_speed_control/` 现面向台架现有的 GM6020（ID=7、电流环，
> 与 `samples/motor/dji_unified` 同一颗电机）：
>
> - `app.overlay` 使用 `dji,gm6020-current`，`current-limit-ma = <300>`，
>   `gear-ratio-num/den = 1/1`。
> - `src/main.cpp` 已去掉 `console_getchar()` 人工 arm 依赖：反馈就绪后打印
>   3 s 倒计时自动开始，跑 6 s 速度轨迹，结束后显式 0 A + `Bus::stop()`。
>
> 本文其余段落仍以 M3508 + C620 / 人工 `a` arm 为叙述对象，作为通用流程与
> 其他硬件的参考；调用链、安全顺序和错误码表格均不变。

## 1. 这份示例解决什么问题

本示例建立下面这条完整链路：

```text
速度请求 requested_velocity_rad_s
        |
        v
斜率限制器 control_slew_rate_step()
        |  velocity_ref、acceleration_ref
        v
Feedforward PID
control_feedforward_pid_step()
        |  输出单位：A
        v
应用层电流限幅
        |
        v
motor::setCurrent(motor, current_a)
        |  只更新这台电机的命令缓存，不发 CAN
        v
dji::Bus::flush()
        |  汇总同一 CAN 上全部电机，本周期只调用一次
        v
C620 -> M3508

M3508 反馈 CAN 帧
        |
        v
DJI RX callback -> motor::Feedback
        |  velocity_rad_s、valid、timestamp_ms
        +-------------------------> 下一拍控制计算
```

虽然类型名是 `control_feedforward_pid_*`，示例把所有前馈系数设为 0，所以第一版实际就是经典 P 控制器。后续可以在不改变调用链的情况下逐步加入 Ki、kS、kV 和 kA。

不要把 PID 写进 CAN callback、`k_timer` callback 或驱动。控制状态和 `Bus` 生命周期都由一个普通 Zephyr 控制线程独占。

## 2. 当前接口的真实边界

### 2.1 反馈单位

`motor::readFeedback()` 返回 `motor::Feedback`：

- `velocity_rad_s` 是 rad/s。
- 驱动已经用设备树的 `gear-ratio-num / gear-ratio-den` 除过电机转子转速，因此它表示配置所定义的输出侧速度。
- `valid & FeedbackVelocity` 非 0 才能使用速度。
- `timestamp_ms` 必须仍在 `CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS` 内。

设备树减速比写错会直接导致速度反馈和 PID 增益的物理含义错误。本文用 `3591/187` 演示 M3508 标称减速箱比例；上电前仍要用你的实物型号和机械传动重新确认。

### 2.2 输出单位

速度环的 PID 输出是安培：

```text
Kp 单位：A / (rad/s)
Ki 单位：A / ((rad/s) * s)
Kd 单位：A / (rad/s^2)
```

控制器输出范围、application 软件限幅和设备树 `current-limit-ma` 是三层边界。建议三者从同一个经过批准的小电流值出发；application 限幅不能大于设备树限幅。

### 2.3 发送语义

`motor::setCurrent()` 只执行校验、A 到协议 raw 的换算和缓存更新。它不会发送 CAN。真正发送发生在 `Bus::flush()`：

```cpp
setCurrent(motor0, current0);
setCurrent(motor1, current1);
setCurrent(motor2, current2);
bus.flush(report); // 所有电机写完以后，本周期只调用一次
```

绝对不要给每台电机分别 `setCurrent()` 后立即 `flush()`。DJI 的一帧命令共享 4 个电机槽位，拆开发送会破坏同周期批次语义，并可能让其他电机的旧命令因 TTL 超时而使 Bus 进入 Fault。

## 3. 要创建的目录

亲手创建以下文件：

```text
samples/motor/dji_speed_control/
  CMakeLists.txt
  prj.conf
  app.overlay
  src/main.cpp
```

本示例不修改 `drivers/motor/` 或 `lib/control/`。

## 4. 构建文件

### 4.1 `samples/motor/dji_speed_control/CMakeLists.txt`

新建文件并写入：

```cmake
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(dji_speed_control)

target_sources(app PRIVATE src/main.cpp)
```

### 4.2 `samples/motor/dji_speed_control/prj.conf`

新建文件并写入：

```ini
CONFIG_CPP=y
CONFIG_REQUIRES_FULL_LIBCPP=y
CONFIG_CAN=y

CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3

CONFIG_SERIAL=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_CONSOLE_SUBSYS=y
CONFIG_CONSOLE_GETCHAR=y

CONFIG_SKYWALKER_LIB_CONTROL=y
CONFIG_SKYWALKER_DRIVER_MOTOR=y
CONFIG_SKYWALKER_MOTOR_DJI=y

# 5 ms 控制周期必须显著短于这两个超时。
CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS=20
CONFIG_SKYWALKER_DJI_COMMAND_TIMEOUT_MS=20
```

`CONFIG_SKYWALKER_LIB_CONTROL=y` 把纯控制库加入构建；`CONFIG_SKYWALKER_DRIVER_MOTOR=y` 与 `CONFIG_SKYWALKER_MOTOR_DJI=y` 把统一电机接口、DJI device 和 Bus 加入构建。

## 5. 设备树实例

### 5.1 第一阶段：只验证链路，保持 0 A

第一次编译、接线和反馈检查时，`current-limit-ma` 必须保持 0：

```dts
/ {
    aliases {
        motor0 = &m3508_1;
    };

    m3508_1: motor-1 {
        compatible = "dji,m3508-c620";
        status = "okay";
        can-bus = <&can1>;
        motor-id = <1>;
        current-limit-ma = <0>;
        gear-ratio-num = <3591>;
        gear-ratio-den = <187>;
    };
};
```

此阶段即使应用误算出非零电流，`setCurrent()` 也会返回 `-ERANGE`，不会把非零指令写进缓存。先用已有的 `samples/motor/dji_unified` 完成 0 A 验证。

### 5.2 第二阶段：经过台架确认后才允许小电流

确认电机架空、急停有效、CAN 终端和方向正确后，才把 `samples/motor/dji_speed_control/app.overlay` 写成下面形式：

```dts
/ {
    aliases {
        motor0 = &m3508_1;
    };

    m3508_1: motor-1 {
        compatible = "dji,m3508-c620";
        status = "okay";
        can-bus = <&can1>;
        motor-id = <1>;

        /* 300 mA 只是低电流链路示例，不是通用安全值。 */
        current-limit-ma = <300>;

        /* 必须按实物电机和外部传动重新确认。 */
        gear-ratio-num = <3591>;
        gear-ratio-den = <187>;
    };
};
```

如果使用 M2006+C610，把 compatible 换成 `dji,m2006-c610`，同时重新确认减速比、电流边界和全部控制参数。不要把本示例参数直接用于 GM6020 舵向位置环。

## 6. 完整 `src/main.cpp`

下面代码包含：设备初始化、反馈等待、人工 arm、控制状态 reset、真实 dt、反馈 freshness、目标斜坡、PID、电流缓存、集中 flush、有限时运行和故障归零。

```cpp
#include <cerrno>
#include <cmath>
#include <cstdint>

#include <zephyr/console/console.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <control/feedforward_pid.h>
#include <control/slew_rate_limiter.h>
#include <drivers/motor/dji_bus.hpp>
#include <drivers/motor/dji_motor.hpp>
#include <drivers/motor/motor.hpp>

LOG_MODULE_REGISTER(dji_speed_control, LOG_LEVEL_INF);

#define MOTOR0_NODE DT_ALIAS(motor0)

namespace {

constexpr std::int64_t kControlPeriodMs = 5;
constexpr std::int64_t kRunDurationMs = 6000;

/*
 * 以下数字只用于展示单位和调用链，不是 M3508 通用调参结果。
 * 第一次非零上电前，必须经现场安全负责人批准。
 */
constexpr float kRequestedVelocityRadS = 1.0f;
constexpr float kRequestedVelocityAbsMaxRadS = 2.0f;
constexpr float kSoftwareCurrentAbsMaxA = 0.30f;

struct VelocityController {
    control_feedforward_pid_config config{};
    control_slew_rate_config reference_config{};
    control_feedforward_pid_state controller_state{};
    control_slew_rate_state reference_state{};
    control_feedforward_pid_result last_result{};
};

struct VelocityControlOutput {
    float velocity_reference_rad_s = 0.0f;
    float acceleration_reference_rad_s2 = 0.0f;
    float current_command_a = 0.0f;
    control_feedforward_pid_result controller{};
};

skywalker::motor::dji::Bus dji_bus;

float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

VelocityController makeVelocityController()
{
    VelocityController loop{};

    loop.config.feedback = {
        /* P-only direction-check value; it may be too small to overcome friction. */
        .kp = 0.05f,
        .ki = 0.0f,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = 0.0f,
        .integral_max = 0.0f,
        .output_min = -kSoftwareCurrentAbsMaxA,
        .output_max = kSoftwareCurrentAbsMaxA,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 0.020f,
    };

    /* 全零前馈使组合控制器在第一阶段等价于经典 PID。 */
    loop.config.feedforward = {
        .k_bias = 0.0f,
        .k_static = 0.0f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
    };

    loop.reference_config = {
        .rising_rate_per_s = 0.5f,
        .falling_rate_per_s = 0.5f,
    };

    return loop;
}

int validateController(const VelocityController &loop)
{
    int ret = control_feedforward_pid_validate(&loop.config);
    if (ret < 0) {
        return ret;
    }
    return control_slew_rate_validate(&loop.reference_config);
}

int waitForFreshFeedback(const struct device *motor)
{
    const std::int64_t deadline_ms = k_uptime_get() + 2000;

    while (skywalker::motor::getState(motor) !=
           skywalker::motor::State::Ready) {
        if (k_uptime_get() >= deadline_ms) {
            return -ETIMEDOUT;
        }
        k_sleep(K_MSEC(5));
    }
    return 0;
}

int readFreshVelocityFeedback(
    const struct device *motor,
    std::uint64_t now_ms,
    skywalker::motor::Feedback &feedback)
{
    int ret = skywalker::motor::readFeedback(motor, feedback);
    if (ret < 0) {
        return ret;
    }

    if (skywalker::motor::getState(motor) !=
        skywalker::motor::State::Ready) {
        return -EHOSTDOWN;
    }
    if ((feedback.valid & skywalker::motor::FeedbackVelocity) == 0U) {
        return -ENODATA;
    }
    if (!std::isfinite(feedback.velocity_rad_s)) {
        return -EINVAL;
    }
    if (feedback.timestamp_ms == 0U ||
        now_ms < feedback.timestamp_ms ||
        now_ms - feedback.timestamp_ms >
            CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS) {
        return -ESTALE;
    }

    return 0;
}

int waitForManualArmToken()
{
    int ret = console_init();
    if (ret < 0) {
        return ret;
    }

    printk("Motor must be suspended; press 'a' to arm the timed test.\n");
    for (;;) {
        const int ch = console_getchar();
        if (ch < 0) {
            return ch;
        }
        if (ch == 'a' || ch == 'A') {
            return 0;
        }
    }
}

int resetController(VelocityController &loop,
                    float first_velocity_rad_s)
{
    /* 目标从 0 rad/s 开始，不在 arm 瞬间制造阶跃。 */
    int ret = control_slew_rate_reset(&loop.reference_state, 0.0f);
    if (ret < 0) {
        return ret;
    }

    /* 捕获当前测量，保证 reset 后第一拍 D 为 0。 */
    return control_feedforward_pid_reset(
        &loop.controller_state,
        first_velocity_rad_s);
}

int calculateVelocityCurrent(
    VelocityController &loop,
    const skywalker::motor::Feedback &feedback,
    float requested_velocity_rad_s,
    float dt_s,
    VelocityControlOutput &output)
{
    if (!std::isfinite(requested_velocity_rad_s) ||
        !std::isfinite(dt_s)) {
        return -EINVAL;
    }
    if (std::fabs(requested_velocity_rad_s) >
        kRequestedVelocityAbsMaxRadS) {
        return -ERANGE;
    }

    /*
     * 两份状态先复制到局部变量。
     * 如果斜率器成功但 PID 失败，真实状态不会前进半拍。
     */
    control_slew_rate_state next_reference_state =
        loop.reference_state;
    control_feedforward_pid_state next_controller_state =
        loop.controller_state;
    VelocityControlOutput next_output{};

    int ret = control_slew_rate_step(
        &next_reference_state,
        &loop.reference_config,
        requested_velocity_rad_s,
        dt_s,
        &next_output.velocity_reference_rad_s,
        &next_output.acceleration_reference_rad_s2);
    if (ret < 0) {
        return ret;
    }

    const control_feedforward_pid_input input = {
        .feedback = {
            .setpoint = next_output.velocity_reference_rad_s,
            .measurement = feedback.velocity_rad_s,
            .dt_s = dt_s,
            .freeze_integrator = false,
        },
        .reference = {
            .position_ref_rad = 0.0f,
            .velocity_ref = next_output.velocity_reference_rad_s,
            .acceleration_ref =
                next_output.acceleration_reference_rad_s2,
        },
    };

    ret = control_feedforward_pid_step(
        &next_controller_state,
        &loop.config,
        &input,
        &next_output.controller);
    if (ret < 0) {
        return ret;
    }

    next_output.current_command_a = clampFloat(
        next_output.controller.output,
        -kSoftwareCurrentAbsMaxA,
        kSoftwareCurrentAbsMaxA);
    if (!std::isfinite(next_output.current_command_a)) {
        return -ERANGE;
    }

    loop.reference_state = next_reference_state;
    loop.controller_state = next_controller_state;
    loop.last_result = next_output.controller;
    output = next_output;
    return 0;
}

int stopAfterFailure(int original_error)
{
    skywalker::motor::dji::FlushReport report{};
    const int stop_ret = dji_bus.stop(report);

    LOG_ERR("control failed: cause=%d, stop=%d, zero=%d, zero_err=%d",
            original_error,
            stop_ret,
            report.zero_sent ? 1 : 0,
            report.zero_tx_error);

    /* 归零发送失败比原始计算错误更危险，优先向上返回。 */
    return stop_ret < 0 ? stop_ret : original_error;
}

float requestedVelocityForTime(std::int64_t elapsed_ms)
{
    /* 先静止 500 ms，再正转，最后留足时间斜坡回 0。 */
    if (elapsed_ms < 500) {
        return 0.0f;
    }
    if (elapsed_ms < 3000) {
        return kRequestedVelocityRadS;
    }
    return 0.0f;
}

} // namespace

int main()
{
    const struct device *motor = DEVICE_DT_GET(MOTOR0_NODE);
    if (!device_is_ready(motor)) {
        LOG_ERR("motor device not ready");
        return -ENODEV;
    }

    skywalker::motor::dji::Descriptor descriptor{};
    int ret = skywalker::motor::dji::describe(motor, descriptor);
    if (ret < 0 || descriptor.can == nullptr ||
        !device_is_ready(descriptor.can)) {
        LOG_ERR("describe/CAN failed: %d", ret);
        return ret < 0 ? ret : -ENODEV;
    }

    VelocityController loop = makeVelocityController();
    ret = validateController(loop);
    if (ret < 0) {
        LOG_ERR("controller config invalid: %d", ret);
        return ret;
    }

    ret = dji_bus.init(descriptor.can);
    if (ret < 0) {
        LOG_ERR("Bus init failed: %d", ret);
        return ret;
    }

    ret = dji_bus.attach(motor);
    if (ret < 0) {
        LOG_ERR("Bus attach failed: %d", ret);
        return ret;
    }

    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("no fresh feedback before arm: %d", ret);
        return ret;
    }

    ret = waitForManualArmToken();
    if (ret < 0) {
        LOG_ERR("manual arm input failed: %d", ret);
        return ret;
    }

    /* 人工等待期间可能移动了电机，因此 arm 前重新取反馈。 */
    ret = waitForFreshFeedback(motor);
    if (ret < 0) {
        LOG_ERR("feedback lost before arm: %d", ret);
        return ret;
    }

    skywalker::motor::Feedback first_feedback{};
    const std::uint64_t reset_time_ms =
        static_cast<std::uint64_t>(k_uptime_get());
    ret = readFreshVelocityFeedback(motor,
                                    reset_time_ms,
                                    first_feedback);
    if (ret < 0) {
        LOG_ERR("initial feedback invalid: %d", ret);
        return ret;
    }

    ret = resetController(loop, first_feedback.velocity_rad_s);
    if (ret < 0) {
        LOG_ERR("controller reset failed: %d", ret);
        return ret;
    }

    skywalker::motor::dji::FlushReport arm_report{};
    ret = dji_bus.arm(arm_report);
    if (ret < 0 || !arm_report.zero_sent) {
        LOG_ERR("arm/zero failed: ret=%d zero=%d zero_err=%d",
                ret,
                arm_report.zero_sent ? 1 : 0,
                arm_report.zero_tx_error);
        return ret < 0 ? ret : -EIO;
    }

    const std::int64_t run_start_ms = k_uptime_get();
    std::int64_t previous_cycle_ms = run_start_ms;
    std::uint32_t telemetry_divider = 0;

    while (k_uptime_get() - run_start_ms < kRunDurationMs) {
        k_sleep(K_MSEC(kControlPeriodMs));

        const std::int64_t now_signed_ms = k_uptime_get();
        if (now_signed_ms <= previous_cycle_ms) {
            return stopAfterFailure(-ERANGE);
        }

        const float dt_s = static_cast<float>(
            now_signed_ms - previous_cycle_ms) / 1000.0f;
        previous_cycle_ms = now_signed_ms;
        const std::uint64_t now_ms =
            static_cast<std::uint64_t>(now_signed_ms);

        skywalker::motor::Feedback feedback{};
        ret = readFreshVelocityFeedback(motor, now_ms, feedback);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        const float requested_velocity_rad_s =
            requestedVelocityForTime(now_signed_ms - run_start_ms);

        VelocityControlOutput output{};
        ret = calculateVelocityCurrent(loop,
                                       feedback,
                                       requested_velocity_rad_s,
                                       dt_s,
                                       output);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        /* 只更新 motor0 的缓存。 */
        ret = skywalker::motor::setCurrent(
            motor,
            output.current_command_a);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        /* 单电机也必须经过 Bus；本周期只 flush 一次。 */
        skywalker::motor::dji::FlushReport flush_report{};
        ret = dji_bus.flush(flush_report);
        if (ret < 0) {
            return stopAfterFailure(ret);
        }

        if (++telemetry_divider >= 40U) {
            telemetry_divider = 0U;
            printk("req=%d ref=%d vel=%d err=%d p=%d i=%d "
                   "ff=%d out=%d sat=%d age=%llu ms\n",
                   static_cast<int>(requested_velocity_rad_s * 1000.0f),
                   static_cast<int>(
                       output.velocity_reference_rad_s * 1000.0f),
                   static_cast<int>(feedback.velocity_rad_s * 1000.0f),
                   static_cast<int>(
                       output.controller.feedback.error * 1000.0f),
                   static_cast<int>(
                       output.controller.feedback.p * 1000.0f),
                   static_cast<int>(
                       output.controller.feedback.i * 1000.0f),
                   static_cast<int>(output.controller.feedforward * 1000.0f),
                   static_cast<int>(output.current_command_a * 1000.0f),
                   output.controller.feedback.saturated ? 1 : 0,
                   static_cast<unsigned long long>(
                       now_ms - feedback.timestamp_ms));
        }
    }

    /* 正常结束也明确提交一次 0 A，再让 Bus stop 再发全零帧。 */
    ret = skywalker::motor::setCurrent(motor, 0.0f);
    if (ret < 0) {
        return stopAfterFailure(ret);
    }

    skywalker::motor::dji::FlushReport final_flush_report{};
    ret = dji_bus.flush(final_flush_report);
    if (ret < 0) {
        return stopAfterFailure(ret);
    }

    skywalker::motor::dji::FlushReport stop_report{};
    ret = dji_bus.stop(stop_report);
    if (ret < 0 || !stop_report.zero_sent) {
        LOG_ERR("normal stop failed: ret=%d zero=%d zero_err=%d",
                ret,
                stop_report.zero_sent ? 1 : 0,
                stop_report.zero_tx_error);
        return ret < 0 ? ret : -EIO;
    }

    LOG_INF("timed PID velocity test completed and motor stopped");
    return 0;
}
```

## 7. 逐段理解调用顺序

### 7.1 启动阶段

严格按以下顺序：

```text
DEVICE_DT_GET(motor0)
  -> device_is_ready(motor)
  -> dji::describe() 得到物理 CAN
  -> control 配置 validate
  -> bus.init(can)
  -> bus.attach(motor)
  -> 等待 Feedback Ready
  -> 等待人工输入 a
  -> 再次读取新鲜速度
  -> reset 斜率器和 PID 状态
  -> bus.arm()，先发送全零帧
```

`reset()` 只初始化算法状态，不发送电流。PID reset 捕获当前测量，使第一拍 D 为 0；斜率器 reset 到 0，使目标从 0 缓慢变化，而不是直接跳到 1 rad/s。

### 7.2 每个 5 ms 控制周期

每拍顺序固定为：

1. 用 `k_uptime_get()` 计算真实 `dt`，不要把 `0.005f` 写死传给 PID。
2. 读取 `Feedback`，检查 Ready、有效位、finite 和时间戳。
3. 斜率器把外部请求变成受限的 `velocity_ref` 和 `acceleration_ref`。
4. PID 用 `velocity_ref - velocity_measurement` 算安培输出。
5. application 再执行一次软件电流限幅。
6. 对每台电机分别调用 `setCurrent()`。
7. 同一 Bus 的全部缓存更新成功后，只调用一次 `flush()`。
8. 低频发布遥测，不能让打印阻塞每一拍控制。

其中任一步失败，都不能继续沿用上一拍非零命令。进入统一故障路径，调用 `Bus::stop()` 尝试发送全零帧并退出当前 arm 会话。

### 7.3 为什么要复制控制状态

`calculateVelocityCurrent()` 先复制：

```cpp
next_reference_state = loop.reference_state;
next_controller_state = loop.controller_state;
```

全部计算成功后才提交回 `loop`。否则可能出现“斜率目标已经前进一拍，但 PID 因非法 dt 失败”的半状态。控制库内部也采用同样的先计算、后提交语义。

## 8. 实际项目应该怎样组织

sample 为了便于阅读，把所有逻辑放在 `main.cpp`。真实项目建议拆成三层，但数据流保持不变：

```text
目标生产者线程
  遥控器 / 上位机 / 底盘规划
  只发布 requested_velocity + sequence + timestamp
                   |
                   v
唯一 motor-control owner 线程
  读取整批目标与整批反馈
  每台电机执行 slew + PID
  对所有电机 setCurrent
  每条物理 CAN 各 flush 一次
  管理 Safe / Armed / Fault / Recover
                   |
                   v
遥测线程
  只读取 owner 发布的结果快照
  降采样输出，不直接改 PID state 或调用 Bus
```

### 8.1 每台电机独占什么

每台电机、每个控制环必须独占：

```cpp
struct MotorVelocitySlot {
    const struct device *motor;
    control_feedforward_pid_config active_config;
    control_feedforward_pid_state controller_state;
    control_slew_rate_config reference_config;
    control_slew_rate_state reference_state;
    control_feedforward_pid_result last_result;
    float software_current_abs_max_a;
};
```

可以复用控制函数，不能让两台电机共享 `controller_state` 或 `reference_state`。Kp/Ki/FF 参数也不能因为型号相同就默认共用；负载、传动和安装方向都会改变结果。

### 8.2 多电机 owner 的核心循环

真实多电机循环应形如：

```cpp
for (std::size_t i = 0; i < motor_count; ++i) {
    readFreshVelocityFeedback(slots[i].motor, now_ms, feedback[i]);
    calculateVelocityCurrent(slots[i],
                             feedback[i],
                             target[i],
                             dt_s,
                             output[i]);
}

/* 只有整批计算全部成功后，才写全部命令缓存。 */
for (std::size_t i = 0; i < motor_count; ++i) {
    ret = skywalker::motor::setCurrent(
        slots[i].motor,
        output[i].current_command_a);
    if (ret < 0) {
        return stopBusAfterControlFailure(bus, ret);
    }
}

skywalker::motor::dji::FlushReport report{};
ret = bus.flush(report);
if (ret < 0) {
    return stopBusAfterControlFailure(bus, ret);
}
```

当前 `Bus` API 并不是强事务式批量提交：如果第 2 台 `setCurrent()` 失败，第 1 台的新值已经进缓存。因此应用必须立刻走 `stop()`，不能继续 `flush()`。若未来需要严格的 all-or-nothing 批次，应新增 Bus 批量接口，不要用多个线程并发写缓存来模拟。

### 8.3 目标输入不能直接跨线程改状态

遥控器、CAN RX、串口解析器或网络线程只应发布小型目标消息：

```text
motor_index
requested_velocity_rad_s
sequence
timestamp_ms
```

owner 在线程周期边界消费消息。其他线程不能直接：

- 调 `control_*_step()` 修改 PID state；
- 调 `setCurrent()`；
- 调 `Bus::arm/flush/stop/recover()`。

原因是 `Bus` 自身的生命周期和整批 flush 没有对外提供总锁。单一 owner 是当前实现下最明确的并发边界。

## 9. 主要错误码与应对

| 返回值         | 常见来源                            | 本示例应对                                    |
| -------------- | ----------------------------------- | --------------------------------------------- |
| `-EINVAL`    | 空指针、NaN/Inf、非法控制配置       | 不发送新命令，立即 stop                       |
| `-ERANGE`    | dt 越界、计算溢出、电流超过 DT 上限 | 立即 stop；检查周期与三层限幅                 |
| `-EACCES`    | 未 reset、未 arm、Bus 状态不允许    | 回到 Safe/Fault 状态机，不重试旧命令          |
| `-EHOSTDOWN` | 电机反馈不再 Ready                  | 立即 stop，检查供电/CAN/反馈周期              |
| `-ESTALE`    | 反馈或命令 TTL 过期、epoch 不匹配   | 立即 stop；Fault 时先 recover，再重新人工 arm |
| `-ETIMEDOUT` | 启动等待反馈超时                    | 保持零输出，不允许 arm                        |

如果 `flush()` 已经让 Bus 进入 `Fault`，不能直接再次 `arm()`。先排除原因，再由 owner 调 `recover()`；recover 成功只回到 Safe，仍需重新读取新鲜反馈、reset 控制器并获得新的人工 arm token。

## 10. 建议的编译命令

创建完四个 sample 文件后，由你执行：

```bash
west build -p always \
  -b dm_mc02/stm32h723xx \
  samples/motor/dji_speed_control \
  --build-dir build/dji_speed_control
```

编译成功后再烧录：

```bash
west flash -d build/dji_speed_control
```

不要为了让编译通过而同时启用旧的 `CONFIG_SKYWALKER_DRIVER_PID`。本示例使用的是 `lib/control`，不是 `drivers/pid` 的旧 device PID。

## 11. 硬件上电顺序

1. 第一次只构建并运行 `samples/motor/dji_unified`，设备树电流上限保持 0 A。
2. 断开电机动力后检查电机 ID=1、CAN 1 Mbps、CAN_H/CAN_L、共地与两端 120 Ω 终端。
3. 把电机和减速输出完全架空，拆除可能击打人员的机构，准备物理断电手段。
4. 手转输出轴，确认遥测速度的正负方向和数量级；同时确认减速比定义。
5. 确认控制周期约 5 ms、feedback age 小于 20 ms，且没有持续丢帧。
6. 只在完成风险评估后，把 DT 从 0 mA 改为经过批准的小电流；软件上限和 PID 输出范围不得更大。
7. 首轮保持 `ki=kd=kS=kV=kA=0`，用很小 Kp 和 1 rad/s 以下的限时目标验证负反馈方向。
8. 输入 `a` 后手不离物理断电；如果给正目标后速度向负方向增长，立即断电，不要靠继续增大 Kp 修正。
9. 试验结束必须看到 0 A flush 和 `Bus::stop()` 成功；再接触机构。

## 12. 预期现象

方向和接线正确时：

- arm 前电机不转；`Bus::arm()` 首先发送全零帧。
- 前 500 ms 请求为 0。
- 目标以 0.5 rad/s² 从 0 上升，不发生目标阶跃。
- `p` 与速度误差同号，正目标时输出为小的正电流。
- 3 s 后目标回到 0，斜率器缓慢降速。
- 6 s 时显式发送 0 A，随后 `Bus::stop()` 再发送全零帧。
- 任何反馈/命令超时或 CAN 发送错误都会退出闭环，不沿用旧非零命令。

示例 Kp 很小，电机因静摩擦完全不转也是合理结果。此时先确认链路和方向，不要直接大幅增加 Kp 或电流上限。

## 13. 常见故障排查

### `setCurrent()` 返回 `-ERANGE`

- 检查 `current-limit-ma` 是否仍为 0；0 A 阶段这是预期保护。
- 检查 PID `output_min/max` 和 application 限幅是否超过 DT 上限。
- 确认控制输出单位是 A，不是 mA 或协议 raw。

### `Bus::arm()` 返回 `-EHOSTDOWN`

- 电机没有反馈或反馈超过 freshness timeout。
- 检查电机 ID、CAN 1 Mbps、供电、终端电阻和实际接入的 CAN 控制器。

### `flush()` 返回 `-ESTALE`

- 某台 attached 电机本拍没有成功执行 `setCurrent()`。
- 控制循环阻塞时间超过 command timeout。
- 旧 arm 会话的命令不能用于新 epoch。

### 正目标时速度持续变负

这是正反馈或方向定义错误。立即硬件断电，检查电机安装方向、反馈方向和应用目标方向。公共 control 库拒绝负 Kp/Ki/Kd，不应通过写负增益偷偷掩盖方向问题。

### dt 偶尔超过 20 ms

控制线程被日志、其他高优先级线程或阻塞调用拖延。不要简单放宽 dt/TTL 掩盖问题；先去掉逐拍打印，检查线程优先级和 CAN 发送时延。遥测应由独立线程读取低频快照。

## 14. 调参进入顺序

1. 先只用小 Kp，Ki、Kd 和全部 FF 保持 0。
2. 正负两个方向分别做相同的短轨迹，确认负反馈和无持续振荡。
3. 用稳定速度下的“维持电流—速度”数据标定 kS/kV，不凭感觉猜。
4. 前馈稳定后仍存在重复稳态误差，才加入小 Ki，并给 I 项设置小的对称范围。
5. 只有加速段存在重复误差时才考虑 kA。
6. 速度反馈噪声足够低且确实需要时才考虑 Kd；速度环通常先保持 Kd=0。

每次只改一个参数，保存目标、反馈、P/I/D/FF、未饱和输出、最终输出、饱和标志、dt、feedback age 和温度。任何参数都不存在跨电机、跨减速机构通用值。

## 15. 最终检查清单

- [ ] `CONFIG_SKYWALKER_LIB_CONTROL=y` 已启用。
- [ ] sample 没有启用或 include 旧 `drivers/pid`。
- [ ] 设备树型号、电机 ID、CAN 和减速比与实物一致。
- [ ] 第一次验证保持 `current-limit-ma=0`。
- [ ] 非零电流值已经过现场安全评审，电机和机构架空。
- [ ] 速度反馈 valid、finite、fresh，单位确认为 rad/s。
- [ ] 控制器在 arm 前用最新反馈 reset。
- [ ] `dt` 来自真实相邻控制周期。
- [ ] 一台电机的控制 state 不与其他电机共享。
- [ ] 只有一个 owner 线程调用 setCurrent 和 Bus 生命周期方法。
- [ ] 同一 Bus 的所有 setCurrent 成功后，本周期只 flush 一次。
- [ ] 任一失败都进入 stop/归零路径，不复用旧非零命令。
- [ ] Fault 只能先 recover，再重新 reset 和人工 arm。
- [ ] 正常退出也显式 0 A、flush、stop。

## 16. 可直接对照的现有实现

- `samples/motor/dji_unified/src/main.cpp`：复用 device、Bus、反馈等待、人工 arm 和全零故障路径。
- `include/drivers/motor/motor.hpp`：复用 `readFeedback()`、`setCurrent()`、`getState()`。
- `include/drivers/motor/dji_bus.hpp`：复用 Bus 生命周期和 `FlushReport`。
- `include/control/feedforward_pid.h`：复用组合控制器配置、状态、输入和结果。
- `include/control/slew_rate_limiter.h`：复用安全目标斜坡。
- `DJI统一电机架构说明.md`：核对共享命令帧、owner、TTL、epoch 和 Fault 语义。

不能照搬的部分：`dji_unified` 当前只持续发送 0 A，不包含 PID；旧 `drivers/pid` 是 device 化旧接口，也不应接入新的电机闭环。
