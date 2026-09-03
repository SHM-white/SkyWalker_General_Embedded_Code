# SkyWalker 古法编程线性实施指南

> 适用仓库：当前 `skywalker_code` 工作区
>
> 整理日期：2026-09-04
>
> Zephyr 基线：`west.yml` 固定提交 `6085aadeb337f27ee7411fa2ecbe8cf49164e360`
>
> 目标平台：`dm_mc02/stm32h723xx`，新增 `rm_typec` 板级定义尚未纳入控制链验证
>
> 语言边界：C11 + C++20
>
> 文档角色：只列从当前代码基线继续向前的施工顺序；已经进入源码的 DJI 重构不再作为待办重复展开。

这是一份“古法编程”手把手实施指南。本文不会替你修改业务源码、测试、构建文件或设备树；代码块是你后续逐文件实施时的参考。每次只完成一个小节，保存测试或抓包证据后再继续。

## 阅读规则：按“完全没学过这套写法”来读

本文从这里开始不假设你已经理解某个缩写、C/C++ 语法或 Zephyr 机制。当前可施工范围是第 1～8 步；新建的纯算法、配置、调参状态机和起步测试文件给出完整内容，旧的硬件调用方则给出“删什么、加什么、放在哪里”的精确局部替换。与未确认实物参数有关的示例只给零输出骨架，并明确标出不能照抄的数值。每种新语法/框架写法在首次出现后紧跟解释。第 9～12 步只是依赖尚未确定的整车路线图，未进入对应步骤前不要照着猜代码。

文档里的操作词只有三种含义：

| 操作词 | 你要做什么 |
| --- | --- |
| “新建文件” | 创建不存在的文件，把该代码块从第一行复制到最后一行 |
| “整文件替换” | 删除目标文件现有内容，再完整粘贴代码块；不能把新旧实现拼在一起 |
| “局部替换” | 用文档给出的旧代码定位，只替换明确指定的一段；未提到的代码保持原样 |

第一次出现的基础词先在这里解释：

| 写法 | 最小解释 |
| --- | --- |
| config | 不随每一拍变化的参数，如 Kp、输出上限；启动时复制到 RAM 后可形成 active/pending 两份 |
| state | 控制器必须记住的历史，如积分和上一拍测量；一台电机的一个环独占一份 |
| input/result | 本拍输入和本拍计算结果；把它们做成结构体是为了不漏字段和便于遥测 |
| setpoint/reference | 想要达到的目标；velocity reference 的单位是 rad/s |
| measurement | 传感器反馈；必须检查有效位、时间戳和 finite |
| `dt` | 相邻两次真正执行控制计算的时间差，单位秒；不是写死的“期望周期” |
| finite | 不是 NaN、正无穷或负无穷；C 用 `isfinite(x)`，C++ 用 `std::isfinite(x)` |
| clamp | 把值限制在 `[min,max]`；小于 min 取 min，大于 max 取 max |
| `const T *p` | `p` 指向一个只读 `T`；函数可以读取 `*p`，不能通过它修改对象 |
| `T *p` | `p` 是地址；`*p` 表示地址里的对象，`&object` 表示取得对象地址 |
| `nullptr` / `NULL` | 空指针；C++ 用 `nullptr`，C 用 `NULL` |
| `extern "C"` | 让 C++ 调用 C 函数时保持 C 的符号名；否则链接器会找不到 C 实现 |
| `static` 局部函数 | 只允许当前 `.c/.cpp` 文件使用，不导出为公共接口 |
| `#include` | 把另一个头文件里的声明引入当前编译单元；尖括号用于公共头，相对引号用于同目录私有头 |
| Kconfig | 决定某项功能是否参与构建；`CONFIG_...=y` 才启用 |
| CMake | 决定启用后具体编译哪些 `.c/.cpp` 文件 |
| devicetree | 描述板上硬件实例和接线，不是运行时 PID 参数数据库 |
| callback | 框架在事件发生时调用的小函数；中断/驱动 callback 不能执行阻塞操作和完整控制计算 |
| owner thread | 唯一拥有可写控制状态、并调用该 Bus 生命周期/flush 的线程 |
| `k_msgq` | Zephyr 固定容量消息队列；callback 复制小消息进去，owner thread 再取出处理 |
| `-EINVAL` | 参数本身非法，例如空指针、NaN 或上下限反向 |
| `-ERANGE` | 参数形式合法，但数值超出允许范围，例如 `dt` 太大或输出超过安全上限 |
| `-EACCES` | 当前状态不允许操作，例如未开始调参会话就写参数 |
| `-EAGAIN` | 条件暂时未满足，保持旧状态，稍后可以重试 |
| `-ESTALE` | 序号重复、回退或缺号，本批调参消息已不可信 |
| `-ENODATA` | 会话存在，但没有任何待提交字段 |
| `-EOVERFLOW` | revision/序号已达到整数上限，拒绝回绕成 0 |
| `-ETIMEDOUT` | 过了允许时间仍未获得新鲜反馈 |

看到函数返回 `int` 时必须先保存和检查：

```c
int ret = some_function();
if (ret < 0) {
    /* 这里进入明确的安全/报错路径，不能假装成功。 */
    return ret;
}
```

看到输出指针时，成功后才读取输出：

```c
float output = 0.0f;
int ret = calculate_something(&output);
if (ret < 0) {
    return ret;
}
/* 只有走到这里，output 才是本次成功结果。 */
```

## 当前基线：这些内容不再列入待办

以下结论来自对当前工作树的静态核对，只表示代码结构已经存在，不等于硬件验收完成：

| 已进入源码的基线                             | 当前落点                                                                        |
| -------------------------------------------- | ------------------------------------------------------------------------------- |
| 型号无关的电机 API                           | `include/drivers/motor/motor.hpp`                                             |
| DJI 纯协议、三型号 profile、统一 device core | `drivers/motor/dji/dji_protocol.cpp`、`dji_profiles.cpp`、`dji_motor.cpp` |
| 每条物理 CAN 的唯一发送 owner                | `drivers/motor/dji/dji_bus.cpp`、`include/drivers/motor/dji_bus.hpp`        |
| M3508/C620、M2006/C610、GM6020 电流模式实例  | 三个`dji_*_instance.cpp` 与四个 motor binding                                 |
| 默认 0 A、人工 arm 的统一示例                | `samples/motor/dji_unified/`                                                  |
| 独立 DJI 架构说明                            | `DJI统一电机架构说明.md`                                                      |

因此，旧文档中的“删除 M3508 旧接口、创建 profile/core/Bus、创建三型号 binding、创建统一 sample”等施工流程已经删除。以后查 DJI 分组发送、生命周期、错误码和并发边界，直接读 `DJI统一电机架构说明.md`，不要在本线性待办里再实现第二套。

仍未获得证据的事项不能伪装成“完成”：

- 当前仓库没有 `tests/`，DJI route、codec、冲突、TTL、epoch 和全零故障路径尚无可见 Twister 测试。
- 没有当前统一 sample 的 clean-build 记录；仓库中旧 `dji_m3508_driver/build/` 产物不能证明新实现可构建。
- M3508、M2006、GM6020 的实物方向、减速比、电流量程、反馈 freshness 和安全电流仍需台架核对。
- GM6020 必须确认实际固件和电流环状态；设备树中的 `current-loop-confirmed` 只是人工声明，不是运行时证明。
- `dji::Bus::flush()` 是集中提交，但当前 API 不是强事务式的多电机批量提交；唯一 owner 线程仍是应用层契约。

这些证据项保留为进入电机闭环前的验收门，不再拆成一串“继续写 DJI 驱动”的待办。

## 当前真正起点与目标

当前 `drivers/pid/pid.c` 有六个结构性问题：

1. 它被注册成 Zephyr device，但 PID 是纯算法，不是硬件驱动。
2. 注释称“增量式”，实际公式是位置式 PID。
3. `dt` 未校验就作为除数，`0`、负值、NaN 或长时间停顿都会污染状态。
4. D 对误差求导，设定值阶跃会产生 derivative kick。
5. deadband 内提前返回，连调用者传入的前馈也一起吞掉。
6. 只有积分项硬钳位，没有结合最终输出饱和做 anti-windup，也没有显式 reset。

唯一现有调用者是 `drivers/imu/imu.c::imu_heat_control()`。它把 PID 状态藏在 `pid_dev->data`，把常量加热偏置作为函数参数传入，同时忽略 `pwm_set()` 返回值。后续电机控制不能沿用这套所有权。

本轮目标是先建立可在 `native_sim` 上验证的控制算法层：

```text
轨迹/目标生成
  position_ref, velocity_ref, acceleration_ref
                    |
                    +--> 前馈模型 kBias/kS/kV/kA/kG -----+
                    |                                    |
测量值 ------------+--> 经典反馈 PID -------------------+--> 合成限幅
                                                         |
                                                         v
                                               应用层安全限幅/执行器
                                                         |
                                                         v
                                             motor::setCurrent + Bus::flush
```

关键裁决：

- “经典 PID”只有反馈 P/I/D，不把前馈参数塞进它的公开输入。
- “前馈 PID”是前馈模型与经典 PID 的组合控制器，不复制一份 PID 算法。
- 组合控制器必须按“PID + FF 的总输出”做饱和和 anti-windup；不能先把 PID 独立钳位再盲加 FF。
- 算法层不 include Zephyr device、CAN、PWM、日志或线程 API，不分配动态内存。
- 控制器配置、运行状态和实例数量由调用者拥有；一台电机的一个环独占一份状态。
- 方向、单位换算、软硬限位、反馈 freshness 和故障停机不属于 PID。

现有代码只复用这些事实，不照搬旧结构：

- 可复用 `pid_update()` 当前的 Kp/Ki/Kd、I 限幅、总输出限幅和 deadband 参数含义，作为迁移输入；旧参数值仍需重新验证。
- 可复用 `imu_heat_control()` 的温度单位、20 ms PWM period 和 `HEAT_OFFSET_NS` 作为行为对照；不能照搬其“忽略返回值、函数外截负值”的错误路径。
- 可复用 `motor::Feedback` 的 SI 单位与 `dji::Bus` 的安全生命周期；PID 不得 include 或调用这些驱动接口。
- 不复用 `pid_dev->data/config`、设备树虚拟 PID device、对 error 求导、deadband 提前返回或调用者随手传入的 `feedforward` 标量。

## 从现在开始的唯一线性流程

```text
第 1 步  建立纯 control library 与公共类型
   ↓
第 2 步  实现并测试经典位置式 PID
   ↓
第 3 步  实现前馈模型与组合式 Feedforward PID
   ↓
第 4 步  增加目标斜坡、角度工具和全部纯测试
   ↓
第 5 步  把 IMU 温控迁移为第一个真实调用者，删除旧 PID device
   ↓
第 6 步  补 DJI 现有实现的纯测试并完成 0 A 台架验收
   ↓
第 7 步  建立唯一 owner 的 Zephyr 控制线程
   ↓
第 8 步  M3508/M2006 速度 PI + 前馈单轴闭环
   ↓
第 9 步  GM6020 舵向位置-速度串级与大小 Yaw 独立闭环
   ↓
第 10 步 舵轮纯数学、单模块、四模块与底盘状态机
   ↓
第 11 步 拆分 chassis/gimbal 两个 application 与板间协议
   ↓
第 12 步 大小 Yaw 协调、裁判 RX、功率策略、UI 与发射机构
```

DM-J4310 MIT 是独立协议族。没有当前整车使用需求时把它放在旁路线，不阻塞上述主线；确认要使用后再在第 6 步与 DJI 做总线 ID 冲突检查。

---

## 第 1 步：建立纯 control library

### 1.1 目标目录

按下面顺序创建文件。控制算法放 `lib/`，不要继续扩展 `drivers/pid/`：

```text
include/control/
  pid.h                    经典反馈 PID 公共接口
  feedforward.h            kBias/kS/kV/kA/kG 纯前馈模型
  feedforward_pid.h        组合控制器公共接口
  slew_rate_limiter.h      目标变化率限制
  angle.h                  角度归一化和连续角工具

lib/control/
  CMakeLists.txt
  control_internal.h       只供本目录使用的 PID 合成核心
  pid.c
  feedforward.c
  feedforward_pid.c
  slew_rate_limiter.c
  angle.c

tests/control/
  CMakeLists.txt
  prj.conf
  testcase.yaml
  src/main.c
```

构建接线分三个文件做。下面不是三种可选写法，而是三个都要做。

第一个文件：`lib/CMakeLists.txt`

操作：整文件替换为下面三行。前两行是仓库已有内容，第三行是新增内容。

```cmake
add_subdirectory_ifdef(CONFIG_SKYWALKER_LIB_MATRIX matrix)
add_subdirectory_ifdef(CONFIG_SKYWALKER_LIB_VOFA vofa)
add_subdirectory_ifdef(CONFIG_SKYWALKER_LIB_CONTROL control)
```

`add_subdirectory_ifdef(A B)` 的意思是：只有 Kconfig 符号 `A` 被选为 `y` 时，CMake 才继续进入子目录 `B`。第三行因此把 `CONFIG_SKYWALKER_LIB_CONTROL=y` 与 `lib/control/` 连起来。

第二个文件：`lib/Kconfig`

操作：在文件最后的 `endmenu` **之前**插入下面整段。`endmenu` 要保留，不能把新配置写在它后面。

```kconfig
config SKYWALKER_LIB_CONTROL
    bool "Pure control algorithms"
    help
        Enable allocation-free PID, feedforward and reference helpers.
```

`config SKYWALKER_LIB_CONTROL` 定义配置符号；`bool` 说明它只有开/关两种状态；`help` 是给 Kconfig 界面和以后阅读者的说明。这个库只依赖 C 标准库，所以此处不需要 `depends on`。

第三个文件：`lib/control/CMakeLists.txt`

操作：新建文件，完整粘贴。

```cmake
zephyr_library()

zephyr_library_sources(
    pid.c
    feedforward.c
    feedforward_pid.c
    slew_rate_limiter.c
    angle.c
)
```

`zephyr_library()` 先创建一个属于当前子目录的 Zephyr library target；`zephyr_library_sources(...)` 再把括号中的源文件加入它。换行只是为了好读，CMake 会把括号内所有文件名当成同一个命令的参数。

公共头只依赖 C 标准头。需要 `bool` 时 include `<stdbool.h>`，需要错误码时实现在 `.c` 中 include `<errno.h>`。每个公共头提供 C++ 链接保护：

```c
#ifdef __cplusplus
extern "C" {
#endif

/* declarations */

#ifdef __cplusplus
}
#endif
```

这样 C 的 IMU 和 C++ 的电机 application 可以共用同一套算法，不需要桥接层。

### 1.2 通用错误语义

所有 `validate/reset/step` 接口采用一致语义：

| 返回值      | 含义                                        | 状态是否改变         |
| ----------- | ------------------------------------------- | -------------------- |
| `0`       | 成功                                        | 按接口说明提交新状态 |
| `-EINVAL` | 空指针、非有限输入、配置上下限反向或负增益  | 不改变               |
| `-ERANGE` | `dt` 超出配置范围或中间计算溢出为非有限值 | 不改变               |
| `-EACCES` | 使用顺序不合法，例如尚未 `reset()` 就 `step()` | 不改变 |

实现时先复制 `next = *state`，所有计算写入 `next` 和局部 `result`，全部验证成功后才执行 `*state = next; *out = local_result;`。错误路径不得留下“积分已加、函数却报错”的半状态。

线程语义也统一：算法对象自身不加锁。一个实例只能由一个控制线程写；遥测线程只能读取控制线程发布的结果快照。ISR、CAN RX callback 和 `k_timer` callback 都不执行 PID。

本步自检：公共头不出现 `zephyr/`、`device`、`can`、`pwm`、`k_mutex`；编译单元没有静态共享控制状态；每个状态对象都能由调用者静态分配。

---

## 第 2 步：封装经典反馈 PID

### 2.1 公共接口

目标文件：`include/control/pid.h`

```c
#ifndef SKYWALKER_CONTROL_PID_H
#define SKYWALKER_CONTROL_PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;

    /* D 项是一阶低通后的 measurement rate；0 表示不滤波。 */
    float derivative_tau_s;

    /* 这里限制的是 I 项输出，不是未乘 Ki 的误差积分。 */
    float integral_min;
    float integral_max;

    /* 最终合成输出范围，允许不对称，例如加热器为 0..period。 */
    float output_min;
    float output_max;

    float deadband;
    float dt_min_s;
    float dt_max_s;
} control_pid_config;

typedef struct {
    float integral_output;
    float previous_measurement;
    float filtered_measurement_rate;
    bool initialized;
} control_pid_state;

typedef struct {
    float setpoint;
    float measurement;
    float dt_s;
    bool freeze_integrator;
} control_pid_input;

typedef struct {
    float error;
    float effective_error;
    float p;
    float i;
    float d;
    float feedback_unsaturated;
    float total_unsaturated;
    float output;
    bool saturated;
} control_pid_result;

int control_pid_validate(const control_pid_config *config);

int control_pid_reset(control_pid_state *state,
                      float current_measurement);

int control_pid_step(control_pid_state *state,
                     const control_pid_config *config,
                     const control_pid_input *input,
                     control_pid_result *result);

#ifdef __cplusplus
}
#endif

#endif
```

参数与状态边界：

- `kp/ki/kd >= 0`；控制方向由传感器/执行器方向配置决定，不靠负增益偷偷修正。
- `derivative_tau_s >= 0`、`deadband >= 0`、`dt_min_s > 0`、`dt_max_s >= dt_min_s`。
- `integral_min <= integral_max`、`output_min <= output_max`，所有配置必须 finite。允许 `output_min=output_max=0` 作为明确的零输出配置；非零闭环必须使用严格递增范围。
- `reset()` 捕获当前测量值并把 D 滤波状态清零，保证恢复后的第一拍 D 为 0；它不向执行器发送任何值。
- `freeze_integrator=true` 用于输出被更外层安全逻辑禁止、模式切换过渡或执行器暂时不可用的场景；它不停止 P/D 的诊断计算。

`reset()` 成功后的状态必须确定：`integral_output=0`、`previous_measurement=current_measurement`、`filtered_measurement_rate=0`、`initialized=true`。measurement 非有限时返回 `-EINVAL`，保留旧状态。

### 2.2 唯一计算顺序

目标文件：`lib/control/pid.c`

经典 PID 使用位置式公式，D 对测量求导：

```text
error = setpoint - measurement
effective_error = abs(error) <= deadband ? 0 : error

P = kp * effective_error
measurement_rate = (measurement - previous_measurement) / dt
D = -kd * low_pass(measurement_rate)
I_candidate = clamp(I + ki * effective_error * dt,
                    integral_min, integral_max)

unsaturated = P + I_candidate + D
output = clamp(unsaturated, output_min, output_max)
```

第一次 `step()` 若状态尚未初始化，应把当前 measurement 当作 previous measurement，所以 D=0。低通系数使用真实 `dt`：

```c
if (config->derivative_tau_s == 0.0f) {
    filtered_rate = measurement_rate;
} else {
    const float alpha =
        input->dt_s / (config->derivative_tau_s + input->dt_s);
    filtered_rate = previous_filtered_rate
                  + alpha * (measurement_rate - previous_filtered_rate);
}
```

deadband 只把本拍 feedback error 变成 0，不提前返回。这样 D 状态仍跟踪测量，已有积分不会被莫名清除，组合控制器的前馈也不会消失。

### 2.3 anti-windup

先算候选积分，再判断它是否把输出继续推向饱和：

```c
const float candidate_total = p + i_candidate + d + additive_output;
const float limited = clampf(candidate_total,
                             config->output_min,
                             config->output_max);

const bool pushes_further =
    (candidate_total - limited) * effective_error > 0.0f;

if (!input->freeze_integrator && !pushes_further) {
    next.integral_output = i_candidate;
}
```

上面的 `additive_output` 在经典 PID 中固定为 0，在第 3 步由组合式前馈 PID 传入。它必须位于 `lib/control/control_internal.h` 声明的目录私有 `control_pid_step_core()` 中；公共 `control_pid_step()` 只是以 0 调用核心。这样外部用户看到的经典 PID API 仍然纯粹，而两种控制器共用同一套 D、积分和饱和逻辑。

目录私有声明固定为：

```c
int control_pid_step_core(control_pid_state *state,
                          const control_pid_config *config,
                          const control_pid_input *input,
                          float additive_output,
                          control_pid_result *result);
```

积分被接受或冻结后，必须用最终的 `next.integral_output` 重新计算 `feedback_unsaturated`、`total_unsaturated`、`output` 和 `saturated`，不能直接返回候选积分对应的旧结果。

如果候选计算、滤波或合成结果出现 Inf/NaN，返回 `-ERANGE`，不提交 `next`。不要依赖最终 clamp 把 Inf“修好”。

### 2.4 两个实现文件的完整内容

目标文件：`lib/control/control_internal.h`

操作：新建文件，完整粘贴。这个头只给 `lib/control/` 内部使用，所以 application 不 include 它。

```c
#ifndef SKYWALKER_CONTROL_INTERNAL_H
#define SKYWALKER_CONTROL_INTERNAL_H

#include <control/pid.h>

int control_pid_step_core(control_pid_state *state,
                          const control_pid_config *config,
                          const control_pid_input *input,
                          float additive_output,
                          control_pid_result *result);

#endif
```

目标文件：`lib/control/pid.c`

操作：新建文件，完整粘贴。

```c
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include <control/pid.h>

#include "control_internal.h"

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static bool pid_config_fields_are_finite(
    const control_pid_config *config)
{
    return isfinite(config->kp) &&
           isfinite(config->ki) &&
           isfinite(config->kd) &&
           isfinite(config->derivative_tau_s) &&
           isfinite(config->integral_min) &&
           isfinite(config->integral_max) &&
           isfinite(config->output_min) &&
           isfinite(config->output_max) &&
           isfinite(config->deadband) &&
           isfinite(config->dt_min_s) &&
           isfinite(config->dt_max_s);
}

int control_pid_validate(const control_pid_config *config)
{
    if (config == NULL) {
        return -EINVAL;
    }
    if (!pid_config_fields_are_finite(config)) {
        return -EINVAL;
    }
    if (config->kp < 0.0f ||
        config->ki < 0.0f ||
        config->kd < 0.0f ||
        config->derivative_tau_s < 0.0f ||
        config->deadband < 0.0f) {
        return -EINVAL;
    }
    if (config->integral_min > config->integral_max ||
        config->output_min > config->output_max) {
        return -EINVAL;
    }
    if (config->dt_min_s <= 0.0f ||
        config->dt_max_s < config->dt_min_s) {
        return -EINVAL;
    }
    return 0;
}

int control_pid_reset(control_pid_state *state,
                      float current_measurement)
{
    if (state == NULL || !isfinite(current_measurement)) {
        return -EINVAL;
    }

    control_pid_state next = {0};
    next.integral_output = 0.0f;
    next.previous_measurement = current_measurement;
    next.filtered_measurement_rate = 0.0f;
    next.initialized = true;

    *state = next;
    return 0;
}

int control_pid_step_core(control_pid_state *state,
                          const control_pid_config *config,
                          const control_pid_input *input,
                          float additive_output,
                          control_pid_result *result)
{
    if (state == NULL || input == NULL || result == NULL) {
        return -EINVAL;
    }

    int ret = control_pid_validate(config);
    if (ret < 0) {
        return ret;
    }

    if (!isfinite(input->setpoint) ||
        !isfinite(input->measurement) ||
        !isfinite(input->dt_s) ||
        !isfinite(additive_output)) {
        return -EINVAL;
    }
    if (input->dt_s < config->dt_min_s ||
        input->dt_s > config->dt_max_s) {
        return -ERANGE;
    }

    if (state->initialized &&
        (!isfinite(state->integral_output) ||
         !isfinite(state->previous_measurement) ||
         !isfinite(state->filtered_measurement_rate))) {
        return -EINVAL;
    }
    if (state->initialized &&
        (state->integral_output < config->integral_min ||
         state->integral_output > config->integral_max)) {
        return -ERANGE;
    }

    control_pid_state next = *state;
    control_pid_result local = {0};
    bool first_step = !next.initialized;

    if (first_step) {
        next.integral_output = 0.0f;
        next.previous_measurement = input->measurement;
        next.filtered_measurement_rate = 0.0f;
        next.initialized = true;
    }

    const float error = input->setpoint - input->measurement;
    if (!isfinite(error)) {
        return -ERANGE;
    }

    const float effective_error =
        fabsf(error) <= config->deadband ? 0.0f : error;

    const float p = config->kp * effective_error;
    if (!isfinite(p)) {
        return -ERANGE;
    }

    float measurement_rate = 0.0f;
    if (!first_step) {
        measurement_rate =
            (input->measurement - next.previous_measurement) /
            input->dt_s;
        if (!isfinite(measurement_rate)) {
            return -ERANGE;
        }
    }

    float filtered_rate = measurement_rate;
    if (config->derivative_tau_s > 0.0f) {
        const float denominator =
            config->derivative_tau_s + input->dt_s;
        const float alpha = input->dt_s / denominator;
        filtered_rate = next.filtered_measurement_rate +
                        alpha * (measurement_rate -
                                 next.filtered_measurement_rate);
    }
    if (!isfinite(filtered_rate)) {
        return -ERANGE;
    }

    const float d = -config->kd * filtered_rate;
    if (!isfinite(d)) {
        return -ERANGE;
    }

    const float integral_delta =
        config->ki * effective_error * input->dt_s;
    const float integral_unclamped =
        next.integral_output + integral_delta;
    if (!isfinite(integral_delta) ||
        !isfinite(integral_unclamped)) {
        return -ERANGE;
    }

    const float integral_candidate =
        clampf(integral_unclamped,
               config->integral_min,
               config->integral_max);

    const float candidate_feedback =
        p + integral_candidate + d;
    const float candidate_total =
        candidate_feedback + additive_output;
    if (!isfinite(candidate_feedback) ||
        !isfinite(candidate_total)) {
        return -ERANGE;
    }

    const bool pushes_high_saturation =
        candidate_total > config->output_max &&
        effective_error > 0.0f;
    const bool pushes_low_saturation =
        candidate_total < config->output_min &&
        effective_error < 0.0f;

    if (!input->freeze_integrator &&
        !pushes_high_saturation &&
        !pushes_low_saturation) {
        next.integral_output = integral_candidate;
    }

    next.previous_measurement = input->measurement;
    next.filtered_measurement_rate = filtered_rate;

    local.error = error;
    local.effective_error = effective_error;
    local.p = p;
    local.i = next.integral_output;
    local.d = d;
    local.feedback_unsaturated =
        local.p + local.i + local.d;
    local.total_unsaturated =
        local.feedback_unsaturated + additive_output;

    if (!isfinite(local.feedback_unsaturated) ||
        !isfinite(local.total_unsaturated)) {
        return -ERANGE;
    }

    local.output = clampf(local.total_unsaturated,
                          config->output_min,
                          config->output_max);
    local.saturated =
        local.total_unsaturated < config->output_min ||
        local.total_unsaturated > config->output_max;

    *state = next;
    *result = local;
    return 0;
}

int control_pid_step(control_pid_state *state,
                     const control_pid_config *config,
                     const control_pid_input *input,
                     control_pid_result *result)
{
    return control_pid_step_core(state,
                                 config,
                                 input,
                                 0.0f,
                                 result);
}
```

逐行理解三个容易忽略的点：

1. `control_pid_state next = *state;` 是先复制旧状态。函数中途失败时没有执行 `*state = next`，旧状态就不会被污染。
2. `first_step` 时 measurement rate 固定为 0，所以第一次调用不会因为“上一拍默认是 0”产生巨大 D。
3. `local.total_unsaturated` 在限幅前保留原值，`local.output` 才是最终限幅值；调参时必须同时看二者。

### 2.5 不要放进 PID 的功能

- 目标限速/限加速度：第 4 步的 trajectory helper。
- 最终电流、力矩、PWM 硬件安全钳位：应用/执行器边界必须再做一次。
- 角度跨圈：`angle.h`。
- feedback freshness、命令 TTL、arm/fault：控制线程和 `dji::Bus`。
- 在线调参传输：独立参数邮箱；控制线程只在周期边界换配置。

本步自检：setpoint 阶跃而 measurement 不变时 D≈0；measurement 改变时 D 方向与变化率相反；正饱和且正误差时积分不继续增加；误差反向时积分允许退出饱和；deadband 内不提前返回；错误路径逐成员证明状态未变。

---

## 第 3 步：封装前馈模型和 Feedforward PID

### 3.1 前馈模型

目标文件：`include/control/feedforward.h`

```c
#ifndef SKYWALKER_CONTROL_FEEDFORWARD_H
#define SKYWALKER_CONTROL_FEEDFORWARD_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CONTROL_GRAVITY_NONE = 0,
    CONTROL_GRAVITY_SIN,
    CONTROL_GRAVITY_COS,
} control_gravity_model;

typedef struct {
    float k_bias;
    float k_static;
    float k_velocity;
    float k_acceleration;
    float k_gravity;
    float velocity_epsilon;
    float acceleration_epsilon;
    control_gravity_model gravity_model;
} control_feedforward_config;

typedef struct {
    float position_ref_rad;
    float velocity_ref;
    float acceleration_ref;
} control_feedforward_reference;

int control_feedforward_validate(
    const control_feedforward_config *config);

int control_feedforward_calculate(
    const control_feedforward_config *config,
    const control_feedforward_reference *reference,
    float *output);

#ifdef __cplusplus
}
#endif

#endif
```

公式为：

```text
FF = k_bias
   + k_static * direction(velocity_ref, acceleration_ref)
   + k_velocity * velocity_ref
   + k_acceleration * acceleration_ref
   + gravity(position_ref)
```

方向选择必须无歧义：

```text
abs(velocity_ref) > velocity_epsilon
    → sign(velocity_ref)
否则 abs(acceleration_ref) > acceleration_epsilon
    → sign(acceleration_ref)
否则
    → 0
```

`gravity(position_ref)` 按配置选择 `0`、`k_gravity*sin(q_ref)` 或 `k_gravity*cos(q_ref)`。只有明确零角定义与重力矩方向后才启用重力项。

`control_feedforward_validate()` 要拒绝非有限系数、负的 `velocity_epsilon/acceleration_epsilon` 和枚举范围外的 gravity mode；`calculate()` 还要拒绝非有限 reference。两个 epsilon 必须分开，因为速度单位是 rad/s，加速度单位是 rad/s²，不能用一个数冒充两种物理量。配置错误返回 `-EINVAL`，有限输入计算后溢出返回 `-ERANGE`，两者都不写 `*output`。

单位必须跟最终执行器输出一致：

- DJI 电流控制：`k_bias/k_static` 为 A，`k_velocity` 为 A/(rad/s)，`k_acceleration` 为 A/(rad/s²)。
- DM 力矩控制：对应输出为 N·m。
- IMU 加热：只使用 `k_bias`，单位为 PWM pulse ns；其余系数为 0。

`acceleration_ref` 必须来自轨迹生成器，不能直接对摇杆或网络阶跃做裸差分。

目标文件：`lib/control/feedforward.c`

操作：新建文件，完整粘贴。

```c
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include <control/feedforward.h>

static bool feedforward_fields_are_finite(
    const control_feedforward_config *config)
{
    return isfinite(config->k_bias) &&
           isfinite(config->k_static) &&
           isfinite(config->k_velocity) &&
           isfinite(config->k_acceleration) &&
           isfinite(config->k_gravity) &&
           isfinite(config->velocity_epsilon) &&
           isfinite(config->acceleration_epsilon);
}

static float direction_from_reference(
    const control_feedforward_config *config,
    const control_feedforward_reference *reference)
{
    if (fabsf(reference->velocity_ref) >
        config->velocity_epsilon) {
        return reference->velocity_ref > 0.0f ? 1.0f : -1.0f;
    }

    if (fabsf(reference->acceleration_ref) >
        config->acceleration_epsilon) {
        return reference->acceleration_ref > 0.0f ? 1.0f : -1.0f;
    }

    return 0.0f;
}

int control_feedforward_validate(
    const control_feedforward_config *config)
{
    if (config == NULL) {
        return -EINVAL;
    }
    if (!feedforward_fields_are_finite(config)) {
        return -EINVAL;
    }
    if (config->velocity_epsilon < 0.0f ||
        config->acceleration_epsilon < 0.0f) {
        return -EINVAL;
    }
    if (config->gravity_model != CONTROL_GRAVITY_NONE &&
        config->gravity_model != CONTROL_GRAVITY_SIN &&
        config->gravity_model != CONTROL_GRAVITY_COS) {
        return -EINVAL;
    }
    return 0;
}

int control_feedforward_calculate(
    const control_feedforward_config *config,
    const control_feedforward_reference *reference,
    float *output)
{
    if (reference == NULL || output == NULL) {
        return -EINVAL;
    }

    int ret = control_feedforward_validate(config);
    if (ret < 0) {
        return ret;
    }

    if (!isfinite(reference->position_ref_rad) ||
        !isfinite(reference->velocity_ref) ||
        !isfinite(reference->acceleration_ref)) {
        return -EINVAL;
    }

    const float direction =
        direction_from_reference(config, reference);

    float gravity = 0.0f;
    switch (config->gravity_model) {
    case CONTROL_GRAVITY_NONE:
        gravity = 0.0f;
        break;
    case CONTROL_GRAVITY_SIN:
        gravity = config->k_gravity *
                  sinf(reference->position_ref_rad);
        break;
    case CONTROL_GRAVITY_COS:
        gravity = config->k_gravity *
                  cosf(reference->position_ref_rad);
        break;
    default:
        return -EINVAL;
    }

    const float local_output =
        config->k_bias +
        config->k_static * direction +
        config->k_velocity * reference->velocity_ref +
        config->k_acceleration * reference->acceleration_ref +
        gravity;

    if (!isfinite(gravity) || !isfinite(local_output)) {
        return -ERANGE;
    }

    *output = local_output;
    return 0;
}
```

上面第一次出现的写法逐个说明：

- `condition ? a : b` 是 C 的条件表达式；条件真时结果为 `a`，否则为 `b`。这里只用它返回 `+1` 或 `-1`。
- `switch/case` 用于穷举枚举值；每个 `break` 防止继续落入下一分支，`default` 防御内存损坏或强制转型产生的未知值。
- `local_output` 是局部候选结果。只有它通过 finite 检查后才写 `*output`，所以函数失败不会把调用方的旧结果覆盖成半成品。
- `fabsf/sinf/cosf` 是单精度 `float` 版数学函数；末尾的 `f` 不要漏掉。

### 3.2 组合控制器

目标文件：`include/control/feedforward_pid.h`

```c
#ifndef SKYWALKER_CONTROL_FEEDFORWARD_PID_H
#define SKYWALKER_CONTROL_FEEDFORWARD_PID_H

#include <control/feedforward.h>
#include <control/pid.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    control_pid_config feedback;
    control_feedforward_config feedforward;
} control_feedforward_pid_config;

typedef struct {
    control_pid_state feedback;
} control_feedforward_pid_state;

typedef struct {
    control_pid_input feedback;
    control_feedforward_reference reference;
} control_feedforward_pid_input;

typedef struct {
    control_pid_result feedback;
    float feedforward;
    float output;
} control_feedforward_pid_result;

int control_feedforward_pid_validate(
    const control_feedforward_pid_config *config);

int control_feedforward_pid_reset(
    control_feedforward_pid_state *state,
    float current_measurement);

int control_feedforward_pid_step(
    control_feedforward_pid_state *state,
    const control_feedforward_pid_config *config,
    const control_feedforward_pid_input *input,
    control_feedforward_pid_result *result);

#ifdef __cplusplus
}
#endif

#endif
```

目标文件：`lib/control/feedforward_pid.c`。顺序必须是：

```text
validate 全部配置和输入
  → calculate FF
  → 调目录私有 control_pid_step_core(..., additive_output=FF)
  → feedback.total_unsaturated 已经等于 P+I+D+FF
  → feedback.output 已经按组合输出范围限幅
  → 一次性提交 state 和 result
```

不要这样写：

```c
/* 错误示例：PID 的 anti-windup 看不到 FF，且总输出可能再次越界。 */
output = control_pid_step(...) + control_feedforward_calculate(...);
```

不要让 `Feedforward PID` 继承或复制另一份 PID 状态。它只组合一个 `control_pid_state`，reset 也只转发到同一状态。

目标文件：`lib/control/feedforward_pid.c`

操作：新建文件，完整粘贴。

```c
#include <errno.h>
#include <stddef.h>

#include <control/feedforward_pid.h>

#include "control_internal.h"

int control_feedforward_pid_validate(
    const control_feedforward_pid_config *config)
{
    if (config == NULL) {
        return -EINVAL;
    }

    int ret = control_pid_validate(&config->feedback);
    if (ret < 0) {
        return ret;
    }

    return control_feedforward_validate(&config->feedforward);
}

int control_feedforward_pid_reset(
    control_feedforward_pid_state *state,
    float current_measurement)
{
    if (state == NULL) {
        return -EINVAL;
    }

    return control_pid_reset(&state->feedback,
                             current_measurement);
}

int control_feedforward_pid_step(
    control_feedforward_pid_state *state,
    const control_feedforward_pid_config *config,
    const control_feedforward_pid_input *input,
    control_feedforward_pid_result *result)
{
    if (state == NULL || input == NULL || result == NULL) {
        return -EINVAL;
    }

    int ret = control_feedforward_pid_validate(config);
    if (ret < 0) {
        return ret;
    }

    float feedforward = 0.0f;
    ret = control_feedforward_calculate(&config->feedforward,
                                        &input->reference,
                                        &feedforward);
    if (ret < 0) {
        return ret;
    }

    control_feedforward_pid_state next = *state;
    control_feedforward_pid_result local = {0};

    ret = control_pid_step_core(&next.feedback,
                                &config->feedback,
                                &input->feedback,
                                feedforward,
                                &local.feedback);
    if (ret < 0) {
        return ret;
    }

    local.feedforward = feedforward;
    local.output = local.feedback.output;

    *state = next;
    *result = local;
    return 0;
}
```

这里的 `&config->feedback` 可以拆成两步理解：`config->feedback` 取出 `config` 指向结构体里的 `feedback` 成员，前面的 `&` 再取该成员的地址。本文先判断 `config != NULL`，所以之后才可以使用 `->`。

`next` 与 `local` 仍然实现错误原子性：内部 PID core 只改临时副本；只有全部成功后，最后两行才一次性替换调用者的 state/result。`local.output` 不再自己加一次 FF，因为 `local.feedback.output` 已经是 core 对 `P+I+D+FF` 做完总限幅后的结果。

### 3.3 典型实例

M3508 速度环：

```text
setpoint/measurement: rad/s
PID output: A
FF: kS + kV*v_ref + kA*a_ref
通常先用 PI，kd=0
```

GM6020 速度内环：

```text
外环位置 P 输出 velocity_ref
轨迹器产生受限 velocity_ref/acceleration_ref
内环 Feedforward PID 输出 A
每个关节拥有独立外环状态、内环状态、零点、方向和限位
```

IMU 温控：

```text
setpoint/measurement: °C
output: PWM pulse ns，范围 0..period
FF: k_bias=当前 HEAT_OFFSET_NS
kS/kV/kA/kG: 0
```

本步自检：FF 单独可测试；经典 PID 的头文件没有 feedforward 字段；组合器只复用 PID core；P/I/D/FF/unsaturated/output 全部可遥测；总输出饱和时积分知道 FF 的存在。

---

## 第 4 步：补齐轨迹、角度工具和纯测试

### 4.1 目标斜坡

目标文件：`include/control/slew_rate_limiter.h`

操作：新建文件，完整粘贴。

```c
#ifndef SKYWALKER_CONTROL_SLEW_RATE_LIMITER_H
#define SKYWALKER_CONTROL_SLEW_RATE_LIMITER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float value;
    bool initialized;
} control_slew_rate_state;

typedef struct {
    float rising_rate_per_s;
    float falling_rate_per_s;
} control_slew_rate_config;

int control_slew_rate_validate(
    const control_slew_rate_config *config);

int control_slew_rate_reset(control_slew_rate_state *state,
                            float current_value);

int control_slew_rate_step(control_slew_rate_state *state,
                           const control_slew_rate_config *config,
                           float requested_value,
                           float dt_s,
                           float *limited_value,
                           float *limited_rate);

#ifdef __cplusplus
}
#endif

#endif
```


目标文件：`lib/control/slew_rate_limiter.c`

操作：新建文件，完整粘贴。

```c
#include <errno.h>
#include <math.h>
#include <stddef.h>

#include <control/slew_rate_limiter.h>

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

int control_slew_rate_validate(
    const control_slew_rate_config *config)
{
    if (config == NULL) {
        return -EINVAL;
    }
    if (!isfinite(config->rising_rate_per_s) ||
        !isfinite(config->falling_rate_per_s) ||
        config->rising_rate_per_s < 0.0f ||
        config->falling_rate_per_s < 0.0f) {
        return -EINVAL;
    }
    return 0;
}

int control_slew_rate_reset(control_slew_rate_state *state,
                            float current_value)
{
    if (state == NULL || !isfinite(current_value)) {
        return -EINVAL;
    }

    control_slew_rate_state next = {
        .value = current_value,
        .initialized = true,
    };
    *state = next;
    return 0;
}

int control_slew_rate_step(control_slew_rate_state *state,
                           const control_slew_rate_config *config,
                           float requested_value,
                           float dt_s,
                           float *limited_value,
                           float *limited_rate)
{
    if (state == NULL ||
        limited_value == NULL ||
        limited_rate == NULL) {
        return -EINVAL;
    }

    int ret = control_slew_rate_validate(config);
    if (ret < 0) {
        return ret;
    }

    if (!isfinite(requested_value) || !isfinite(dt_s)) {
        return -EINVAL;
    }
    if (dt_s <= 0.0f) {
        return -ERANGE;
    }
    if (!state->initialized) {
        return -EACCES;
    }
    if (!isfinite(state->value)) {
        return -EINVAL;
    }

    const float delta = requested_value - state->value;
    if (!isfinite(delta)) {
        return -ERANGE;
    }

    const float rate_limit = delta >= 0.0f
                           ? config->rising_rate_per_s
                           : config->falling_rate_per_s;
    const float maximum_delta = rate_limit * dt_s;
    if (!isfinite(maximum_delta)) {
        return -ERANGE;
    }

    const float applied_delta =
        clampf(delta, -maximum_delta, maximum_delta);
    const float next_value = state->value + applied_delta;
    const float next_rate = applied_delta / dt_s;
    if (!isfinite(next_value) || !isfinite(next_rate)) {
        return -ERANGE;
    }

    control_slew_rate_state next = *state;
    next.value = next_value;

    *state = next;
    *limited_value = next_value;
    *limited_rate = next_rate;
    return 0;
}
```

`rising_rate_per_s` 和 `falling_rate_per_s` 都是非负的“大小”；下降时实现自己加负号，所以配置中不写负数。例如当前值 0，请求值 10，上升率 2 unit/s，`dt=0.1 s`，本拍 `maximum_delta=2*0.1=0.2`，输出只从 0 走到 0.2，`limited_rate=0.2/0.1=2`。

`limited_rate` 就是前馈所需的一阶导数。调用方在进入控制模式时必须先 `reset()` 到当前安全目标；未 reset 就 step 会返回 `-EACCES`，而不是偷偷跳到第一个请求值。

### 4.2 角度工具

目标文件：`include/control/angle.h`

操作：新建文件，完整粘贴。

```c
#ifndef SKYWALKER_CONTROL_ANGLE_H
#define SKYWALKER_CONTROL_ANGLE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float last_wrapped_rad;
    float continuous_rad;
    bool initialized;
} control_angle_unwrapper;

int control_angle_unwrap_reset(control_angle_unwrapper *state,
                               float wrapped_rad);

int control_angle_unwrap_step(control_angle_unwrapper *state,
                              float wrapped_rad,
                              float *continuous_rad);

int control_shortest_angle_error(float target_rad,
                                 float measurement_rad,
                                 float *error_rad);

#ifdef __cplusplus
}
#endif

#endif
```

目标文件：`lib/control/angle.c`

操作：新建文件，完整粘贴。

```c
#include <errno.h>
#include <math.h>
#include <stddef.h>

#include <control/angle.h>

static const float control_pi = 3.14159265358979323846f;
static const float control_two_pi = 6.28318530717958647692f;

static float wrap_to_minus_pi_inclusive(float angle_rad)
{
    float wrapped = remainderf(angle_rad, control_two_pi);

    /* 统一采用 [-pi, pi)，所以把精确的 +pi 映射为 -pi。 */
    if (wrapped >= control_pi) {
        wrapped -= control_two_pi;
    }
    return wrapped;
}

int control_angle_unwrap_reset(control_angle_unwrapper *state,
                               float wrapped_rad)
{
    if (state == NULL || !isfinite(wrapped_rad)) {
        return -EINVAL;
    }

    const float normalized =
        wrap_to_minus_pi_inclusive(wrapped_rad);
    if (!isfinite(normalized)) {
        return -ERANGE;
    }

    control_angle_unwrapper next = {
        .last_wrapped_rad = normalized,
        .continuous_rad = normalized,
        .initialized = true,
    };
    *state = next;
    return 0;
}

int control_angle_unwrap_step(control_angle_unwrapper *state,
                              float wrapped_rad,
                              float *continuous_rad)
{
    if (state == NULL || continuous_rad == NULL) {
        return -EINVAL;
    }
    if (!isfinite(wrapped_rad)) {
        return -EINVAL;
    }
    if (!state->initialized) {
        return -EACCES;
    }
    if (!isfinite(state->last_wrapped_rad) ||
        !isfinite(state->continuous_rad)) {
        return -EINVAL;
    }

    const float normalized =
        wrap_to_minus_pi_inclusive(wrapped_rad);
    const float delta = wrap_to_minus_pi_inclusive(
        normalized - state->last_wrapped_rad);
    const float next_continuous = state->continuous_rad + delta;
    if (!isfinite(normalized) ||
        !isfinite(delta) ||
        !isfinite(next_continuous)) {
        return -ERANGE;
    }

    control_angle_unwrapper next = *state;
    next.last_wrapped_rad = normalized;
    next.continuous_rad = next_continuous;

    *state = next;
    *continuous_rad = next_continuous;
    return 0;
}

int control_shortest_angle_error(float target_rad,
                                 float measurement_rad,
                                 float *error_rad)
{
    if (error_rad == NULL ||
        !isfinite(target_rad) ||
        !isfinite(measurement_rad)) {
        return -EINVAL;
    }

    const float difference = target_rad - measurement_rad;
    if (!isfinite(difference)) {
        return -ERANGE;
    }

    const float local_error =
        wrap_to_minus_pi_inclusive(difference);
    if (!isfinite(local_error)) {
        return -ERANGE;
    }

    *error_rad = local_error;
    return 0;
}
```

这里固定返回半开区间 `[-pi, pi)`：包含 `-pi`，不包含 `+pi`。因此目标为 `+pi`、测量为 0 时最短误差是 `-pi`；两条路一样短，必须人为选一个唯一规则，否则边界会一会儿正一会儿负。

`remainderf` 是常数时间归一化，不使用可能被 Inf 卡死的无界 `while`。解包裹的前提是两次样本间真实转角绝对值小于 pi；如果可能一拍转过半圈，只凭 wrapped angle 数学上无法判断真实圈数。DJI encoder 的 8191→0 圈数累计已经属于电机 core；这里不再解 raw encoder。

### 4.3 Twister 测试矩阵

目标目录：`tests/control/`。至少覆盖：

经典 PID：

- `kp=2`、setpoint=3、measurement=1 时 P=4。
- 首次 step 与 reset 后第一拍 D=0。
- setpoint 阶跃、measurement 不变时 D≈0。
- measurement 上升时 D 为负；tau>0 时按期望平滑。
- deadband 内 effective error=0，但状态和组合 FF 仍正常更新。
- 正/负输出饱和、积分冻结、误差反向后积分释放。
- `freeze_integrator` 不改变积分。
- `dt=0`、负数、过小、过大、NaN，配置 NaN/Inf、上下限反向。
- 所有失败路径调用前后逐成员比较 `control_pid_state` 均一致；不要用 `memcmp` 比较可能含 padding 的结构体。

前馈与组合器：

- `k_bias=0.5, kS=0.2, kV=1, kA=0.5, v=2, a=1` 得到 3.2。
- 速度接近 0、加速度为负时静摩擦方向为负；二者都近 0 时为 0。
- sin/cos 重力项在 0、π/2 的结果正确。
- PID 候选 4 A、FF 3 A、总上限 5 A 时输出 5 A，且积分不继续推高饱和。
- classic PID 与 FF PID 使用不同 state 时互不影响；两个电机实例也互不影响。

轨迹与角度：

- 上升/下降使用各自 rate；输出不越过目标；返回的有限 rate 与实际 delta/dt 一致。
- `+π`/`-π` 边界、连续多圈、首次初始化、NaN 和 Inf。

下面先给出一份可运行的起步测试，不只写“应该测什么”。它已覆盖每类算法的主路径和关键错误原子性；上面矩阵中的其余边界仍需用同一写法追加，不得因为起步文件通过就删除矩阵。

目标文件：`tests/control/CMakeLists.txt`

操作：新建文件，完整粘贴。

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(skywalker_control_tests)

target_sources(app PRIVATE src/main.c)
```

`$ENV{ZEPHYR_BASE}` 读取 shell 环境里的 Zephyr 根目录；`target_sources(app PRIVATE ...)` 把测试源文件加到当前 Zephyr application target，`PRIVATE` 表示不把源文件作为传递给其他 target 的公共依赖。

目标文件：`tests/control/prj.conf`

操作：新建文件，完整粘贴。

```ini
CONFIG_ZTEST=y
CONFIG_SKYWALKER_LIB_CONTROL=y
```

目标文件：`tests/control/testcase.yaml`

操作：新建文件，完整粘贴。YAML 用缩进表示层级，只用空格，不要用 Tab。

```yaml
tests:
  skywalker.control:
    tags:
      - control
    platform_allow:
      - native_sim
```

目标文件：`tests/control/src/main.c`

操作：新建文件，完整粘贴。

```c
#include <errno.h>
#include <math.h>

#include <zephyr/ztest.h>

#include <control/angle.h>
#include <control/feedforward.h>
#include <control/feedforward_pid.h>
#include <control/pid.h>
#include <control/slew_rate_limiter.h>

#define TEST_EPSILON 0.0001f

static void assert_float_near(float actual, float expected)
{
    zassert_true(fabsf(actual - expected) <= TEST_EPSILON,
                 "float value outside tolerance");
}

static control_pid_config make_pid_config(void)
{
    const control_pid_config config = {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = -100.0f,
        .integral_max = 100.0f,
        .output_min = -100.0f,
        .output_max = 100.0f,
        .deadband = 0.0f,
        .dt_min_s = 0.001f,
        .dt_max_s = 2.0f,
    };
    return config;
}

static control_feedforward_config make_zero_feedforward(void)
{
    const control_feedforward_config config = {
        .k_bias = 0.0f,
        .k_static = 0.0f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
    };
    return config;
}

ZTEST(control_algorithms, test_pid_p_and_measurement_derivative)
{
    control_pid_config config = make_pid_config();
    config.kp = 2.0f;
    config.kd = 0.5f;

    control_pid_state state = {0};
    control_pid_input input = {
        .setpoint = 3.0f,
        .measurement = 1.0f,
        .dt_s = 0.01f,
        .freeze_integrator = false,
    };
    control_pid_result result = {0};

    int ret = control_pid_step(&state, &config, &input, &result);
    zassert_equal(ret, 0, "first step failed: %d", ret);
    assert_float_near(result.p, 4.0f);
    assert_float_near(result.d, 0.0f);

    /* 只改设定值，measurement 没动，D 仍为 0。 */
    input.setpoint = 4.0f;
    ret = control_pid_step(&state, &config, &input, &result);
    zassert_equal(ret, 0, "setpoint step failed: %d", ret);
    assert_float_near(result.d, 0.0f);

    /* measurement 上升 0.1，dt=0.01，rate=10，D=-0.5*10=-5。 */
    input.measurement = 1.1f;
    ret = control_pid_step(&state, &config, &input, &result);
    zassert_equal(ret, 0, "measurement step failed: %d", ret);
    assert_float_near(result.d, -5.0f);
}

ZTEST(control_algorithms, test_pid_invalid_dt_does_not_change_state)
{
    control_pid_config config = make_pid_config();
    control_pid_state state = {0};
    int ret = control_pid_reset(&state, 2.0f);
    zassert_equal(ret, 0, "reset failed: %d", ret);

    state.integral_output = 1.5f;
    state.filtered_measurement_rate = -2.0f;
    const control_pid_state before = state;

    const control_pid_input input = {
        .setpoint = 3.0f,
        .measurement = 2.0f,
        .dt_s = 0.0f,
        .freeze_integrator = false,
    };
    control_pid_result result = {0};

    ret = control_pid_step(&state, &config, &input, &result);
    zassert_equal(ret, -ERANGE, "unexpected return: %d", ret);
    assert_float_near(state.integral_output,
                      before.integral_output);
    assert_float_near(state.previous_measurement,
                      before.previous_measurement);
    assert_float_near(state.filtered_measurement_rate,
                      before.filtered_measurement_rate);
    zassert_equal(state.initialized, before.initialized,
                  "initialized changed");
}

ZTEST(control_algorithms, test_feedforward_numeric_example)
{
    control_feedforward_config config = make_zero_feedforward();
    config.k_bias = 0.5f;
    config.k_static = 0.2f;
    config.k_velocity = 1.0f;
    config.k_acceleration = 0.5f;

    const control_feedforward_reference reference = {
        .position_ref_rad = 0.0f,
        .velocity_ref = 2.0f,
        .acceleration_ref = 1.0f,
    };
    float output = -99.0f;

    const int ret = control_feedforward_calculate(
        &config, &reference, &output);
    zassert_equal(ret, 0, "feedforward failed: %d", ret);
    assert_float_near(output, 3.2f);
}

ZTEST(control_algorithms, test_feedforward_acceleration_direction)
{
    control_feedforward_config config = make_zero_feedforward();
    config.k_static = 0.2f;
    config.velocity_epsilon = 0.01f;
    config.acceleration_epsilon = 0.01f;

    const control_feedforward_reference reference = {
        .position_ref_rad = 0.0f,
        .velocity_ref = 0.0f,
        .acceleration_ref = -1.0f,
    };
    float output = 0.0f;

    const int ret = control_feedforward_calculate(
        &config, &reference, &output);
    zassert_equal(ret, 0, "feedforward failed: %d", ret);
    assert_float_near(output, -0.2f);
}

ZTEST(control_algorithms, test_combined_limit_blocks_integrator)
{
    control_feedforward_pid_config config = {
        .feedback = make_pid_config(),
        .feedforward = make_zero_feedforward(),
    };
    config.feedback.kp = 4.0f;
    config.feedback.ki = 1.0f;
    config.feedback.output_min = -5.0f;
    config.feedback.output_max = 5.0f;
    config.feedforward.k_bias = 3.0f;

    control_feedforward_pid_state state = {0};
    int ret = control_feedforward_pid_reset(&state, 0.0f);
    zassert_equal(ret, 0, "reset failed: %d", ret);

    const control_feedforward_pid_input input = {
        .feedback = {
            .setpoint = 1.0f,
            .measurement = 0.0f,
            .dt_s = 1.0f,
            .freeze_integrator = false,
        },
        .reference = {
            .position_ref_rad = 0.0f,
            .velocity_ref = 0.0f,
            .acceleration_ref = 0.0f,
        },
    };
    control_feedforward_pid_result result = {0};

    ret = control_feedforward_pid_step(&state,
                                       &config,
                                       &input,
                                       &result);
    zassert_equal(ret, 0, "combined step failed: %d", ret);
    assert_float_near(state.feedback.integral_output, 0.0f);
    assert_float_near(result.feedback.total_unsaturated, 7.0f);
    assert_float_near(result.output, 5.0f);
    zassert_true(result.feedback.saturated, "expected saturation");
}

ZTEST(control_algorithms, test_deadband_does_not_remove_feedforward)
{
    control_feedforward_pid_config config = {
        .feedback = make_pid_config(),
        .feedforward = make_zero_feedforward(),
    };
    config.feedback.deadband = 0.1f;
    config.feedforward.k_bias = 0.5f;

    control_feedforward_pid_state state = {0};
    const control_feedforward_pid_input input = {
        .feedback = {
            .setpoint = 0.01f,
            .measurement = 0.0f,
            .dt_s = 0.01f,
            .freeze_integrator = false,
        },
        .reference = {0},
    };
    control_feedforward_pid_result result = {0};

    const int ret = control_feedforward_pid_step(&state,
                                                  &config,
                                                  &input,
                                                  &result);
    zassert_equal(ret, 0, "combined step failed: %d", ret);
    assert_float_near(result.feedback.effective_error, 0.0f);
    assert_float_near(result.output, 0.5f);
}

ZTEST(control_algorithms, test_slew_uses_separate_rates)
{
    const control_slew_rate_config config = {
        .rising_rate_per_s = 2.0f,
        .falling_rate_per_s = 4.0f,
    };
    control_slew_rate_state state = {0};
    int ret = control_slew_rate_reset(&state, 0.0f);
    zassert_equal(ret, 0, "reset failed: %d", ret);

    float value = 0.0f;
    float rate = 0.0f;
    ret = control_slew_rate_step(&state,
                                 &config,
                                 10.0f,
                                 0.1f,
                                 &value,
                                 &rate);
    zassert_equal(ret, 0, "rising step failed: %d", ret);
    assert_float_near(value, 0.2f);
    assert_float_near(rate, 2.0f);

    ret = control_slew_rate_step(&state,
                                 &config,
                                 -10.0f,
                                 0.1f,
                                 &value,
                                 &rate);
    zassert_equal(ret, 0, "falling step failed: %d", ret);
    assert_float_near(value, -0.2f);
    assert_float_near(rate, -4.0f);
}

ZTEST(control_algorithms, test_angle_wrap_and_unwrap)
{
    float error = 0.0f;
    int ret = control_shortest_angle_error(
        3.14159265358979323846f, 0.0f, &error);
    zassert_equal(ret, 0, "shortest error failed: %d", ret);
    assert_float_near(error, -3.14159265358979323846f);

    control_angle_unwrapper state = {0};
    ret = control_angle_unwrap_reset(&state, 3.0f);
    zassert_equal(ret, 0, "unwrap reset failed: %d", ret);

    float continuous = 0.0f;
    ret = control_angle_unwrap_step(&state, -3.0f, &continuous);
    zassert_equal(ret, 0, "unwrap step failed: %d", ret);
    assert_float_near(continuous, 3.28318530717958647692f);
}

ZTEST_SUITE(control_algorithms, NULL, NULL, NULL, NULL, NULL);
```

`ZTEST(suite, name)` 定义一个用例，`ZTEST_SUITE(...)` 注册整个套件。`zassert_equal` 比较整数/枚举/布尔值；浮点数不直接用 `==`，而是用 `assert_float_near()` 判断误差是否小于容差。断言消息不打印浮点，因此测试不需要额外开启 Zephyr 浮点格式化配置。

注意测试中的 `const control_pid_state before = state;` 是保存调用前快照。失败后逐成员比较，不用 `memcmp()`，因为结构体成员间可能有未定义值的对齐填充字节。

建议由你执行，不由本文代跑：

```bash
west twister -T tests/control -p native_sim -v \
  -O build/twister/control

# 若当前 Zephyr 环境缺少 32 位 multilib，再使用其支持的 64 位 native target：
west twister -T tests/control -p native_sim/native/64 -v \
  -O build/twister/control-64
```

本步出口：不是“能编译”，而是上述正常、边界、错误原子性和多实例隔离用例全部通过。

---

## 第 5 步：迁移 IMU 温控并删除旧 PID device

这是纯控制库的第一个真实调用者。固定顺序如下，任一步未编译通过都不要先删旧实现：

1. 添加并测试 `lib/control`。
2. 在 `imu_config` 中保存 `control_feedforward_pid_config` 默认参数，在 `imu_data` 中保存独占的 `control_feedforward_pid_state` 与最近结果。
3. `skywalker_imu_init()` 不再检查 `pid_dev`，而是 validate 配置并把状态留为未初始化。第一次成功获取真实温度后才 reset；不得用启动默认值 `0 °C` 伪装当前测量。
4. `imu_heat_control()` 改用 `control_feedforward_pid_step()`，检查返回值、finite、0..PWM period 和 `pwm_set()` 返回值。
5. 将 `imu_heat_control()` 从 `void` 改为 `int`；调用者必须处理错误并将加热输出置 0。
6. 把 PID 默认参数移到 `skywalker,imu` binding 的 heater 属性，删除 `pid-dev` phandle 和独立 `temp_pid` 节点。
7. 所有调用方迁移后，删除 `drivers/pid/`、`include/drivers/pid/pid.h`、`dts/bindings/pid/skywalker,pid.yaml`、`CONFIG_SKYWALKER_DRIVER_PID` 及对应 CMake 行。

逐文件修改范围如下：

| 文件                                            | 亲手修改的内容                                                                                                               |
| ----------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `include/drivers/imu/imu.h`                   | include 新控制头；`imu_config` 用配置值取代 `pid_dev`；`imu_data` 增加状态/最近结果；`imu_heat_control` 返回 `int` |
| `drivers/imu/imu.c`                           | DT 配置展开、init/reset、组合控制计算、PWM 错误处理与安全置零                                                                |
| `dts/bindings/imu/skywalker,imu.yaml`         | 删除`pid-dev`；增加 heater PID、dt、输出范围和 `k-bias` 属性                                                             |
| `samples/imu_test/boards/dm_mc02.overlay`     | 参数搬进`imu` 节点；删除独立 `temp_pid` 节点                                                                             |
| `samples/imu_test/prj.conf`                   | 启用`CONFIG_SKYWALKER_LIB_CONTROL`；删除旧 PID driver 开关                                                                 |
| `samples/imu_test/src/main.c`                 | 检查`imu_heat_control()` 错误并进入安全路径                                                                                |
| `drivers/Kconfig`、`drivers/CMakeLists.txt` | 调用方全部迁移后移除旧 PID driver 接线                                                                                       |

### 5.1 先整文件替换 IMU 公共头

目标文件：`include/drivers/imu/imu.h`

操作：整文件替换为下面内容。

```c
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#include <control/feedforward_pid.h>

struct imu_filter_api {
    void (*init)(const struct device *dev);
    void (*predict)(const struct device *dev,
                    const float gyro[3],
                    float dt,
                    float angle[3]);
    void (*correct)(const struct device *dev,
                    const float accel[3]);
    void (*get_angle)(const struct device *dev,
                      float angle[3]);
};

typedef struct {
    const char *name;
    const struct imu_filter_api api;
} imu_estimator;

typedef struct {
    const struct device *accel_dev;
    const struct device *gyro_dev;
    const struct device *heat_dev;
    const struct device *filter_dev;
    const char *estimator;

    control_feedforward_pid_config heater_controller;
    uint32_t heater_feedback_timeout_ms;
} imu_config;

typedef struct {
    float accel[3];
    float gyro[3];
    float temp;
    float angle[3];

    bool temp_valid;
    int64_t temp_timestamp_ms;
    control_feedforward_pid_state heater_controller_state;
    control_feedforward_pid_result heater_last_result;
} imu_data;

int imu_fetch(const struct device *dev);
void imu_estimate(const struct device *dev, float dt);
int imu_heat_control(const struct device *dev,
                     float target_temp,
                     float dt);

#endif
```

`void (*predict)(...)` 是“函数指针”：该成员保存一个函数的地址，因此 IMU 可以通过同一套 API 调用不同滤波器实现。`const float gyro[3]` 在函数参数中本质上是指向至少 3 个 `float` 的只读指针。

`imu_config` 是 Zephyr device 的只读启动配置；`imu_data` 是每个 device 实例独占的 RAM。所以 heater PID 的 config 在前者，积分/上拍测量在后者。`temp_valid + temp_timestamp_ms` 是温度反馈有效性证据，不能只检查 `temp` 看起来像数字。

### 5.2 把 heater 启动参数放回 IMU binding

目标文件：`dts/bindings/imu/skywalker,imu.yaml`

操作：整文件替换为下面内容。这里保留 string 表示浮点字面量，与仓库当前约定一致。

```yaml
# SPDX-License-Identifier: Apache-2.0

description: Skywalker IMU

compatible: "skywalker,imu"

include: [base.yaml]

properties:
  accel-dev:
    type: phandle
    required: true
    description: Accelerometer device
  gyro-dev:
    type: phandle
    required: true
    description: Gyroscope device
  heat-dev:
    type: phandle
    required: true
    description: PWM controller used by the heater
  filter-dev:
    type: phandle
    required: true
    description: Attitude estimator device
  estimator:
    type: string
    required: true
    enum:
      - "ekf"
    description: Attitude estimator name

  heater-kp:
    type: string
    required: true
    description: Heater proportional gain in ns per degree Celsius
  heater-ki:
    type: string
    required: true
    description: Heater integral gain in ns per degree Celsius-second
  heater-kd:
    type: string
    required: true
    description: Heater derivative gain in ns-second per degree Celsius
  heater-derivative-tau-s:
    type: string
    required: true
    description: Heater measurement-derivative low-pass time constant in seconds
  heater-integral-min-ns:
    type: string
    required: true
    description: Minimum heater integral contribution in nanoseconds
  heater-integral-max-ns:
    type: string
    required: true
    description: Maximum heater integral contribution in nanoseconds
  heater-output-min-ns:
    type: string
    required: true
    description: Minimum combined heater pulse in nanoseconds
  heater-output-max-ns:
    type: string
    required: true
    description: Maximum combined heater pulse in nanoseconds
  heater-deadband-c:
    type: string
    required: true
    description: Heater temperature error deadband in degrees Celsius
  heater-dt-min-s:
    type: string
    required: true
    description: Minimum accepted heater update interval in seconds
  heater-dt-max-s:
    type: string
    required: true
    description: Maximum accepted heater update interval in seconds
  heater-bias-ns:
    type: string
    required: true
    description: Constant heater feedforward pulse in nanoseconds
  heater-feedback-timeout-ms:
    type: int
    required: true
    description: Maximum permitted temperature sample age in milliseconds
```

binding 中的 `kP` 单位例子：误差是 °C，P 输出是 ns，所以 `Kp × °C = ns`，Kp 的单位就是 ns/°C。Ki 还会乘 `dt`，因此是 ns/(°C·s)；Kd 乘的是 °C/s，因此是 ns·s/°C。属性名中写出单位，是为了防止将 ms/us/ns 混用。

目标文件：`samples/imu_test/boards/dm_mc02.overlay`

操作：只整段替换根节点 `/ { ... };`；文件后面已有的 `&timers3 { ... };` 保持不变。

```dts
/ {
	imu: imu {
		compatible = "skywalker,imu";
		accel-dev = <&bmi08x_accel>;
		gyro-dev = <&bmi08x_gyro>;
		heat-dev = <&pwm_heat>;
		filter-dev = <&ekf_filter>;
		estimator = "ekf";

		heater-kp = "6000000";
		heater-ki = "0";
		heater-kd = "0.02";
		heater-derivative-tau-s = "0";
		heater-integral-min-ns = "0";
		heater-integral-max-ns = "0";
		heater-output-min-ns = "0";
		heater-output-max-ns = "20000000";
		heater-deadband-c = "0";
		heater-dt-min-s = "0.05";
		heater-dt-max-s = "0.20";
		heater-bias-ns = "6750000";
		heater-feedback-timeout-ms = <250>;
	};

	ekf_filter: ekf_filter {
		compatible = "skywalker,kalman_filter";
		state-dim = <4>;
		measure-dim = <3>;
	};
};
```

`<&bmi08x_accel>` 是 phandle，可理解为“引用另一个已命名设备树节点”。`<250>` 是 32 位整数 cell，而 `"0.20"` 是字符串；当前 devicetree 浮点约定借助后者把数值字面量送入 C 预处理器。这组数字是旧参数的迁移起点，不是已完成热系统标定的结论；新 D 对 measurement 求导，旧 D 对 error 求导，`heater-kd` 必须重新验证。

### 5.3 精确替换 `imu.c` 中与温控有关的四段

这个文件后半还有大段 EKF，不要整文件覆盖。只做以下四个局部替换。

第一段：在文件顶部删除：

```c
#include "drivers/pid/pid.h"
```

并在已有 include 区加入：

```c
#include <errno.h>
#include <math.h>

#include <zephyr/kernel.h>

#include <control/feedforward_pid.h>
```

第二段：从旧的 `#define IMU_CONFIG_DEFINE(inst)` 开始，到该宏结束的 `};` 为止，整段替换为：

```c
#define IMU_HEATER_PERIOD_NS PWM_MSEC(20)
#define IMU_HEATER_CHANNEL 4U

#define IMU_DT_FLOAT(inst, property) \
    ((float)DT_STRING_UNQUOTED(DT_DRV_INST(inst), property))

#define IMU_CONFIG_DEFINE(inst)                                      \
    static const imu_config imu_config_##inst = {                    \
        .accel_dev = DEVICE_DT_GET(                                  \
            DT_INST_PHANDLE(inst, accel_dev)),                       \
        .gyro_dev = DEVICE_DT_GET(                                   \
            DT_INST_PHANDLE(inst, gyro_dev)),                        \
        .heat_dev = DEVICE_DT_GET(                                   \
            DT_INST_PHANDLE(inst, heat_dev)),                        \
        .filter_dev = DEVICE_DT_GET(                                 \
            DT_INST_PHANDLE(inst, filter_dev)),                      \
        .estimator = DT_INST_PROP(inst, estimator),                  \
        .heater_controller = {                                       \
            .feedback = {                                            \
                .kp = IMU_DT_FLOAT(inst, heater_kp),                 \
                .ki = IMU_DT_FLOAT(inst, heater_ki),                 \
                .kd = IMU_DT_FLOAT(inst, heater_kd),                 \
                .derivative_tau_s = IMU_DT_FLOAT(                    \
                    inst, heater_derivative_tau_s),                  \
                .integral_min = IMU_DT_FLOAT(                        \
                    inst, heater_integral_min_ns),                   \
                .integral_max = IMU_DT_FLOAT(                        \
                    inst, heater_integral_max_ns),                   \
                .output_min = IMU_DT_FLOAT(                          \
                    inst, heater_output_min_ns),                     \
                .output_max = IMU_DT_FLOAT(                          \
                    inst, heater_output_max_ns),                     \
                .deadband = IMU_DT_FLOAT(                            \
                    inst, heater_deadband_c),                        \
                .dt_min_s = IMU_DT_FLOAT(inst, heater_dt_min_s),     \
                .dt_max_s = IMU_DT_FLOAT(inst, heater_dt_max_s),     \
            },                                                       \
            .feedforward = {                                         \
                .k_bias = IMU_DT_FLOAT(inst, heater_bias_ns),        \
                .k_static = 0.0f,                                    \
                .k_velocity = 0.0f,                                  \
                .k_acceleration = 0.0f,                              \
                .k_gravity = 0.0f,                                   \
                .velocity_epsilon = 0.0f,                            \
                .acceleration_epsilon = 0.0f,                        \
                .gravity_model = CONTROL_GRAVITY_NONE,               \
            },                                                       \
        },                                                           \
        .heater_feedback_timeout_ms = DT_INST_PROP(                  \
            inst, heater_feedback_timeout_ms),                       \
    };
```

`\` 放在物理行末尾，表示 C 预处理器应把下一行也当成同一个宏；反斜杠后不能再放空格或注释。`imu_config_##inst` 中的 `##` 是 token paste：当 `inst` 为 0 时生成标识符 `imu_config_0`。`DT_STRING_UNQUOTED` 去掉 devicetree string 外层引号，使 `"0.20"` 变成 C 的 `0.20` 数值字面量。

`DT_DRV_INST(inst)` 表示当前 `DT_DRV_COMPAT skywalker_imu` 下的第 `inst` 个 enabled 设备树实例。`DT_INST_PHANDLE(inst, accel_dev)` 取该实例 `accel-dev` 属性指向的节点，`DEVICE_DT_GET(...)` 再从节点得到 Zephyr `struct device` 地址。`DT_INST_PROP(inst, estimator)` 直接取普通属性的生成值；devicetree 名字中的连字号在 C 宏里会变成下划线，例如 `heater-kp` 对应 `heater_kp`。

第三段：整个替换 `skywalker_imu_init()`。

```c
static int skywalker_imu_init(const struct device *dev)
{
    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;

    if (!device_is_ready(cfg->accel_dev) ||
        !device_is_ready(cfg->gyro_dev) ||
        !device_is_ready(cfg->heat_dev) ||
        !device_is_ready(cfg->filter_dev)) {
        return -ENODEV;
    }

    int ret = control_feedforward_pid_validate(
        &cfg->heater_controller);
    if (ret < 0) {
        return ret;
    }
    if (cfg->heater_controller.feedback.output_min < 0.0f ||
        cfg->heater_controller.feedback.output_max >
            (float)IMU_HEATER_PERIOD_NS) {
        return -ERANGE;
    }
    if (cfg->heater_feedback_timeout_ms == 0U) {
        return -EINVAL;
    }

    const struct imu_filter_api *api =
        imu_get_api(cfg->estimator);
    if (api == NULL) {
        return -EINVAL;
    }

    *data = (imu_data){0};
    api->init(cfg->filter_dev);
    return 0;
}
```

`*data = (imu_data){0};` 是 C11 复合字面量：先构造一个所有成员为 0/false 的临时 `imu_data`，再整体赋给 device RAM。此时 heater state 的 `initialized=false`，这正是需要的状态。

第四段由两个函数组成。先整个替换旧 `imu_fetch()`：

```c
int imu_fetch(const struct device *dev)
{
    if (dev == NULL) {
        return -EINVAL;
    }

    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;
    struct sensor_value accel_value[3];
    struct sensor_value gyro_value[3];
    struct sensor_value temp_value;

    data->temp_valid = false;

    int ret = sensor_sample_fetch(cfg->accel_dev);
    if (ret < 0) {
        return ret;
    }
    ret = sensor_channel_get(cfg->accel_dev,
                             SENSOR_CHAN_ACCEL_XYZ,
                             accel_value);
    if (ret < 0) {
        return ret;
    }
    ret = sensor_channel_get(cfg->accel_dev,
                             SENSOR_CHAN_DIE_TEMP,
                             &temp_value);
    if (ret < 0) {
        return ret;
    }

    ret = sensor_sample_fetch(cfg->gyro_dev);
    if (ret < 0) {
        return ret;
    }
    ret = sensor_channel_get(cfg->gyro_dev,
                             SENSOR_CHAN_GYRO_XYZ,
                             gyro_value);
    if (ret < 0) {
        return ret;
    }

    float next_accel[3];
    float next_gyro[3];
    for (size_t i = 0; i < 3U; ++i) {
        next_accel[i] = sensor_value_to_float(&accel_value[i]);
        next_gyro[i] = sensor_value_to_float(&gyro_value[i]);
        if (!isfinite(next_accel[i]) || !isfinite(next_gyro[i])) {
            return -ERANGE;
        }
    }

    const float next_temp =
        sensor_value_to_float(&temp_value);
    if (!isfinite(next_temp)) {
        return -ERANGE;
    }

    memcpy(data->accel, next_accel, sizeof(next_accel));
    memcpy(data->gyro, next_gyro, sizeof(next_gyro));
    data->temp = next_temp;
    data->temp_timestamp_ms = k_uptime_get();
    data->temp_valid = true;
    return 0;
}
```

`sizeof(next_accel)` 在这里是整个 3 元素数组的字节数，`memcpy(destination, source, byte_count)` 把局部候选数组一次提交到 device data。`size_t` 是表示对象大小/数组下标的无符号标准类型；`++i` 把 `i` 增加 1。

再从旧的 `#define HEAT_OFFSET_NS` 开始，到旧 `imu_heat_control()` 结束为止，整段替换为：

```c
static int imu_heater_enter_safe(const imu_config *cfg,
                                 imu_data *data,
                                 int cause)
{
    data->heater_controller_state =
        (control_feedforward_pid_state){0};
    data->heater_last_result =
        (control_feedforward_pid_result){0};

    int off_ret = pwm_set(cfg->heat_dev,
                          IMU_HEATER_CHANNEL,
                          IMU_HEATER_PERIOD_NS,
                          0U,
                          PWM_POLARITY_NORMAL);
    if (off_ret < 0) {
        return off_ret;
    }
    return cause;
}

int imu_heat_control(const struct device *dev,
                     float target_temp,
                     float dt)
{
    if (dev == NULL) {
        return -EINVAL;
    }

    const imu_config *cfg = dev->config;
    imu_data *data = dev->data;

    if (!isfinite(target_temp) || !isfinite(dt)) {
        return imu_heater_enter_safe(cfg, data, -EINVAL);
    }
    if (!data->temp_valid || !isfinite(data->temp)) {
        return imu_heater_enter_safe(cfg, data, -EAGAIN);
    }

    const int64_t now_ms = k_uptime_get();
    const int64_t age_ms = now_ms - data->temp_timestamp_ms;
    if (age_ms < 0 ||
        age_ms > (int64_t)cfg->heater_feedback_timeout_ms) {
        return imu_heater_enter_safe(cfg, data, -ETIMEDOUT);
    }

    if (!data->heater_controller_state.feedback.initialized) {
        int ret = control_feedforward_pid_reset(
            &data->heater_controller_state,
            data->temp);
        if (ret < 0) {
            return imu_heater_enter_safe(cfg, data, ret);
        }
    }

    const control_feedforward_pid_input input = {
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
    control_feedforward_pid_result result = {0};

    int ret = control_feedforward_pid_step(
        &data->heater_controller_state,
        &cfg->heater_controller,
        &input,
        &result);
    if (ret < 0) {
        return imu_heater_enter_safe(cfg, data, ret);
    }

    if (!isfinite(result.output) ||
        result.output < 0.0f ||
        result.output > (float)IMU_HEATER_PERIOD_NS) {
        return imu_heater_enter_safe(cfg, data, -ERANGE);
    }

    const uint32_t pulse_ns = (uint32_t)result.output;
    ret = pwm_set(cfg->heat_dev,
                  IMU_HEATER_CHANNEL,
                  IMU_HEATER_PERIOD_NS,
                  pulse_ns,
                  PWM_POLARITY_NORMAL);
    if (ret < 0) {
        return imu_heater_enter_safe(cfg, data, ret);
    }

    data->heater_last_result = result;
    return 0;
}
```

`(uint32_t)result.output` 把已经检查为 `[0, period]` 的浮点 ns 截断为 PWM API 需要的无符号整数。类型转换必须放在 finite 和范围检查之后；直接把负数/NaN 转成无符号整数不是安全限幅。

`imu_heater_enter_safe()` 会先丢弃积分/导数历史，再尝试向 PWM 写 0。如果“写 0”也失败，返回更紧急的 PWM 错误；上层不能宣称加热已关闭，必须执行硬件断电/故障告警。

### 5.4 修改 sample 调用方和构建开关

目标文件：`samples/imu_test/prj.conf`

操作：删除 `CONFIG_SKYWALKER_DRIVER_PID=y`，在 library 配置附近增加：

```ini
CONFIG_SKYWALKER_LIB_CONTROL=y
```

目标文件：`samples/imu_test/src/main.c`

操作：把 `while (1) { ... }` 整段替换为下面内容。变量 `now/t_last/t_heat/t_send/data/vofa` 仍由旧函数在循环之前创建。

```c
while (1) {
    now = k_uptime_get_32();
    float dt = (now - t_last) / 1000.0f;
    t_last = now;

    int fetch_ret = imu_fetch(imu_dev);
    if (fetch_ret == 0) {
        imu_estimate(imu_dev, dt);
    }

    if (now - t_heat >= 100U) {
        const float heat_dt = (now - t_heat) / 1000.0f;
        int heat_ret = imu_heat_control(imu_dev, 50.0f, heat_dt);
        if (heat_ret < 0) {
            printk("IMU heater safe-off/error: %d\n", heat_ret);
        }
        t_heat = now;
    }

    if (now - t_send >= 10U) {
        float out[3] = {
            data->angle[0],
            data->angle[1],
            data->angle[2],
        };
        vofa_send(&vofa, out, 3);
        t_send = now;
    }

    k_usleep(1000);
}
```

`fetch_ret == 0` 才用新样本更新姿态。温控函数还会自己查 `temp_valid` 和时间戳，这是必要的二次边界检查，不能假设所有未来调用方都像本 sample 一样写对。

### 5.5 最后才移除旧 PID device

只有全仓库执行 `rg 'drivers/pid|pid_dev|skywalker,pid|SKYWALKER_DRIVER_PID'` 已经只剩待删文件自身时，才做以下操作：

1. 在 `drivers/CMakeLists.txt` 删除 `add_subdirectory_ifdef(CONFIG_SKYWALKER_DRIVER_PID pid)` 整行。
2. 在 `drivers/Kconfig` 删除从 `config SKYWALKER_DRIVER_PID` 到它 `help` 文本结束的整段，不要删到后面 Kalman Filter 配置。
3. 删除目录 `drivers/pid/`、头文件 `include/drivers/pid/pid.h` 和 binding `dts/bindings/pid/skywalker,pid.yaml`。
4. 再次执行上面的 `rg`，预期零匹配；`rg` 返回 1 在这里表示“没找到”，不是删除命令失败。

不要在 IMU 迁移这一步同时改在线调参。设备树生成的 `dev->config` 是只读启动默认值；未来调参时，在 application RAM 中维护候选配置，由控制 owner 线程在周期边界完成 validate 和切换。Ki/Kd 或输出范围大幅改变时先 reset。

温控边界：

- `k_bias` 暂时映射当前 `HEAT_OFFSET_NS=6750000`，但它只是旧参数迁移，不代表已经标定正确。
- PWM period 当前是 20 ms，因此组合控制器 `output_min=0`、`output_max<=20000000 ns`。
- 读取温度失败、温度非有限、控制器报错或 PWM 写失败时，返回负错误码并尝试写 0 pulse。
- 加热器不能制冷；使用不对称输出范围，不要先得到负数后在函数外悄悄截断。
- 每个 IMU device 有自己的温控状态，不允许多个实例共享全局 PID data。

建议由你执行：

```bash
west build -b dm_mc02/stm32h723xx \
  samples/imu_test \
  -d build/imu_test -p always \
  -- -DBOARD_ROOT=$PWD
```

预期：编译图中不再出现旧 PID driver/device；加热禁用或传感器失败时 PWM 为 0；冷启动时输出有限且没有 D 尖峰；温度接近目标时无持续饱和。首次通电要限制加热时间并监视真实温度，不能只看软件数值。

---

## 第 6 步：封闭 DJI 验证缺口

此步不继续发明驱动接口，只给当前基线补证据。

### 6.1 纯测试

建立 `tests/motor/dji/`，覆盖：

- 三型号 motor-id → feedback-id/command-id/slot 映射。
- A↔raw 的 0、正负端点、越界、NaN/Inf 和饱和策略。
- CAN 8 字节大端 codec、错误 DLC、IDE/RTR/FDF 拒绝。
- 重复 feedback ID、重复 command group/slot、跨 CAN attach。
- arm 前全零、command TTL、feedback timeout、epoch 失配、Fault 后不能直接 arm。
- 任一失败时所有已知 command group 都尝试全零，并准确报告 `zero_sent`。

### 6.2 构建和 0 A 台架

```bash
west twister -T tests/motor/dji -p native_sim -v \
  -O build/twister/dji

west build -b dm_mc02/stm32h723xx \
  samples/motor/dji_unified \
  -d build/dji_unified -p always \
  -- -DBOARD_ROOT=$PWD
```

上电顺序：动力侧先断开，电机架空，确认 CAN ID/1 Mbps/终端；保持 `current-limit-ma=<0>`；先只看新鲜反馈；输入人工 `a` 后抓到所有已知 group 的全零帧；持续验证 0 A；断动力后才改接线。只有物理急停、方向和零帧证据都通过，才批准一个极小、限时的非零方向试验。

本步出口：三型号软件向量通过，统一 sample clean build，实物 RX-only 和 0 A 抓包通过。未知电流换算、固件模式或安全电流都必须保持“未验证”，不能靠放大 PID 增益绕过。

---

## 第 7 步：建立唯一 owner 的控制线程

控制周期数据流固定为：

```text
k_timer callback 只 give semaphore
  → owner thread 被唤醒
  → 取整批 feedback snapshot
  → 验证 state/valid/timestamp/finite/实际 dt
  → 每个新 feedback generation 最多更新一次 controller
  → 计算整批输出并做应用层安全钳位
  → 对全部电机 setCurrent()，只写缓存
  → 同一物理 CAN 的 dji::Bus::flush() 恰好一次
  → 发布轻量遥测快照
```

硬规则：

- CAN RX callback 只解码和提交反馈；timer callback 只发信号。
- PID、日志、`setCurrent()`、`flush()` 均在线程上下文。
- `dt` 来自单调时钟；超出每个控制器范围时 reset 并停机，不补算一大步。
- timestamp/generation 不变时不重复积分；可以在 command TTL 允许范围内重发同一安全命令。
- 一个 Bus 只有一个 owner 线程调用 `attach/arm/flush/stop/recover`。
- 任一必要反馈 stale、输出非有限、设置命令失败或 flush 失败，整条 Bus 停机并 reset 所有相关控制器。
- `FlushReport.zero_sent=false` 时软件没有停机证据，立即走物理动力切断；停止线程不等于电调收到 0。
- recover 后必须收到新的人工 arm token；旧目标和旧积分不得自动复活。

### 7.1 电机 PID 参数到底放在哪里

结论是：电机 PID/前馈参数不放 motor 设备树节点。当前 motor binding 中的字段都是硬件或接线事实，应保持这个边界：

| 参数 | 放置位置 | 修改方式 |
| --- | --- | --- |
| `compatible`、`can-bus`、`motor-id`、减速比 | devicetree overlay | 断电核对硬件后修改并重新构建 |
| `current-limit-ma` | devicetree overlay | 机构安全评审后修改；它是 driver 拒绝越界命令的硬上限，不做在线调参 |
| `current-loop-confirmed` | GM6020 overlay | 核对固件/Assistant 后声明，不允许运行时伪造 |
| Kp/Ki/Kd、D 滤波、deadband、I 限幅 | application 控制配置 | 编译期默认值复制到 RAM；通过 pending 配置微调 |
| kBias/kS/kV/kA/kG、目标斜坡 | application 控制配置 | 与对应电机、机构和输出单位一起标定 |
| PID state、积分、上一拍测量、当前目标 | application 运行状态 | 只存在 RAM，绝不写设备树或持久化 |

不要给 `dji-motor-base.yaml` 增加 `kp/ki/kd`。同一台 M3508 换到驱动轮、摩擦轮或其他减速机构后控制参数不同，说明这些值不是电机硬件描述。设备树也不是运行时数据库：生成的 `dev->config` 位于只读启动配置，在线强转写入属于错误用法。

这与第 5 步把 IMU heater 默认值放进 `skywalker,imu` binding 不冲突：IMU 加热环是固定在该板上的传感器/加热器热系统，由 IMU device 自己拥有；电机闭环则取决于底盘、舵向、摩擦轮等 application 机构和任务。如果以后同一块 IMU 的温控策略也要随 application 改变，就应把那组默认值同样上移，不能因为“它也是 PID”就一律塞进设备树。

两级上限必须同时存在：

```text
协议绝对上限 protocol_current_max_a       profile 固定
                  ≥
设备/机构硬上限 configured_current_limit_a  来自 DT current-limit-ma
                  ≥
本次调试软件上限 software_current_limit_a   application 配置
                  ≥
PID 合成输出绝对值                          每拍结果
```

运行时调参只能在已批准的软件范围内改变 PID/FF 输出，不能突破 `Descriptor::configured_current_limit_a`。第一版调参协议不暴露 `software_current_abs_max_a`、目标速度上限或 DT 硬上限；要改变这些安全边界，先停机断动力、评审后修改 application 默认配置或 overlay，并重新构建。

### 7.2 单电机调试阶段的文件结构

第 8 步先建立独立 sample，别把第一轮调参代码塞回 `dji_unified` 的 0 A 安全 sample：

```text
samples/motor/dji_speed_control/
  CMakeLists.txt
  prj.conf
  app.overlay                  只写 motor/CAN/硬电流上限
  src/
    main.cpp                   Bus 生命周期和控制线程
    motor_control_config.hpp   参数结构、编译期安全默认值、校验声明
    motor_control_config.cpp   默认值与跨字段校验
    motor_tuning.hpp           调参命令的枚举和消息结构
    motor_tuning.cpp           key 映射、pending 修改、commit
```

进入真实底盘后，把同样职责移到 `application/chassis/src/control/`，不要把 sample 的全局变量直接复制成整车架构。纯 `control_pid_*` 算法仍位于 `lib/control`，不知道电机型号；application 配置才知道“motor0 的速度环输出单位是 A”。

目标文件：`samples/motor/dji_speed_control/CMakeLists.txt`

操作：新建文件，完整粘贴。

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dji_speed_control)

target_sources(app PRIVATE
    src/main.cpp
    src/motor_control_config.cpp
    src/motor_tuning.cpp
)
```

目标文件：`samples/motor/dji_speed_control/prj.conf`

操作：新建文件，完整粘贴。先不启用 VOFA 调参输入；第 7.3 节列出的 parser/TX ownership 验收完成后再加对应开关。

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
CONFIG_SKYWALKER_DJI_FEEDBACK_TIMEOUT_MS=20
CONFIG_SKYWALKER_DJI_COMMAND_TIMEOUT_MS=20
```

目标文件：`samples/motor/dji_speed_control/app.overlay`

操作：下面是 M3508/C620 的**零电流构建骨架**，只能用于验证 device/feedback/0 A。整文件粘贴后，先与实物核对 CAN、ID 和传动比；不核对就不允许把 `current-limit-ma` 改为非零。

```dts
/ {
    aliases {
        motor0 = &speed_test_motor0;
    };

    speed_test_motor0: motor-1 {
        compatible = "dji,m3508-c620";
        status = "okay";
        can-bus = <&can1>;
        motor-id = <1>;
        current-limit-ma = <0>;
        gear-ratio-num = <1>;
        gear-ratio-den = <1>;
    };
};
```

这里的 `gear-ratio-num/den=<1>/<1>` 只让零输出骨架按电机转子轴单位工作，不声称机构输出轴传动比是 1。非零闭环前必须把它改为实际“电机转数/输出轴转数”的正整数比，例如精确分数用 num/den 表示，不写浮点近似。使用 M2006/C610 时把 compatible 换成 `dji,m2006-c610`，其他字段仍然要逐项核对；不能同时声明两个 compatible。

`speed_test_motor0:` 是节点 label，`&speed_test_motor0` 可以在别处引用它；`motor0 = ...` 是 alias，C/C++ 里的 `DT_ALIAS(motor0)` 会找到该节点。`status = "okay"` 表示启用该实例。`compatible` 决定匹配哪个 binding/driver，所以它是型号事实，不是一个可在调参时来回切换的字符串。

目标文件：`samples/motor/dji_speed_control/src/motor_control_config.hpp`

```cpp
#pragma once

#include <cstdint>
#include <control/feedforward_pid.h>
#include <control/slew_rate_limiter.h>
#include <drivers/motor/dji_motor.hpp>

namespace skywalker::app {

struct MotorVelocityControlConfig {
    control_feedforward_pid_config controller{};
    control_slew_rate_config reference_limiter{};
    float requested_velocity_abs_max_rad_s = 0.0f;
    float software_current_abs_max_a = 0.0f;
};

struct MotorVelocityControlSlot {
    MotorVelocityControlConfig active{};
    MotorVelocityControlConfig pending{};
    control_feedforward_pid_state controller_state{};
    control_slew_rate_state reference_state{};
    control_feedforward_pid_result last_result{};
    std::uint32_t active_revision = 0;
    bool pending_dirty = false;
};

int makeSafeVelocityDefaults(
    float nominal_period_s,
    float approved_software_current_a,
    MotorVelocityControlConfig &out);

int validateMotorVelocityConfig(
    const MotorVelocityControlConfig &config,
    const skywalker::motor::dji::Descriptor &motor);

} // namespace skywalker::app
```

`makeSafeVelocityDefaults()` 的 Kp/Ki/Kd 和全部 FF 系数先为 0。`approved_software_current_a` 未经批准时也传 0，于是 PID `output_min=output_max=0`，即使误 arm 也只能得到零输出。函数返回 `int` 是因为周期和限流也可能非法；只有返回 0 时才会写出 `out`。`dt_min_s/dt_max_s` 围绕实际 owner 周期设置，不直接复制示例数字。只有确认反馈方向、物理急停和 DT 硬上限后，才把软件上限改为一个更小的台架值。

调试稳定后再在 `motor_control_config.cpp` 增加按“具体机构实例”命名的默认函数，例如 `makeChassisDriveFrontLeftDefaults()`；不要写成一个看似适用于所有 M3508 的全局 `m3508_pid`。函数返回最后一组有测试记录的参数，仍须经过 `validateMotorVelocityConfig()`，校验失败就退回零输出配置并保持未 arm。

`MotorVelocityControlConfig &out` 是 C++ 引用：它必须绑定到一个真实对象，函数内对 `out` 赋值就是修改调用者的对象。它与 C 中的 `T *out` 都是“写出参数”，但调用时不写 `&`。

目标文件：`samples/motor/dji_speed_control/src/motor_control_config.cpp`

操作：新建文件，完整粘贴。不要保留“继续校验”一类占位注释。

```cpp
#include <cerrno>
#include <cmath>

#include "motor_control_config.hpp"

namespace skywalker::app {

int makeSafeVelocityDefaults(
    float nominal_period_s,
    float approved_software_current_a,
    MotorVelocityControlConfig &out)
{
    if (!std::isfinite(nominal_period_s) ||
        !std::isfinite(approved_software_current_a) ||
        nominal_period_s <= 0.0f ||
        approved_software_current_a < 0.0f) {
        return -EINVAL;
    }

    const float dt_min_s = nominal_period_s * 0.5f;
    const float dt_max_s = nominal_period_s * 2.0f;
    if (!std::isfinite(dt_min_s) ||
        !std::isfinite(dt_max_s)) {
        return -ERANGE;
    }

    MotorVelocityControlConfig next{};
    next.controller.feedback = {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .derivative_tau_s = 0.0f,
        .integral_min = 0.0f,
        .integral_max = 0.0f,
        .output_min = -approved_software_current_a,
        .output_max = approved_software_current_a,
        .deadband = 0.0f,
        .dt_min_s = dt_min_s,
        .dt_max_s = dt_max_s,
    };
    next.controller.feedforward = {
        .k_bias = 0.0f,
        .k_static = 0.0f,
        .k_velocity = 0.0f,
        .k_acceleration = 0.0f,
        .k_gravity = 0.0f,
        .velocity_epsilon = 0.0f,
        .acceleration_epsilon = 0.0f,
        .gravity_model = CONTROL_GRAVITY_NONE,
    };
    next.reference_limiter = {
        .rising_rate_per_s = 0.0f,
        .falling_rate_per_s = 0.0f,
    };
    next.requested_velocity_abs_max_rad_s = 0.0f;
    next.software_current_abs_max_a =
        approved_software_current_a;

    out = next;
    return 0;
}

int validateMotorVelocityConfig(
    const MotorVelocityControlConfig &cfg,
    const skywalker::motor::dji::Descriptor &motor)
{
    int ret = control_feedforward_pid_validate(&cfg.controller);
    if (ret < 0) {
        return ret;
    }

    ret = control_slew_rate_validate(&cfg.reference_limiter);
    if (ret < 0) {
        return ret;
    }

    if (!std::isfinite(cfg.software_current_abs_max_a) ||
        !std::isfinite(cfg.requested_velocity_abs_max_rad_s) ||
        cfg.software_current_abs_max_a < 0.0f ||
        cfg.requested_velocity_abs_max_rad_s < 0.0f) {
        return -EINVAL;
    }

    if (!std::isfinite(motor.protocol_current_max_a) ||
        !std::isfinite(motor.configured_current_limit_a) ||
        !std::isfinite(motor.gear_ratio) ||
        motor.protocol_current_max_a <= 0.0f ||
        motor.configured_current_limit_a < 0.0f ||
        motor.gear_ratio <= 0.0f) {
        return -EINVAL;
    }
    if (motor.configured_current_limit_a >
            motor.protocol_current_max_a ||
        cfg.software_current_abs_max_a >
            motor.configured_current_limit_a ||
        cfg.software_current_abs_max_a >
            motor.protocol_current_max_a) {
        return -ERANGE;
    }

    if (cfg.controller.feedback.output_min > 0.0f ||
        cfg.controller.feedback.output_max < 0.0f) {
        return -ERANGE;
    }
    if (cfg.controller.feedforward.gravity_model !=
            CONTROL_GRAVITY_NONE ||
        cfg.controller.feedforward.k_gravity != 0.0f) {
        return -EINVAL;
    }
    if (cfg.controller.feedback.output_min <
            -cfg.software_current_abs_max_a ||
        cfg.controller.feedback.output_max >
            cfg.software_current_abs_max_a) {
        return -ERANGE;
    }

    return 0;
}

} // namespace skywalker::app
```

上面 include 中的引号表示先从当前源文件目录查找这个 sample 私有头；尖括号留给仓库公共 include 路径。

`MotorVelocityControlConfig next{};` 是 C++ 值初始化，会先把整个对象清零；随后的 `.kp = ...` 是 C++20 指定成员初始化。它们必须按结构体声明顺序出现，不要随意重排。

这里直接复用 `dji::describe()` 已取得的 `Descriptor`，不重新从 devicetree 宏复制一份硬上限。校验失败时调用方不得写 active；`makeSafeVelocityDefaults()` 也只在所有局部计算成功后才 `out = next`。

参考调用方必须同时检查两个函数：

```cpp
skywalker::app::MotorVelocityControlConfig defaults{};
int ret = skywalker::app::makeSafeVelocityDefaults(
    0.005f,
    0.0f,
    defaults);
if (ret < 0) {
    return ret;
}

ret = skywalker::app::validateMotorVelocityConfig(
    defaults,
    descriptor);
if (ret < 0) {
    return ret;
}

slot.active = defaults;
slot.pending = defaults;
```

`0.005f` 代表 5 ms 周期，`0.0f` 代表尚未批准非零软件电流上限；它们是启动安全演示，不是实车调参值。

### 7.3 在线微调使用 active/pending 双配置

“在线调参”是在线接收参数，不等于 UART callback 可以在线改控制器。数据流固定为：

```text
PC/VOFA 文本 key=value
  → UART RX callback 立即把 key 映射成 enum，并复制 value/sequence
  → k_msgq_put(K_NO_WAIT)
  → control owner 在线程周期边界取消息
  → 只修改 slot.pending
  → 收到 commit 后做整组 validate + 安全静止检查
  → 先提交 0 A，再 reset 状态，再交换 active
  → 目标继续保持 0，等待操作者发起下一次短测试
```

目标文件：`samples/motor/dji_speed_control/src/motor_tuning.hpp`

```cpp
#pragma once

#include <cstdint>

#include "motor_control_config.hpp"

namespace skywalker::app {

enum class TuneField : std::uint8_t {
    Kp,
    Ki,
    Kd,
    DerivativeTau,
    IntegralMin,
    IntegralMax,
    OutputMin,
    OutputMax,
    Deadband,
    KBias,
    KStatic,
    KVelocity,
    KAcceleration,
    VelocityEpsilon,
    AccelerationEpsilon,
    VelocityRateUp,
    VelocityRateDown,
    BeginSession,
    Commit,
    Discard,
};

struct MotorTuneRequest {
    std::uint8_t motor_index;
    TuneField field;
    float value;
    std::uint32_t sequence;
};

struct MotorTuningSession {
    bool open = false;
    std::uint32_t last_sequence = 0;
};

int decodeMotorTuneKey(const char *key, TuneField &field);

void invalidateMotorTuneSession(MotorVelocityControlSlot &slot,
                                MotorTuningSession &session);

int stageMotorTuneRequest(MotorVelocityControlSlot &slot,
                          MotorTuningSession &session,
                          const MotorTuneRequest &request,
                          bool &commit_requested);

int commitPendingMotorConfig(
    MotorVelocityControlSlot &slot,
    MotorTuningSession &session,
    const skywalker::motor::dji::Descriptor &motor,
    float current_measurement_rad_s,
    bool in_tuning_mode,
    bool target_is_zero,
    bool speed_is_quiet,
    bool zero_command_committed);

} // namespace skywalker::app
```

`enum class` 是带作用域的 C++ 枚举，使用时必须写 `TuneField::Kp`，不会把 `Kp` 这个普通名字泄漏到命名空间。后面的 `: std::uint8_t` 指定它的底层存储类型为 8 位无符号整数，让队列消息大小稳定。

`decodeMotorTuneKey()` 只解决文本key到枚举的转换；`stageMotorTuneRequest()` 只写 pending；`commitPendingMotorConfig()` 只由 owner thread 在已证明静止和 0 A 提交的时刻调用。把三件事拆开后，纯状态机可在 `native_sim` 测试，不需 UART 或电机。

目标文件：`samples/motor/dji_speed_control/src/motor_tuning.cpp`

操作：新建文件，完整粘贴。

```cpp
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "motor_tuning.hpp"

namespace skywalker::app {
namespace {

struct TuneKeyEntry {
    const char *key;
    TuneField field;
};

constexpr TuneKeyEntry tune_keys[] = {
    {"m0.vel.kp", TuneField::Kp},
    {"m0.vel.ki", TuneField::Ki},
    {"m0.vel.kd", TuneField::Kd},
    {"m0.vel.d_tau", TuneField::DerivativeTau},
    {"m0.vel.i_min", TuneField::IntegralMin},
    {"m0.vel.i_max", TuneField::IntegralMax},
    {"m0.vel.out_min", TuneField::OutputMin},
    {"m0.vel.out_max", TuneField::OutputMax},
    {"m0.vel.deadband", TuneField::Deadband},
    {"m0.vel.kbias", TuneField::KBias},
    {"m0.vel.ks", TuneField::KStatic},
    {"m0.vel.kv", TuneField::KVelocity},
    {"m0.vel.ka", TuneField::KAcceleration},
    {"m0.vel.velocity_epsilon", TuneField::VelocityEpsilon},
    {"m0.vel.acceleration_epsilon", TuneField::AccelerationEpsilon},
    {"m0.vel.rate_up", TuneField::VelocityRateUp},
    {"m0.vel.rate_down", TuneField::VelocityRateDown},
    {"m0.vel.session", TuneField::BeginSession},
    {"m0.vel.commit", TuneField::Commit},
    {"m0.vel.discard", TuneField::Discard},
};

void closeAndRestorePending(MotorVelocityControlSlot &slot,
                            MotorTuningSession &session)
{
    slot.pending = slot.active;
    slot.pending_dirty = false;
    session = MotorTuningSession{};
}

} // namespace

int decodeMotorTuneKey(const char *key, TuneField &field)
{
    if (key == nullptr) {
        return -EINVAL;
    }

    for (const TuneKeyEntry &entry : tune_keys) {
        if (std::strcmp(key, entry.key) == 0) {
            field = entry.field;
            return 0;
        }
    }
    return -ENOENT;
}

void invalidateMotorTuneSession(MotorVelocityControlSlot &slot,
                                MotorTuningSession &session)
{
    closeAndRestorePending(slot, session);
}

int stageMotorTuneRequest(MotorVelocityControlSlot &slot,
                          MotorTuningSession &session,
                          const MotorTuneRequest &request,
                          bool &commit_requested)
{
    commit_requested = false;

    if (request.motor_index != 0U ||
        !std::isfinite(request.value)) {
        if (session.open) {
            closeAndRestorePending(slot, session);
        }
        return -EINVAL;
    }

    if (request.field == TuneField::BeginSession) {
        slot.pending = slot.active;
        slot.pending_dirty = false;
        session.open = true;
        session.last_sequence = request.sequence;
        return 0;
    }

    if (!session.open) {
        return -EACCES;
    }

    if (session.last_sequence ==
            std::numeric_limits<std::uint32_t>::max() ||
        request.sequence != session.last_sequence + 1U) {
        closeAndRestorePending(slot, session);
        return -ESTALE;
    }
    session.last_sequence = request.sequence;

    if (request.field == TuneField::Discard) {
        closeAndRestorePending(slot, session);
        return 0;
    }
    if (request.field == TuneField::Commit) {
        if (!slot.pending_dirty) {
            return -ENODATA;
        }
        commit_requested = true;
        return 0;
    }

    MotorVelocityControlConfig next = slot.pending;
    switch (request.field) {
    case TuneField::Kp:
        next.controller.feedback.kp = request.value;
        break;
    case TuneField::Ki:
        next.controller.feedback.ki = request.value;
        break;
    case TuneField::Kd:
        next.controller.feedback.kd = request.value;
        break;
    case TuneField::DerivativeTau:
        next.controller.feedback.derivative_tau_s = request.value;
        break;
    case TuneField::IntegralMin:
        next.controller.feedback.integral_min = request.value;
        break;
    case TuneField::IntegralMax:
        next.controller.feedback.integral_max = request.value;
        break;
    case TuneField::OutputMin:
        next.controller.feedback.output_min = request.value;
        break;
    case TuneField::OutputMax:
        next.controller.feedback.output_max = request.value;
        break;
    case TuneField::Deadband:
        next.controller.feedback.deadband = request.value;
        break;
    case TuneField::KBias:
        next.controller.feedforward.k_bias = request.value;
        break;
    case TuneField::KStatic:
        next.controller.feedforward.k_static = request.value;
        break;
    case TuneField::KVelocity:
        next.controller.feedforward.k_velocity = request.value;
        break;
    case TuneField::KAcceleration:
        next.controller.feedforward.k_acceleration = request.value;
        break;
    case TuneField::VelocityEpsilon:
        next.controller.feedforward.velocity_epsilon = request.value;
        break;
    case TuneField::AccelerationEpsilon:
        next.controller.feedforward.acceleration_epsilon = request.value;
        break;
    case TuneField::VelocityRateUp:
        next.reference_limiter.rising_rate_per_s = request.value;
        break;
    case TuneField::VelocityRateDown:
        next.reference_limiter.falling_rate_per_s = request.value;
        break;
    case TuneField::BeginSession:
    case TuneField::Commit:
    case TuneField::Discard:
    default:
        closeAndRestorePending(slot, session);
        return -EINVAL;
    }

    slot.pending = next;
    slot.pending_dirty = true;
    return 0;
}

int commitPendingMotorConfig(
    MotorVelocityControlSlot &slot,
    MotorTuningSession &session,
    const skywalker::motor::dji::Descriptor &motor,
    float current_measurement_rad_s,
    bool in_tuning_mode,
    bool target_is_zero,
    bool speed_is_quiet,
    bool zero_command_committed)
{
    if (!std::isfinite(current_measurement_rad_s)) {
        return -EINVAL;
    }
    if (!session.open || !in_tuning_mode ||
        !target_is_zero || !speed_is_quiet) {
        return -EACCES;
    }
    if (!slot.pending_dirty) {
        return -ENODATA;
    }
    if (!zero_command_committed) {
        return -EAGAIN;
    }

    int ret = validateMotorVelocityConfig(slot.pending, motor);
    if (ret < 0) {
        return ret;
    }
    if (slot.active_revision ==
        std::numeric_limits<std::uint32_t>::max()) {
        return -EOVERFLOW;
    }

    control_feedforward_pid_state next_controller_state{};
    ret = control_feedforward_pid_reset(&next_controller_state,
                                        current_measurement_rad_s);
    if (ret < 0) {
        return ret;
    }

    control_slew_rate_state next_reference_state{};
    ret = control_slew_rate_reset(&next_reference_state, 0.0f);
    if (ret < 0) {
        return ret;
    }

    const MotorVelocityControlConfig next_active = slot.pending;
    const std::uint32_t next_revision =
        slot.active_revision + 1U;

    slot.active = next_active;
    slot.pending = next_active;
    slot.controller_state = next_controller_state;
    slot.reference_state = next_reference_state;
    slot.last_result = control_feedforward_pid_result{};
    slot.active_revision = next_revision;
    slot.pending_dirty = false;
    session = MotorTuningSession{};
    return 0;
}

} // namespace skywalker::app
```

`namespace { ... }` 是 C++ 匿名命名空，里面的表和辅助函数只在当前 `.cpp` 可见，效果类似 C 文件级 `static`。`constexpr` 表示表项能在编译期构造。`for (const TuneKeyEntry &entry : tune_keys)` 是 range-for：按顺序访问数组每个元素，`entry` 是对当前元素的只读引用。

`std::strcmp(a, b)` 完全相等时返回 0；不能用 `a == b` 比较 C 字符串，因为那只会比较两个指针地址。`std::numeric_limits<std::uint32_t>::max()` 是 32 位无符号整数最大值；在此拒绝再加 1，防止序号/revision 回绕成 0。

注意：普通字段只改 `slot.pending`，不调 validate，因为一组合法配置可能需要先改 min 再改 max，中间态暂时非法。整组 validate 只在 Commit 时做。任何未知枚举、非有限值或序号缺口都调 `closeAndRestorePending()`，所以不会留下一组残缺 pending。

在 `main.cpp` 只定义一个有界队列。下面完整代码中的 `16` 是容量示例，应按最坏命令突发量和 RAM 预算确认；队列满时拒绝新命令并设置传输故障，不覆盖尚未应用的旧请求。

下面是 `main.cpp` 中接收侧和 owner 取队列的完整起步函数。操作：先在 include 区加入所列头，再把对象/函数放在 `main()` 之前。

```cpp
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <limits>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "motor_tuning.hpp"

K_MSGQ_DEFINE(motor_tune_queue,
              sizeof(skywalker::app::MotorTuneRequest),
              16,
              alignof(skywalker::app::MotorTuneRequest));

static atomic_t tune_transport_fault;
static std::uint32_t tune_rx_sequence;

static void onVofaCommand(const char *key, float value)
{
    skywalker::app::TuneField field{};
    int ret = skywalker::app::decodeMotorTuneKey(key, field);
    if (ret < 0 || !std::isfinite(value)) {
        atomic_set(&tune_transport_fault, 1);
        return;
    }

    if (tune_rx_sequence ==
        std::numeric_limits<std::uint32_t>::max()) {
        atomic_set(&tune_transport_fault, 1);
        return;
    }
    ++tune_rx_sequence;

    const skywalker::app::MotorTuneRequest request = {
        .motor_index = 0U,
        .field = field,
        .value = value,
        .sequence = tune_rx_sequence,
    };

    ret = k_msgq_put(&motor_tune_queue,
                     &request,
                     K_NO_WAIT);
    if (ret < 0) {
        atomic_set(&tune_transport_fault, 1);
    }
}

static int drainMotorTuneQueue(
    skywalker::app::MotorVelocityControlSlot &slot,
    skywalker::app::MotorTuningSession &session,
    bool &commit_requested)
{
    commit_requested = false;

    if (atomic_cas(&tune_transport_fault, 1, 0)) {
        k_msgq_purge(&motor_tune_queue);
        skywalker::app::invalidateMotorTuneSession(slot, session);
        return -ESTALE;
    }

    skywalker::app::MotorTuneRequest request{};
    while (k_msgq_get(&motor_tune_queue,
                      &request,
                      K_NO_WAIT) == 0) {
        bool this_request_wants_commit = false;
        int ret = skywalker::app::stageMotorTuneRequest(
            slot,
            session,
            request,
            this_request_wants_commit);
        if (ret < 0) {
            return ret;
        }

        if (this_request_wants_commit) {
            if (atomic_cas(&tune_transport_fault, 1, 0)) {
                k_msgq_purge(&motor_tune_queue);
                skywalker::app::invalidateMotorTuneSession(
                    slot, session);
                return -ESTALE;
            }
            commit_requested = true;
            return 0;
        }
    }

    return 0;
}
```

`K_MSGQ_DEFINE(name, item_size, max_items, alignment)` 在静态内存中定义队列；`sizeof(Request)` 是每项字节数，`16` 是最多项数，`alignof(Request)` 是 C++ 查询该类型的对齐要求。`K_NO_WAIT` 表示队列满/空时立即返回，callback 不睡眠等待。

`atomic_t` 是 Zephyr 的原子整数。`atomic_set(..., 1)` 让 callback 只发布一个“这批传输已不可信”标志；`atomic_cas(address, 1, 0)` 只在当前值确实为 1 时改成 0，并返回是否成功。owner 观察到标志后 purge 未处理队列并废弃 pending，不继续猜测哪条丢了。

`onVofaCommand()` 里的 `key` 只在本次 callback 内使用，进队的是已经转成枚举的 `field`，因此 DMA buffer 被复用后队列中不会留下悬空字符串指针。`drainMotorTuneQueue()` 收到 Commit 就停止取后续消息；本批是否真正可提交还由第 7.4 节的静止状态机决定。

`sequence` 由接收侧对每一条完整命令单调递增后复制进消息，不依赖 PC 提供。owner 若看到重复、回退或缺号，立即 `pending=active`、清 dirty，并拒绝后续字段和 Commit；只有 `BeginSession` 可以建立新的连续序列。这样某个字段因队列满而丢失时，不会把“残缺的一组参数”提交为 active。

串口可以使用如下命令名：

```text
m0.vel.session=1
m0.vel.kp=0.0
m0.vel.ki=0.0
m0.vel.kd=0.0
m0.vel.ks=0.0
m0.vel.kv=0.0
m0.vel.ka=0.0
m0.vel.out_max=0.0
m0.vel.commit=1
m0.vel.discard=1
```

数值只是格式示例，不是实车参数。`BeginSession` 先执行 `pending=active`、清 dirty 并记录新的 sequence 起点；没有有效 session 时拒绝普通字段。收到本批第一条普通字段且 `pending_dirty=false` 时确认 pending 仍来自 active；后续字段继续修改同一个 pending，不能每条都重新复制 active。每接收一个有效字段都置 `pending_dirty=true`；收到 `Discard` 时恢复 `pending=active` 并结束 session；成功 Commit 后令 `active=pending`、清 dirty、`active_revision++` 并结束 session。revision 只用于遥测和回退记录，不参与 arm 或安全判断。不要逐行立即生效，否则先收到 `integral_min`、后收到 `integral_max` 的中间状态可能非法。

UART callback 只做有界 key→enum 映射和非阻塞入队：

- 不保存指向 RX DMA buffer 的 `key` 指针，buffer 很快会被复用。
- 不调用 PID、`setCurrent()`、`Bus::stop()`、日志格式化或持久化。
- 未知 key、非有限 value、过长 key 和队列满都拒绝，并增加错误计数。
- 当前 `vofa.c` 的简易 parser 不支持科学计数法，也没有可靠保留跨 callback 半行；拿它做实车调参前，先补流式 RX 测试。
- 当前 `vofa_send()` 使用异步 UART 时还要解决 TX buffer ownership；未解决前降低遥测频率，不能在控制周期里连续复用同一缓冲区。

上面两个函数**不能让当前 `lib/vofa/vofa.c` 立即具备实车可用性**。当前 parser 将每个 `UART_RX_RDY` 事件当成独立文本，一行被拆成两个事件时会丢失前半行；`parse_float()` 也不拒绝空值、尾部垃圾和溢出。在完成“跨 callback 行缓冲 + 严格浮点解析 + 队列满测试”前，只可在断开电机动力的条件下验证命令映射，不得用它调非零电流参数。

### 7.4 第一版 commit 必须“静止切换”，不做运动中热更新

提交接口已在第 7.3 节的 `motor_tuning.hpp/.cpp` 完整给出，其精确签名为：

```cpp
int commitPendingMotorConfig(
    MotorVelocityControlSlot &slot,
    MotorTuningSession &session,
    const skywalker::motor::dji::Descriptor &motor,
    float current_measurement_rad_s,
    bool in_tuning_mode,
    bool target_is_zero,
    bool speed_is_quiet,
    bool zero_command_committed);
```

返回和状态语义：

| 返回值 | 含义 | active/state 行为 |
| --- | --- | --- |
| `0` | pending 合法且安全切换完成 | active 更新、revision+1，PID/斜坡 reset，目标保持 0 |
| `-EINVAL` | pending 字段、NaN/Inf 或跨字段关系非法 | 全部不变 |
| `-ERANGE` | PID 输出/速度/斜坡超过软件或 DT 上限 | 全部不变 |
| `-EACCES` | 目标非零、速度未静止或不在单电机 Tuning 模式 | 全部不变 |
| `-EAGAIN` | 本周期还没有成功提交 0 A | 全部不变，下周期可重试 |
| `-ENODATA` | 没有待提交的 pending 字段 | 全部不变 |
| `-EOVERFLOW` | revision 已达上限 | 全部不变，进入维护故障 |

owner 收到 Commit 后按以下状态机处理，而不是直接调用上表接口：

```text
TuningRun
  → 把 requested velocity 置 0
  → 连续 N 个新反馈周期确认 abs(velocity) 小于实测 quiet threshold
  → setCurrent(0 A) + 本 Bus 唯一一次 flush，且发送成功
  → commitPendingMotorConfig() 内部先构造 reset 后的临时状态，
    再一次性提交 active/pending/state/revision
  → TuningIdle；继续输出 0
  → 操作者显式开始下一次短测试
```

下面给出 owner 侧的完整起步状态。这些代码放在 sample `main.cpp` 的文件作用域；需要已 include `<cerrno>`、`<cmath>`、`<cstdint>`、motor 和 tuning 头。

```cpp
struct TuningCommitState {
    bool requested = false;
    std::uint32_t consecutive_quiet_samples = 0;
};

static int stopBusAfterControlFailure(
    skywalker::motor::dji::Bus &bus,
    int original_error)
{
    skywalker::motor::dji::FlushReport stop_report{};
    const int stop_ret = bus.stop(stop_report);
    if (stop_ret < 0) {
        return stop_ret;
    }
    if (!stop_report.zero_sent) {
        return -EIO;
    }
    return original_error;
}

static int advanceTuningCommit(
    TuningCommitState &commit_state,
    skywalker::app::MotorVelocityControlSlot &slot,
    skywalker::app::MotorTuningSession &session,
    skywalker::motor::dji::Bus &bus,
    const struct device *motor,
    const skywalker::motor::dji::Descriptor &descriptor,
    float current_velocity_rad_s,
    bool feedback_is_new,
    float quiet_threshold_rad_s,
    std::uint32_t required_quiet_samples)
{
    if (!commit_state.requested) {
        return 0;
    }
    if (motor == nullptr ||
        !std::isfinite(current_velocity_rad_s) ||
        !std::isfinite(quiet_threshold_rad_s) ||
        quiet_threshold_rad_s < 0.0f ||
        required_quiet_samples == 0U) {
        return stopBusAfterControlFailure(bus, -EINVAL);
    }

    int ret = skywalker::motor::setCurrent(motor, 0.0f);
    if (ret < 0) {
        return stopBusAfterControlFailure(bus, ret);
    }

    skywalker::motor::dji::FlushReport flush_report{};
    ret = bus.flush(flush_report);
    if (ret < 0) {
        return stopBusAfterControlFailure(bus, ret);
    }

    if (feedback_is_new) {
        if (std::fabs(current_velocity_rad_s) <=
            quiet_threshold_rad_s) {
            if (commit_state.consecutive_quiet_samples <
                required_quiet_samples) {
                ++commit_state.consecutive_quiet_samples;
            }
        } else {
            commit_state.consecutive_quiet_samples = 0U;
        }
    }

    if (commit_state.consecutive_quiet_samples <
        required_quiet_samples) {
        return -EAGAIN;
    }

    ret = skywalker::app::commitPendingMotorConfig(
        slot,
        session,
        descriptor,
        current_velocity_rad_s,
        true,
        true,
        true,
        true);
    if (ret < 0) {
        return ret;
    }

    commit_state = TuningCommitState{};
    return 0;
}
```

owner 在 `drainMotorTuneQueue()` 返回 `commit_requested=true` 时必须立即把 requested velocity 设为 0，再执行：

```cpp
requested_velocity_rad_s = 0.0f;
tuning_commit.requested = true;
tuning_commit.consecutive_quiet_samples = 0U;
```

之后的每个控制周期，只要 `tuning_commit.requested` 为 true，就调 `advanceTuningCommit()` **代替**普通 PID + flush 分支，不能两个分支都 flush。函数返回 `-EAGAIN` 只表示电机还在慢停，owner 继续下一拍输出 0；其他负值才记录错误/决定是否进入 Fault。

`feedback_is_new` 来自 feedback generation/时间戳变化；只有新样本才增加静止计数，否则同一个旧速度值在快速循环中被重复读 N 次就会伪造“连续 N 个新样本静止”。`quiet_threshold_rad_s` 和 `required_quiet_samples` 必须根据真实反馈噪声/频率确定，本文不伪造实车数值。

只看到软件变量为 0 不等于零帧已发送；必须使用本周期 `setCurrent(0)` 与成功 `flush()` 的证据。若 flush 失败，沿 Bus Fault/物理断电路径处理，绝不切配置后自动 recover。

第一版所有 Kp/Ki/Kd/FF 改动都 reset，行为简单、可测试。以后确有“运动中无扰切换”需求，再单独设计 tracking/bump-less transfer，例如以 `I_new = last_output - P_new - D_new - FF_new` 反算积分并钳位；在有完整测试和安全论证前不要加入。

为调参层建立 `tests/application/motor_tuning/`。下面是可执行的起步测试，不是用例名列表。

目标文件：`tests/application/motor_tuning/CMakeLists.txt`

操作：新建文件，完整粘贴。

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(skywalker_motor_tuning_tests)

set(SPEED_SAMPLE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../samples/motor/dji_speed_control/src)

target_sources(app PRIVATE
    src/main.cpp
    ${SPEED_SAMPLE_DIR}/motor_control_config.cpp
    ${SPEED_SAMPLE_DIR}/motor_tuning.cpp
)

target_include_directories(app PRIVATE ${SPEED_SAMPLE_DIR})
```

`set(NAME value)` 在 CMake 里定义变量，之后用 `${NAME}` 展开。`${CMAKE_CURRENT_SOURCE_DIR}` 是当前 `CMakeLists.txt` 所在绝对目录；三个 `..` 从 `tests/application/motor_tuning` 回到仓库根。`target_include_directories` 让测试找到 sample 私有头，不会把它们伪装成公共 API。

目标文件：`tests/application/motor_tuning/prj.conf`

```ini
CONFIG_ZTEST=y
CONFIG_CPP=y
CONFIG_REQUIRES_FULL_LIBCPP=y
CONFIG_SKYWALKER_LIB_CONTROL=y
```

目标文件：`tests/application/motor_tuning/testcase.yaml`

```yaml
tests:
  skywalker.application.motor_tuning:
    tags:
      - control
      - motor
    platform_allow:
      - native_sim
```

目标文件：`tests/application/motor_tuning/src/main.cpp`

操作：新建文件，完整粘贴。

```cpp
#include <cerrno>
#include <cmath>

#include <zephyr/ztest.h>

#include "motor_control_config.hpp"
#include "motor_tuning.hpp"

namespace app = skywalker::app;

static constexpr float test_epsilon = 0.0001f;

static void assertFloatNear(float actual, float expected)
{
    zassert_true(std::fabs(actual - expected) <= test_epsilon,
                 "float value outside tolerance");
}

static skywalker::motor::dji::Descriptor makeDescriptor()
{
    skywalker::motor::dji::Descriptor descriptor{};
    descriptor.protocol_current_max_a = 20.0f;
    descriptor.configured_current_limit_a = 2.0f;
    descriptor.gear_ratio = 19.0f;
    return descriptor;
}

static int makeSlot(app::MotorVelocityControlSlot &out)
{
    app::MotorVelocityControlConfig config{};
    const int make_ret = app::makeSafeVelocityDefaults(
        0.005f,
        1.0f,
        config);
    if (make_ret < 0) {
        return make_ret;
    }

    config.requested_velocity_abs_max_rad_s = 10.0f;
    config.reference_limiter.rising_rate_per_s = 20.0f;
    config.reference_limiter.falling_rate_per_s = 20.0f;

    const auto descriptor = makeDescriptor();
    const int validate_ret =
        app::validateMotorVelocityConfig(config, descriptor);
    if (validate_ret < 0) {
        return validate_ret;
    }

    app::MotorVelocityControlSlot slot{};
    slot.active = config;
    slot.pending = config;
    out = slot;
    return 0;
}

static app::MotorTuneRequest request(app::TuneField field,
                                     float value,
                                     std::uint32_t sequence)
{
    return app::MotorTuneRequest{
        .motor_index = 0U,
        .field = field,
        .value = value,
        .sequence = sequence,
    };
}

ZTEST(motor_tuning, test_field_without_session_is_rejected)
{
    app::MotorVelocityControlSlot slot{};
    const int setup_ret = makeSlot(slot);
    zassert_equal(setup_ret, 0, "setup failed: %d", setup_ret);
    if (setup_ret < 0) {
        return;
    }
    app::MotorTuningSession session{};
    bool commit_requested = true;

    const int ret = app::stageMotorTuneRequest(
        slot,
        session,
        request(app::TuneField::Kp, 0.2f, 1U),
        commit_requested);

    zassert_equal(ret, -EACCES, "unexpected return: %d", ret);
    zassert_false(commit_requested, "commit flag changed");
    zassert_false(session.open, "session opened");
    zassert_false(slot.pending_dirty, "pending became dirty");
    assertFloatNear(slot.active.controller.feedback.kp, 0.0f);
}

ZTEST(motor_tuning, test_complete_session_commits_atomically)
{
    app::MotorVelocityControlSlot slot{};
    const int setup_ret = makeSlot(slot);
    zassert_equal(setup_ret, 0, "setup failed: %d", setup_ret);
    if (setup_ret < 0) {
        return;
    }
    app::MotorTuningSession session{};
    bool commit_requested = false;

    int ret = app::stageMotorTuneRequest(
        slot,
        session,
        request(app::TuneField::BeginSession, 1.0f, 10U),
        commit_requested);
    zassert_equal(ret, 0, "begin failed: %d", ret);

    ret = app::stageMotorTuneRequest(
        slot,
        session,
        request(app::TuneField::Kp, 0.2f, 11U),
        commit_requested);
    zassert_equal(ret, 0, "stage failed: %d", ret);
    assertFloatNear(slot.active.controller.feedback.kp, 0.0f);
    assertFloatNear(slot.pending.controller.feedback.kp, 0.2f);

    ret = app::stageMotorTuneRequest(
        slot,
        session,
        request(app::TuneField::Commit, 1.0f, 12U),
        commit_requested);
    zassert_equal(ret, 0, "commit request failed: %d", ret);
    zassert_true(commit_requested, "commit not requested");

    const auto descriptor = makeDescriptor();
    ret = app::commitPendingMotorConfig(slot,
                                        session,
                                        descriptor,
                                        0.0f,
                                        true,
                                        true,
                                        true,
                                        true);
    zassert_equal(ret, 0, "commit failed: %d", ret);
    assertFloatNear(slot.active.controller.feedback.kp, 0.2f);
    assertFloatNear(slot.pending.controller.feedback.kp, 0.2f);
    zassert_equal(slot.active_revision, 1U, "bad revision");
    zassert_false(slot.pending_dirty, "pending still dirty");
    zassert_false(session.open, "session still open");
    zassert_true(slot.controller_state.feedback.initialized,
                 "controller not reset");
    zassert_true(slot.reference_state.initialized,
                 "reference not reset");
}

ZTEST(motor_tuning, test_sequence_gap_discards_pending)
{
    app::MotorVelocityControlSlot slot{};
    const int setup_ret = makeSlot(slot);
    zassert_equal(setup_ret, 0, "setup failed: %d", setup_ret);
    if (setup_ret < 0) {
        return;
    }
    app::MotorTuningSession session{};
    bool commit_requested = false;

    int ret = app::stageMotorTuneRequest(
        slot,
        session,
        request(app::TuneField::BeginSession, 1.0f, 20U),
        commit_requested);
    zassert_equal(ret, 0, "begin failed: %d", ret);

    ret = app::stageMotorTuneRequest(
        slot,
        session,
        request(app::TuneField::Kp, 0.3f, 22U),
        commit_requested);
    zassert_equal(ret, -ESTALE, "gap not rejected: %d", ret);
    zassert_false(session.open, "session not closed");
    zassert_false(slot.pending_dirty, "pending still dirty");
    assertFloatNear(slot.pending.controller.feedback.kp,
                    slot.active.controller.feedback.kp);
}

ZTEST(motor_tuning, test_invalid_pending_keeps_active_and_state)
{
    app::MotorVelocityControlSlot slot{};
    const int setup_ret = makeSlot(slot);
    zassert_equal(setup_ret, 0, "setup failed: %d", setup_ret);
    if (setup_ret < 0) {
        return;
    }
    slot.controller_state.feedback.integral_output = 0.25f;
    slot.controller_state.feedback.previous_measurement = 1.0f;
    slot.controller_state.feedback.filtered_measurement_rate = -2.0f;
    slot.controller_state.feedback.initialized = true;
    const control_feedforward_pid_state state_before =
        slot.controller_state;
    const app::MotorVelocityControlConfig active_before = slot.active;
    const std::uint32_t revision_before = slot.active_revision;

    app::MotorTuningSession session{};
    bool commit_requested = false;
    int ret = app::stageMotorTuneRequest(
        slot,
        session,
        request(app::TuneField::BeginSession, 1.0f, 30U),
        commit_requested);
    zassert_equal(ret, 0, "begin failed: %d", ret);

    ret = app::stageMotorTuneRequest(
        slot,
        session,
        request(app::TuneField::OutputMax, 3.0f, 31U),
        commit_requested);
    zassert_equal(ret, 0, "stage failed: %d", ret);

    ret = app::stageMotorTuneRequest(
        slot,
        session,
        request(app::TuneField::Commit, 1.0f, 32U),
        commit_requested);
    zassert_equal(ret, 0, "commit request failed: %d", ret);

    const auto descriptor = makeDescriptor();
    ret = app::commitPendingMotorConfig(slot,
                                        session,
                                        descriptor,
                                        0.0f,
                                        true,
                                        true,
                                        true,
                                        true);
    zassert_equal(ret, -ERANGE,
                  "unsafe pending accepted: %d", ret);

    assertFloatNear(slot.active.controller.feedback.output_max,
                    active_before.controller.feedback.output_max);
    assertFloatNear(
        slot.controller_state.feedback.integral_output,
        state_before.feedback.integral_output);
    assertFloatNear(
        slot.controller_state.feedback.previous_measurement,
        state_before.feedback.previous_measurement);
    assertFloatNear(
        slot.controller_state.feedback.filtered_measurement_rate,
        state_before.feedback.filtered_measurement_rate);
    zassert_equal(slot.controller_state.feedback.initialized,
                  state_before.feedback.initialized,
                  "initialized changed");
    zassert_equal(slot.active_revision,
                  revision_before,
                  "revision changed");
}

ZTEST(motor_tuning, test_key_decode_is_exact)
{
    app::TuneField field{};
    int ret = app::decodeMotorTuneKey("m0.vel.kp", field);
    zassert_equal(ret, 0, "known key failed: %d", ret);
    zassert_equal(field, app::TuneField::Kp, "wrong field");

    ret = app::decodeMotorTuneKey("m0.vel.kp.extra", field);
    zassert_equal(ret, -ENOENT, "prefix was accepted: %d", ret);
}

ZTEST_SUITE(motor_tuning, NULL, NULL, NULL, NULL, NULL);
```

`namespace app = skywalker::app;` 是命名空间别名，只是把后面的长名缩短，不会复制任何数据。`const auto descriptor = ...` 让编译器从函数返回类型推导出 `Descriptor`；`auto` 只是编译期类型推导，不是动态类型。

这是起步文件，还要用相同模式追加：Discard、sequence 重复/回退、NaN/Inf、上下限反向、目标非零、速度未静止、缺少 0 A 提交证据以及 `main.cpp` 集成层的队列满/purge。所有失败用例都像 `test_invalid_pending...` 一样逐成员确认 active、state、last_result 和 revision 不变。

### 7.5 调好后怎样固化，掉电保存放到后期

第一版推荐把最终参数回填到 application 源码，而不是设备树：

1. 导出当前 active revision 的完整 PID、FF、斜坡、软件上限、控制周期和单位。
2. 目标归零，确认 0 A flush 成功，停止 Bus 并切断动力。
3. 把有证据的数值写入 `motor_control_config.cpp` 对应机构实例的 defaults 函数，同时记录电机、减速比、负载和测试日期。
4. clean build 后先验证默认仍不自动 arm，再重复同一条正/负短轨迹。
5. 新结果与调参时一致后才提交代码；若不一致，恢复最后一个有证据的 revision，不靠继续增大电流掩盖。

下面是“回填到 application 源码”的确切形状。先在 `motor_control_config.hpp` 的 `validateMotorVelocityConfig()` 声明后增加：

```cpp
int makeChassisDriveFrontLeftDefaults(
    const skywalker::motor::dji::Descriptor &motor,
    MotorVelocityControlConfig &out);
```

再在 `motor_control_config.cpp` 中、已有函数之后但仍然位于 `namespace skywalker::app` 内加入以下完整代码：

```cpp
namespace {

/*
 * 启动安全记录：所有输出/目标为 0。
 * 只用某一个已验收 active revision 的完整导出值
 * 逐字段替换本初始化器，不从多次实验拼凑。
 */
constexpr MotorVelocityControlConfig
    chassis_drive_front_left_record = {
        .controller = {
            .feedback = {
                .kp = 0.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .derivative_tau_s = 0.0f,
                .integral_min = 0.0f,
                .integral_max = 0.0f,
                .output_min = 0.0f,
                .output_max = 0.0f,
                .deadband = 0.0f,
                .dt_min_s = 0.0025f,
                .dt_max_s = 0.0100f,
            },
            .feedforward = {
                .k_bias = 0.0f,
                .k_static = 0.0f,
                .k_velocity = 0.0f,
                .k_acceleration = 0.0f,
                .k_gravity = 0.0f,
                .velocity_epsilon = 0.0f,
                .acceleration_epsilon = 0.0f,
                .gravity_model = CONTROL_GRAVITY_NONE,
            },
        },
        .reference_limiter = {
            .rising_rate_per_s = 0.0f,
            .falling_rate_per_s = 0.0f,
        },
        .requested_velocity_abs_max_rad_s = 0.0f,
        .software_current_abs_max_a = 0.0f,
    };

} // namespace

int makeChassisDriveFrontLeftDefaults(
    const skywalker::motor::dji::Descriptor &motor,
    MotorVelocityControlConfig &out)
{
    const MotorVelocityControlConfig next =
        chassis_drive_front_left_record;

    const int ret = validateMotorVelocityConfig(next, motor);
    if (ret < 0) {
        return ret;
    }

    out = next;
    return 0;
}
```

这段代码本身是可验证的零锁定默认配置，不含“待补”字段。`dt_min_s=0.0025`、`dt_max_s=0.0100` 只对应前文 5 ms nominal sample；若 owner 周期不是 5 ms，先按实测周期重算这两项，不能照抄。调好后从同一个 active revision 导出并一次性替换 PID、FF、斜坡、目标上限和软件电流上限的数值；`k_gravity/gravity_model` 在纯速度环继续保持 0/NONE。函数在复制给 `out` 前再次拿真实 `Descriptor` 做完整校验，所以 overlay 的硬上限一旦收紧，旧 application 参数不会悄悄越界启动。

设备树仍只改硬件事实。例如更换 motor-id、实际传动比或经安全评审收紧 `current-limit-ma` 才改 `app.overlay`；只把 Kp 从已验证 revision A 微调到 revision B 时，改上面的 application 记录，不改 overlay。若 PID 和 DT 同时改变，无法判断响应变化来自控制器还是硬件描述，必须拆成两次独立变更和两套证据。

初次调参只保存在 RAM：重启恢复代码中的安全默认值，并重新人工 arm。参数稳定后若需要持久化，再使用 Zephyr settings/NVS 保存一个带 `schema_version`、payload length 和 CRC 的参数快照。

启动加载顺序必须是：读取快照 → 校验版本/长度/CRC/finite → `validateMotorVelocityConfig()` → 成功才复制到 pending；失败使用编译期默认值并保持未 arm。不要持久化积分、上一拍测量、目标值、Bus epoch、arm token 或 Fault 状态。持久化写入由低频 worker 执行，绝不在 UART/控制 callback 或每次字段修改时写 flash。

本步自检：制造 dt 超时、反馈停更、命令过期、CAN 发送失败；每条路径都可观察到 reset、Fault、全零报告和人工重新 arm 门控。再验证调参队列满、非法字段、超 DT 电流上限、非静止 Commit、Discard，以及未启用持久化时重启回源码默认值；每条拒绝路径 active/state 均不变。

---

## 第 8 步：先做 M3508/M2006 单速度环

第一条真实电机闭环使用：

```text
requested velocity
  → slew-rate limiter，得到 velocity_ref 和 acceleration_ref
  → Feedforward PID（首轮 kd=0，实际是 PI + kS/kV/kA）
  → 机构安全电流 A clamp
  → motor::setCurrent()
  → 本 Bus 本周期唯一一次 flush()
```

### 8.1 单拍速度环的完整计算函数

把下面结构体和函数放入 `samples/motor/dji_speed_control/src/main.cpp` 的 `main()` 之前。它只做纯计算和提交 application 状态，不调 CAN；owner 在它成功后再调 `setCurrent()` 和本拍唯一的 `flush()`。

```cpp
struct VelocityControlOutput {
    float velocity_reference_rad_s = 0.0f;
    float acceleration_reference_rad_s2 = 0.0f;
    float current_command_a = 0.0f;
    control_feedforward_pid_result controller{};
};

static float clampFloat(float value,
                        float minimum,
                        float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int calculateVelocityCurrent(
    skywalker::app::MotorVelocityControlSlot &slot,
    const skywalker::motor::Feedback &feedback,
    bool feedback_is_fresh,
    float requested_velocity_rad_s,
    float dt_s,
    VelocityControlOutput &output)
{
    if (!feedback_is_fresh ||
        (feedback.valid &
         skywalker::motor::FeedbackVelocity) == 0U) {
        return -EAGAIN;
    }
    if (!std::isfinite(feedback.velocity_rad_s) ||
        !std::isfinite(requested_velocity_rad_s) ||
        !std::isfinite(dt_s)) {
        return -EINVAL;
    }
    if (std::fabs(requested_velocity_rad_s) >
        slot.active.requested_velocity_abs_max_rad_s) {
        return -ERANGE;
    }

    control_slew_rate_state next_reference_state =
        slot.reference_state;
    control_feedforward_pid_state next_controller_state =
        slot.controller_state;
    VelocityControlOutput next_output{};

    int ret = control_slew_rate_step(
        &next_reference_state,
        &slot.active.reference_limiter,
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
        &slot.active.controller,
        &input,
        &next_output.controller);
    if (ret < 0) {
        return ret;
    }

    next_output.current_command_a = clampFloat(
        next_output.controller.output,
        -slot.active.software_current_abs_max_a,
        slot.active.software_current_abs_max_a);
    if (!std::isfinite(next_output.current_command_a)) {
        return -ERANGE;
    }

    slot.reference_state = next_reference_state;
    slot.controller_state = next_controller_state;
    slot.last_result = next_output.controller;
    output = next_output;
    return 0;
}
```

调用前必须在进入 Armed 控制模式时初始化两份状态：

```cpp
int ret = control_slew_rate_reset(&slot.reference_state, 0.0f);
if (ret < 0) {
    return ret;
}

ret = control_feedforward_pid_reset(
    &slot.controller_state,
    first_feedback.velocity_rad_s);
if (ret < 0) {
    return ret;
}
```

`feedback.valid` 是位掩码，一个整数的不同 bit 表示不同反馈字段是否可用。`a & b` 是按位与；只有两边同一 bit 都为 1，结果的那一 bit 才为 1。所以与 `FeedbackVelocity` 按位与后为 0，就表示速度字段无效。

`next_reference_state` 和 `next_controller_state` 是两份候选副本。若斜坡成功、PID 失败，函数也不会把斜坡状态偷偷前进一拍；三个 `slot... = next...` 只在最后一起提交。算法内已有 output range，application 还是用 `software_current_abs_max_a` 再 clamp 一次，它是跨层安全防线，不是替代 config validate。

owner 每拍的最小成功路径如下：

```cpp
VelocityControlOutput control_output{};
ret = calculateVelocityCurrent(slot,
                               feedback,
                               feedback_is_fresh,
                               requested_velocity_rad_s,
                               dt_s,
                               control_output);
if (ret < 0) {
    return stopBusAfterControlFailure(dji_bus, ret);
}

ret = skywalker::motor::setCurrent(
    motor,
    control_output.current_command_a);
if (ret < 0) {
    return stopBusAfterControlFailure(dji_bus, ret);
}

skywalker::motor::dji::FlushReport report{};
ret = dji_bus.flush(report);
if (ret < 0) {
    return stopBusAfterControlFailure(dji_bus, ret);
}
```

这一段不可原样塞进无返回值的无限循环；实际 owner 要把负值交给 Fault 状态分支，reset controller，取消旧目标，并要求新的人工 arm token。关键是成功路径每拍只 flush 一次；调参 commit 进行中则由第 7.4 节分支取代它。

### 8.2 每次微调前要记录什么

至少发布一份低频遥测快照，字段固定为：`timestamp`、`dt`、`velocity_requested`、`velocity_ref`、`acceleration_ref`、`velocity_measurement`、`error`、P/I/D/FF、`total_unsaturated`、`output`、`saturated`、feedback age、active revision 和 Bus state。控制线程只写快照；遥测线程降采样发送。

每轮使用相同的短测试轨迹、相同软件电流上限和相同机械状态。参数记录至少包含 Git commit、电机/减速机构、DT `current-limit-ma`、软件电流上限、控制周期、测试参数 revision、峰值误差、稳态误差、超调、饱和占比和异常现象。没有这些对照数据，“感觉更顺”不能作为保留参数的证据。

### 8.3 推荐调试顺序

1. 架空、只收反馈，手转确认正方向与单位。
2. DT `current-limit-ma` 和 application 软件上限都保持已批准的小值；确认 application 上限更小。
3. `ki=kd=kBias=kS=kV=kA=0`，用极小 `kp` 和限时正/负目标确认负反馈方向；误差扩大立即清零断电。
4. 每次只改一个字段，将新值写 pending；目标归零、速度静止、0 A flush 成功后 Commit，再开始同一条短轨迹。
5. 小步增加 `kp` 到响应清晰但没有持续振荡；若噪声被明显放大或正负方向行为差异大，先查反馈/机械，不继续加 Kp。
6. 保持 `ki=0` 记录多个正负稳态点，用“维持该速度所需输出”估计 kS/kV；前馈接近后 PID 只负责修正误差。
7. 再少量增加 `ki` 消除剩余稳态误差，同时限制 I 输出并验证正/负饱和 anti-windup。
8. 只有轨迹加速段存在可重复误差时才评估 `kA`；只有任务确实需要且测量足够干净时才评估 D。速度环通常保持 `kd=0`。
9. 最后分别验证正转、反转、目标归零、急停、feedback stale、命令 TTL、重新 arm 和长时间热状态。

参数现象只作为排查方向，不是自动调参规则：

| 现象 | 优先检查/调整 |
| --- | --- |
| 一给正目标误差迅速变大 | 方向或符号错误，立即断电；不是 Kp 太小 |
| 响应迟缓且从不接近软件电流上限 | 在安全范围内小步增加 Kp，确认目标斜坡没有限制过严 |
| 持续振荡或噪声直接进入电流 | 降低 Kp；查反馈周期、机械间隙和滤波，不急着加 D |
| 稳态误差随速度近似线性变化 | 先标定 kV，再考虑 Ki |
| 正负低速都需近似固定且符号相反的输出 | 标定 kS 与 velocity/acceleration epsilon |
| 加速段误差明显，稳态已好 | 从受限轨迹数据评估 kA |
| 输出长期饱和 | 降目标/加速度或重新评估机构；不要靠提高 DT 电流上限“调好” |
| 目标归零后积分拖尾 | 查 anti-windup、I 限幅、reset 和 Ki，不在调用方强减积分 |

### 8.4 每个参数到底写什么

先固定物理单位。速度环输入是 rad/s，输出是 A，因此参数单位如下：

| 字段 | 单位 | 初始写法 | 什么时候才改 |
| --- | --- | --- | --- |
| `kp` | A/(rad/s) | `0.0f` | 方向验证后，用受限短速度阶跃逐次试 |
| `ki` | A/((rad/s)·s) | `0.0f` | Kp 与 FF 已稳定，仍有可重复稳态误差 |
| `kd` | A/(rad/s²) | `0.0f` | 只在有可重复动态需求且速度反馈噪声可控时试；速度环通常保持 0 |
| `derivative_tau_s` | s | `0.0f` | `kd>0` 且确实需要 D 低通时才标定 |
| `integral_min/max` | A | 两者都 `0.0f` | 启用 Ki 时在已批准软件电流内设小范围，必须含 0 |
| `output_min/max` | A | 两者都 `0.0f` | 硬件与人员安全评审后，仍不得超过 application 软上限 |
| `deadband` | rad/s | `0.0f` | 只在实测静止噪声/机械回差需要时使用 |
| `k_bias` | A | `0.0f` | 确认机构存在方向无关的固定输出才使用 |
| `k_static` | A | `0.0f` | 正负低速都存在符号相反的固定克服摩擦电流 |
| `k_velocity` | A/(rad/s) | `0.0f` | 稳态所需电流与速度近似线性 |
| `k_acceleration` | A/(rad/s²) | `0.0f` | kS/kV 后，加速段仍有可重复的输出残差 |
| `velocity_epsilon` | rad/s | `0.0f` | 根据零速噪声设定静摩擦方向切换阈值 |
| `acceleration_epsilon` | rad/s² | `0.0f` | 根据轨迹器返回的零加速度量化噪声设定 |
| `rate_up/rate_down` | rad/s² | `0.0f` | 先根据机构/软电流上限确定；0 代表目标锁住，不代表无限快 |

“小步改参数”不是每次随手加 1。它的可执行含义是：

1. 试验前记录当前 active revision 和完整参数，只选一个字段作为本轮变量。
2. 本轮的新值必须位于上一个“稳定值”与第一个“明显变差值”之间；若尚未观察到上界，每次只能用现场安全计划预先批准的增量。
3. 每次都重跑完全相同的正/负短轨迹，比较误差、超调、饱和占比和温度；不同时改目标、载荷和电流上限。
4. 任何持续振荡、噪声放大、方向错误或饱和时间变长都立即回到上一 revision，不用另一个参数去“对消”新问题。

第一次验证命令链时保持输出上下限都为 0，例如：

```text
m0.vel.session=1
m0.vel.kp=0.1
m0.vel.out_min=0
m0.vel.out_max=0
m0.vel.commit=1
```

这只验证“解析→入队→pending→静止 commit→revision”；因为最终 output range 是 `[0,0]`，`kp=0.1` 也不能产生非零电流。实车上的任意非零上限都必须先由安全评审写入 application 默认配置，不通过命令“试出一个够大的值”。

正式改一个字段时，命令仍然必须是完整会话。下面的 0.10→0.11 只是文本协议示范，不是任何电机的推荐 Kp：

```text
m0.vel.session=1
m0.vel.kp=0.11
m0.vel.commit=1
```

Commit 成功只表示新配置被验证并切换，目标仍为 0，也不自动 arm/开始新轨迹。操作者必须再发起一次独立的限时试验。

### 8.5 前馈不是猜值：用遥测算 kS/kV/kA

先保持 `ki=kd=k_acceleration=0`，在不饱和、转速已稳定的正向试验中取两个点 `(v1,u1)` 与 `(v2,u2)`，其中 v 是 rad/s，u 是 A。一阶近似为：

```text
kV = (u2 - u1) / (v2 - v1)
kS = u1 - kV * v1
```

纯算术示例：若测量记录是 `(10 rad/s, 0.8 A)` 和 `(20 rad/s, 1.3 A)`，则 `kV=(1.3-0.8)/(20-10)=0.05 A/(rad/s)`，`kS=0.8-0.05*10=0.3 A`。这组数字只演示如何算，不是 M3508/M2006 的参数。

同样记录负向多个点。若正负向结果明显不对称，第一版不强行用一个 kS 拟合所有点；先查机械偏载、方向换算和电流反馈。需要正/负不对称模型时，应扩展前馈结构和测试，不在调参表中暗藏符号补丁。

当 kS/kV 已固定后，在轨迹加速段记录 `(a,v,u)`，先扣除静摩擦和速度项：

```text
u_residual = u - kS * sign(reference) - kV * v
kA_sample = u_residual / a
```

不在 `abs(a) <= acceleration_epsilon` 的样本上做除法，不用饱和样本，不将反馈噪声的单点当结论。对多次相同轨迹的有效 `kA_sample` 取稳健中值，再用独立轨迹验证。

### 8.6 Ki 和 D 的实际引入顺序

FF 完成后仍有可重复稳态误差才引入 Ki。先保持 `integral_min=integral_max=0`，确认当前响应；然后在已批准电流内设一个小的对称 I 输出范围，最后才把 Ki 从 0 改成第一个试验值。如果先改 Ki 而 I 上下限仍是 0，数学上积分仍被锁死；如果先放开很大的 I 上限，又会失去安全边界。

每次 Ki 试验都要包含：正向稳态、负向稳态、输出正饱和后误差反向、输出负饱和后误差反向、目标归零与 reset。遥测中 `i` 在推向饱和时必须停止增长，误差反向时必须能退出饱和。

D 最后引入。先看原始 measurement rate 和遥测噪声，再确定 `derivative_tau_s`，最后才试非零 Kd。若增加 Kd 后电流高频抖动或 D 项主导输出，立即回退；不用无限增大 tau 把错误的 D 项“滤到看不见”。

Kp/Ki/Kd 和 FF 系数没有跨电机通用值。M3508 与 M2006、同型号不同减速机构、同机构不同负载都分别保存配置和测试记录。对参数的任何小幅变更也要生成新的 revision，便于回退到最后一组有证据的 active 配置。

M3508 和 M2006 可以复用控制器类型，不能复用未经标定的参数。每台电机必须有自己的 state；多轴结果先写应用层批次缓冲，再由 owner 整批提交。

---

## 第 9 步：GM6020 串级与大小 Yaw 独立闭环

舵向或 Yaw 的第一版结构：

```text
continuous position target
  → position P（通常先不加 I/D）
  → velocity target clamp + slew-rate limiter
  → velocity PI + kS/kV/kA
  → current A clamp
  → motor API / Bus
```

位置外环输出单位是 rad/s，速度内环输出单位是 A。两个环各有独立配置和状态，不能把外环 `pid_state` 传给内环。软限位外只允许向安全方向退出；硬限位直接锁存 fault，不靠 PID 饱和硬顶。

大 Yaw、小 Yaw 先分别在 Independent 模式稳定。此阶段完全不做协调；各自保存型号、传动比、零点、方向、软硬限位和控制器状态。若小 Yaw 到中心附近仍抖动，先查角度跨圈、反馈时间戳、机械间隙和内环，再谈协调器。

---

## 第 10 步：舵轮纯数学到整底盘

顺序不可颠倒：

```text
纯运动学测试
  → 单个 steering/drive module 架空闭环
  → 四模块只做 +vx
  → +vy
  → +wz
  → 组合运动
  → 底盘 Disabled/Aligning/Ready/Running/Fault 状态机
```

纯数学至少包含：零速保持上一舵角、最短转向并允许驱动速度反号、四轮共同缩放而非逐轮 clamp、正逆解往返和奇异输入拒绝。任一模块未对齐时所有驱动输出保持 0；任一关键电机 stale 时整车按同一故障策略停机。

---

## 第 11 步：拆双固件和板间协议

把当前空 application 拆为 `application/chassis` 与 `application/gimbal`，但共享的只能是纯协议、控制算法和设备 API，不能共享可写全局控制状态。

板间协议先在 native_sim 做纯 codec/parser：固定 magic/version/type/length/sequence/timestamp/payload/CRC；显式端序；逐字节重同步；半帧跨 callback 保留；未知版本和长度拒绝；sequence 回退、重复和 timeout 有确定行为。只有纯测试通过后才接 Zephyr UART/CAN-FD transport。

接收 callback 只搬字节，worker 解析并发布不可变 snapshot。enable/arm 必须是有时效的一次性 token，不是可永久重放的布尔位。

---

## 第 12 步：后续业务严格后移

基础闭环稳定后再按以下顺序推进：

1. 大小 Yaw 协调：先中心卸载，再考虑低频运动分配；每轴前馈只承担分配给该轴的轨迹，不能重复加入两份完整 FF。
2. 当前赛季裁判 RX/stream parser/decoder/store；不要把 packed struct 直接压到 UART 字节上。
3. 功率策略：裁判数据 stale 时保守降级；四轮命令共同缩放，策略层不直接控制 CAN。
4. UI scene/diff 与独立 TX scheduler；稳定 ID，重连重建，不每周期 DeleteAll。
5. 发射机构：左右摩擦轮独立速度控制器、热量/供电 interlock、拨弹控制器和 shot ledger。裁判弹速是稀疏观测，不是高速 PID 反馈。

实弹、功率器件和双 Yaw 限位都必须另有现场安全方案、独立急停人员和硬件保护；软件状态机不能替代物理防护。

---

## 附录 A：建议构建与验证命令

本文没有执行这些命令，因为它们会生成工作区产物。由你在完成对应文件后运行：

```bash
# 第 4 步：纯控制算法
west twister -T tests/control -p native_sim -v \
  -O build/twister/control

# 第 5 步：IMU 调用方迁移
west build -b dm_mc02/stm32h723xx \
  samples/imu_test -d build/imu_test -p always \
  -- -DBOARD_ROOT=$PWD

# 第 6 步：DJI 纯测试和统一 0 A sample
west twister -T tests/motor/dji -p native_sim -v \
  -O build/twister/dji

west build -b dm_mc02/stm32h723xx \
  samples/motor/dji_unified -d build/dji_unified -p always \
  -- -DBOARD_ROOT=$PWD

# 第 7/8 步：调参状态机纯测试和单电机速度环 sample
west twister -T tests/application/motor_tuning -p native_sim -v \
  -O build/twister/motor-tuning

west build -b dm_mc02/stm32h723xx \
  samples/motor/dji_speed_control \
  -d build/dji_speed_control -p always \
  -- -DBOARD_ROOT=$PWD

# 第 11 步后才存在
west build -b dm_mc02/stm32h723xx \
  application/chassis -d build/chassis -p always \
  -- -DBOARD_ROOT=$PWD

west build -b dm_mc02/stm32h723xx \
  application/gimbal -d build/gimbal -p always \
  -- -DBOARD_ROOT=$PWD
```

每次先看第一条编译或测试失败，不要在底层尚未通过时继续写上层 application。

## 附录 B：硬件上电总顺序

1. 动力电源关闭，烧录默认零锁定固件。
2. 驱动轮架空；舵向、Yaw 和发射机构卸载或可靠限位。
3. 全断电检查 CAN/PWM、唯一 ID 和终端；两端 120 Ω 时总线约 60 Ω。
4. 实测物理急停或动力切断有效，安排独立观察者。
5. 只收反馈，不允许任何非零命令。
6. 核对 profile、motor-id、feedback/command/slot、方向、零点和单位。
7. 每条 `dji::Bus` arm 前发送所有已知分组全零并抓包。
8. 在 current-limit=0 或动力级隔离下验证 feedback timeout、command TTL、bus-off、stop/recover 和零帧失败升级。
9. 每次启动收到新的人工 arm token 后，才允许单电机极小限时命令。
10. 单速度环通过后才做单关节串级；单模块通过后才接四模块。

出现反馈时间戳停止、输出非有限、电流持续饱和、误差越控越大、机构逼近硬限位、CAN error 快速增加或 `zero_sent=false`，立即物理断电，不继续靠调参试错。

## 附录 C：故障索引

| 现象                            | 先检查                                        | 不要先做                      |
| ------------------------------- | --------------------------------------------- | ----------------------------- |
| PID 返回`-EINVAL`             | 配置 finite、上下限、负增益、空指针           | 忽略错误继续输出上一值        |
| PID 返回`-ERANGE`             | 实际 dt、停顿、运算溢出                       | 把 dt_max 无限制放大          |
| setpoint 阶跃有 D 尖峰          | 是否误用 error derivative、reset 是否捕获测量 | 同时乱改三项增益              |
| 有 FF 时积分仍增长              | anti-windup 是否按 P+I+D+FF 总和判断          | 给积分限幅继续加大            |
| deadband 内输出全丢             | 是否提前 return、FF 是否由组合 core 合成      | 在调用方再复制一个 PID        |
| 两台电机互相影响                | 是否共享 state 或静态临时状态                 | 加一个大 mutex 掩盖所有权错误 |
| device ready 但 timestamp=0     | feedback ID、CAN 接线、电调 ID、filter        | 改 PID                        |
| 0 A 仍有输出                    | 抓全部 group 帧、方向/单位、唯一 Bus owner    | 只清软件缓存后停发            |
| Bus Fault 且`zero_sent=false` | CAN/供电/接线，立即物理断电                   | 自动重新 arm                  |
| recover 后旧命令复活            | epoch、目标缓存、reset 和新 arm token         | 增大 TTL                      |
| 8191→0 位置跳变                | DJI core 回绕累计和 gear ratio                | 在 PID 里补一圈               |
| 舵轮停车回零                    | 零速保持上一舵角                              | 使用`atan2(0,0)`            |
| 四轮限速后方向改变              | 是否按共同因子缩放                            | 分别放宽各轮上限              |
| 高频遥测花屏                    | async TX buffer ownership 和发送预算          | 在控制线程持续 printk         |

## 附录 D：当前未验证假设

- M3508 标准减速比是否适用于当前实物，或机构已经换过齿轮组。
- 三种 DJI 电机反馈 `current_raw` 与真实安培的关系是否已用 Assistant/台架交叉验证。
- 每个机构的安全电流、反馈 timeout、command TTL 和最大允许控制周期。
- DM-MC02 与新 `rm_typec` 板的 CAN 收发器、终端和供电拓扑。
- GM6020 实物固件、Assistant 版本、电流环开关、零点和限位。
- 当前 `HEAT_OFFSET_NS`、温控 PID 参数和 20 ms PWM 是否经过热稳定性验证。
- 四舵轮几何、轮径、零点、方向和真实允许电流。
- 大/小 Yaw 型号、传动比、同轴程度、IMU 层级和限位。
- 双板最终 transport、总线负载和当赛季裁判协议版本。

任何一项不确定时，保持输出为 0，并把实测结论写入独立参数记录；不要把试验猜测固化成控制库默认值。

## 附录 E：最终进度板，只保留尚未完成的工作

- [ ] 第 1 步：`lib/control` 目录、Kconfig/CMake 和统一错误/所有权契约完成。
- [ ] 第 2 步：经典 PID、测量微分、滤波、reset 和 anti-windup 完成。
- [ ] 第 3 步：纯前馈与组合式 Feedforward PID 完成，总饱和能约束积分。
- [ ] 第 4 步：轨迹/角度工具及 control 全测试矩阵通过。
- [ ] 第 5 步：IMU 温控迁移完成，旧 PID device/binding/driver 零引用并删除。
- [ ] 第 6 步：DJI 纯测试、clean build、RX-only 和 0 A 台架证据完整。
- [ ] 第 7 步：唯一 owner 控制线程、参数 active/pending、静止 Commit、实际 dt、新反馈门控和故障归零完整。
- [ ] 第 8 步：M3508/M2006 单速度环分别标定并通过安全验收。
- [ ] 第 9 步：GM6020 串级、大/小 Yaw 独立闭环和软硬限位通过。
- [ ] 第 10 步：舵轮纯数学、单模块、四模块与底盘状态机通过。
- [ ] 第 11 步：双 application、纯板间协议和 transport 通过。
- [ ] 第 12 步：Yaw 协调、裁判、功率、UI 与发射业务按需完成。

## 附录 F：每次收工的证据清单

- [ ] 只完成了当前步骤，没有跨层夹带重构。
- [ ] 正常、边界、错误和恢复路径都有测试或台架证据。
- [ ] 所有控制输出单位在接口处写明，raw 只存在于协议层。
- [ ] 一个环一份状态；一个 Bus 一个 owner；ISR 不执行控制算法。
- [ ] stale、非法 dt、非有限输出、发送失败都会 reset 并进入安全状态。
- [ ] 上电先 RX-only、再全零、最后才是极小限时非零。
- [ ] 参数、构建日志、抓包、Git commit 和预期/实际现象已保存。

## 附录 G：固定参考入口

DJI 当前代码职责、分组帧、生命周期与并发说明见仓库根目录 `DJI统一电机架构说明.md`。电机协议优先使用 `motor_docs/` 中固定保存的 C610、C620、M2006、M3508、GM6020 和 DM-J4310 手册，并保留 `DOWNLOAD_LINKS.txt` 的来源记录。Zephyr 接线以当前工作区固定提交的官方头和文档为准；裁判系统实施前重新核对当赛季官方协议和规则。
