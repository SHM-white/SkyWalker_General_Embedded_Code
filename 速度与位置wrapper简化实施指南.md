# 速度与位置 wrapper 简化实施指南

> 实施状态：后续已按用户授权直接实现为 `VelocityMotor` / `PositionMotor`，旧 C++ wrapper 已移除。
> 当前接口、限制和五个迁移示例见 [统一速度与位置控制](samples/motor/MOTOR_CONTROL.md)。下文保留设计讨论，拟议类名不代表最终 API。

## 结论：能优化，优先减少调用方的工作

当前 `VelocityController`、`PositionController` 仅管理控制算法的配置和历史状态。调用复杂的主要原因是应用还要负责初始化总线、读反馈、计算周期、调用算法、写力矩缓存、flush、错误停机和位置坐标适配。

建议保留这两个算法类，先增加一层 **DM MIT 单电机控制封装**，让示例主循环只需 `update(目标)`。第一版放在现有 `samples/motor/dm_common`，复用已经存在的 Session 和保护逻辑，不急着建立全品牌控制框架。

本指南为拟议接口和手工实施步骤，所有新类、结构、方法尚不存在；本次没有修改业务源码或运行构建。适用范围是当前 DM MIT 软件速度/位置控制示例，不包含原生模式切换、多电机总线托管或新 PID 算法。实际固件的位置回绕行为仍待验证。

## 大疆兼容性补充：共用外观，分开驱动适配

用户进一步明确需要考虑大疆兼容性。上述 DM 单电机方案可作为首个实现，但若第一版就要同时支持大疆，应将它调整为下面的结构；本文原先的 `MitVelocityAxis` / `MitPositionAxis` 用法仅表示 DM 专用版本，不是已经兼容大疆的实现。

**可以共用 begin/update/stop 的调用方式，不能把内部的 DM Session 原样换一个 device 就使用。** 当前算法类已经同时用于 DM 和 DJI，新增工作是统一电机执行流程的接口，分别适配各自驱动。

```text
应用：VelocityAxis / PositionAxis 的 begin、update、stop、telemetry
   ├ 现有 VelocityController / PositionController（共用）
   └ 电机适配（初始化时显式选择）
       ├ DmMitBackend：DM 反馈/模式检查、setTorque、DM Bus
       └ DjiCurrentBackend：DJI 反馈/能力检查、setCurrent、DJI Bus
```

这是拟议命名，不是当前已有类。第一版可以用两个具体适配类加少量共用流程实现，不必为此引入动态分配或插件系统。调用方只在创建时选择适配器和参数；业务循环仍调用 `axis.update(target)`。

### 已从当前源码确认的差异

| 项目 | DM MIT | 当前 DJI 驱动 | 封装要求 |
| --- | --- | --- | --- |
| 控制输出 | `motor::setTorque()`，N·m | `motor::setCurrent()`，A；set_torque 为空 | 后端明确输出种类，配置与其匹配 |
| 速度反馈 | rad/s | rad/s，驱动已按 gear_ratio 换算输出轴速度 | 算法无需改动，不要重复除减速比 |
| 连续位置 | 驱动直出协议位置，当前在应用解包 | 驱动累计编码器并换算输出轴位置 | 后端向算法交付统一连续位置 |
| 固定零点单圈角 | 当前未声明通用绝对位置能力 | 仅合适的传感器且传动比为 1:1 时声明 | 按能力检查，不按品牌一概支持 |
| 反馈保护 | 可检查力矩、MOS/转子温度、Enabled 状态 | 电流反馈；温度取决于 profile；没有 DM 状态字段 | 不能照搬 DM readSafeFeedback |
| arm | Enable/状态确认及中性命令 | 新鲜反馈检查、发送零电流组帧、软件 armed | begin 的外观相同，动作由后端实现 |
| stop | DM Disable 路径 | 清除软件命令状态并发送零电流组帧 | 共用“撤销输出”，不承诺机械制动或断电 |
| CAN flush | 各电机独立控制帧 | 按 command_id/slot 组成组帧 | 两种 Bus 分别保留，不共用打包代码 |

主要依据：`include/drivers/motor/dji_bus.hpp`、`dji_motor.hpp`、`drivers/motor/dji/dji_motor.cpp` 的 getCapabilities/反馈解码/API 表、`dji_bus.cpp` 的 arm/stop，以及现有 DJI 速度/位置示例。

### 最小适配契约

前面的 axis 生命周期、错误、线程和周期约定继续有效；后端只负责设备差异，不自己运行 PID。以下接口是设计契约，类型可在落地时用普通结构体实现：

| 拟议接口 | 返回/副作用 |
| --- | --- |
| `prepare()` | 0 或底层负 errno；检查设备类型/能力和配置，初始化并挂接对应 Bus，等待反馈；不启动闭环 |
| `readMeasurement(Measurement &out)` | 0 或负 errno；返回经过新鲜度/状态检查的速度、连续位置及可用字段标志；失败不发布新测量 |
| `effortKind()` / `effortLimit()` | 返回固定的 CurrentA 或 TorqueNm 及对应上限；begin 时验证控制器配置单位/限幅匹配 |
| `arm()` | 0 或负 errno；DM 走使能确认，DJI 检查零帧发送成功；保存各自详细报告 |
| `stageEffort(float value)` | 暂存命令；DM 调 setTorque，DJI 调 setCurrent；值单位由 effortKind 固定，不隐式换算 |
| `flush()` | 发送对应 Bus 的命令；返回底层错误并保留原始报告 |
| `stop()` | 尽力撤销输出并保存停机结果；不同品牌保留不同物理语义 |

对同一轴，控制器配置、最终限幅和遥测输出必须使用相同 effortKind。不要“检测到 CommandCurrent 就把原来 N·m 的数值当 A 发出”。换电机必须换匹配的配置，但不必改业务目标函数。当前驱动未提供可靠的统一力矩转换模型，本轮不新增 A↔N·m 自动换算。

通用运行检查至少包括所需位置/速度字段、时间戳、有限值和在线状态；温度若被配置为必需却无能力支持，begin 返回 `-ENOTSUP`。若明确配置为可选，则按字段有效标志展示“不可用”，不能将默认 0°C 当作真实反馈。DM 特有的 Enabled 和 MOS 温度检查留在 DM 后端。

### 位置共同基线与总线归属

最容易跨品牌统一的是“相对 begin 时位置的连续角度”：DJI 后端保存 begin 时 position_rad 并做偏置，DM 后端从 begin 时反馈开始累计解包。这样 `update(π/2)` 对两者都表示同一个起始坐标下的 +90°，不会混淆“驱动首次反馈”与“应用开始控制”的时刻。

固定零点、最短路径和连续正向旋转作为显式目标策略保留；DJI 不满足绝对位置能力时，不能静默降级成相对零点。DM 保存零点语义和跨量程行为仍需要实机确认。

单电机便利接口可以把后端拥有的 Bus.flush 隐藏进 update。多台 DJI 共用发送组时，Bus 必须由组级对象统一持有，各轴只 stage，所有轴本拍计算成功后统一 flush；不能每个轴创建独立 Bus 并各发一份完整组帧，否则可能覆盖组内其它电机的命令。DM 多轴也沿用组级管理，不用每轴重复供电/握手。

### 实施顺序对原方案的调整

1. 保持 `include/control/*controller.hpp` 与 C 算法内核不变。
2. 将 DM 方案的设备操作收进 DmMitBackend；从现有 DJI 示例提取 DjiCurrentBackend，保留 DJI 的能力检查和 FlushReport。两种适配只引用各自驱动，原生 DM 模式在 DM MIT 后端返回 `-ENOTSUP`。
3. 让共用 Axis 承担计时、controller.reset/step、生命周期和错误汇总；begin 前校验 effort 种类匹配。第一版限定单电机使用，组级扩展按上述边界单独实现。
4. 迁移 DM 和 DJI 的速度示例，再迁移连续相对位置示例。自检：只改变初始化中的后端和调参配置，周期调用外观一致；反馈已经换算到输出轴，不重复除减速比。
5. 共用 Axis/契约可放入拟新增的 `include/control/motor_axis.hpp`，平台适配和启动辅助先分别留在 sample common 目录；为 DM 与 DJI 各自 CMake 只链接相应后端，避免单品牌构建强依赖另一品牌 Kconfig。

手工实现后，除本文原有两个 DM 构建命令，再构建现有 DJI 入口；命令仅供用户执行，本次未运行：

```sh
west build -b dm_mc02/stm32h723xx -d /tmp/skywalker-axis-dji-velocity samples/motor/dji_speed_control
west build -b dm_mc02/stm32h723xx -d /tmp/skywalker-axis-dji-position samples/motor/dji_position_control
```

上板仍使用各自匹配的电流/力矩上限和供电配置，先固定机身、悬空输出并准备物理断电。DJI stop 成功仅说明零电流发送路径成功，不代表输出轴已停止转动。最终检查还应包括后端/effort 匹配、温度能力缺失处理、相对 begin 的零点一致性和同一 DJI 组只有一个发送管理者。

## 希望调用方最终写成什么

以下是目标用法，不是当前可编译 API。`makeVelocityConfig()` / `makePositionConfig()` 由应用提供具体参数，封装不自动猜测电机增益。

```cpp
// DM MIT 软件速度控制；静态对象保证注册到 CAN 的对象长期有效。
static MitVelocityAxis axis{DEVICE_DT_GET(DT_ALIAS(motor0)), makeVelocityConfig()};

int main() {
    int ret = axis.begin();
    if (ret < 0) return ret;

    for (;;) {
        k_sleep(K_MSEC(5));
        ret = axis.update(targetVelocityRadS()); // rad/s；内部已处理故障停机
        if (ret < 0) return ret;
        // 可选：从 axis.telemetry() 取数据交给 VOFA。
    }
}
```

位置控制用法相同，只更换类与目标单位：

```cpp
static MitPositionAxis axis{DEVICE_DT_GET(DT_ALIAS(motor0)), makePositionConfig()};
// begin() 成功后，每拍：
ret = axis.update(targetPositionRad()); // rad，第一版采用连续位置目标
```

业务层仍保留目标生成和线程调度：多久转到哪里、运行多久属于应用。停止运行时显式调用 `stop()`；不要依靠析构函数完成总线停机或销毁仍有异步回调的对象。

## 两层职责

```text
应用：目标函数、线程睡眠、遥测展示
   ↓ begin / update(target) / stop
MitVelocityAxis / MitPositionAxis
   ├ Session：CAN、供电、握手、反馈保护、arm/flush/stop
   ├ 周期计时、错误记录、位置反馈适配
   └ 现有 VelocityController / PositionController：纯算法
```

算法 wrapper 继续接受显式的测量和 dt，保留用于其它平台和精确控制周期的能力。新层负责使用 Zephyr 时间和 DM 驱动，调用方不会再到处重复设备操作。

不要把 CAN、设备对象、时钟直接加到 `include/control/*controller.hpp`：现有 DJI 路径也在用这些类。也不需要第一版就引入模板后端、继承树或自动选择电机模式。

## 拟议接口与约定

建议在 `samples/motor/dm_common/include/dm_mit_axis.hpp` 声明下面两个类。配置、遥测和诊断结构随实现补齐，先按本节约定固定语义。

```cpp
class MitVelocityAxis {
public:
    MitVelocityAxis(const device *motor, const VelocityAxisConfig &config);
    int begin();
    int update(float target_velocity_rad_s);
    int stop();
    const VelocityTelemetry &telemetry() const;
    const AxisStatus &status() const;
    // 禁止复制和移动：Session::bus 注册的 CAN 回调保存其地址。
};

class MitPositionAxis {
public:
    MitPositionAxis(const device *motor, const PositionAxisConfig &config);
    int begin();
    int update(float continuous_target_rad);
    int stop();
    const PositionTelemetry &telemetry() const;
    const AxisStatus &status() const;
    // 同样禁止复制和移动。
};
```

### 生命周期和错误

- 构造函数只保存配置，不访问设备、不上电；构造不意味着已经可以控制。
- `begin()` 合并 validate、prepare、首次安全反馈、controller.reset 和 arm。调用它会尝试给电机使能，应在设备准备好后显式执行。返回 0 才进入 Running。
- 第一版 `begin()` 每个对象只允许尝试一次；后续调用返回 `-EALREADY`。失败后的 Bus 可能已经注册过滤器，不能通过再次 prepare 假装回到全新状态。重新运行和故障恢复另行设计，不在本轮添加。
- `begin()` 中先完成可离线检查的配置校验，再调用 prepare；prepare 尝试过后发生失败，若 Bus 已初始化则尽力 stop，记录原始错误和停机错误，进入 Fault。对象仍需长期存活。
- `update()` 要求 Running；否则返回 `-EACCES`。每次检查目标、计算真实 dt、读取安全反馈、运行算法、写力矩命令、flush。成功 0。
- 更新时非有限目标返回 `-EINVAL`；目标越界、周期越界或软件保护触发返回 `-ERANGE`；反馈/驱动故障等保留底层 errno。任何运行中失败进入 Fault，统一尽力 stop；本拍失败以后不发送正常控制命令。
- `update()` 的返回值保留触发故障的原始错误；`status()` 另存 `stop_error` 和 `TxReport`，避免停机发送失败覆盖真正故障原因。`stop_error != 0` 时不能报告“已经物理失能”。
- `stop()`：未 begin 返回 0，不调用未初始化 Bus；已初始化时调用 Bus.stop，成功进入 Stopped；失败进入 Fault 并允许再次尝试 stop。软件 Stopped 不作为物理停机确认。
- `telemetry()` 保存最近成功周期的反馈、目标、算法输出、时间戳和 valid 标志。故障后保留诊断数据但将有效标志置假，不能把旧值当作新反馈。
- 单控制线程调用；begin 包含供电等待和握手，会阻塞，update/stop 也可能等待 CAN 发送。不要在中断里调用，不自动创建后台线程，不承诺硬实时截止时间。遥测若交给其它线程，需要复制快照并同步。

### 周期：自动算 dt，不假定睡眠等于真实周期

将时间戳保存在新类中。`begin()` 在 arm 完成后才记录起点，因为 arm 会等待反馈。`update()` 取两次调用起点的实际时间差；要求 dt 落在控制器允许区间内，位置模式使用两级控制器允许区间的交集。

保留当前示例 1～20 ms 的控制器允许范围与超期停机语义。调用方仍每 5 ms 左右调用一次，但不可将 dt 固定写成 0.005，也不可把过大的 dt 截断后继续运行。第一次 update 也要经过一个有效周期，不能在 begin 返回后立即无间隔调用。

超时检查只有线程继续运行时才有机会执行；包装 update 不会凭空产生独立看门狗。

## 配置也应简化，但不要隐藏单位

第一版配置可以直接包含已有 `Controller::Config` 加保护参数；这样能先移走主循环样板代码，不改变调参含义。再按实际重复情况增加命名配置工厂，把 PID 之外的零前馈等重复字段集中填充。

配置建议分为：

| 配置 | 应用必须明确 | 可集中填写的内容 |
| --- | --- | --- |
| 速度 | PID 增益、积分输出限幅、速度上限 rad/s、力矩上限 N·m | 零前馈、滤波、速度斜率、dt 范围 |
| 位置 | 位置增益、内层速度配置、零点语义 | 位置输出限幅由内层速度上限生成 |
| 保护 | 超速阈值、温度阈值 | 通用检查流程与失败处理 |

速度 PID 输出限幅、最终 effort 限幅从同一个 `torque_limit_nm` 生成，仍检查不超过驱动 `Descriptor::torque_limit_nm`。PID 积分限幅有独立含义，不应为了少一个参数直接等同于总输出限幅。DM 配置避免使用没有单位的 `max_output` 名称。

不要直接复用 DJI 增益：同一个 effort 字段在 DJI 中表示 A，在 DM 中表示 N·m。不要默默更改当前示例增益。首次上板的保守限幅由应用配置明确给出。

## 位置封装必须明确的事

第一版 `update()` 接受连续位置 rad：0、π/2、π、3π/2、2π 表示持续正向推进；3π/2 后输入 0 表示目标回到原来那一圈的 0。不要根据目标变小就自动猜“转下一圈”，也不要默认走最短路。

把现有 DM MIT 示例按时间生成的目标圈数保留在应用目标函数里。首次“向前找保存零点”的动作同样作为业务目标策略，不让 begin 自动发起找零运动。

`PositionAxisConfig` 要求显式选择零点：

- `StartupRelative`：首次有效反馈定义为 0，适合相对上电位置的运动。反馈转换层保存前一原始位置并累计位移。
- `SavedZero`：以原始反馈的保存零点坐标初始化连续位置；仅在确认固件反馈坐标和回绕语义后使用。软件不保证恢复上电前历史圈数，也不自动调用 savePositionZero。

两者都先从有效反馈初始化解包状态，按 `remainder(raw - previous_raw, 2*PMAX)` 累计。它依赖实际回绕且相邻有效样本位移小于半周期；反馈失效则停机，不尝试跨长时间丢帧继续累积。

当前 `PositionController` 的 C 内核冻结位置积分，即使 ki 非零也不累积；本轮简化调用时保留现有行为并写入配置说明，不顺带改变控制算法。

通用位置 wrapper 应接收同一稳定坐标系中的目标和测量。不要照搬示例的“目标固定 0，测量减去每拍目标”作为唯一实现：将来启用测量微分时，移动坐标会把目标阶跃混入测量微分。非常长时间的连续圈数精度处理可以单独设计，不能靠改变坐标掩盖。

## 文件级手工实施顺序

1. **新增 `samples/motor/dm_common/include/dm_mit_axis.hpp`。** 声明配置、两类接口、状态枚举、诊断与遥测；两个类都禁止复制移动，明确仅支持 MIT。自检：用户是否只需配置、begin、update、stop；头文件是否记录单位和对象寿命。
2. **新增 `samples/motor/dm_common/src/dm_mit_axis.cpp`，先做速度。** 迁入速度示例中反复出现的检查和执行流程；复用 dm_sample_support 的 prepare/readSafeFeedback/arm/flush/stop。为保留完整诊断，需要 TxReport 时直接调用 Session.bus 对应接口。自检：构造无副作用、arm 后计时、运行失败均经过统一停机入口。
3. **在同一文件实现位置类。** 复用速度类的错误/计时辅助逻辑，调用已有 PositionController；位置解包集中在私有辅助对象中，目标轨迹仍在 main。自检：跨量程周期使用 `2*PMAX`，零点策略显式，未增加自动保存零点动作。
4. **调整两个 MIT 示例 `src/main.cpp`。** 保留配置、目标生成、VOFA 与睡眠；改为上述使用方式。Session 由 axis 内部拥有，应用不再创建第二份 Session。自检：对照旧代码，保护阈值、目标时序与错误路径没有无意丢失；位置旧目标策略按选定坐标明确迁移。
5. **调整两个 MIT 示例 `CMakeLists.txt`。** 添加 `../dm_common/src/dm_mit_axis.cpp`，保留已有共享支持源文件与 include 路径。不改原生速度/位置示例，也不改底层算法。

update 的内部实现可参照以下伪代码：

```cpp
if (state_ != Running) return -EACCES;
// fail(error) 必须记录原因、标记 Fault/遥测失效，并尽力 stop。
if (!finite(target)) return fail(-EINVAL);
auto now = k_uptime_get();
if (!validInterval(now, previous_cycle_ms_)) return fail(-ERANGE);
float dt_s = (now - previous_cycle_ms_) / 1000.0f;
int ret = readSafeFeedback(session_, speed_cutoff_, temperature_cutoff_, fb, raw);
if (ret < 0) return fail(ret);
// 位置路径：先连续化反馈，再传 continuous target/position。
ret = controller_.step(target, fb.velocity_rad_s, dt_s, out);
if (ret < 0) return fail(ret);
ret = motor::setTorque(session_.motor, out.effort_command);
if (ret < 0) return fail(ret);
ret = session_.bus.flush(tx_report_);
if (ret < 0) return fail(ret);
previous_cycle_ms_ = now;
publishSuccessfulTelemetry(fb, raw, target, out, now);
return 0;
```

失败后算法状态可能已更新而 CAN 发送未成功；进入 Fault 后不继续 step，不能无缝重试旧运行状态。重新运行需要将来专门定义恢复与 reset 流程。

## 多电机边界

上述封装限定一个 axis 独占一个 Session/Bus 控制上下文。不要把这种单电机对象直接复制成同一 CAN 上的多轴框架：供电握手和总线生命周期会重复，且某个 axis 的 flush 不代表整组本拍命令已经更新。

真正扩展到多个电机时，把 Session/Bus 提升到组级：各轴只读取反馈和暂存命令，所有轴成功后统一 flush，任一轴失败由组级决定停机范围。轴提供内部 `stage(target, now)`，组提供 update；当前单电机便利接口可以保留，不必提前实现这个扩展。

## 验证和交付检查

本次是指南交付，没有编译、烧录或上板。用户完成手工修改后可执行：

```sh
west build -b dm_mc02/stm32h723xx -d /tmp/skywalker-axis-velocity samples/motor/dm_mit_velocity_control
west build -b dm_mc02/stm32h723xx -d /tmp/skywalker-axis-position samples/motor/dm_mit_position_control
```

首次上板保持电机固定、输出轴悬空且能立即断电；先确认 MIT 模式、ID、量程与 overlay 一致，使用保守力矩限制。begin 的供电顺序沿用现有支持代码：启动 CAN/注册过滤器、MC02 开启 XT30_1、等待 1.5 s、反馈握手、控制器初始化、arm。

预期速度示例仍按原参数跟踪目标，位置示例仍按明确迁移后的连续目标运动，VOFA 数据可从 telemetry 获取。begin 后第一次 update 报周期错误时先检查是否保留睡眠；`-EALREADY` 表示重复 begin；跨量程跳变检查解包周期和固件行为；故障后的 `stop_error` 非零表示失能发送未成功。

- [ ] 新接口把日常调用收敛为 begin、update、stop。
- [ ] 现有算法 wrapper 和 C 内核保持复用。
- [ ] 初始化顺序、周期检测、温度/速度/力矩保护均保留。
- [ ] 配置明确单位，位置明确零点和连续目标语义。
- [ ] CAN 回调对象持久存活，禁止复制移动。
- [ ] 失败保留原始原因和停机错误，不自动恢复输出。
- [ ] 手工修改后完成两个示例构建，再进行受控硬件验证。
