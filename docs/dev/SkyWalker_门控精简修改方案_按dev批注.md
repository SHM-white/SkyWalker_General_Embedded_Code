# SkyWalker 门控精简修改方案：按 dev 批注实施

基线：`dev@dc258ec0b9dda91524c2b35f0061f2fb2a4ab356`，2026-11-10。依据：[你批注后的 180 项清单](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/docs/dev/SkyWalker_%E9%97%A8%E6%8E%A7%E5%88%A0%E7%95%99%E5%86%B3%E7%AD%96%E6%B8%85%E5%8D%95_180%E9%A1%B9.md)。原清单的源码行号基于较早的 main；本方案以当前 dev 源码为准。

**本文只交付方案和伪代码，没有改业务源码、配置、测试，也没有提交或推送。** 伪代码表达修改后的逻辑，函数名带说明性质；不能整段复制后假设已有对应 API。

文件放进仓库 `docs/dev/` 后，“本地文件”链接可在 VS Code 打开对应源码；“基线源码”链接指向本次分析的固定提交及位置。下载后单独阅读时使用基线源码链接。

## 1. 给本地执行者的总要求

目标是减少运行时拒绝执行、重复许可、故障连坐和不必要的恢复动作。**不要在删一个判断后补一个新的 ready/confirmed/armed/permission 状态来代替它。**

1. 按本文“建议本轮实施”修改；明确标成“保留”“延期”的项目不顺手重构。附录逐项保留用户原话，未定或问号不等于同意删除。
2. 固定硬件拓扑、参数范围、模式组合在编译期发现；运行时硬件返回值、收包内容、反馈与计算结果仍按实际数据处理，不能谎称编译期能知道。
3. 上层意图与下层执行分开：某个轮子输出失败不能撤销遥控授权，底盘不能动不能推导大 Yaw 不能动。真正的左 Down 停止指令仍传给对应机构。
4. 有限、有效的数值直接进入控制算法；算法需要的反馈不存在时，只处理依赖该反馈的本轴。禁止用 NaN、过期数据伪造“有效”来凑齐执行条件。
5. 参数集中，但不建设新的配置框架、权限服务或故障管理器。沿用 `vehicle/calibration.hpp` 及现有 profile 的分工；可调数值只保留一处来源，profile 负责装配。
6. 没启用的视觉、射击、MIT 航向锁、双 IMU 配对、热控，不因为这次整理而启用。已有 CRC、长度解码、总线恢复和用户明确保留的检查不重写。
7. 不新增正常运动前的“连续 N 帧”“重新拨杆”“确认恢复”“等待若干毫秒”。本文保留的原有机制也不得扩大适用范围。

### 实施顺序

| 批次 | 修改范围 | 主要结果 |
| --- | --- | --- |
| P1 | A 配置；B 四轮门控；C 零速；D 机构解耦；E 软限位 | 首先解决正常运动被打断、零速舵轮松掉和故障连坐 |
| P2 | F 周期处理；G 重复校验；H 遥控启动；I 功率与裁判 | 减少无意义 disable/re-enable、重复拒绝和缺数据停控 |
| P3 | J 原始 gyro 发布；K 通信参数整理 | 解开一个明确的数据发布依赖；整理心跳与参数 |
| 延期 | L Boot/IMU 航向/自动模式；M 尚未启用的功能 | 给出说明与后续方向，本轮不批量改动 |

本地执行时先连续完成选定批次；不把本文行为描述转成新测试框架或新运行门槛。按仓库当前工作流，在全部实现结束后至多做一次最终编译检查，是否进行实机调试由用户安排。本方案未运行固件编译或实机测试。

<a id="a"></a>

## A. 固定配置前移到编译期，统一可调数值

涉及 005、007、021、026、087、088、090、098、122、126、129、163、165、170、176；001—004 按批注暂保留。

入口：calibration.hpp（[本地文件](../../include/robotics/vehicle/calibration.hpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/include/robotics/vehicle/calibration.hpp#L16)）；swerve_profile.hpp（[本地文件](../../include/robotics/vehicle/swerve_profile.hpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/include/robotics/vehicle/swerve_profile.hpp#L7)）；big_yaw_profile.hpp（[本地文件](../../include/robotics/vehicle/big_yaw_profile.hpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/include/robotics/vehicle/big_yaw_profile.hpp#L6)）；motion_profile.hpp（[本地文件](../../include/robotics/vehicle/motion_profile.hpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/include/robotics/vehicle/motion_profile.hpp#L13)）；manual_profile.hpp（[本地文件](../../include/robotics/vehicle/manual_profile.hpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/include/robotics/vehicle/manual_profile.hpp#L13)）。

### A1. 哪些能编译期判断

| 数据 | 处理位置 | 不能混淆的点 |
| --- | --- | --- |
| 电机 ID、协议模式、同一总线 ID 冲突、数量/静态容量、方向 ±1、减速比、轮径、零点范围 | constexpr 配置 + static_assert | 不同物理总线 ID 相同是正常的；检查实际协议占用 ID，而不只检查表面电机编号 |
| PID 常量、机械角范围、速度/电流上限、死区、倍率、任务周期配置 | constexpr 配置 + static_assert | 编译期检查的是配置，不是运行中的 PID 输出 |
| 固定控制器绑定到哪个 Motor | 静态装配中明确唯一归属 | 任意运行时指针是否重复绑定不能普遍 static_assert；不要为此引入一套模板注册框架 |
| CAN/SPI/UART 是否 ready、设备是否应答、发送结果 | 初始化或驱动调用处 | 编译通过不能证明线已接好 |
| 实时帧 CRC、长度、传感器 NaN、运算后溢出、陈旧回调 | 最靠近数据入口/提交处 | 编译期没有这些值，不能无条件删除 |

建议让纯 profile 构造函数在现有类型允许时成为 `constexpr`，把纯配置校验改为可常量求值函数，正式配置用 `static_assert(valid(...))`；不要继续在每个 update 周期重复校验常量。若底层 C 校验器不能 constexpr，先对实际配置写少量静态断言，不能为了消除几个 if 重写整个控制库。通用库仍可能供动态配置样例使用；必要的动态入口检查保留一次，不重复层层检查。

```cpp
// 伪代码：集中到现有 calibration/profile；沿用项目 C++ 标准。
constexpr bool finite_constant(float x) {
    return x == x && x >= -FLT_MAX && x <= FLT_MAX;
}
static_assert(finite_constant(wheel_radius_m) && wheel_radius_m > 0);
static_assert(all_motor_ratios_positive(steer, wheel));
static_assert(all_directions_are_plus_or_minus_one(steer, wheel));
static_assert(no_id_conflict_on_each_physical_bus(topology));
static_assert(topology.count <= existing_bus_capacity);
// 不依赖当前工具链尚不支持 constexpr 的 std::isfinite。
```

### A2. 大 Yaw 后端只留一个编译期选项

现在有 native/MIT/heading lock 等多个布尔量，虽然已经有部分 static_assert，但执行器仍到处判断。改成现有 profile 里的一个枚举选择，当前值 `NativeVelocity`。模式选择决定硬件配置、执行路径与相应配置校验；不要在正常周期反复确认实际选择。

```cpp
enum class BigYawBackend { NativeVelocity, MitVelocity, MitHeading };
inline constexpr auto big_yaw_backend = BigYawBackend::NativeVelocity;

hardware_config = build_hardware_for(big_yaw_backend);
// 用现有构建组织或简单 if constexpr 分开，任选一种，不做插件式后端系统。
if constexpr (big_yaw_backend == BigYawBackend::NativeVelocity) {
    update_native_velocity(request);
} else if constexpr (big_yaw_backend == BigYawBackend::MitVelocity) {
    update_mit_velocity(request);
} else {
    update_mit_heading(request);
}
```

这是程序选择，不是让 MCU 自动替电机修改固件里的 CTRL_MODE。设备实际模式与编译配置必须一致；通信失败不能用编译期断言代替处理。MIT 当前不使用，维持原实现，不顺手调参。

### A3. 一处调电流、一处最终输出裁剪

目前 `swerve_profile` 通过 `min(bench_current, effort_limit)` 设置 PID/环路限制，`ChassisExecutor::update` 又裁剪一次；Motor 还有自己的范围验证。正式底盘的舵向/驱动 bench 值分别为 **0.8 A / 0.5 A**。合并后先保持这些有效值，不借删门控放大电流。

建议将每轴 `effort_limit` 作为唯一可调电流值，删除重复的 `bench_*_current_limit_a`、独立 bench 常量与层层 `min()`。`power_control_calibrated` 不能再偷偷切换第二套电流限值。PID 的输出/积分抗饱和使用同一 limit；Motor 提交边界做一次最终幅值裁剪或范围转换。**抗积分饱和不是另一个需要删掉的许可门。**

```cpp
constexpr float drive_limit = wheel[i].effort_limit;
pid.output_min = -drive_limit;
pid.output_max = +drive_limit;
motor.current_limit_a = drive_limit;

raw_current = velocity_pid.step(target, measured, dt); // PID 按同一 limit 抗饱和
requested_current = raw_current * power_scale;        // 无额外 bench 比例暗中再压一次
motor.setCurrent(requested_current, generation);      // 最终边界处理协议可表示范围
// 删除 Executor 再次 clamp 到另一组 bench_* 的循环。
```

Motor 面向不同调用者仍需拒绝非有限值、错误命令类型；普通有限超幅值可统一裁剪到配置上限，而不是层层 suspend。要区分 DJI 电流 A、DM 力矩 N·m，不把两种 effort 混成一个全车数字。保留 DM 当前原生速度后端的实际速度约束。

### A4. 速度和倍率

手动 `vx/vy/wz`、yaw/pitch 速率；轮速/舵速；小陀螺平移 `.3/.4` 倍率；大 Yaw 转换中的 `.25` 反馈增益和 `±10` 上限，全部移到现有 calibration 的分组常量。profile 只引用。轮径、减速比只换算一次。

轮速总缩放继续保持四轮比例，不能分别裁剪四个轮子的速度破坏运动方向。当前 swerve profile 名义上写 `3 m/s`，又被轮径 × 轮轴上限压低；以当前参数 `0.05 m × 10 rad/s` 得到 **0.5 m/s**。建议只保留一个可调的有效上限，另一种单位由它推导；不要保留两个互相取 min 的调参入口。

<a id="b"></a>

## B. 删除四轮集体准入，保留每轮 cos³ 连续缩放

涉及 099、104—112。入口：SwerveChassis::step（[本地文件](../../lib/robotics/swerve.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/swerve.cpp#L291)）；SwerveModule::finishDrive（[本地文件](../../lib/robotics/swerve.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/swerve.cpp#L200)）；swerve_chassis.hpp（[本地文件](../../include/robotics/swerve/swerve_chassis.hpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/include/robotics/swerve/swerve_chassis.hpp#L5)）。

### 现在的链路

所有轮子 healthy、都不是重新初始化、代次一致、最大舵误差低于 0.10 rad，才置 `drive_permitted_`；普通运动舵误差超过 0.40 rad 撤销，旋转状态变化也撤销；任意模块计算出错，再把四个模块一起 `finishDrive(..., false)`。每个模块另外还有 `max(cos(error), 0)^3`。

### 改后结构

删除全局 `drive_permitted_`、只服务这套门控的 `rotating_`、`drive_generation_`、`align_enter_rad/align_exit_rad/rotation_epsilon_rad_s` 及对应 validate。删除 `healthy/max_error/moving` 集体放行计算。单模块不再接收外部 `permitted` 参数，`cosine_scale == 0` 是一个正常的零轮速目标，不清 PID 历史。

```cpp
SwerveChassis::step(command, feedback, dt, out):
    if command.mode == Disabled:
        execute_existing_explicit_stop()
        return

    if kinematics.solve(command, targets) fails:
        report_solve_error()
        return no_new_outputs_for_this_cycle
        // 不重置四轮、不清授权；旧 Motor 指令仍按原 20ms 寿命失效。

    for i in 0..3:
        result[i] = module[i].prepare(targets[i], feedback[i], dt)
        // prepare 完成舵向、翻转优化、误差和 cos³。
        if result[i].steer_output_valid:
            module[i].drive(
                target = result[i].optimized_wheel_rate * result[i].cosine_scale,
                feedback = feedback[i], dt)
        else:
            result[i].drive_output_valid = false
            // 只该轮；没有可信舵角时不能声称已完成方向补偿。

    publish_each_valid_axis(result)
    report_per_module_errors_without_global_stop()
```

每轮驱动 PID 仍需要本轮速度反馈；无反馈就不计算本轮电流。舵向反馈失效影响同一轮的方向计算，因此允许该轮驱动无新输出，但其他轮继续；这不是四轮集体许可。某个驱动失效时，同轮舵向仍可保持角度。

**调用链一并修改**：`SwerveHardware::stage` 已按每轴 `output_valid` 独立提交，保留这个形态；`ChassisExecutor` 不能因为 `first_error != 0` 再清整组；样例、诊断读到 `drive_permitted` 时只能显示计算结果，不能重新形成准入条件。同步清理 `include/robotics/swerve/swerve_module.hpp` 的 `finishDrive` 签名，以及 `swerve_types.hpp` 中只供旧门控使用的输出字段。删无调用的旧字段，避免保留一个“兼容”门在其他路径继续生效。

错误输入不能通过“沿用上次并刷新时间戳”无限续命。运动学整包不合法时，本周期不产生新输出；单轮出错只影响单轮；下一份有效输入自然恢复，没有确认、计数或解锁流程。按你保留的 130，持续没有有效输出仍受原 Motor 20ms 命令期限约束。

<a id="c"></a>

## C. 零速保持上次舵角，恢复运动直接追新角

涉及 097、100、103、109、166。入口：SwerveKinematics::solve（[本地文件](../../lib/robotics/swerve.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/swerve.cpp#L33)）；SwerveModule::prepare（[本地文件](../../lib/robotics/swerve.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/swerve.cpp#L223)）；idle_behavior（[本地文件](../../include/robotics/vehicle/swerve_profile.hpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/include/robotics/vehicle/swerve_profile.hpp#L34)）。

运动学已经在零速时保存 `last_angle_rad_`，问题在后级 profile 设置 `IdleBehavior::Coast`，零速会绕开舵向闭环。最小修改是改为现有 `Hold`，同时 B 删除“没在 moving 就撤销驱动许可”的全局状态。

```cpp
if command is explicit Disabled:
    existing_stop()                         // 左 Down 的停控语义不变
else if wheel_speed is within existing zero-speed hysteresis:
    steer_target = last_optimized_steer_target
    drive_target = 0
    run_steer_pid(steer_target)
    run_drive_velocity_pid(0)               // Hold 下零速闭环，不积累运动目标
else:
    steer_target, drive_target = optimize_180deg(new_target)
    run_steer_pid(steer_target)             // 使用已有舵角速率，不加对齐等待
    run_drive_velocity_pid(drive_target * cos_error_cubed)
```

注意 `last_angle_rad_` 是运动学角，模块可能有 180° 翻转；零速时沿用当前 `flipped_`，得到上次实际使用的舵目标，不能把原始角与优化后角混用导致停车转半圈。当前 `prepare` 只有非零速时更新 `flipped_`，可以沿用，不新增一套零速状态机。

<a id="d"></a>

## D. 底盘行走与大 Yaw 解耦

涉及 113；关联 084、085、086、114 保持明确边界。入口：底盘主循环（[本地文件](../../applications/sentry_chassis/src/main.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/applications/sentry_chassis/src/main.cpp#L145)）。

直接删除 `if (input.command.mode == Disabled) big_input.request.mode = Disabled` 以及仅服务它的 `BigYawGate::ChassisDisabled`。大 Yaw 使用自己的请求模式、有效性、硬件状态；底盘使用自己的运动命令。共同的人工停机仍由发送端分别发出两个 Disabled，不从“底盘执行失败”推导。

```cpp
chassis_input.command = rx.control.chassis_command
chassis_input.transport_ready = chassis_hardware_ready && rx.online
big_input.request = rx.big_yaw_request
big_input.transport_ready = big_yaw_hardware_ready && rx.online

chassis.update(chassis_input)
big_yaw.update(big_input)
// 不读取另一个执行器的 ready/active/error/mode 来覆盖自己。
```

不要把上例的名称误做新运行标志：`*_hardware_ready` 指现有初始化结果组合，不新增“确认一次才能动”。001—004 暂保留。共享电源真不可用、链路真离线会分别影响两条链，属于同一输入缺失，不是机构故障连坐。

**尚未全部解耦的地方如实保留**：你把 054—060、161 标成“后面改”，云台侧 `big_channel_ready` 仍依赖小 Yaw 执行、航向和反馈；本轮删 113 不等于这些上游门也消失。后续单独重构这一段，不能宣传“大 Yaw 已完全独立”。

底盘平移方向目前用大 Yaw 编码器定义云台朝向，因此“电机不能输出”和“编码器没数据”要分开：大 Yaw 执行器故障但编码器仍可读时，平移转换照常；编码器也失效时没有可靠方向，不能偷换成另一坐标系继续。086 零位需实测；本轮不捏造新的零点。可在以后明确选择车体系直接驾驶作为独立控制模式，不能暗中自动切换。

<a id="e"></a>

## E. 软件限位管目标，不把越界反馈当作轴失效

涉及 038、041、174。入口：feedbackError（[本地文件](../../lib/robotics/gimbal_axis.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/gimbal_axis.cpp#L37)）；GimbalAxis::update（[本地文件](../../lib/robotics/gimbal_axis.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/gimbal_axis.cpp#L117)）。

删除 `feedbackError` 中“实际角度在软件范围外 → -ERANGE”。仍保留角度有数据、参考成立、值有限这三项；实际越限记录到现有 telemetry，不撤销输出。

```cpp
feedbackError(snapshot):
    check_required_feedback_and_reference()
    check_angle_is_finite()
    return OK                 // 不拿实际角度与软件 min/max 比较来拒绝

update(command):
    measured = current_valid_feedback
    if controller_really_needs_initialization:
        target = clamp(measured, software_min, software_max)
    target = update_target_using_existing_rate_or_absolute_mode(command)
    target = clamp(target, software_min, software_max)
    position_loop(target, measured)
```

效果：人工把轴掰到软件上限与硬限位之间，启用控制时目标仍在软件范围内，闭环可向内回到边界；不会因为越界撤权。有限的反馈不能先 clamp 成边界，否则会掩盖真实位置、使回位误差消失。到边界以外继续给向外命令只会把目标夹在边界，向内命令可以继续改变目标。**主动控制且零速保持时，也可能回拉到软件边界**，这是本方案的明确语义。

041 的“跟不上就丢目标”需要分开定位：小 Yaw/Pitch 的 Rate 目标会在正常有效周期积分并夹机械范围；`max_lead_rad` 当前为 0，没有启用目标领先裁剪。大 Yaw Native 模式发送的是**速度目标**，不存在同样的绝对角目标。停跟随更可能来自上游 disabled、反馈判断、超周期 suspend 或恢复清历史；仅凭这段源码不能断言某一项就是实机原因。本轮删除 B/D/F 的相关打断，保留普通机械目标限幅与必要速率约束，不以“保留目标”为由跨真实参考重置沿用旧坐标。

<a id="f"></a>

## F. 超周期只记录并跳过本次计算，不 disable/re-enable

涉及 035、093、119、158，保留 130 的 Motor 指令期限。入口：GimbalExecutor 周期判断（[本地文件](../../lib/robotics/gimbal_executor.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/gimbal_executor.cpp#L74)）；BigYawExecutor 周期判断（[本地文件](../../lib/robotics/big_yaw_executor.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/big_yaw_executor.cpp#L117)）；ChassisExecutor 周期判断（[本地文件](../../lib/robotics/chassis_executor.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/chassis_executor.cpp#L178)）。

当前 Gimbal 和 Native BigYaw 在 `!cycle` 时 suspend，导致 disable、目标丢失和使能代次变化；第一周期没有 previous time 也走这条路。建议把周期处理与真正的停止输入分开。

```cpp
update(input, now):
    // 明确停止、当前输入失效等原有终止条件先处理；不能被 dt 分支吞掉。
    handle_explicit_stop_and_input_lifetime()

    if no_previous_time:
        previous_time = now
        request_enable_using_existing_async_path()
        return waiting_for_first_interval      // 没有错误、没有新的解锁条件

    dt = now - previous_time
    previous_time = now
    if dt not in existing_controller_valid_interval:
        diagnostic.cycle_error_count++
        diagnostic.last_dt = dt
        publish_rate_limited_error()
        return without_staging_new_output     // 不 suspend，不 withdraw，不 reset group

    calculate_and_stage_with_this_dt()
```

不向 PID 传 dt=0 再让 PID 清历史；底盘也在进入 `chassis.step` 前跳过不合法周期。下一有效周期直接按现有状态计算，不能要求连续正常 N 次。

这里保留一个现实边界：**跳过更新不会保证电机永远保持输出。** 没有新输出超过原来保留的 20ms，Motor 本身会停止沿用旧命令；不能为了“不断”偷偷刷新旧命令的时间戳。删除的是 Executor 主动 disable/re-enable 和授权撤销，不是用户已保留的命令寿命。若驱动已经真实进入 Fault/新代次，就遵守原有恢复过程。

158 是调度跳过已经错过的截止点，避免落后后忙着补跑历史循环；它本身不禁止控制，保留。日志只说明 `dt_us`、预期周期、发生次数，不能新增必须 clear 的故障锁存。035 要报错，这里满足；157 的历史粘性诊断照常只用于观察。

<a id="g"></a>

## G. 合并重复检查，未用字段不参与判定

涉及 020、032、033、077、091、115、118、129、170、171。入口：仲裁配置校验（[本地文件](../../lib/robotics/command.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/command.cpp#L139)）；Gimbal 输入校验（[本地文件](../../lib/robotics/gimbal_executor.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/gimbal_executor.cpp#L62)）；轴输入校验（[本地文件](../../lib/robotics/gimbal_axis.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/gimbal_axis.cpp#L117)）；大 Yaw 时效（[本地文件](../../lib/robotics/big_yaw_executor.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/big_yaw_executor.cpp#L75)）。

### G1. 只验证当前模式实际使用的字段

```cpp
validate_axis_command(mode, target, rate):
    switch mode:
      Disabled: return OK
      Hold: return OK
      Rate: return finite(rate)
      AbsoluteAngle: return finite(target)
      default: return invalid_mode

// 共享 mode 不合法时拒绝这一条消息；数值按 yaw/pitch 各自判断。
for axis in [yaw, pitch]:
    if validate_axis_command(mode, axis.target, axis.rate):
        axis.update(command)
    else:
        report_this_axis_input_error()
        skip_this_axis_new_output()   // 不 suspend 整组，也不刷新旧输出的时间戳
```

Executor 和 GimbalAxis 必须一起修改：只改上层，轴里仍无条件验证 `target_rad`，Rate 模式照样会失败。绝对角模式无需验证不用的速率字段。动态坏值丢弃本条对应轴输入，不把其他轴一起 suspend；显式 Disabled 独立执行。

配置同理：纯手动路径不应因为视觉或射击参数不合法而启动失败。静态未编译功能不校验；实际启用的模块在配置装配处检查自己的参数。对于原来一份大 Config 混有所有功能的情况，按现有启用字段分支校验即可，不新建完整的“功能能力注册器”。

### G2. 时间戳不是全部重复，但每种来源只检查一次

| 时间 | 真正含义 | 建议 |
| --- | --- | --- |
| source stamp / source_age | 遥控器或其他原始生产者上次真正更新 | 保留一次；防止循环重复发送旧输入却被当成新输入 |
| command stamp / command_age | 仲裁/业务命令上次生成 | 与 source 独立才有价值；相同来源同一阈值合并 |
| permission age | 大 Yaw 另一个授权有效期 | 对本次由同一操作模式生成的冗余授权，不再增加独立停控期限 |
| 电机 command 20ms | 控制输出多久没有更新 | 用户保留，和遥控 100ms 不是同一个量 |
| 电机反馈 20/50ms | 当前测量是否还可用于控制 | 与命令年龄不同，不按“都叫超时”一刀删 |

底盘正式 main 的 `source_stamp` 直接从 `input.command.stamp` 换单位得到，所以 091 的两次判断在这条路径重复：正式底盘输入只保留 command 新鲜度，删由同一 stamp 派生的 source 检查/字段传递；库若供真正独立来源使用，就在那个入口明确检查，不保留虚假的双重来源。

Gimbal 的 `sourceStamp(frame, command.source)` 可能是原始 RC，而 command stamp 是仲裁生成时刻，不是天然重复；可以在进入执行器前生成一个既有命令有效期限 `deadline=min(source_deadline, command_deadline)`，执行器只比较一次，也可以保留现有两字段并写清语义。**本轮建议不为了减少 if 引入新消息结构**，只删已证实同源的重复判断。

BigYaw 的 source/command 两个预算目前均为 100ms，可就地计算 `effective_age = max(source_age, command_age) + receiver_elapsed` 后比较一次；相加用宽整数或饱和运算。权限字段不是裁判权限时，删除执行器对它的独立 300ms 租约，以 `request.mode` 表达本条请求的启停；发送端产生 Disabled 时按现有协议撤销当前请求，不续发缓存启用态。暂不改变线上 payload，保留解析兼容字段并不意味着继续参与执行许可。

同一修改要覆盖 Endpoint 发送请求（[本地文件](../../lib/communication/interboard_endpoint.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/communication/interboard_endpoint.cpp#L242)） 的 `!request.permission.valid/!request.permission.enabled → Disabled` 分支，否则底层删了上游仍拦。当前 main 用同一 `big_channel_ready` 同时生成 mode 和 permission；保留 mode 作为唯一操作意图。接收 Endpoint 的 source 新鲜度仍会在公开快照时过滤，Executor 在稍后使用时检查实际年龄；这两个时刻可能不同，合并的是同一时刻同源预算，不要求消费过的缓存永不过期。Codec 的字段尺寸/布尔合法性校验保留。

对“同源权限”的删除只适用于当前正式 bigYaw 操作授权：先沿发送端检查赋值来源；不得把真正的人工停止当成过期许可直接忽略。Boot 和对端会话字段本轮另见 L，暂不整体删除。

### G3. 死区和角速度转换

020：`RcControlAdapter::normalize` 的 3% 死区与 `ManualCommandMapper` 的死区是两个可见实现，但正式路径传递原始 `RemoteState`，不能据此声称它必定串联两次。选择 mapper 负责归一化和模拟量死区；样例要复用同一函数，不能先 normalize 再送 mapper。解锁归中阈值 ±33 是手势判定，和操纵手感死区分开命名，不互相充当运行门。

115：世界角速度到关节角速度，基本关系是 `joint_rate = desired_world_rate - body_rate`；原代码还加 `.25 * (desired_world_rate - head_rate)` 做反馈修正。轮径用于 `线速度 = 轮轴角速度 × 半径`，解决的不是这件事。保留必要坐标转换，把 `.25` 移到统一配置；删转换函数里硬编码 `±10`，由大 Yaw 当前选定后端的一处速率上限处理，再按减速比和方向变成电机轴量。别把“不满足世界变换条件”改成偷偷把世界速率当关节速率。

<a id="h"></a>

## H. 遥控上电归中可直接进入对应模式

涉及 013—018、020。入口：RcControlAdapter::update（[本地文件](../../samples/robotics/common/rc_controls.hpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/samples/robotics/common/rc_controls.hpp#L95)）。

### H1. 启动快捷路径

你的要求解释为：首次接到有效 RC 帧，摇杆/轮归中、右拨 Middle，就不必先 Down 保持 500ms 再 Down→Middle→Up。可以直接在 Up 进入手动，Middle 进入导航选择；Down 仍是停止，不能把“已经归中”解释成忽略 Down。

```cpp
on_first_valid_rc_frame:
    if analogs_centered && right == Middle:
        startup_neutral_available = true

while startup_neutral_available:
    if !analogs_centered || right != Middle:
        startup_neutral_available = false
    else if left != Down:
        armed = true
        startup_neutral_available = false
        emit_existing_start_event_once()

if left == Down:
    stop_now()
    neutral_ready = analogs_centered && right == Middle  // 无 500ms 等待
else if not armed:
    use_existing_Down_to_Middle_transition(neutral_ready)

derive_manual_or_navigation_from_left_switch()
use_existing_right_middle_before_spin_or_fire()
```

`startup_neutral_available` 只是一遍上电快捷分支，可替换现有 `safe_baseline_` 等无必要重复状态；不能在每次失联恢复时重新当成刚上电。进入过活动模式后的 Down 停止与重新拨档仍按 017 的既有流程，暂不改成无限自动复活。首次就是非归中输入时仍走原拨档流程。删除 `neutral_since_/neutral_timing_/neutral_arm_ms` 及只显示 500ms 的诊断，减少旧状态；“归中即 ready”不新增另一段等待。启动时停在 Down 只禁止当前动作，不把启动归中资格擦掉；一旦真正失联或出现非归中输入则取消这次快捷资格，不能在后续恢复时重建。调整 `withdraw()` 清理范围时把这种启动资格与运行 armed 区分，避免照搬旧清零代码使快捷路径失效。

### H2. 时间倒退

设备本地 64 位单调 uptime 正常不会因为校时倒退。看到倒退，通常应检查不同来源时钟混用、缓存旧样本、新旧会话混杂、测试注入或错误截断；不能把它写成正常运动经常发生的条件。

014 建议：旧序号/倒退时间只丢弃这份候选 RC 帧，记录诊断，不 `withdraw()`；保留上一份已接受帧及其原始时间戳，由原 100ms 新鲜度决定是否仍可用。**需把 `state_.remote = remote`、`last_sequence_` 的提交移到帧检查通过之后**，避免先污染状态再说“忽略”。旧候选帧被丢弃后，新鲜度应检查“已接受的最后一帧”，不能仍用刚拒绝的旧候选帧触发顶部的失联 withdraw；真正接收器 offline 或最后有效帧超过100ms仍执行原失联处理。收到有效左 Down 应优先按用户停止语义处理；新旧会话的整体恢复本轮不重写。

### H3. 程控模式忽略 RC 离线：作为后续接入要求

当前正式应用仍是手动来源，不能只加一个 bool 就宣布导航/自瞄可离线运行。建议统一配置 `autonomous_requires_rc = false`，但仅在已有且明确选中的 Autonomous 来源真实存在时适用：

```cpp
if selected_source == Manual:
    use_existing_rc_freshness()
else if selected_source == Autonomous:
    use_autonomous_source_own_stamp()
    if autonomous_requires_rc:
        also_require_rc_freshness()
// 收到明确人工停止，仍停止；单纯 RC 没包不伪造成新的人工停止。
```

真正接入时要同时处理 `RcControlAdapter.withdraw()`、`CommandManager` 外部门、仲裁器 RC 前提及发送端 run_allowed 的连锁，否则常量没有效果。013 是该后续功能的设计要求，**本轮不伪造导航来源，也不重写你在 028 暂缓的整个仲裁模块**。

<a id="i"></a>

## I. 功率数据沿用最后有效值；裁判缺数据默认允许

涉及 095、096、153、173；029 尚未定，现有未启用状态不改变。

入口：ChassisExecutor::power（[本地文件](../../lib/robotics/chassis_executor.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/chassis_executor.cpp#L133)）；CommandArbiter 裁判许可（[本地文件](../../lib/robotics/command.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/robotics/command.cpp#L223)）；RefereeParser（[本地文件](../../lib/communication/referee.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/communication/referee.cpp#L70)）；RefereePowerState（[本地文件](../../include/robotics/messages/referee.hpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/include/robotics/messages/referee.hpp#L4)）。

### I1. 预算超时不再变成全车零输出

当前 `power()` 对预算、limit_stamp、实测功率任一不可用返回 `-ESTALE`，调用方把两种 scale 置 0 并清全部模块 output_valid。删这条全局清零路径。预算上限与实测功率是不同数据，分开缓存，不能用状态帧给功率测量续期。

```cpp
// 已有对象内的少量缓存，不新建服务。
last_valid_limit = none
last_valid_scale = 1.0

power(inputs, dt):
    if received_new_valid_limit(inputs.limit, inputs.limit_stamp):
        last_valid_limit = inputs.limit

    if last_valid_limit does not exist:
        return scale(1.0)                // 从未获得预算，不缩放

    if measurement_is_fresh_and_valid(inputs.measurement):
        // buffer 与 limit 分别按各自 stamp 处理；旧 buffer 不能假装实时。
        if buffer_available_for_current_algorithm:
            result = existing_limiter(measured, last_valid_limit, buffer, dt)
            if result succeeded and finite(result.scale):
                last_valid_scale = clamp(result.scale, 0, 1)
        // 算法缺必需输入或失败，沿用最后一次算出的比例。

    return scale(last_valid_scale)
```

预算旧了仍用最后合法预算；测量也旧了就沿用最后有效缩放比例，初始比例为 1；不拿陈旧功率继续积分，不按旧缓冲能量虚构新状态。已有功率模型估计若已配置可继续用，不能为了填缺数据自动启用未标定估计。

缓存仅在新样本合法时更新，解析错误不覆盖；不刷新原时间戳。显式用户停机仍停，但不应顺带把预算缓存清空到“重新等待裁判才可动”。当前 formal profile 未启用 require_power_budget，本轮只改该可选路径的行为，不自动启用功率模型。

### I2. 裁判许可只影响选定动作

```cpp
bool referee_allows(permission, now):
    if !permission.valid || !isFresh(permission.stamp, now, existing_timeout):
        return true
    return permission.enabled

motion = requested_motion
if use_referee_for_chassis_motion && !referee_allows(ref.chassis_output, now):
    motion.vx = motion.vy = motion.wz = 0
    // 不设全局 Disabled、不撤销 RC，不连带大 Yaw；舵角仍可保持。

// 云台默认不接裁判门；如以后明确启用，只影响云台自身。
```

配置使用几个现有 profile 的 constexpr 动作开关即可；本轮沿用未接入状态，未来若要限制移动，明确开启 `use_referee_for_chassis_motion`。不要写一套通用权限矩阵。当前有效且明确禁止的裁判位，按选定动作限制；缺失/过期按你要求允许，不保留“最后一次 false 永久锁住”。

CRC 和帧长度错误仍丢帧，不能“默认允许”就接受垃圾字节。默认允许发生在业务读取层，不篡改 `RefereeState` 使其看起来 online/valid。2026/2027 已通过 Kconfig 选择解析器，本轮不动赛季协议格式。

把 `require_permission` 设 false 只能关掉某一个执行器的重复门；若上游仲裁已把请求改 Disabled，底层放行也无效。因此实现时统一更改实际启用的读取点，防止一层默认允许、另一层继续 fail-closed。发射机构、热量、自瞄许可按 149/154 延期，不顺带扩展。

<a id="j"></a>

## J. 原始 gyro 独立发布，不等待 EKF 成功

涉及 044—052 的说明、167 的局部修改；053—060 航向重构仍延期。

入口：Bmi088Imu::service（[本地文件](../../drivers/imu/bmi088_imu.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/drivers/imu/bmi088_imu.cpp#L33)）；ImuState::publish（[本地文件](../../drivers/imu/imu_state.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/drivers/imu/imu_state.cpp#L23)）；云台 EKF 配置（[本地文件](../../applications/sentry_gimbal/src/main.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/applications/sentry_gimbal/src/main.cpp#L66)）。

EKF 用来估计姿态和陀螺零偏；重力可约束 roll/pitch，不能单靠加速度得到绝对 yaw。800 是启动估计使用的样本数量，不是硬件必须产出 800 帧才能读 gyro。实际等待时长取决于被接纳样本的速率和静止条件，不能直接按线程 500us 推成精确 0.4 秒。

当前 `correct_gyro_bias=true` 时，gyro 只有 EKF 本次 `update()==0` 才加入发布；初始化、拒绝某份 accel 或 EKF 暂时没更新时，即便 gyro 读取成功也不发布。建议拆开：

```cpp
if gyro_read_succeeded:
    gyro = raw_gyro
    if correct_gyro_bias && estimator_has_established_finite_bias:
        gyro -= estimator.last_established_bias()
    publish_gyro(rotate_sensor_to_body(gyro), original_gyro_stamp)

if accel_read_succeeded:
    publish_accel(rotated_accel, original_accel_stamp)

if accel_and_gyro_available_and_existing_pairing_rule_satisfied:
    result = estimator.update(...)
    if result == success:
        publish_orientation_and_quality()
    // EAGAIN 或姿态不可用只影响姿态，不阻止前面的 gyro。
```

“已建立 bias”复用估计器已有初始化/质量信息；不新增一个 gyro_armed 或等待 N 次稳定的许可。初始化前用原始 gyro；已经建立后允许在短暂未更新期间用最后 bias。估计器真实 reset 时按它已有生命周期回到原始 gyro。

实现可采用分次 `ImuState::publish`：有效 gyro/accel 单独发布，姿态独立发布。这样一个非法 quaternion 不把同批有效 gyro 一并拒绝；不需要把 `ImuState` 重写成容错框架。169 的帧/参考一致性仍保留，禁止把不同 epoch 的姿态与新 gyro 伪造成一个共同姿态样本。

**范围限制**：这只消除发布耦合。上层 `ImuHeadingReference` 还要姿态、参考、质量，单改这里不会自动解除 049—060。不得把原始 `gyro.z` 无条件标成世界 yaw rate；有俯仰时两者不等价。完整航向连续恢复见 L，保持延期。

<a id="k"></a>

## K. 心跳频率、解析与诊断

涉及 061—082、135—145、156—160、175、180。

入口：发送调度（[本地文件](../../lib/communication/interboard_endpoint.cpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/lib/communication/interboard_endpoint.cpp#L300)）；Endpoint 配置（[本地文件](../../include/communication/interboard/interboard_endpoint.hpp) · [基线源码](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dc258ec0b9dda91524c2b35f0061f2fb2a4ab356/include/communication/interboard/interboard_endpoint.hpp#L15)）。

目前板间计划周期：心跳 **20ms（50Hz）**，ChassisControl / BigYawRequest / BigYawFeedback **10ms（100Hz）**，GimbalIMU **20ms（50Hz）**；心跳在线超时 **100ms**。这是计划上限，UART 忙或调度延迟会使实际发送更低。

069/072 要降低频率，建议先把心跳周期集中成 `heartbeat_period_ms = 50`（20Hz），在线超时设 `heartbeat_timeout_ms = 200`。这是拟议参数，不是已测最优值；降低了心跳流量，也把只靠心跳判离线的最坏延迟放宽到约 200ms，运动命令自己的 100ms 超时保持。200ms 不是“必须收到四帧才能上线”，首个有效心跳就按原逻辑建立在线状态。其他业务帧周期不一起降。

发送代码按消息种类读取周期，替换 `i==1||i==4||i==5 ? 10 : 20`，避免调心跳误调其他消息。只增加参数来源，不新增排队策略。

其余通信用现有实现：CRC/长度/角色/序号检查、UART 分片、TX busy 延后、超时 abort/retry、CAN BusOff 恢复都保留。解析器 4 帧队列若被一次性灌入很多完整帧且不及时 pop，可能覆盖；正式 Endpoint 的消费方式通常不会积压到这种情况，不能仅凭“4”就认定必须扩容。保留统计，未见溢出不重写。

“硬件正常不会出现”不能推出 UART/CAN API 永远返回成功；负载、调度、接收队列处理不及时也可能导致忙碌/溢出。175 用户已说保留，不替换成另一种整总线恢复。180 是非阻塞读取抢锁失败沿用旧快照，旧时间戳不变；无需加等待锁或人为续期。

<a id="l"></a>

## L. Boot、source、generation：解释及延期方案

涉及 019、022—028、033、073—085、117、120、124—128、131、135—138、151、177。

### L1. 三个容易混淆的概念

| 名称 | 本项目里的含义 | 举例 |
| --- | --- | --- |
| Boot ID | 一块主控这一次启动的会话编号 | 云台不断电，底盘重启后 Boot ID 换了；它不是许可证、密码或编译版本 |
| source | 命令原始生产者/输入来源及其时间 | 一份 RC 帧被主循环重复读了 50 次，source 时间仍是原来那次接收 |
| generation / revision | 本地状态代数，区分旧计算与恢复后的当前对象 | PID 算到一半电机被 disable 再 enable，旧结果不能覆盖新状态；参考重新定零后旧角度不能当成新坐标 |

同一 Boot 的序号才有可比性；重启后 seq 从小值重新开始，若完全不分会话，可能把新包永远当旧包，或把重启前旧包当本次新命令。receiver_boot 表示“发给接收板这一次启动”，sender_boot 表示“由发送板哪一次启动产生”。它不是额外人机解锁本身；**收到新会话之后清旧帧**与**要求用户重新拨杆**是两种不同策略，当前代码把它们部分绑定了。

### L2. 哪些冗余，哪些先别动

- 019/084：目前对端重启或离线会撤销 RC / suspend 两个执行器。你尚未决定 Boot 相关项，本轮保留；后续可改为“清旧会话缓存，接收新会话的新命令自然恢复”，不加人工解锁。不能只删比较而留下旧 seq 基线。
- 074：退休 Boot 列表的容量约束会使多次重启变成另一种永久拒绝点。后续建议只持有当前已建立会话，切换时清缓存/序号基线；不要再累计永久拒绝名单。如何识别旧心跳重放需结合现有会话握手一次处理，而不是本轮悄悄替换协议。
- 022/023：RC adapter 的运行态与 CommandManager 外部门是重复层，但边界/旧队列处理还防止解锁前帧被用作解锁后新手势。你 028 说仲裁暂缓，因此不把它们全拆；未来统一到一个操作态来源，并清一次旧队列，不再两套 armed。
- 024/180：无新帧、抢锁失败不续期本来就正确；“暂时读不到”不立即等于断线。
- 127/131：异步 CAN 中计算、提交、完成回调可能跨 disable/recovery；代次用来丢弃旧结果，属于并发一致性。先保留底层一处，不继续往业务层传播成为解锁条件。
- 177：64 位计数每秒加一千次也需约 5.8 亿年才耗尽，不值得增加运行停机路径。建议仅对确认为 64 位、纯相等比较的本地计数去掉耗尽锁死，按 unsigned 自然推进；0 有哨兵含义则统一跳过 0。有限位宽线帧序号保留现有回绕比较，不能机械套用。

### L3. IMU 航向重构留待下一轮

053—060、161 是明确延期。建议后续目标是删 `blocked_` 永久锁存，区分“当前角速度可用”和“世界角参考需要重建”，只让依赖世界角的功能等待；普通手动、车体平移不等双 IMU。参考变化时重新基准化连续角，而不是加更多次数/时间门槛。先确定世界角连续性语义，再写重构，不在本轮凭猜测修改标定与零点。

```cpp
// 后续草案，不纳入本轮执行。
on_heading_sample(sample):
    publish_available_rate_with_its_real_stamp()
    if orientation_reference_changed:
        rebase_angle_reference_using_current_valid_sample()
        discard_only_old_reference_dependent_state()
    if current_orientation_unavailable:
        mark_this_orientation_sample_unavailable()
    // 没有永久 blocked_，也没有 clear/unlock 命令。
```

必须处理“旧目标在新参考中怎么表达”，不能只把 `blocked_` 改 false 而保留旧绝对角。双 IMU 151 与 MIT 152 仍未启用，不为了这一轮研究而激活。

<a id="m"></a>

## M. 用户疑问的具体发生条件与处理边界

| 条目 | 回答 | 本轮处置 |
| --- | --- | --- |
| 008、009、168 | 线程创建成功不等于传感器初始化成功；SPI ready、设备 ID/配置写入等可能失败。接收器只 init 一次，BMI088 init 失败后 service 并不能自动补齐初始化；RS485 与 BMI 的恢复路径不同 | 说明现状，暂保留初始化结构；不要为此给所有设备加无限 begin 重试。以后做 BMI 专用可重入初始化时再改 |
| 012 | 队列过满表示鼠标边沿可能丢了，清的是手势会话；不等于所有 RC 控制都需停止 | 按批注保留，诊断文字可直说“丢帧导致点击序列不完整” |
| 032、129、171 | 绝对角/速率来自运行时计算或外部输入，不可能全靠编译器判定；Rate 模式还查 unused target 则确实多余 | 静态配置前移；动态字段只校验实际使用项，坏值不连坐 |
| 034、092 | main 会记录初始化错误并继续跑诊断循环，所以可能进入 Executor；并非 setup 失败后整个程序已经退出 | 001—004 暂保留，不另造启动门；固定配置错误提前编译失败 |
| 039、135 | DJI 一圈编码器断流期间若跨圈，累计圈数可能无法可靠恢复；reseeding 是恢复坐标参考，不是让轴重新请求许可 | 保留真正坐标恢复；现象可能是停控直到参考恢复，不能保证只抖一下就继续 |
| 040、102、120、145 | 真正 disable/enable 或参考变更后，积分、旧位置目标和新反馈不一定同一基准 | 保留局部必要重置；不要把零速/cos=0/旋转切换当新代次 |
| 043 | 手动载体速率不可用时补偿取 0，意味着只少一项补偿，不再拒绝手动命令 | 保留，不扩成 IMU 许可 |
| 045、047 | accel/gyro 是不同读取，可能有 SPI 错误、时序偏移、调度长间隙；旧时间常见于缓存/源混用，不应说本地单调钟正常会倒退 | 原始数据与姿态发布解耦；滤波算法本体暂不删检查 |
| 048—052 | 运动加速度并非重力，EKF 可暂时拒绝该重力观测继续预测；姿态奇异点/无有效四元数不能通过“允许”变成可计算角度 | 不把数学异常升级为整机停机；航向层完整修改延期 |
| 067、070、071、077 | UART 错误恢复、旧 seq 或源年龄过大不只由断线产生，也可能是旧缓存、发送源停更、线程延迟 | 保留经典丢帧/重试，重复年龄按 G 合并 |
| 089、151、161 | 正式底盘走 Body 配置，普通平移用大 Yaw 编码器旋转；不要求双 IMU 配对。Body IMU 仍用于大 Yaw 角速度补偿 | 不误删补偿，不把未启用配对写成目前平移必经门 |
| 091、093、099、104 | 缓存旧输入、主循环长间隙、运行时无效值或 PID 参数/状态出错都可能发生；不全是接线问题 | 重复 source 判断删；数值问题只局部不提交，不撤整车授权 |
| 112 | `LegacyCandidate/Reference...Candidate` 是编译期零点/方向配置选择，本身不是运行权限 | 改名字为明确标定集即可；不要自动来回切换，实际零位要测 |
| 116、121、122 | 模式/请求范围在 BigYawExecutor；native 速度有 4 rad/s² 斜坡；setVelocity 还可能因运行状态而失败 | 后端静态选；斜坡保留并集中参数；非法有限超幅统一限幅，错误命令类型不能当成功 |
| 123、124、134 | 请求 enable 之后还需驱动应答才能 Active；没反馈时控制器没有当前状态。底层 20/50ms 期限与上层命令 100ms 用途不同 | 原有驱动状态与反馈期限先保留，不新增外层同类门 |
| 136—138 | 不是说 CAN 线必然乱序，而是恢复前后的缓存、回调和 DM 状态帧可能属于不同阶段；驱动还可能报告自身 Fault | 保留当前协议流程，不因“没遇到”全部删掉 |
| 142、143 | BusOff、提交忙和恢复是运行时 CAN 机制；正常使用没遇到不证明永远不发生 | 保留，别在业务层重复拉全车停止 |
| 144、164 | PID 饱和、滤波、斜坡和死区属于控制器数值行为，删掉可能直接改变整定效果 | 参数集中，重复上限合并，不当作许可门批量删除 |
| 155 | emergency_stop 是显式请求，不是设备状态自动投票；clear_estop 也当 suspend 容易让调用语义混乱 | 先删除正式路径里未使用的独立 clear_estop 输入口和传播；保留明确 Stop 语义及尚在使用的样例接口，不添加另一种解锁手势 |
| 159、036 | `Active` 只要求任一轴有效是状态汇总，不表示全部成功，也不是准入 | 保留局部输出；诊断显示 active_count/member_count，不反向拿 Active 作执行条件 |
| 162 | 加热器功率相同，在散热、环境改变时稳态温度不同；限最大功率不能等价于知道当前温度 | 当前未启用，延期，不把缺温度输入改成持续加热，也不新增热控功能 |
| 172—174 | 当前额外逐轴许可、独立裁判门、max_lead 未启用；不是正式路径每次都在触发 | 裁判默认允许按 I；其余保持关闭，不添加新使用点 |
| 176 | 静态数量/拓扑可编译期发现；运行时 attach 顺序、实体 owner 冲突仍是运行事实 | 静态部分前移，动态绑定入口只检查一次 |
| 178、179 | 替代航向跟随器和替代 CAN/RS485 板间传输当前不在正式 UART 路径 | 本轮不激活、不重构 |

<a id="n"></a>

## N. 本地实施交接与完成定义

按 P1→P2→P3 分成几个有意义的提交；先完成代码再进行仓库允许的最终编译。以下是交付行为说明，不要求额外新增自动测试或新的运行检查：

| 场景 | 预期行为 |
| --- | --- |
| 四轮只有一个舵角落后 | 该轮 cos³ 缩放，其余轮不因最大误差被一起置零 |
| 一轮反馈异常 | 依赖该反馈的本轮输出不可算；其他模块继续；不撤 RC |
| 零速后重新推摇杆 | 零速维持原优化舵角，恢复即追新角，没有全轮对齐许可 |
| 底盘运动命令 Disabled，大 Yaw 仍有独立有效请求 | 大 Yaw 正常执行自己的请求 |
| 软件限位外、反馈有效 | 能向软件范围内回位，不撤该轴授权 |
| 偶发周期超限 | 记录原因，不主动 disable/re-enable；旧输出不续命，沿用原 20ms 期限 |
| 初次有效 RC 归中、右 Middle、左 Up | 可直接手动；左 Down 仍立即停止 |
| 裁判数据缺失/过期 | 业务默认允许；有效明确禁止仅限制配置选中的动作 |
| 功率预算旧但曾有效 | 沿用预算；缺测量沿用最后比例；从未有效默认比例 1 |
| EKF 未产出本次姿态但 gyro 读取成功 | gyro 按自己时间戳发布；不宣称世界航向已恢复 |
| 固定配置非法 | 编译期给出具体字段错误；不是开机后静默 ENODEV/ERANGE |

本轮不应新增：全车 ready 汇总后才能动、健康连续计数、冷却等待、人为清故障手势、许可续租线程、自动扩大的故障传播范围。上述正常场景如果又需要一个新的“允许”位才能通过，就没有达到本轮目标。

### 可直接交给本地执行者的提示

> 以当前 dev 和《SkyWalker_门控精简修改方案_按dev批注.md》为依据，用户要求减少门控与故障连坐。仅实现文中 P1—P3 和附录标为“本轮”的项目，保留/延期项不要顺手改。重点删除四轮全局 drive_permitted/对齐阈值/旋转切换撤权、底盘 Disabled 连带禁用大 Yaw、零速 Coast、超周期主动 suspend、重复电流上限和未用字段检查；固定配置前移编译期；裁判缺数据默认允许、旧功率预算不再清全部输出；gyro 发布独立于 EKF。不得新增许可状态机、N 帧稳定门槛、解锁手势或故障连坐。本文伪代码不是现成 API，按现有类最小改动实现。严格区分 source 时间与输出时间，旧缓存不得刷新时间戳。Boot/航向大改、导航来源、射击/热控保持延期。按仓库工作流先完成实现；没有额外测试授权，不写测试或扩大验证，最终至多编译一次并报告结果。

## 附录：180 项批注逐项处置

下面保留你的原始回复，编号与原表一致。主文提出的建议没有被冒充成你已确认的决定。“本轮”是推荐交给本地执行者的范围；“解释/保留”“延期”不应被自动实施为删除。

| 编号 | 项目 | 你的原始批注 | 建议处置 | 修改点/说明 |
| --- | --- | --- | --- | --- |
| 001 | 主控资源确认总开关；正式-云台 | 想不到怎么改，暂时保留，但想删了太屎了 | 保留 | [A](#a)：主控资源确认按原话暂留；只前移其中确定的常量校验，不整段删除 setup_ready。 |
| 002 | 三路CAN与电源确认；正式-底盘 | 想不到怎么改，暂时保留，但想删了太屎了 | 保留 | [A](#a)：三路 CAN 与电源初始化暂留；不把其错误扩散到不相关独立机构。 |
| 003 | 双轴 begin 共用初始化门槛；正式-云台 | 又是初始化，同上 | 保留 | [A](#a)：双轴 begin 共用初始化暂留，不在本轮顺手改对象生命周期。 |
| 004 | 底盘硬件八电机拓扑确认；正式-底盘 | 初始化同上 | 保留 | [A](#a)：八电机装配暂留；固定方向/数量/ID 可按 A 静态断言，硬件事实仍在初始化检查。 |
| 005 | Motor/CAN注册期模式与ID检查；正式-双板 | 电机config是否合法不能放编译器断言一下吗，直接不让编译不要留到运行期间报错退出 | 本轮 | [A](#a)：固定电机参数、模式组合、同总线 ID 冲突前移编译期；移除周期内重复常量检查。 |
| 006 | 物理CAN独占与设备ready；正式-双板 | 保留 | 保留 | [K](#k)：物理 CAN 单 owner 和 device ready 保留；不是许可续租。 |
| 007 | 原生速度/MIT策略互斥；正式-大Yaw | 大yaw的速度模式和mit用个编译期常量切换不就行了，好屎，写这么长判断是给人读的吗 | 本轮 | [A](#a)：用一个 constexpr 后端枚举替换 native/MIT 多布尔组合，当前选择 NativeVelocity。 |
| 008 | IMU线程启动与传感器初始化分离；正式-云台 | hyw为啥要这么设计 | 解释/保留 | [M](#m)：线程 start 与设备 init 是两个动作；说明失败传播，不重建通用初始化状态机。 |
| 009 | 一次性初始化不可自动重新begin；正式-双板 | ？为什么要重新begin | 解释/保留 | [M](#m)：begin 是注册/装配，不是正常运行周期；不能把故障恢复理解为每次重新 begin。 |
| 010 | DR16解码格式及通道范围；正式-云台 | 应该不用改 | 保留 | [K](#k)：DR16 帧格式/通道范围保留，非法帧不进入控制层。 |
| 011 | DR16半帧10ms间隙清空；正式-云台 | 应该是正确的吧，如果用dma是不是就要改？先不动 | 保留 | [K](#k)：半帧间隙逻辑先不动；未来 DMA 调整接收边界时再处理。 |
| 012 | DR16帧队列溢出清鼠标手势会话；正式-云台 | 帧列队应该没问题，至少用下来暂时没发现问题，如果故障发生的描述太抽象没看懂 | 保留 | [M](#m)：队列溢出可能丢鼠标边沿，清的是手势序列；保留并改善说明。 |
| 013 | 遥控时间戳新鲜度；正式-云台 | 没问题，但最好在程控模式下不影响（导航和自瞄模式），最终上场万一遥控器离线需要确保不会进入安全模式锁死，加个常量开关就行 | 后续接入 | [H](#h)：设计 autonomous_requires_rc=false，但当前没真实导航来源，不把手动旧 RC 永久保活。 |
| 014 | 旧序号与时间倒退处理；正式-云台 | 时间倒退基本不可能吧，要防这个干嘛，告诉我啥时候会出现，否则删了或者忽略不要直接撤销 | 本轮 | [H](#h)：旧序号/时间倒退丢候选帧而不 withdraw；检查通过后才更新已接受快照和序号。 |
| 015 | 左Down人工停车；正式-云台 | 调试时是正常情况不用改，后续可能要修改 | 保留 | [H](#h)：左 Down 明确停机保留；默认允许不能覆盖人工停机。 |
| 016 | 上电安全档静止500ms基线；正式-云台 | 可以改成如果启动时遥控器没有动作指令时可以直接解锁不等500ms下再手动上吗，因为遥控器拨杆都拨到无动作的位置时也不会发生什么直接解锁允许控制也没关系吧 | 本轮 | [H](#h)：上电首个有效归中输入允许直入对应档；删 500ms 基线等待，仍尊重 Down。 |
| 017 | Down→Middle明确解锁边沿；正式-云台 | 暂时影响不大，但为了上场稳定后续需要更改 | 保留 | [H](#h)：启动快捷分支之外，用户主动停机后的 Down→Middle 语义暂留，后续再改。 |
| 018 | 右拨Middle之后才允许小陀螺；正式-云台 | 不用改应该就是这样 | 保留 | [H](#h)：右拨先回 Middle 再触发旋转/射击保留，不扩大成额外全车门。 |
| 019 | 对端失联或Boot ID变化强制撤销RC授权；正式-云台 | 同一boot又是啥玩意，没看懂 | 解释/延期 | [L](#l)：Boot 是对端本次启动号；会话切换与人工再解锁可分开，但本轮不替用户决定整体协议。 |
| 020 | 遥控量化死区；正式-云台 | 意思是这个链路有两个死区置0逻辑吗，能不能改掉 | 本轮整理 | [G](#g)：正式链路不确定串联了两次死区；统一由 mapper 归一化，样例复用同一实现。 |
| 021 | 注册来源及后台启动完整性；正式-云台 | 这些应该算是配置问题吧，配置问题全部在编译器断言好，后面的也一样 | 本轮局部 | [A](#a)：静态来源类型/配置前移；运行线程启动返回值仍需实际处理，不重写注册器。 |
| 022 | 外部Operator Gate二次授权；正式-云台 | 没看懂，怎么又门控上了 | 解释/延期 | [L](#l)：外部 Operator Gate 与 RC armed 有重复；你暂停仲裁重构，本轮不拆完整链。 |
| 023 | 授权变更边界/接收队列老帧；正式-云台 | ？ | 解释/延期 | [L](#l)：边界用于隔离解锁前旧手势帧；将来与单一操作态合并，不能直接删除后仍消费旧队列。 |
| 024 | 采样-EAGAIN缓存不续期；正式-云台 | ？ | 保留 | [L](#l)：EAGAIN 是暂无新快照，沿用缓存不续期符合预期。 |
| 025 | 命令仲裁真实遥控100ms窗口；正式-云台 | 这个门控到时候再整个完整的说明吧 | 解释/保留 | [G](#g)：100ms 针对原始 RC，不是每次循环生成的新命令；保留其作用说明。 |
| 026 | 仲裁配置、时钟/无效映射拒绝；正式-云台 | 和之前一样配置问题 | 本轮局部 | [A](#a)：只迁移静态配置校验；仲裁整体时钟/模式结构留待后续。 |
| 027 | Safe与不可用Auto模式拒绝；正式-云台 | ？怎么又有撤销 | 解释/保留 | [H](#h)：Safe 是明确停机；Auto 没来源不能把手动旧目标当自动执行，不新增锁存。 |
| 028 | 首次快照未发布/读取失败；正式-云台 | 算了这个模块后面不看了本来是仲裁用的复杂点是应该的，后续再慢慢看 | 延期 | [L](#l)：按原话暂停整个仲裁模块重构；只做独立明确的小改如 unused 配置和裁判默认值。 |
| 029 | 裁判输出许可门控；未启用-裁判授权 | 未定 | 未定/保留关闭 | [I](#i)：尚未启用的仲裁裁判门不自动打开；153/173 的缺数据默认允许作为未来接入语义。 |
| 030 | 自动瞄准新帧、参考与手动夺权；未启用-视觉 | 未定 | 未定/保留关闭 | [M](#m)：视觉目标/参考和手动夺权保持现状，不借本轮启用或重构。 |
| 031 | 瞄准未控禁止发射；未启用-发射 | 未定 | 未定/保留关闭 | [M](#m)：射击对准条件未定，保持当前未启用状态。 |
| 032 | GimbalExecutor模式/数值校验；正式-云台 | 何时会出现这种情况？ | 本轮局部 | [G](#g)：按模式校验实际使用的动态字段，单轴无效不连坐其他轴；静态枚举配置前移。 |
| 033 | GimbalExecutor命令及来源双重时效；正式-云台 | source又是啥玩意？ | 解释/合并重复 | [G](#g)：source 是原始生产者；与 command 独立时保留语义，相同 stamp 不双重检查。 |
| 034 | GimbalExecutor通信/可用性门槛；正式-云台 | setup都炸了还能跑到这边吗 | 解释/保留 | [M](#m)：初始化失败后 main 仍可跑诊断，确会进 Executor；本轮不删初始化结果传播。 |
| 035 | GimbalExecutor控制周期超限；正式-云台 | 周期超限一般不就是写的有问题吗，需要给出报错 | 本轮 | [F](#f)：超周期打印受限频率错误及 dt/count；不主动整组 suspend，不新增清错要求。 |
| 036 | 双轴任一有效即可报告Active；正式-云台 | 应该没问题 | 保留 | [M](#m)：任一轴有效报告 Active 是汇总口径，显示轴数，不用其结果作另一个授权。 |
| 037 | 本轴反馈和机械参考可信检查；正式-云台 | 貌似也没问题，但是看上去好像有点冗余？ | 保留/不重复 | [E](#e)：反馈存在、有限、坐标参考有效仍有意义；删除的软件范围拒绝见038。 |
| 038 | 反馈已越软件限位时整轴拒绝；正式-云台 | 越软件限位不应直接撤销，因为有可能是手动掰到软件限位到硬件限位之间，这种情况干嘛要撤授权停控？ | 本轮 | [E](#e)：越软件限位不撤轴；保持真实反馈，目标限在范围内允许向内回位。 |
| 039 | 失效反馈后自动reseeding；正式-云台 | 没看懂 | 解释/保留 | [M](#m)：reseed 恢复多圈坐标参考，非重新发许可证；不和普通目标更新混合。 |
| 040 | 恢复代次变化清积分目标；正式-云台 | 这个是不是就是该这么写 | 保留 | [M](#m)：真实使能/参考代次变化后重置局部控制历史，不因零速/对齐/旋转变化触发。 |
| 041 | 云台目标速率与机械范围截断；正式-云台 | 这个应该是有问题的，大yaw跟不上的时候好像直接丢目标了 | 本轮局部/待定位 | [E](#e)：保留正常目标积分与机械范围；删相关周期打断，不能凭感觉归因；大 Yaw Native 只有速度目标。 |
| 042 | PID时间窗与反馈要求；正式-云台 | 应该是正常的写法 | 保留 | [F](#f)：PID 正常 dt/反馈要求保留；上层无效周期不再传 dt=0 引发清历史。 |
| 043 | 手动载体角速度缺失置零补偿；正式-云台 | 没看懂喵 | 解释/保留 | [M](#m)：缺载体补偿用0不会额外拒绝手动，保持此行为。 |
| 044 | 板载IMU硬件与EKF初始化校验；正式-云台 | 加ekf的作用是啥 | 解释/局部优化 | [J](#j)：EKF 是姿态/偏置估计；原始 gyro 不等它，配置前移，算法本体不重写。 |
| 045 | BMI088双传感器采样失败或跨传感器偏移；正式-双板 | ？这个是啥意思呢，应该不会出现吧 | 解释/保留 | [M](#m)：读取错误和双采样偏移可能发生；不把一份姿态失败连坐有效 gyro。 |
| 046 | EKF初始化需要静止800样本；正式-云台 | ？为啥要先有800个样本 | 解释/延期调参 | [J](#j)：800 是初始化样本设置，不是硬件要求；本轮不凭空换样本数，先消除原始数据等待。 |
| 047 | EKF拒绝倒序/过密样本，长间隙及数值失败reset；正式-双板 | ？ | 解释/保留 | [J](#j)：旧样本/过密/长间隙是滤波时间语义；不直接删除滤波数值条件。 |
| 048 | EKF重力观测门控/Degraded；正式-双板 | 所以为啥会有这个玩意 | 解释/保留 | [J](#j)：运动加速度不能全当重力；Degraded 可继续预测，勿升级为整机禁止。 |
| 049 | Degraded最多100ms补偿；正式-云台 | ？ | 延期 | [L](#l)：Degraded 补偿时限随航向层一起改，原始 gyro 可先独立发布。 |
| 050 | 航向Running/姿态/陀螺50ms门槛；正式-云台 | ？ | 延期 | [L](#l)：Running/姿态/gyro 组合门需按世界角/速率用途拆分，本轮不改航向定义。 |
| 051 | IMU数学奇异点和旋转速率极限；正式-云台 | ？ | 解释/保留 | [J](#j)：奇异姿态和非有限计算不能用默认允许解决；缺哪种结果只影响其消费者。 |
| 052 | 航向静止零点门槛；正式-云台 | ？ | 延期 | [L](#l)：静止航向零点条件随参考语义重构，不在本轮增删静止样本数。 |
| 053 | 航向参考切换锁存；正式-云台 | 这部分感觉还要大改 | 延期 | [L](#l)：建议未来删永久 blocked_ 并重新基准化角度；按用户“还要大改”单独做。 |
| 054 | 航向连续采样丢失500ms锁存；正式-云台 | 后面改吧 | 延期 | [L](#l)：明确后面改，保留500ms航向采样间隙锁存待统一重构。 |
| 055 | 应用对航向二次20ms门槛；正式-云台 | 后面改吧 | 延期 | [L](#l)：应用20ms航向门本轮不动，后续合并重复时效。 |
| 056 | 大Yaw许可绑定小Yaw执行及多项上游条件；正式-云台 | 后面改吧 | 延期 | [D](#d)：云台侧大 Yaw 对小 Yaw/IMU 的依赖仍在；本轮删113不宣称全链独立。 |
| 057 | 大Yaw启动零速等待远端反馈；正式-云台 | 后面改吧 | 延期 | [L](#l)：等待远端反馈的启动零速按原话后续处理。 |
| 058 | 远端反馈速率250ms/数值/上限检查；正式-云台 | 后面改吧 | 延期 | [L](#l)：远端反馈250ms/数值/范围按原话后续处理，不先调阈值。 |
| 059 | 回中控制自身前提与周期检查；正式-云台 | 后面改吧 | 延期 | [L](#l)：回中控制前提/周期检查在后续航向重构范围。 |
| 060 | 回中死区/迟滞/速率斜坡；正式-云台 | 后面改吧 | 延期 | [L](#l)：回中死区/斜坡原话后面改，本轮不重新整定。 |
| 061 | v4帧头、版本、角色检查；正式-双板 | 应该是正常的 | 保留 | [K](#k)：帧头、版本、角色保留，不放行无法解码的包。 |
| 062 | payload长度/CRC16/ID检查；正式-双板 | 应该没问题 | 保留 | [K](#k)：payload长度、CRC16、消息ID按现状保留。 |
| 063 | 分帧拼装超时和串口断片；正式-双板 | 应该问题不大 | 保留 | [K](#k)：分帧与串口断片逻辑保留。 |
| 064 | 通信读取与TX一次轮询预算；正式-双板 | ？好像没问题吧，经典实现就行 | 保留 | [K](#k)：有界轮询预算防止通信线程占满周期；不增加额外运行许可。 |
| 065 | UART接收队列世代隔离；正式-双板 | 是经典实现就行 | 保留 | [K](#k)：UART接收世代隔离保留，防旧缓冲内容混入新接收。 |
| 066 | UART TX忙碌与超时abort；正式-双板 | 这个是啥情况，是经典实现就行 | 解释/保留 | [K](#k)：TX busy延期、超时abort属于异步传输流程，保留。 |
| 067 | UART RX自动100ms重试；正式-双板 | 一般应该不会出错自己关吧 | 解释/保留 | [M](#m)：UART恢复可由驱动错误触发；按现有100ms重试，不新增后台多级恢复。 |
| 068 | 帧级消息类型及payload语义拒绝；正式-双板 | 应该是对的 | 保留 | [K](#k)：不符合消息语义的payload丢弃。 |
| 069 | 同源角色与无心跳业务帧拒绝；正式-双板 | 心跳帧现在的频率大概多少，应该可以再调低点吧 | 本轮参数 | [K](#k)：心跳现20ms/50Hz；建议50ms/20Hz，配合200ms在线期限；同源角色语义不删。 |
| 070 | 线帧seq必须单调；正式-双板 | 理论上应该不会出现异常 | 保留 | [K](#k)：线帧seq用于去旧包，保留回绕规则；异常不能变成全车锁存。 |
| 071 | 业务生产者seq额外单调；正式-双板 | 理论上也不会发生吧，发生就丢弃？应该是经典做法吧 | 保留 | [K](#k)：业务生产者seq避免重复旧样本变新数据，按现有方式丢弃。 |
| 072 | 心跳100ms在线判定；正式-双板 | 可以把频率调低点吧 | 本轮参数 | [K](#k)：配合降低心跳频率调在线期限；命令100ms另算，不把心跳200ms当命令续命。 |
| 073 | Boot ID切换清历史命令；正式-双板 | ？这玩意是啥 | 解释/延期 | [L](#l)：切Boot清历史是区分重启会话；不要把清缓存和强制人工解锁当同一件事。 |
| 074 | 旧Boot ID退休列表锁定；正式-双板 | 这又是啥 | 解释/延期 | [L](#l)：退休列表满后拒新会话确实可能过度；后续收敛会话策略，不本轮偷改协议。 |
| 075 | 新对端会话基线禁止旧目标复用；正式-双板 | ？为啥要分这种，是不是只影响调试，还有正常情况下只重启其中一块板的时候为了保证安全需要重新解锁之类的 | 解释/延期 | [L](#l)：单板重启后不能沿用旧seq/目标；可做到新会话新命令恢复，不必天然人工解锁。 |
| 076 | 板间平移命令新鲜度与已禁用显式发送；正式-双板 | boot到底是啥 | 解释/保留 | [L](#l)：帧的Boot号指启动会话；显式Disabled与命令时间仍保留。 |
| 077 | 大Yaw源年龄+命令年龄双重校验；正式-双板 | 理论上不会出现，接线正常就没事 | 本轮合并 | [G](#g)：source/command同100ms预算可计算max年龄一次；源停更并非只有接线错误。 |
| 078 | 大Yaw许可年龄有效性；正式-双板 | 应该不会吧，怎么又有许可 | 本轮局部 | [G](#g)：当前操作许可由同一run状态产生，去独立租约与重复停控；人工Disabled仍生效。 |
| 079 | 0x0404方向/会话/采样50ms检查；正式-双板 | 为啥这个的boot还要匹配 | 解释/延期 | [L](#l)：IMU信息也属于对应启动与坐标参考；不与其他会话姿态混用，本轮保留。 |
| 080 | 0x0404样本序号/零位代次不倒退；正式-双板 | 这个又是啥 | 解释/延期 | [L](#l)：样本序号与零点代次分别区分新样本/新坐标；按航向重构再简化。 |
| 081 | 大Yaw反馈发送/接收两侧100ms，上层再250ms；正式-双板 | 应该没问题不用改 | 保留 | [K](#k)：按用户不用改保留反馈发送/接收寿命；应用250ms归延期航向范围。 |
| 082 | 发送端忙/批量长度不足延后不同消息；正式-双板 | 是经典实现就行 | 保留 | [K](#k)：发送忙时延后不同消息是正常异步调度，不新增队列系统。 |
| 083 | 0x0301合法性/来源会话检验；未启用-操作授权帧 | 所以这个到底是啥 | 解释/保留关闭 | [L](#l)：0x0301是独立操作授权帧，正式路径未启用，不让它变新必经条件。 |
| 084 | 对端启动号变化立即suspend两个执行器；正式-底盘 | 所以这个到底是什么，后续我就不填有关boot id的项目了后面再看 | 解释/延期 | [L](#l)：对端boot变化目前同时suspend；会话恢复整体另议，本轮按用户暂缓。 |
| 085 | 底盘控制global_action、online与receiver boot三条件；正式-底盘 | 未定 | 未定/保留 | [L](#l)：global_action/online/receiver_boot保留现状；不伪称删除113就等于删除这些共同条件。 |
| 086 | 大Yaw原生角决定平移坐标变换；正式-底盘 | 现在零位不准需要重新校准 | 需实测 | [D](#d)：统一零位配置入口，实际值由车上测量；不凭旧参考工程猜新零点。 |
| 087 | 大Yaw原生零位与传动比参数有效性；正式-底盘 | 不合法参数请在编译期报错 | 本轮 | [A](#a)：零位/方向/传动比常量静态断言，转换周期不反复验证这些常量。 |
| 088 | 小陀螺边平移倍率切换；正式-底盘 | 这些倍率统一写到配置文件里，不符合阈值就在编译期报错 | 本轮 | [A](#a)：旋转平移倍率集中，明确范围和单位，编译期断言。 |
| 089 | 正式Body模式绕开双IMU配对门；范围说明 | 啥玩意儿 | 解释/保留 | [M](#m)：Body模式绕开双IMU配对是当前路径事实，不额外补配对门。 |
| 090 | ChassisMotionAdapter输入与配置有效性；正式-底盘 | 配置不合法，同上 | 本轮局部 | [A](#a)：配置有效性静态处理；实时输入有限值仍在最近入口处理。 |
| 091 | 底盘执行器来源stamp与本地命令双超时；正式-底盘 | 什么情况下就坏成这样啊 | 本轮 | [G](#g)：正式底盘source_stamp从command直接复制，删这条路径的重复时效。 |
| 092 | 底盘通信准备/电源/执行初始化否决；正式-底盘 | 什么情况下会出现这种神秘的报错 | 解释/保留 | [M](#m)：诊断主循环可在初始化失败后继续；初始化结果保留，独立机构不连坐。 |
| 093 | 底盘控制周期异常退化；正式-底盘 | 什么情况下会出现这种神秘的状况 | 本轮 | [F](#f)：无效周期在进入舵轮PID前跳过，不用dt=0逼下层重置。 |
| 094 | 非标定功率控制下电流限制；正式-底盘 | 应该是正常现象 | 本轮合并 | [A](#a)：保留目前有效电流值，把非标定分支的重复上限合到唯一配置。 |
| 095 | 裁判功率预算/实测反馈失效零输出；未启用-功率模型 | 怎么预算过期就不允许输出了？照着之前有效继续呗，之前没有有效就默认不缩放不就行了 | 本轮 | [I](#i)：旧预算沿用最后有效；无预算scale=1；无测量沿用上次scale，不清所有输出。 |
| 096 | 底盘Executor独立require_permission；未启用-裁判额外许可 | 裁判系统的许可可以加上但是可以限制一下只限制某些行为吗，比如只限制移动指令的执行 | 本轮可选路径 | [I](#i)：裁判仅限制配置指定动作；缺数据允许；不因裁判平移禁止关掉大Yaw或舵向保持。 |
| 097 | 四轮速度死区和启停迟滞；正式-底盘 | 应该是正常现象，保留配置的地方就行 | 保留/集中参数 | [C](#c)：速度死区/启停迟滞保留，零速按Hold处理。 |
| 098 | 轮速上限整体缩放；正式-底盘 | 应该没问题但是不要在各个地方散一堆配置的地方，集中到一处 | 本轮整理 | [A](#a)：四轮整体比例缩放保留，上限统一来源，去互相取min的多处调参入口。 |
| 099 | 轮目标NaN/越界直接失败；正式-底盘 | 尝试忽略吗？直接停可能不太好吧，要么就是算法有问题才能出现这种情况 | 本轮局部 | [B](#b)：非法求解不重置全车；本周期不生成新输出且不续期旧缓存，下一有效输入自然继续。 |
| 100 | 舵向角目标180度翻转迟滞；正式-底盘 | 是经典实现就行 | 保留 | [C](#c)：180度最短转向翻转及迟滞保留，停车不改变已选翻转状态。 |
| 101 | 舵向新鲜反馈/周期/Coast检查；正式-底盘 | 应该是正常的 | 本轮局部 | [C](#c)：反馈/数值计算要求保留，正式零速Coast改Hold。 |
| 102 | 舵向使能或参考代次变化初始化；正式-底盘 | 应该也是正常的吧 | 保留 | [M](#m)：真实使能/参考变化局部重初始化保留，不能触发全轮等待。 |
| 103 | 舵向目标角速率限制；正式-底盘 | 应该没问题 | 保留/集中参数 | [A](#a)：舵向目标速率限幅保留，参数唯一来源，不新增对齐许可。 |
| 104 | 舵向PID计算失败清历史；正式-底盘 | 理论上不会出现 | 本轮局部 | [B](#b)：PID失败只影响本模块，不全车清许可；下一正常周期按原局部恢复。 |
| 105 | 全四模块health集体放行；正式-底盘 | 正常情况下都会看出来的不用管 | 本轮删除 | [B](#b)：删除四模块healthy集体放行。 |
| 106 | 四舵向全局对齐准入/退出；正式-底盘 | 这个许可不是有那个cosine的三次方函数来限制了吗为什么要再加一个 | 本轮删除 | [B](#b)：删除全局0.10/0.40对齐准入退出，仅保留每轮cos³。 |
| 107 | 旋转状态切换打断全局驱动许可；正式-底盘 | 那为啥又要重新许可呢怎么又有这个限制 | 本轮删除 | [B](#b)：删除旋转状态切换撤销drive_permitted。 |
| 108 | 任一轮错误清全局驱动；正式-底盘 | 正常情况下会看出来是车出问题了不需要在这边加门控 | 本轮删除 | [B](#b)：删除任意轮错误后四轮finishDrive(false)的连坐。 |
| 109 | cos³随舵误差压低轮速；正式-底盘 | 这个是正常的呀 | 保留 | [B](#b)：cos³作为连续目标比例，比例0不视为需重新授权。 |
| 110 | 轮驱PID失去反馈/周期失效；正式-底盘 | 理论上不会出现吧 | 保留/局部化 | [B](#b)：没本轮反馈不能算本轮PID；只该轮无新输出，不传播全车。 |
| 111 | 单模块drive_permitted附加要求；正式-底盘 | 怎么又在清许可，为什么整个的逻辑不应该是默认有许可，除非出问题了再加限制吗 | 本轮删除 | [B](#b)：删单模块外部permitted参数/许可状态，保留计算所需反馈，不在cos=0时清PID。 |
| 112 | 方向与零点候选切换；正式-底盘 | 这个又是什么玩意儿怎么这个还要有门槛 | 解释/整理 | [M](#m)：只是编译期标定集选择，不是运行门；明确命名，实际零位仍要测量。 |
| 113 | 底盘行走Disabled连带撤销大Yaw；正式-底盘 | 底盘的门控和大yaw解耦，要么就都给我删了，不允许其中一个出故障的时候另外一个也不允许让他动 | 本轮删除 | [D](#d)：删底盘Disabled覆盖BigYawRequest.mode，两个执行器独立消费各自请求。 |
| 114 | 双陀螺世界转关节前提；正式-底盘 | 这个应该不会失效吧 | 解释/延期 | [G](#g)：世界速率转换确实要有效body/head测量；本轮不把缺参考当可用，后续拆IMU依赖。 |
| 115 | 速度补偿二级限幅；正式-底盘 | 为什么世界速度和车体速度要有补偿不能直接配置轮的直径直接算出到底有多少的速度吗 | 本轮合并 | [G](#g)：保留角速度坐标转换，删硬编码二次限幅、集中反馈增益；轮径不替代角速度转换。 |
| 116 | BigYawExecutor禁用/数值/命令模式验证；正式-底盘 | 所以这个限制又是在哪里写的 | 本轮局部 | [A](#a)：模式编译期选择，实时值按使用字段判定并统一限幅，不增加授权。 |
| 117 | 请求Boot绑定/peerOnline/transport；正式-底盘 | ？ | 解释/延期 | [L](#l)：boot/online/transport属于会话及设备可用性，待用户决定会话简化。 |
| 118 | 源、业务请求与权限三层时效；正式-底盘 | 三层时效你是想造火箭吗 | 本轮合并 | [G](#g)：合并源/命令同阈值年龄，删除同源操作许可独立寿命；Motor20ms另保留。 |
| 119 | BigYaw控制周期越界硬停；正式-底盘 | 理论上不会出现，出现了就是硬件有问题需要修，不需要你兜底 | 本轮 | [F](#f)：Native大Yaw超周期不主动suspend；记录并跳过一次输出，下周期自动继续。 |
| 120 | 原生速度反馈恢复分界；正式-底盘 | ？ | 解释/保留 | [M](#m)：真实驱动恢复是新反馈基准；局部斜坡起点重置保留，普通周期抖动不制造恢复。 |
| 121 | 大Yaw直接速度斜坡每周期限幅；正式-底盘 | 怎么还有限制 | 解释/集中参数 | [A](#a)：当前原生速度4rad/s²斜坡是加速度整形；先保留数值移配置，不改成解锁门。 |
| 122 | Native速度setter拒绝导致整轴suspend；正式-底盘 | 所以为什么会出现这种情况，不应该直接在编译期发现吗 | 本轮局部 | [A](#a)：后端/类型/配置错编译期发现；运行setter仍可能状态错误，有限超幅统一限幅不整轴反复suspend。 |
| 123 | Motor反馈时间窗独立于Executor；正式-双板 | 理论上不会出现，出现了也不需要你兜底 | 解释/保留 | [M](#m)：反馈超时不只硬件坏，也可能调度/总线堵塞；本轮保留底层期限，避免扩散到别的机构。 |
| 124 | 使能是异步请求，不直接等于Active；正式-双板 | ？ | 解释/保留 | [M](#m)：enable是异步设备命令，等应答Active不是人为许可；不新增第二层等待。 |
| 125 | disable清命令与切断代次；正式-双板 | 应该不用改 | 保留 | [L](#l)：明确disable后丢旧输出与旧代次保留。 |
| 126 | Motor指令来源独占绑定；正式-双板 | 重复绑定不应该直接在编译的时候报错吗 | 本轮静态部分 | [A](#a)：静态装配单一控制者可前移；通用运行时指针绑定保留一次必要检查，不模板化重写。 |
| 127 | Motor计算输出使能代次版本一致；正式-双板 | 可能确实应该如此吧 | 保留 | [L](#l)：计算与提交之间可能切代，保留底层一致性判断。 |
| 128 | Motor计算输出可被invalidate；正式-双板 | 可能也许是合理的 | 保留 | [L](#l)：invalidate只丢该轴待发输出，别据此撤操作态。 |
| 129 | Motor命令类型、有限值、幅度验证；正式-双板 | 超限不应该是在编译期就能发现的问题吗怎么还能在运行的时候报错呢 | 本轮合并 | [A](#a)：常量范围静态检查；动态结果不能静态断言，最终一处有限值/类型处理和裁剪。 |
| 130 | Motor命令仅20ms有效；正式-双板 | 确实应该如此 | 保留 | [F](#f)：Motor命令20ms有效保留；任何“忽略异常”不能借刷新旧stamp绕过它。 |
| 131 | CAN发送前再验证版本/权限/新鲜度；正式-双板 | 所以代次又是个什么鬼 | 解释/保留 | [L](#l)：CAN提交前版本检查避免异步旧结果覆盖新状态；不继续往业务层复制代次门。 |
| 132 | DJI电机缺目标自动发零槽位；正式-双板 | 是正常的 | 保留 | [K](#k)：DJI无目标的槽位发0是正常帧编码。 |
| 133 | DM控制失效发送中性帧；正式-大Yaw/云台Pitch | 理论上没有问题 | 保留 | [K](#k)：DM无有效输出按现有模式发中性帧。 |
| 134 | 电机反馈20/50ms过期Fault；正式-双板 | 应该没有影响 | 保留 | [M](#m)：20/50ms反馈超时保留，先不改变驱动故障模型。 |
| 135 | DJI连续编码器断流无效参考；正式-双板 | 真的会出现吗，出现的会是什么现象，是停一下继续转吗 | 解释/保留 | [M](#m)：断流可能丢多圈信息，不能保证停一下自愈；保留坐标参考处理。 |
| 136 | DM原生多圈恢复与先后时序校验；正式-DM | 怎么可能会出乱序 | 解释/保留 | [L](#l)：可能是恢复前后缓存/回调跨阶段，不仅线上的乱序；保留DM原生位置时序。 |
| 137 | DM驱动状态禁止运行或上报Fault；正式-DM | ？ | 解释/保留 | [M](#m)：DM自身状态禁止或Fault与人为门不同，保留驱动处理。 |
| 138 | DM enable/clear fault超时与100ms重试；正式-DM | ？ | 解释/保留 | [M](#m)：enable/clear-fault确认及重试是驱动协议；不删除也不叠加业务重试层。 |
| 139 | CAN RX帧型/长度/ID/解码校验；正式-双板 | 应该对的吧 | 保留 | [K](#k)：CAN帧型/长度/ID检查保留。 |
| 140 | CAN发送回调超时及控制器BusOff；正式-双板 | 应该是正常的，是经典实现就行 | 保留 | [K](#k)：发送回调超时、BusOff按现有总线恢复处理。 |
| 141 | 总线Recovering传播给所有Motor；正式-双板 | 应该没问题 | 保留 | [K](#k)：同一物理总线不可用会影响其电机，不能当跨机构人为连坐误删。 |
| 142 | CAN控制器停启与自动重试；正式-双板 | 真会出现吗 | 解释/保留 | [M](#m)：总线错误可触发控制器重启，不需为未发生过另设新流程。 |
| 143 | CAN提交线程忙与协议消息优先级；正式-双板 | 一直用的好的真没发现出现过 | 保留 | [M](#m)：线程忙与协议消息优先级按现有异步实现，不重写。 |
| 144 | Position/Velocity PID内部反馈与dt准入；正式-双板 | ？ | 解释/保留 | [M](#m)：PID必需反馈和dt保留；无效周期由F提前跳过，避免层层重置。 |
| 145 | 控制器代次重置首周期只初始化；正式-双板 | 是经典实现就行 | 保留 | [M](#m)：真正新代次首周期初始化保留，不因零速或切旋转人为换代。 |
| 146 | VisionLink源格式/时间戳与参考验证；未启用-视觉 | 应该没有问题 | 保留关闭 | [M](#m)：视觉格式/参考处理保持，当前未启用。 |
| 147 | Shooter执行器命令周期、许可与急停；未启用-射击 | 是不是有点太冗余了 | 延期 | [M](#m)：射击尚未启用，重复许可待射击整体适配时处理，不自动放开所有发射条件。 |
| 148 | 摩擦轮反馈转速与稳定时间；未启用-射击 | 理论上不会出现除非车坏了 | 保留关闭 | [M](#m)：摩擦轮到速判断属于射击功能要求，不因当前没故障就删。 |
| 149 | 枪口准星、剩余热量、拨盘回零、卡弹锁；未启用-射击 | 先不管热量这种东西后面再说 | 延期 | [M](#m)：按原话暂不动热量/回零/卡弹。 |
| 150 | 外置头部IMU惯性云台适配器；未启用-惯性云台 | 后续再慢慢适配调整 | 延期 | [M](#m)：外置头部IMU适配等后续调整，不在本轮引入。 |
| 151 | 非Body模式的双IMU启动配对与锁存；未启用-IMU同步 | ？ | 解释/保留关闭 | [L](#l)：非Body双IMU配对不在正式普通平移路径，保留关闭。 |
| 152 | 大Yaw MIT头部IMU角度锁与扭矩PI；未启用-MIT航向锁 | 目前不是m i t模式所以不用管 | 保留关闭 | [A](#a)：MIT当前不用，后端选择清楚即可，内部控制不调。 |
| 153 | 裁判帧CRC、超时与权限数据源；未启用-Referee | 不满足的时候就不要撤销就当它是有的 | 本轮 | [I](#i)：CRC错误仍丢；业务层对缺失/过期权限默认允许，不把原数据伪造为有效。 |
| 154 | 鼠标点击手势与释放检测；未启用-键鼠射击 | 后续再慢慢调和测试 | 延期 | [M](#m)：鼠标手势与释放按原话后续调试，不扩展修改。 |
| 155 | 执行器emergency_stop/clear_estop可选参数；未启用-额外安全API | 这个又是何意味，冗余就删 | 本轮清理未用部分 | [M](#m)：清除正式路径无用clear_estop传播，保留明确Stop及仍用到的样例接口，不新增手势。 |
| 156 | 状态灯与诊断理由优先级映射；仅观测 | 什么时候再优化一下吧 | 延期 | [K](#k)：状态灯优先级后续优化；本轮只补必要报错，不重建诊断系统。 |
| 157 | 短暂失效的粘性诊断记录；仅观测 | 就这样 | 保留 | [F](#f)：粘性诊断只记历史，不能参与新运行许可。 |
| 158 | 主循环过期周期跳步；调度行为 | ？ | 解释/保留 | [F](#f)：错过周期后跳到未来截止点是调度行为，保留，实际超周期另记录。 |
| 159 | Ready/Active只要求任意轴output_valid；正式-底盘 | 啥意思？ | 解释/保留 | [M](#m)：Active=有任意轴有效是状态汇总，保持局部输出，展示轴数减少误读。 |
| 160 | 解析器4帧队列满覆盖旧帧（需区分调用方式）；解析器能力/正式路径通常不积压 | 应该不会出啥大问题吧，因为发送端也不会发太快，真一起发应该也不至于解析太慢不及时pop挤掉帧吧，保证解析是经典实现就行 | 保留 | [K](#k)：4帧队列是否满取决于调用方式；正式路径通常及时pop，保留现实现与统计。 |
| 161 | Body IMU仍起线程但不限制常规平移；范围说明 | 后面改吧 | 延期/现状保留 | [L](#l)：Body IMU不阻普通平移的现状保留；大Yaw补偿相关后面改。 |
| 162 | IMU加热器温度无效降功率；未启用-热控 | 什么情况下会发生呢，限制一下最大功率不就行了吗，真有问题烧了会发现的 | 延期/保留关闭 | [M](#m)：最大功率不等于最高温度；未启用热控先不改，不新增持续加热路径。 |
| 163 | 手动命令最大速度配置；正式-输入限幅 | 参数在统一的配置文件里面统一配置 | 本轮 | [A](#a)：手动线速度/角速度最大值集中配置，单位明确，profile只装配。 |
| 164 | 云台PID死区、输出饱和、速度斜坡与滤波；正式-控制限幅 | ？ | 解释/集中参数 | [M](#m)：PID死区/滤波/斜坡不是权限；参数统一，保留当前整定。 |
| 165 | 舵轮电流在profile、PID、Executor、Motor重复受限；正式-多层限流 | 不要有多层上限，给我删了或者统一在某个地方 | 本轮 | [A](#a)：profile/PID/Executor/Motor同一电流来源；删Executor第二套bench裁剪，保留同限值抗饱和。 |
| 166 | 舵轮零速Coast而非保持舵角；正式-零速策略 | 零速时就保持之前的舵角啊很难理解吗，恢复运动就按恢复运动时他请求舵角转不就好了吗 | 本轮 | [C](#c)：正式IdleBehavior改Hold，零速保留优化舵角，恢复追新目标，不等全轮对齐。 |
| 167 | 云台校正gyro绑定EKF成功更新；正式-IMU发布依赖 | ？为什么传感器e kf还能阻断执行 | 本轮局部 | [J](#j)：成功读gyro即发布；EKF只影响姿态/可用bias，不发布成功不再连坐gyro。 |
| 168 | BMI088异步初始化失败后不重新init；正式-初始化恢复 | 真的会初始化失败吗 | 解释/延期 | [M](#m)：BMI异步init确会失败且目前不重试；不在本轮加通用重试状态机。 |
| 169 | ImuState入库校验与整批拒绝；正式-数据发布校验 | 还是老问题会出现吗 | 本轮局部 | [J](#j)：gyro/accel与姿态分次发布，非法姿态不拖掉有效gyro；入库数学/参考校验保留。 |
| 170 | 手动模式仍验证视觉/鼠标/发射配置；正式-未用参数也校验 | 为什么不直接丢弃非法参数呢 | 本轮 | [G](#g)：未启用功能不验证其闲置参数；实际用到的静态配置编译期报错，动态输入只丢对应输入。 |
| 171 | Rate模式仍要求绝对角目标字段有限；正式-未用字段也校验 | ？ | 本轮 | [G](#g)：Rate只查rate，Absolute只查target；Executor与Axis同步改，否则下层仍拦。 |
| 172 | yaw_output_valid/pitch_output_valid附加门；未启用-云台逐轴许可 | ？ | 解释/保留关闭 | [M](#m)：额外逐轴许可当前未用，不新增其使用点，后续可随API收敛。 |
| 173 | GimbalExecutor独立require_permission；未启用-云台额外许可 | 是裁判系统来的许可就默认为允许 | 本轮可选路径 | [I](#i)：云台裁判权限缺失默认允许；有效禁用是否作用由本机构配置决定，不开新的总门。 |
| 174 | max_lead_rad限制目标领先反馈角；未启用-目标领先限制 | ？ | 保留关闭 | [E](#e)：max_lead当前0；没有据此证明丢目标，本轮不启用、不另加领先门。 |
| 175 | CAN接收队列溢出升级总线Recovery；正式-CAN整总线恢复 | ？真的会出现吗，真出现了就说明电机接线有问题，保留吧 | 保留 | [K](#k)：按用户保留CAN队列溢出恢复；原因可能是线/负载/调度，不仅接线。 |
| 176 | CAN注册数量/启动阶段/Owner容量限制；正式-资源容量校验 | ？ | 本轮静态部分 | [A](#a)：固定数量、ID与容量前移编译期，动态注册时序/owner仍在单一入口检查。 |
| 177 | 64位代次/命令revision/回调顺序耗尽拒绝；正式-极端计数溢出 | ？ | 本轮小清理 | [L](#l)：仅去除64位纯本地计数耗尽导致的运行锁死；保留回绕语义和0哨兵处理，不动线帧序号规则。 |
| 178 | WorldYawWindowFollower授权/窗口/样本一致性门；未启用-替代跟随器 | ？ | 解释/保留关闭 | [M](#m)：替代WorldYawWindowFollower未启用，不作为正式路径删改对象。 |
| 179 | CAN/RS485板间传输的双批次队列清空；未启用-替代板间传输 | ？ | 解释/保留关闭 | [M](#m)：替代CAN/RS485板间传输未用；正式UART不受这项影响。 |
| 180 | Latest非阻塞锁失败沿用旧遥控快照；正式-遥控快照争用 | ？ | 保留 | [K](#k)：Latest抢锁失败沿用旧帧且不续期，正常；不新增锁等待或假新帧。 |
