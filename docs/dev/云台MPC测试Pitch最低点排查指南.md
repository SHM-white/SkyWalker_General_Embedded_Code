# 云台 MPC 响应测试：Pitch 停在最低点排查指南

**当前正式云台已改为 schema 113、32通道。** 新 Pitch 实测关节角是 I21、头部世界 Pitch 是 I12、轴位置参考是 I20、接受的参考速度是 I22；限幅/恢复事件看 I31，状态看 I7。以下涉及 I33/I28、I38～I43 的编号均为旧 schema 112 的历史排查方案，不得直接用于新固件，也不再需要按旧方案覆盖这些通道。新布局与使用方式以[云台双轴响应VOFA通道说明](云台双轴响应VOFA通道说明.md)为准。

## 当前结论与范围

2026-11-15 二次更新：用户补充，**手动抬高后重新启动 MPC 仍自行回到最低点，而撤销全部工作区改动的上一版正常**。这组对比将排查重点转向本轮下位机改动；此前“优先查上位机旧中心”的判断不足以解释回归，不能继续将其作为首要结论。截图中约正 18°的目标只说明截屏时轨迹处在低位，不能确定低位是原因还是接管过程形成的结果。尚无本轮同步 CSV、编码器和轴参考记录，不能确认是哪条新增行为导致全部低位现象。

用户本轮已明确授权直接修复。已修改正式应用的 Pitch 恢复播种、执行器重启事件保留，并把 Pitch 速度前馈从 0.25 恢复为 0；具体见下节。没有修改上位机捕获中心策略。硬件上的回归是否完全消失，仍需重新烧录后的中部接管记录确认。

当前未提交优化改动相对仓库 HEAD 保留了原来的 Pitch 世界射线变换、`vision_pitch_horizontal_rad=3.00`、`pitch.direction=-1` 和 `[2.60, 3.40]` 限位；改动增加了 Rate 会话重建、速度参考前馈，并缩小位置死区。**测试中心偏低、关节目标重新播种、机械水平零位变化是三件不同的事。** 本轮直接修复只处理新增参考链路及未经基线验证的 Pitch 前馈，不改变方向、机械限位、水平零位或 PID 增益。下文中心操作保留作辅助诊断，重新捕获中心不能替代对下位机回归的确认。

上位机 `~/RM2026-AutoAim/tests/planning/gimbal_mpc_response_test.cpp` 的中心在反馈新鲜、重连、按 C、或进入自动模式时捕获。它发送 `center.pitch + MPC 相对角`，单位 rad，是真实反馈所在世界参考系的绝对欧拉角；图上大部分曲线和 XY 则是**相对中心**。默认高度 3° 是全高，不是从当前位置向上 3°。如果自动接管时已在低位，中心就在那里；按 C 也是捕获当时位置，不会自动回水平。

下位机 `lib/communication/vision_ab_protocol.cpp` 回传的 pitch 是从同一头部四元数提取的欧拉 pitch；上位机直接使用回传的 `state.pitch`，因此不应把收到的目标直接当作 2.60～3.40 rad 的电机角，也不应给上位机加 3.00 rad。两边约定都是正 pitch 低头。下位机实际把世界射线变换到 yaw 级坐标后，计算编码器目标，硬件方向在 `calibration.hpp` 是 `-1`，目标夹在 `[2.60, 3.40]`；`3.00` 水平零位仍是未实测初值。

理想检验：头部四元数和目标使用同一时刻、同一参考，目标 yaw/pitch 等于当前头部 yaw/pitch 时，几何计算应返回当前编码器位置（只要该位置在限位内）。因此即使水平零位不准确，也不能单凭回中停在低处断言它是根因；安装方向、四元数坐标、时间不同步、轴的参考和实际电机反馈也可能造成偏差。需要实际数据排除。

## 本轮已直接实施的修复

1. `applications/sentry_gimbal/src/main.cpp`：删除执行前更新的 `vision_pitch_was_active`。只有上一周期属于视觉 Rate、执行器没有提前返回、Pitch 输出有效且轴目标有效时，才记录参考已建立；下一周期还检查控制间隔、驱动 Active / 输出许可以及电机使能和位置参考代次是否连续。
2. 参考尚未建立或不连续时，本周期 Pitch 追赶 Rate 为零，并请求轴重建会话；成功播种后，下一连续周期才用 `pitchTargetRad()` 计算追赶。成功播种不要求 `reference_accepted=true`，否则恢复周期禁止前馈会让轴永远无法退出等待。
3. `include/robotics/gimbal/gimbal_executor.hpp` / `lib/robotics/gimbal_executor.cpp`：两轴分别保留重启事件。漏周期会将两轴标为待恢复；只有对应 Rate 轴成功输出才消费事件，不能被另一轴的 Active 或提前返回吞掉。Yaw 从绝对轨迹回 Rate 时也重新播种。
4. `include/robotics/vehicle/gimbal_profile.hpp`：`pitch_velocity_feedforward_gain=0.0f`，保留参考桥接和合成速度限幅。先恢复零增益基线，再判断是否重开 Pitch 前馈；Yaw 增益仍为 0.5，Pitch 死区仍为 0.005 rad。

这些修改修复了可以从源码确定的恢复缺口，并撤回尚未完成基线验证的 Pitch 前馈。没有实机同步记录，不能把其中一项宣称为已经独立证实的唯一根因。所有源码和文档改完后只进行编译检查，不运行测试程序、烧录或设备运动。

烧录后沿用同一上位机程序和配置，从安全中部、小幅低频开始，观察是否还出现无请求的大幅下扎；记录进入自动时的 `center_pitch_deg`、绝对 REF/MPC/RX 与 I33。若仍异常，立即解除使能，保留数据，按第 2 节区分几何目标与轴积分参考；不要靠改变上位机中心掩盖现象。

## 0. 本轮截图实际说明了什么

### 0.1 先看绝对角，区分大幅追赶和稳态往复

| 截图读数 | 可以得出的结论 |
|---|---|
| 绝对 Pitch：REF `+18.370°` / MPC `+18.367°` / RX `+19.676°` | 发出的目标本身处在低头方向；当前瞬时 `MPC-RX≈-1.309°`。不是在要求世界 Pitch 为 0°。 |
| Raw q YPR 的 Pitch `+19.68°` | 本帧报文 Pitch 与原始回传四元数解出的 Pitch 同号、近似一致；没有显示出这两种反馈之间的符号反转。它们来自同一传感器，不能据此证明机械安装标定正确。 |
| 高度 `3.0°`、频率 `1.0 Hz` | 正常展开后的相对 Pitch 范围为中心上下各 1.5°。它不会因为云台接近端点就自动把中心移回中部。 |
| 开头相对 Pitch RX 约 `-25°`，Pitch 误差峰值 `25.041°` | 可见窗口包含一次远大于轨迹全高的追赶。可能涉及旧中心恢复、重启或其他瞬态；截图不能唯一确定触发事件。 |
| Pitch REF-RX RMSE `3.323°`、CMD-RX RMSE `3.319°` | 这 6 s 的统计包含上述大瞬态，不能直接当作稳定 1 Hz 跟踪性能来调 PID。 |
| MPC 收敛 `Y/Y`、限幅 `N/N` | 上位机规划正常不代表 MCU 几何目标、轴参考或机械端点没有触限。 |

右侧 XY 图把相对 Pitch 为负的点画在上方，且横纵轴共用比例。约 25°的纵向瞬态会把正常的 `6°×3°` 小矩形压得很小；等瞬态离开显示窗口再判断稳态形状。按 R 只清历史，不能改变实际目标中心。

### 0.2 “参考位置变了”需要区分哪一个参考

| 名称 | 当前代码中的含义 | 如何确认 |
|---|---|---|
| 头部世界 Pitch | 头部 IMU 四元数提取的欧拉角，正值低头 | 下位机 I28，对照上位机绝对 RX / Raw q，注意 rad 与 deg 换算 |
| 测试中心 `center[1]` | 上位机捕获的某一帧世界 Pitch，不是水平标定值 | 上位机 CSV 的 `center_pitch_deg` |
| 几何水平参数 `vision_pitch_horizontal_rad` | 建模时水平枪口对应的电机编码器角，当前 3.00 rad | `include/robotics/vehicle/calibration.hpp`；本轮未提交优化未改它 |
| 轴执行参考 `gimbal.pitchTargetRad()` | Rate 积分器的关节目标，与电机反馈同坐标 | 临时 I42，对照 I33；视觉会话切换等事件会把它重新播种到实测位置 |

真实的数据流是：

```text
头部四元数 → 回传世界 Pitch → 上位机 state.pitch
                              ↓ 捕获一次
                         测试中心 center[1]
                              ↓ 加相对轨迹
               发出的世界目标 = center[1] + MPC 相对 Pitch
                              ↓
下位机本地年龄外推 → 世界射线转关节几何目标 → 限位 → Rate → 轴积分参考 → 电机
```

上位机关键位置在 `ResponseWorker::loop()`：

- 第 348～356 行：首次取得反馈、串口 generation 变化、反馈中断后恢复、按 C、启用控制时进入 AUTO_AIM，都会用当前 `feedback->state.pitch` 捕获中心。
- 第 358～364 行：改参数或暂停后恢复，只重新计算 `start = 当前实测 - 原中心`，并重启过渡；不会单独重捕获中心。
- 第 452～454 行：发送 `center[1] + sample.cmd.p[1]`，不是直接发送相对角。
- 第 847～862 行：CSV 已保存 `center_pitch_deg`，并把 REF/MPC/RX 角度列保存成绝对角。无需先改程序就能查中心。

按键行为如下，不能互相替代：

| 操作 | 中心行为 | 对本问题的影响 |
|---|---|---|
| C | 下一帧新鲜反馈处重新捕获当前姿态 | 在低位按 C，会继续以低位为中心；它不会自动抬到水平 |
| Enter / Apply | 沿用中心，从当前实测位置过渡到新参数的轨迹 | 手动抬高后只按 Enter，最终仍可能回到旧低位中心 |
| Space 恢复 | 沿用中心，从当前实测位置重新过渡 | 仅暂停再恢复也不会把终点中心换成当前位置 |
| R | 沿用中心，只清历史和统计 | 图看起来重画了，但实际参考没变 |
| 从非自动进入 AUTO_AIM | 启用控制时重捕获当前反馈 | 应记录切换时的真实位置；下位机报 mode=1 只表示自主模式，不能证明已经由视觉成功执行 |

补充：下位机 `main.cpp` 第 613～614 行在获准自主运行时回传 AUTO_AIM，巡航/扫描也可能处于此模式。若反馈一直是 mode=1，只在自主子模式间切换，不会触发上位机 `!was_auto → auto_mode` 的重捕获条件。中心可以早于本次实际视觉接管时刻形成。

### 0.3 为什么不能看到约 20°就给上位机加减 3.00 rad

3.00 rad 是电机坐标，约 18°是世界姿态，两者不能直接抵消。当前几何同时使用水平参数来移除实测 Pitch 关节旋转、再还原目标编码器角。目标方向恰好等于当前头部方向时，这两个操作应相互抵消，返回当前关节位置。

在横滚很小、目标 Yaw 接近当前 Yaw 的近似条件下，可以帮助理解为：

```text
q_raw ≈ q_measured + direction × (pitch_command - pitch_measured)
direction = -1
本帧 pitch_command - pitch_measured ≈ (18.367 - 19.676)° ≈ -0.02285 rad
所以 q_raw ≈ q_measured + 0.02285 rad，即本帧几何意图略微抬头。
```

这只是对截图当前时刻的近似解释，精确值必须看世界射线变换后的 I39；不能用它推算机械端点。它说明“实物在低位”不等于“本帧命令还在持续向下顶”。

## 1. 先确认现象，不改软件

### 1.1 辅助诊断：中部中心与下位机参考分开记录

1. 在改变中心前，按 S 保存当前 CSV，保留本轮 `center_pitch_deg`。C 和模式重捕获都会清掉旧历史；改完再保存就拿不到旧中心了。记录目前实物在低位以及对应 I33 / I28。
2. 退出测试的控制发送，回手动模式，将云台低速移到安全行程中部，确认能保持且没有端点拉扯。保持无弹、可立即解除使能。不要在当前最低位置直接按 C 期待回水平。
3. 在手动状态、姿态稳定时启动响应工具，使用小幅低频参数，例如 `--width=1 --height=0.5 --frequency=0.2`；沿用已知的程序路径和配置路径。工具启用控制时，手动模式下应显示停止控制，保持反馈新鲜。
4. 在这个中部姿态按 C，再按 S，确认 `center_pitch_deg` 接近此时绝对 RX，而非仍是先前的低位数值。然后在可立即解除使能的条件下切入允许视觉的自动模式，避免在巡航扫描过程中捕获中心；进入 AUTO_AIM 可能再次捕获，保存切入后的 CSV。
5. 分别观察中心、REF/MPC 和 RX：正常展开后 REF 应在新中心上下约 0.25°内往复。若 REF/MPC 仍落在旧低位，继续查上位机捕获事件；若 REF/MPC 明确在中部而实物向低位跑，立即解除使能，再走第 2 节的下位机诊断。
6. 若中部小幅跟踪恢复，原低位轨迹首先由旧中心解释。待过渡结束再按 R 或等待它移出可见窗口，另存稳定段；不要将清历史视为修复，也不要据此宣称原来的响应优化已经验证通过。

这里需要的四组数据是：旧记录的 `center_pitch_deg`、中部手动的绝对 RX、自动接管后的 `center_pitch_deg`、同一时刻 I33。**若新中心在中部却仍被压到低位，才进入下游追踪，不继续靠反复按 C 掩盖。**

### 1.2 方向、反馈与动态影响的后续隔离

1. 空载、无弹，确保有人能立即解除使能，并防止云台碰撞。暂时关闭响应测试的控制发送（`--enable-control=false`），用手动模式低速验证 Pitch 双向可控。观察 VOFA 的 I33（电机位置，rad）和 I28（头部世界 pitch，rad）；按代码的预期，物理抬头时 I33 增大、I28 减小。记录抬头和低头各一次；若相反、反馈不变或接近机械端点，停止，不要继续视觉试验。
2. 将云台手动移到安全行程中部，再进入测试并等待反馈稳定；在 GUI 读绝对 Pitch 的 REF/MPC/RX 与 `Raw q YPR`，记录进入自动前后数值及 mode/state。注意图中负 Pitch 代表抬头，XY 和多数波形以捕获中心为零。确认所说的“最低点”是实物还是曲线的最低位置。
3. 保持 `--enable-control=false` 时，观察上位机 RX 与 VOFA I28 是否同号同量级（GUI 是度、VOFA 是 rad，1 rad ≈ 57.3°）。GUI 的 Raw q YPR 来源于回传四元数，与 RX 的 pitch 都应近似一致。检查 I26=0、I31 中头部有效位、I33/I36 与 I44；这些现有通道不能单独证明视觉目标夹到了限位。I30 是惯性适配器的观测误差，**不是**当前 Pitch 几何执行误差。
4. 非零前馈实验尚未取得 gain=0 基线：在实际修改并重新构建前，把 `include/robotics/vehicle/gimbal_profile.hpp` 的 `pitch_velocity_feedforward_gain` 暂设为 `0.0f`，其他参数保持不变。这仅是隔离动态影响，不保证修复位置问题。不得通过放宽限位或增大力矩来试。

## 2. 加一组短期诊断通道

现有 I28/I33/I44 缺少目标、夹紧前值和参考实际值，无法区分几何问题与执行问题。只在调试版本中，在 `applications/sentry_gimbal/src/main.cpp` 的控制循环内，把以下变量声明在 `if (vision_active)` 之前，每周期初始化为 `NaN`；不要使用上周期残留值。计算 Pitch 时分别保存：

```cpp
float dbg_pitch_raw_target = std::numeric_limits<float>::quiet_NaN();
float dbg_pitch_clamped_target = std::numeric_limits<float>::quiet_NaN();
float dbg_pitch_seed = std::numeric_limits<float>::quiet_NaN();
// vision_active 分支内，在计算 target 的原位置：
const float raw_target = installed::vision_pitch_horizontal_rad +
    installed::pitch.direction *
        std::atan2(-joint_ray.z, std::hypot(joint_ray.x, joint_ray.y));
const float target = std::clamp(raw_target, installed::pitch_min_rad, installed::pitch_max_rad);
dbg_pitch_raw_target = raw_target;
dbg_pitch_clamped_target = target;
// 原有 seed 定义之后：
dbg_pitch_seed = seed;
```

保留原来的 `target`、`seed` 及速率计算，不重复定义 `target`。在 `gimbal.update(...)` 之后、VOFA `trace` 构造前，通过 `gimbal.pitchTargetRad()` 和 `gimbal.pitchTargetDiagnostics()` 取本周期执行结果。临时替换 trace 的 **I38～I43**，保持 `std::array<float, 30>` 的数量不变，I19 另换一个未与现有接收端冲突的 schema 值并同步通道说明：

| 通道 | 调试值 | 作用 |
|---|---|---|
| I38 | `vision_active ? vision_pitch_rad : unavailable` | 预测后的世界 pitch，rad |
| I39 | `dbg_pitch_raw_target` | 限位前关节目标，rad |
| I40 | `dbg_pitch_clamped_target` | 限位后几何目标，rad |
| I41 | `dbg_pitch_seed` | 计算本周期追赶速度的旧目标，rad |
| I42 | `float(gimbal.pitchTargetRad())` | 执行器更新后的实际积分参考，rad |
| I43 | `float(gimbal.pitchTargetDiagnostics().restart_reason)` | 参考重启原因的枚举值 |

保留 I26～I28、I31、I33、I36、I44、I45、I48。`pitchTargetDiagnostics()` 也有 `target_valid`、`reference_accepted`、`mechanical_limited`、`lead_limited` 等字段；若参考无法解释，再临时替换其他已确认不需要的通道输出它们，明确记录 schema。调试字段会覆盖原 I38～I43，不要按原说明解读；调完恢复原通道布局。该诊断不修改控制命令或机械保护。主循环局部变量在 `#if CONFIG_SKYWALKER_VEHICLE_DIAGNOSTICS` 之外使用时也需保证作用域正确；使用源码现有的 `unavailable` 位置，不要在其声明之前引用它。

## 3. 对照一段低速记录

修改一次后统一构建并烧录调试固件，按照项目现有构建命令操作；不要在同一轮继续改多个参数。先在中部、无弹、有人守停的条件下，用很小的高度和低频开始，持续观察，异常立即解除使能。保存上位机 CSV（S）及同步 VOFA，标出自动接管、运动和停止时刻。上位机 `--height 3` 是 3° 全幅；首次应显著减小，频率也降低。保持接管时姿态在行程中部，不要在端点重捕获中心。

| 观察 | 优先结论及下一步 |
|---|---|
| GUI 相对 RX 在波形最低，但实物在中部、绝对角及 I33 正常变化 | 这是相对中心或负角方向的显示含义，不是电机被压在底部。 |
| I38 不接近 GUI MPC 绝对角 | 对比两边时刻、预测量、接管状态与串口；检查选中目标和新鲜度，不能先改方向。I38 已含下位机按本地报文年龄与配置延迟外推。 |
| 刚接管时目标近似实测（GUI MPC≈RX、yaw 也近似），但 I39 与 I33 明显不同 | 优先核对头部四元数是否为 body→world、IMU 机械安装轴与云台模型是否一致、关节角和姿态是否同一时刻；读 `head_sensor_to_body` 与实际安装。先做静态同姿态对照；不可仅因这一项就武断翻转符号。 |
| I39 超过 2.60～3.40，I40 卡在端点 | 几何目标被限位。核对上位机中心、目标幅值、上一步姿态约定及实际编码器范围；不可放宽限位来掩盖。 |
| I39/I40 随轨迹正常变化，但 I42 卡住或反复跳回 I33，I43 或参考接受状态异常 | 查看会话重播种、输出有效状态和循环间隔。`vision_pitch_was_active` 仅表示视觉有效，不代表上一周期 Pitch 成功执行；原优化指南第 7 节已经指出恢复首周期需与轴播种同步，不能把它当成已排除的故障。 |
| I42 跟随目标，I33 不跟或方向相反 | 关注电机反馈、实际转向、使能、力矩和机械负载；停止视觉控制，回手动模式排查，不能靠提高前馈或放宽端点解决。 |
| I33 双向正常而 I28 随手动运动的方向或幅值不符 | 优先排查 IMU 安装轴、报文方向和姿态参考，再继续视觉控制。 |

I43 的数字需对照当前 `include/robotics/gimbal/gimbal_axis.hpp` 的枚举，不要假设所有非零原因相同。I44 是本周期发送给轴的速率命令，I42 是轴更新后目标，两者与 GUI 中的世界角不能直接相减。

当前 `AxisRestartReason` 对应：`0=None`、`1=CommandSession`、`2=RequestEdge`、`3=UnconsumedCycle`、`4=ExplicitReseed`。恢复周期不积分、不接受速度参考可以是正常行为；`reference_accepted=false` 本身不等于电机故障。VOFA 的周期采样可能漏掉单个 5 ms 恢复事件，未观察到非零 I43 不能证明没有发生重播种；必要时由操作者增加事件计数或窗口锁存后再采集，不能直接提高日志发送频率。

## 4. 后续修复选择与完成条件

如果证据指向安装关系，先标定头部传感器到机械坐标的旋转及其在 IMU 驱动中的应用位置，保证反馈与执行共用约定，避免在协议两侧各翻一次符号。如果指向会话恢复，按原《云台MPC轨迹跟踪响应优化实施指南》第 7 节实现：恢复周期的 Pitch 追赶速率置零，之后仅从成功播种并执行的参考追赶；依赖轴状态而非单独依赖 `vision_pitch_was_active`。如果根因是机械参考或电机方向，先测得实际端点和安全方向，再由操作者做对应标定；不应凭当前猜测改常量。

修复前的恢复接缝是：应用按 `vision_pitch_was_active` 选种子，且在 `gimbal.update()` 前更新它；轴内部可能随后重播种、等待或拒绝输出。本轮已按文首说明改为执行结果与参考代次联合门控，不再使用该布尔变量。下列步骤保留为此类修改的接口约束，不表示仍需重复实施。

需要实施恢复修复时，按以下文件顺序连续完成，再统一构建；不边写边运行：

1. 在 `include/robotics/gimbal/gimbal_axis.hpp` / `lib/robotics/gimbal_axis.cpp` 明确“本周期重新播种”“实际执行参考有效”的诊断，保留已有限位、反馈新鲜度和恢复周期零积分规则。
2. 在执行器头文件及 `lib/robotics/gimbal_executor.cpp` 保持这些状态向正式应用可见，等待或漏周期不可被解读为成功执行。
3. 在 `applications/sentry_gimbal/src/main.cpp` 用上次实际执行结果决定是否可从旧轴参考追赶；进入/恢复/参考代次变化时先输出零 Pitch 追赶 Rate，使轴播种，之后再使用已建立的 `pitchTargetRad()`。不能只将布尔变量移到 `gimbal.update()` 后而仍然赋值为 `vision_active`。
4. 全部改完后，使用正式应用构建命令进行一次编译；记录并确认烧录镜像身份，再按本文中部小幅步骤人工取数。本轮编译结果以任务回复为准，没有运行测试、烧录或设备运动。

## 5. 如仍不清楚中心何时变了，先补上位机显示与日志

这一步只增加观测，不默认改变中心策略，也不默认把世界 0°当作机械安全中位。文件为 `~/RM2026-AutoAim/tests/planning/gimbal_mpc_response_test.cpp`，本次未修改该外部工程。

在 `render()` 中现有 `row(...)` 可用的位置、绝对角读数附近添加以下一行；适当给侧栏留出高度，避免后续文本超出画布：

```cpp
row(fmt::format("Latched center Y/P {:+.3f} / {:+.3f} deg",
  view.center[0] * deg, view.center[1] * deg), muted);
```

在 `ResponseWorker::loop()` 原 `if (center_changed)` 内，`center = ...;` 之后、`centered = true;` 之前添加日志，原有加速度清理与历史重启逻辑全部保留：

```cpp
tools::logger()->info(
  "[MPC response] center Y/P={:+.3f}/{:+.3f} deg; generation={}; "
  "initial={} generation_changed={} fresh_edge={} C={} auto_entry={}",
  center[0] * deg, center[1] * deg, feedback->generation,
  !centered, generation_changed, !was_fresh, recenter != input.recenter,
  enable_control_ && auto_mode && !was_auto);
```

这些标志可以同时成立，不要仅取第一个作为唯一原因。`generation` 是上位机串口会话代次，不是 MCU 的头部姿态 epoch。两个观测点连续完成后，使用上位机现有构建目录编译 `gimbal_mpc_response_test` 目标；不要在不知道真实目标中心的情况下先改捕获规则。如果最终需要支持“固定中心”或“恢复时重新捕获”，应作为显式选项设计，并沿用 `start` 的平滑过渡，避免改变中心后直接跳目标。

## 6. 本轮完成清单

- [ ] 记录物理最低点对应 I33、I28，确认正方向与范围。
- [ ] 保存旧 CSV，读取精确 `center_pitch_deg`；记录手动中部和自动接管后的中心。
- [ ] 用中部新中心、小幅低频轨迹区分“目标本身在低位”和“下游把中部目标推向低位”。
- [ ] 确认 Enter / Space 恢复沿用旧中心，R 只清历史；C 不负责回水平。
- [ ] 获取同步 GUI CSV 和 VOFA 的几何目标、轴参考及执行状态。
- [ ] 按证据只修一个根因，在安全中部先做低速小幅确认，再逐步恢复原参数。
- [ ] 清除临时诊断、恢复通道定义；Pitch 前馈仍需先有零增益基线再逐步评估。
