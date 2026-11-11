# SkyWalker 双 Yaw 加速度协同控制实施方案

复核日期：2026-11-11。主仓库基准为 [SkyWalker `dev@4dc29fb65dc564e9cacd88005fb44c7fe4e75440`](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/commit/4dc29fb65dc564e9cacd88005fb44c7fe4e75440)。参考底盘基准为 [`JJL0612/sentry_onlychassis@dcc8d4ebfb5f91b0836849af7999e3b45849ed2e`](https://github.com/JJL0612/sentry_onlychassis/commit/dcc8d4ebfb5f91b0836849af7999e3b45849ed2e)。

## 1. 交给实现者的任务

1. 先确认实车底盘烧录的是参考 HAL 固件还是本仓库 Zephyr 固件，并从当前 `origin/dev` 开工作分支。若提交已前进，先复核本文锚定的入口和参数。
2. 实现云台侧双 Yaw 加速度分配。手动持续转向时，大 Yaw 尽早承担枪头的持续角速度，小 Yaw 回到机械中心附近。目标先以稳态中心偏差 ±5°作为试车指标，之后按机械间隙与静止噪声收敛。
3. 高频自瞄由小 Yaw 跟踪角度和高加速度分量。大 Yaw 跟随可持续的低频运动，并根据小 Yaw 剩余行程提前接管。轨迹超出实测能力时，主动压低向外的枪头速度和加速度。
4. 按第 7 节分阶段提交。参考底盘先做云台单板验证；完整的持续高速目标使用配套 Zephyr 双板方案。真实视觉执行以姿态、参考系和超时链路验收为前提。每阶段给出构建结果、回归用例和实机待测项。

## 2. 已核对的源码和前版修正

| 环节 | 当前行为 | 对实施的影响 |
| --- | --- | --- |
| 正式云台入口 | `sentry_gimbal` 控制周期 5 ms，约每 10 ms 发大 Yaw 请求与载体 IMU。遥控 Yaw 上限取标定中的 3 rad/s，启动日志里的 1 rad/s 文案已过时。现有回中 P=6、请求速度上限 1 rad/s、斜率 4 rad/s² | 在应用入口分离原始枪头指令与经过补偿的关节指令；启动日志改为报告真实配置 |
| 小 Yaw | `GimbalAxis::Rate` 将速度积分成位置目标，最后由 `PositionMotor` 的位置、速度串级环控制。标定候选总行程约 69°，位置环死区 0.012 rad；速度内环已有 20 rad/s² 参考斜率 | 第一阶段能改变关节轨迹和目标斜率。要把自瞄加速度真正用于驱动，须检查串级环输入、限幅与电流路径 |
| 参考 HAL 底盘 | README 指定兼容对象。先把 `0x0402` 世界速度意图裁到 ±1 rad/s，再积分航向，叠加 P 修正（修正项 ±1.5 rad/s）、速度 PI 与底盘自转补偿；最后直接发原生速度命令。回传约 100 ms 一帧，`0x0403.actual_rate_rad_s` 取电机原生速度 | 云台单改可验证低速主动分配。稳定居中持续转向的速度预算须低于 1 rad/s，给回中和负载留余量；即使瞬时实际速度超过 1，也无法凭此预算承诺 3 rad/s 持续转向。该回传用于健康观测 |
| SkyWalker Zephyr 底盘 | `BigYawWorldHeadingFollower` 积分世界速度并加航向 P；当前 P=2，修正项上限 6 rad/s，角速度 PI 为 0。减车体陀螺速度后，执行器按 ±10 rad/s 物理关节限速、安装方向、传动比和 20 rad/s² 原生速度斜率执行 | 云台规划的加速度只约束线上意图。最终速度和斜率预算要覆盖航向修正以及自转补偿；原生斜率须换算到关节侧并经负载实测 |
| 两种底盘的零速捕获 | 参考底盘在请求从非零变成精确零时重新捕获航向；Zephyr 用 `release_epsilon_rad_s=1e-4` 的穿越判定 | 换向或回中与低频项抵消可触发捕获。参考底盘阶段接受并量化这一行为；配套 Zephyr 阶段实现稳定停车捕获 |
| 视觉 | 正式入口目前 `allow_auto=false`，未启动视觉或 CAN3 头部 IMU。`CommandArbiter` 已校验并限幅 `decision.selected_vision` 的加速度，`GimbalCommand` 仅携带角度和速度 | 从已裁决的 `selected_vision` 构造轨迹，避开绕过权限与参考系校验的旁路 |

源码锚点：[云台入口](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/applications/sentry_gimbal/src/main.cpp#L170-L246) · [小 Yaw 标定](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/include/robotics/vehicle/gimbal_profile.hpp#L48-L96) · [参考底盘限速及航向环](https://github.com/JJL0612/sentry_onlychassis/blob/dcc8d4e/Skywalker/Application/Src/app_robot.c#L1418-L1517) · [参考底盘回传](https://github.com/JJL0612/sentry_onlychassis/blob/dcc8d4e/Skywalker/Application/Src/app_robot.c#L1690-L1724) · [Zephyr 底盘执行入口](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/applications/sentry_chassis/src/main.cpp#L156-L201) · [视觉裁决](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/lib/robotics/command.cpp#L281-L326)。

## 3. 坐标、执行归属和反馈

近似水平、Yaw 转轴近似平行时，统一使用逆时针为正。角度单位 rad，角速度 rad/s，角加速度 rad/s²，加加速度 jerk 为 rad/s³。

| 符号 | 含义及来源 |
| --- | --- |
| `h` | 枪头世界 Yaw，手动由轨迹规划生成，视觉由已对齐的世界目标给出 |
| `b` | 大 Yaw 载体世界 Yaw，使用云台板载 BMI088 的独立 planar yaw 和去零偏 gyro-z |
| `s` | 小 Yaw 物理关节角。中心偏差 `e = direction_s * (q_encoder - q_center)`，关节速度 `w_s_meas = direction_s * qdot_encoder` |
| `c` | 底盘车体世界 Yaw，底盘板本地 BMI088 提供车体偏航速度 |
| `j` | 大 Yaw 相对车体的物理关节。底盘侧有 `w_j = w_b - w_c`；电机原生命令由底盘侧乘方向和传动比一次 |

有 `w_h = w_b + w_s`、`a_h = a_b + a_s`。小 Yaw 使用 **实际载体运动** `w_s_ref = w_h_plan - w_b_meas`。速度、位置、加速度输出转回编码器坐标时各乘一次 `direction_s`。例如枪头目标 +1.5、载体实测 +0.6 rad/s，则小 Yaw 物理速度目标为 +0.9 rad/s。枪头保持、载体向 + 方向回中时，小 Yaw 给出负相对速度。回中律使用正号 `+Kc*e`。正向和反向各做一次实机低速符号检查。

板间 v4 继续传 `0x0402 mode=1` 的载体世界速度意图，以及 `0x0404` 的载体 BMI088 航向和速度。请求在云台侧直接用世界坐标，底盘侧负责减 `w_c`。视觉姿态回传独立生产，`0x0404` 始终使用载体。主控校验 `0x0403` 的身份、时效及执行状态；其速度语义在两套固件间有差异，运动分配闭环统一取本地载体 gyro。 [板间协议 v4](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/docs/protocols/interboard-v4.md) · [独立平面航向](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/applications/sentry_gimbal/README.md)。

## 4. 云台侧分配器

### 4.1 接口与调用顺序

建议新增 `include/robotics/gimbal/dual_yaw_motion_allocator.hpp`、`lib/robotics/dual_yaw_motion_allocator.cpp`，在 `lib/robotics/CMakeLists.txt` 接入。实例归 `applications/sentry_gimbal/src/main.cpp`。算法只读已裁决指令和新鲜测量，只生产轨迹及诊断。使用固定大小状态，单执行线程，无动态分配。

- **输入**包括原始 `GimbalCommand` 与源时间戳、手动或视觉来源、上一周期实际提交的大 Yaw 意图、载体 gyro 值与原始采样标记、小 Yaw 编码器角度与速度、执行器状态、所选底盘标定与控制间隔。视觉阶段增加被接受的目标轨迹和参考代次。
- **输出**包括大 Yaw 世界请求速度和该请求的实际斜率，小 Yaw 关节位置/速度/可选加速度参考，枪头可实现轨迹，当前限位、降级、跟踪受限状态。第一阶段小 Yaw 只消费关节速度参考。5 ms 分配器斜率按规划时钟计算，10 ms 线上请求的斜率另外按实际发送间隔计算。
- **每周期顺序**为读取快照和既有授权 → 判断本地、小 Yaw 速度与载体反馈时效 → 计算行程包络及轨迹 → 调一次 `gimbal.update()` → 检查本周期 Yaw 执行结果与 CAN 提交结果 → 按现有发送节拍选择正常请求、零速等待或 Disabled。命令源时间戳沿用既有 `sourceStamp()`。本地执行失败须覆盖本周期的非零板间请求；如调换发送与 CAN 提交顺序，保留每条物理总线每周期一个写入方和一次 commit。
- `RunStatus::Active` 可以仅代表 Pitch 有效。用当前周期有效时长、Yaw 反馈/代次、本周期确实更新过的 Yaw 执行结果确认大 Yaw 授权；错周期时历史 `yawTelemetry().output_valid` 仍可能为真。启用前保持原有 `mode=1,target=0` 握手，获取当前会话新鲜反馈后才允许非零。
- 手动入口在应用层将分配后的关节速度直接交给 `GimbalExecutor`，把原先应用内的载体减法并入分配步骤。降级路径继续用现有手动编码器行为。大 Yaw 停用但 gyro 仍新鲜时，可继续补偿载体实际转动；gyro 失效时退到本地编码器速率。Pitch、行走/小陀螺、解锁及停机仍由各自现有路径负责。

### 4.2 枪头目标与大 Yaw 加速度规划

手动遥控以原始世界速度为目标，使用有界加速度/jerk 的轨迹生成器产生可实现的枪头参考 `w_h_plan,a_h_plan`。禁止对量化摇杆原始值做单步差分求加速度。高频自瞄接入后的输入构造见第 6 节。

把已受行程约束的枪头速度送入一阶低通，使用真实 `dt`。`w_slow += (1-exp(-dt/tau)) * (w_h_plan-w_slow)`，从滤波前后状态求 `a_slow=(w_slow-w_slow_prev)/dt`，其中 `w_slow_prev` 是本轮更新前的滤波状态。复位首帧导数取零。手动建议先试 `tau=0.08–0.12 s`，视觉试 `0.18–0.30 s`。例如 `tau=0.20 s` 的一阶低通对 5 Hz 速度分量保留约 16%，其余动态优先由小 Yaw 及实际反馈承担。

中心修正采用滤波后的 `e`，正常路径可用连续软死区 `D(e)=sign(e)*max(abs(e)-deadband,0)`。在行程预警区用原始角度及速度触发保护，以免滤波延迟。若加迟滞，软区出口从零平滑增益并单测切换连续性。回中项要单独限幅和保留可用速度余量，靠近端点时使用朝释放行程方向的回中加权，最后由加速度规划统一整形。

```text
w_slow, a_slow = lowPass(feasible_head_rate, dt)
center_rate = clamp(Kc * softDeadband(filtered_center_error),
                    -center_rate_max, +center_rate_max)
w_big_goal = clamp(w_slow + center_rate, -big_intent_max, +big_intent_max)
a_big_goal = a_slow + Kv * (w_big_goal - w_big_plan)
w_big_plan, a_big_plan = stepRateTrajectory(a_big_goal, big_bounds, dt)
w_small_ref = feasible_head_rate - carrier_rate_measured
```

`Kv` 的单位为 s⁻¹，反馈的是规划速度误差。由载体实测与请求的偏差做跟踪受限检测与防积累，给比较值对齐发送、反馈延迟。实测长期落后时，限制同向规划超前、降低枪头可持续速度预算，并通过实测小 Yaw 行程继续做保护。超过跟踪阈值须可观测，阈值按波形整定。避免再叠加一个未经整定的实测速度高增益外环。

**速度、加速度与 jerk 一起约束。** 大 Yaw 和枪头都可复用一个标量轨迹步进。固定边界、`J>0` 时按以下方法实现：

1. 可选下一步加速度区间为 `[max(a_min,a0-J*dt), min(a_max,a0+J*dt)]`，常规对称上限取 `a_min=-A,a_max=+A`。沿一个周期按恒定 jerk 走，`w1=w0+(a0+a1)*dt/2`。
2. 给加速度平滑归零预留速度余量。上侧要求 `w1 + max(a1,0)^2/(2*J) <= w_max`，下侧要求 `w1 - max(-a1,0)^2/(2*J) >= w_min`。同时把周期内的速度极值纳入二分判定。用 `j=(a1-a0)/dt` 和 `w(t)=w0+a0*t+j*t²/2`，检查起点、终点，以及落在 `(0,dt)` 的零加速度时刻 `t=-a0/j`。同一时刻的 `w(t)` 随 `a1` 单调递增，周期内最小值和最大值也单调递增。下侧二分须同时通过最小速度和下侧归零余量，上侧二分须同时通过最大速度和上侧归零余量。将目标加速度投影至交集，数值容差按控制量精度配置。
3. 用积分得到的 `w1,a1` 写回状态和诊断。固定边界的正常轨迹用提前制动接近限速。保护导致边界骤缩、初值越界或可行区间为空时，优先满足安全限速与刹停，记录约束冲突，按安全输出重置/对齐规划状态；这类周期单独标记，允许超出舒适性 jerk 指标。停机、权限撤销、非法 `dt` 走独立既有停机路径。
4. 第一阶段可先关闭 jerk，仅用 `w1=clamp(w0+clamp(a_cmd,a_min,a_max)*dt,w_min,w_max)` 和 `a1=(w1-w0)/dt` 验证。初期优先验证实际斜率、方向和恢复，启用 jerk 后再验收反向和边界。

### 4.3 小 Yaw 行程保护和持续速度预算

机械软边界先用装机测量确定，在现有硬裁剪范围内给两侧各留裕量。把两端和中心统一转换为物理正向坐标后排序，分别计算朝正、负端点剩余行程。上电位置越界时，只允许朝内部恢复。

每侧独立用制动距离判断。令 `d` 为扣去标定、位置误差裕量后的该侧剩余距离，`v` 为朝该侧的非负速度，`L` 为采样到实际产生制动的延迟上界，`A_out` 为延迟期间可能的最坏向外加速度，`A_brake` 为实测保证能达到的最低制动减速度。保守估算：

```text
v_after_delay = v + A_out * L
distance_needed(v) = v*L + 0.5*A_out*L^2 + v_after_delay^2/(2*A_brake)
```

- 根据 `distance_needed(v) <= d` 用单调求根获得每侧安全速度 `v_pos,v_neg`，与小 Yaw 可执行速度上限取交集。若 `distance_needed(0)>d`，该侧向外预算为零。模型参数覆盖电流受限、负载、滤波、CAN 周期和制动建立时间；若 jerk 引入额外刹停时间，应计入 `L` 或改用实测制动包络。按实际运动的联合日志确定安全余量。
- 用新鲜实测小 Yaw 速度判断当前是否已超出制动包络。已超包络时主动向零或内侧制动，按反馈侧位置裁剪/重新播种关节目标，清除继续向外的参考与前馈，避免位置积分仍拉着电机向外跑。内侧回退继续可用。硬软件限位作为最后一层，记录每次触发；制动包络待真机确认。
- 正常协调模式下，把小 Yaw 物理速度区间 `[-v_neg,+v_pos]` 加上实际载体速度，得到当轮枪头可执行速度区间。对载体估计误差和短时减速留余量。用此区间约束枪头轨迹，然后计算小 Yaw 实际残差。靠近边界时大 Yaw 提前加大释放行程方向的回中份额，达到大 Yaw 能力上限后由枪头轨迹降速。大 Yaw 不可用时，只要关节反馈仍新鲜，就继续在本地手动路径使用同一向外行程包络。
- 给持续运动单独设置预算 `steady_head_rate <= verified_big_steady_rate - centering_reserve`。参考底盘初次试车可从 ≤0.8 rad/s 的枪头持续预算和 ≤1.0 rad/s 的大 Yaw 意图起步。高于持续预算的短动作须经过制动包络与可用行程判断。开发初版可统一限制手动速度，先验证低速持续居中。
- 开启 `GimbalAxisConfig::max_lead_rad` 或同等的轨迹目标超前保护时，在新模式专用配置里启用，按跟踪波形标定；旧默认值为 0。第一阶段 `Rate` 路径积累了超前且需要急刹时，可在应用本地复用现有进入 `Hold` 时的重新播种语义，给恢复提供向内速率通路。第三阶段采用显式轨迹目标时，把最终关节位置投影到软边界及允许的实测超前区间；与位置投影方向冲突的速度、加速度前馈同步削减，连续几何变换与投影使用同一参考。

单次起转的理想估算 `delta_s = w_h²/(2*A_b)` 仅适用于枪头速度阶跃、大 Yaw 恒加速且无延迟的模型。例如 3 rad/s 与 12 rad/s² 对应约 21.5°。实际设计按枪头已整形的速度和载体**实测**速度差积分，并纳入起转延迟、jerk、跟踪偏差和反向初速。用日志复算偏角，决定加速度预算与可放开的峰值。

### 4.4 状态转移

| 事件 | 分配器与输出 |
| --- | --- |
| 正常松杆 | 枪头轨迹按限制减速；大 Yaw 继续执行有界回中。到中心带且滤波速度、规划速度与加速度均进入静止阈值后，令规划器收敛并锁存精确零请求。阈值下的归零跳变量须落入单步加速度/jerk 预算，小 Yaw 继续补偿真实载体运动 |
| 正常换向、手动接管视觉 | 保留上一已执行轨迹速度和加速度，通过规划器过渡。新视觉目标需由裁决器重新放行。若必须更换坐标参考，先对齐或从实测位置/速度重新播种 |
| 上电、执行器或对端恢复 | 清理旧轨迹、积分、差分和待发非零值。小 Yaw 用当前新鲜编码器播种，大 Yaw 从当前会话的零请求握手进入有界斜坡。首个有效周期用于建立基准，随后才积分 |
| 云台 IMU 失效 | 撤大 Yaw 请求并清空协同状态。小 Yaw/Pitch 走现有本地手动编码器控制，服从已有遥控、反馈与授权条件 |
| 对端重启或曾在线后失联、遥控超时、人工停车 | 沿用现有撤权、重解锁和停机规则。下一次可发送时优先发布 Disabled，接收端失联靠既有超时处理。安全撤销跳过普通减速斜坡 |
| 错周期、编码器代次变化或反馈失效 | 该周期停止推进分配状态，停止非零大 Yaw 意图。丢弃故障前的位置目标超前量；恢复时重新获取有效反馈和参考。Pitch 独立保有原执行语义 |

参考底盘启用了 DT7 本地后备。测试中把“收到有效 Disabled”和“UART 断开、请求过期后底盘本地接管”分别验证，并记录参考底盘当前 DT7 状态。 [参考底盘授权与后备处理](https://github.com/JJL0612/sentry_onlychassis/blob/dcc8d4e/Skywalker/Application/Src/app_robot.c#L1470-L1538)。

## 5. 配套 Zephyr 底盘的完整控制预算

这一节用于 Zephyr 双板配套阶段。现有接收方无法通过 v4 判断对端是否启用了新分配器，先用明确的构建标定选择配套模式，默认旧配置可继续搭配参考云台。正式 v4 的字段和帧长固定。

### 5.1 航向修正、车体补偿与执行器饱和

1. `BigYawWorldHeadingFollower::step()` 继续拥有世界航向目标。每个合法周期按已接受的世界速度意图推进目标，用现有的真实采样年龄预测当前载体航向。航向误差取最短弧并限制目标超前量。保留会话/零点代次捕获。
2. 先得到世界控制速度候选 `w_b_raw = w_intent + heading_correction + optional_rate_PI`。第一轮维持角速度 PI 为零。按单独的修正项上限及总世界速度包络限制候选，再对**叠加修正后的总世界运动目标**做加速度整形；若需要承诺 jerk 指标，在这一层同样启用 jerk。
3. 用实测 `w_c` 计算可实现的世界速度区间 `[w_c-W_joint, w_c+W_joint]`。先使用标定后的世界运动预算，随后 `w_joint_req=w_b_shaped-w_c` 并执行最后的关节裁剪。若底盘自转本身耗尽关节能力，向最接近可实现的速度饱和并诊断。车体补偿走本地新鲜 gyro，保留原生速度执行器的最后斜率。分别记录世界整形触限、关节限速、原生斜率触限和实际执行滞后。
4. 跟踪饱和时消除航向目标积欠：先计算试推进的航向误差和未限幅输出，若正在受限且目标继续加深同方向误差，丢弃这部分新增积分，并把已有航向误差钳在标定的小超前角内；反向释放允许正常推进。若以后启用速度 PI，积分须使用最终物理限幅/斜率信息做条件积分或回算。若需用上一周期执行器信息，将饱和与命令的符号、代次对应，防止换向后的陈旧饱和状态反向冻结。
5. 原生速度斜率为电机轴量，关节能力约束为 `A_native/ratio`。理论上 `a_joint=a_b-a_c`，因此世界预算必须给实测车体加速度留空间。快速小陀螺启停/反向时，记录世界角速度误差与关节饱和；有必要时下调协同可用运动预算。关节命令的最终裁剪优先于世界轨迹平滑指标。

对应文件：[航向跟随器](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/lib/robotics/big_yaw_heading_follower.cpp#L31-L101) · [原生速度后端](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/lib/robotics/big_yaw_executor.cpp#L121-L147) · [底盘标定接口](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/include/robotics/vehicle/big_yaw_profile.hpp#L44-L71)。

### 5.2 正常停车的航向捕获

配套模式可在接收端用停留时间推断停车，启动建议按 10 ms 请求周期试验 50–80 ms。接收新鲜连续的低速意图进入停车候选，持续低于进入阈值后捕获一次实测航向，清除旧修正积分。期间若速度重新高于退出阈值，取消候选。保持状态继续稳定在已捕获世界航向，并做车体补偿。计时以有效会话内的真实时间为准，候选要求期间收到多帧不同的请求序号 `BigYawRequest.stamp.sequence`（payload +32）。遥控源 `source_sequence` 可被多帧请求复用。重复读缓存不充当新帧。普通零点穿越应在计时完成前退出。捕获前后的速度仍经世界整形，避免修正项跳变直接进入电机。

这是 v4 速度字段上的停车推断，慢速经过零带也可能被识别为停车。把该情形列为回归用例。若台架证明推断无法满足实车低速操作，再另立任务讨论双方显式停车意图及兼容协商。第一版保持 mode 0 的停机语义和 mode 1 的世界速度语义。

## 6. 小 Yaw 轨迹接口与高频自瞄

### 6.1 先建立可测量的执行链

建议在 `GimbalExecutionInputs` 给小 Yaw 添加可选的已校验关节轨迹输入，经过 `GimbalAxis` 和 `PositionMotor` 单一执行链，复用权限、反馈代次、位置限位、CAN 发布和现有 PID。先用模拟轨迹验证。轨迹包含有限值 `position_rad, rate_rad_s, acceleration_rad_s2`、有效字段、时间戳，明确其为编码器坐标。只在当轮轨迹有效并通过既有门控时消费。旧 `Rate/Hold/AbsoluteAngle` 调用继续采用原输入含义。

- **位置和速度前馈。** 以受限位置轨迹为小 Yaw 目标。位置环输出作为速度修正，把关节参考速度加到速度请求，合成值执行速度上限和斜率管理。参考位置、速度、加速度使用同一可行轨迹，避免独立限幅后互相打架。现有速度参考斜率为 20 rad/s²，若已规划的轨迹仍被内环二次削斜，先从遥测定位并用同一验证预算协调两层，最后一道驱动限制继续生效。位置、速度环的反馈增益先沿用可运行值。
- **可选加速度努力前馈。** 预留 `i_ff = K_a * a_s_ref`，`K_a` 单位 A/(rad/s²)，缺标定默认 0。经台架得到同方向电流响应和限幅余量后再启用。把有界 `i_ff` 和速度环输出合成后统一限流，饱和时速度环积分需感知合成输出，使用现有抗饱和接口或按实际输出回算。当前控制链可从 [`motor_position.c`](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/lib/control/motor_position.c#L54-L104) 和 [`PositionMotor`](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/include/control/position_motor.hpp) 开始追踪。
- **载体加速度估计。** 只在两个真实的新 gyro 样本之间用原始时间差求导，低通后限幅。重复读到同一序号就保持滤波值及其原始时间戳；新样本断档、代次变化、异常间隔或样本过期时清空导数历史，禁用加速度前馈，速度补偿按其独立有效性决定。用时间对齐后的 `a_s_ref = a_h_plan - a_b_meas_filtered`，并应用小 Yaw 的实测加速度边界。估计有效时，将 `[a_b_est - A_s_neg, a_b_est + A_s_pos]` 与枪头加速度区间求交，作为当轮枪头规划器的加速度边界。两侧余量要扣除载体估计误差，并随制动包络收紧。这样直接约束可执行的枪头斜率，速度、位置、加速度参考可以从同一受限轨迹派生。若区间突变或为空，按第 4.2 节冲突路径优先执行小 Yaw 制动。高频噪声若影响电流，先通过输出日志确认测量带宽和延迟。

### 6.2 视觉接入

1. 从裁决通过的 `frame.decision.selected_vision` 取得已限幅的 `theta_v,w_v,a_v`，且要求最终 `gimbal.source==Vision` 与对应参考、时间戳匹配。复用 `CommandArbiter` 的合法源、模式、手动覆盖、新命令序号和超时判定。依据真实采样年龄有界预测，是否使用 `theta+w*t+0.5*a*t²` 取决于上位机发送的是采样时刻轨迹还是已前推目标，先核对时间语义。沿同一视觉参考追踪角度，过 ±π 用最短弧。
2. 用枪头规划角 `theta_h_plan` 与已对齐的视觉目标角误差生成有界速度修正，例如 `w_v+K_theta*wrap(theta_v-theta_h_plan)`。规划器以该速度为目标，使用视觉 `a_v` 作为有界前馈，再经过第 4 节的枪头加速度、jerk 和行程约束。积分最终可行的速度得到 `theta_h_plan`，避免积累不可执行的位置目标。
3. 用已对齐的实测载体角度解算小 Yaw 关节位置，选当前有限行程内最近的角度分支；速度和加速度使用载体实测残差。若位置投影、速度投影改变了动作，重新生成一致的三元参考并压掉向限位外的前馈。若复用 `InertialGimbalAdapter` 的参考系或姿态检查，明确补偿只由一处执行，避免把其已经输出的关节速度再减一遍载体速度。
4. 视觉外置 IMU/CAN3 启动、两套航向参考对齐和接管、AB 上行字段及串口分配按已有 [视觉姿态回传与外置 IMU 接管方案](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/docs/dev/云台视觉姿态回传与外置IMU接管方案.md) 单独验证。视觉失联、越时或参考变化时，立即停止消费旧目标和前馈，按裁决进入手动或本地 Hold；进入 Hold 时从当下反馈播种，避免继续向旧视觉绝对角运动。手动在缺少外置 IMU 时可继续运行。

## 7. 实施顺序与交付

| 阶段 | 文件和动作 | 完成条件 |
| --- | --- | --- |
| A．手动加速度分配 | 新增分配器和标定，接 `sentry_gimbal` 的手动 Rate 入口。保留现有编码器闭环和 v4 编码。先运行参考底盘低速配置 | 正反向都能主动跟转并居中；软行程能提前降速；松杆/换向/断流有日志。固件默认配置与实车匹配 |
| B．配套 Zephyr 底盘 | 调整 `big_yaw_heading_follower.cpp/.hpp`、`sentry_chassis/src/main.cpp` 和 `big_yaw_profile.hpp`。必要时由 `big_yaw_executor` 报告原生斜率触限。先处理捕获和世界/关节预算，再提高持续速度 | 记录双板签名与标定。按 1→2→3 rad/s 逐档验证，已过档位才开放；同步测试底盘小陀螺启停、反向和饱和释放 |
| C．小 Yaw 轨迹与模拟视觉 | 扩展 `GimbalExecutionInputs → GimbalAxis → PositionMotor`，先开位置/速度前馈，验证后再用实测标定开加速度努力前馈 | 离线或台架正弦/扫频能比较大小 Yaw 分工，日志能从可行轨迹追到速度环与电流 |
| D．真实视觉 | 接入被裁决的 `selected_vision`、对齐的姿态和参考代次；使用独立的视觉姿态链 | 视觉断流、参考切换、手动覆盖均能退出旧目标。确认高频响应、剩余行程与热/电流指标后启用自动 |

起始标定建议集中在正式应用的标定或配置文件。优先实测大、小 Yaw 的加载起转加速度、正反向刹停能力、速度误差和执行延迟。参考底盘的大 Yaw 起步可沿用 4 rad/s² 的请求斜率作为首轮对照；配套 Zephyr 底盘实测有余量后，可按 8→12→16 rad/s² 逐级试大 Yaw **世界规划加速度**，同时考虑自转补偿。小 Yaw 的高频优势由带负载的加速度和带宽实测确认。中心死区、前馈和增益逐项调参，留下每次改变后的对照波形。

## 8. 测试和真机日志

### 8.1 自动回归

在现有 `tests/sentry_control` 加分配器、Zephyr 航向和原生速度链的回归。涉及视觉裁决或 v4 才改对应的 `tests/command_manager`、`tests/protocols` 等现有套件。

| 场景 | 可断言结果 |
| --- | --- |
| 稳态匀速、松杆 | 预算内持续转向时 `w_b` 跟上 `w_h`，中心误差收敛；松杆后经可控减速进入保持 |
| 正负符号与车体自转 | 小 Yaw 补偿实测载体；关节映射仅应用一次方向和传动比；大 Yaw 世界意图与小陀螺体速正确相减 |
| 加速度、jerk、可行边界 | 验证实际写回的轨迹状态；覆盖两侧限速、改变 `dt`、反转、边界收缩、已越界、不可行区间与恢复 |
| 行程不足 | 双侧接近限位、不同初始速度、载体低于预期；比较预测制动距离，限制向外并允许回退；释放后无旧目标补跑 |
| 航向捕获与饱和 | 穿越零点、低速穿越、持续零速、Boot ID/参考代次变化；启停小陀螺时记录世界/关节/原生斜率各层触限，解除饱和后无追赶旧航向 |
| 故障与恢复 | 重复时间戳、未来值/超期、超长周期、反馈丢失、对端重启、Yaw 失败而 Pitch Active、遥控撤权。停机优先于轨迹斜坡 |
| 视觉路径 | 角度过 ±π、有限值与参考代次检查、高频交变、缺少新样本时导数失效、前馈限流/积分、手动接管 |

从仓库目录编译双板，从 west 工作区根执行 Twister。构建目录按本地工作区实际布局调整。

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d ../build/applications/sentry_gimbal
west build -b dm_mc02/stm32h723xx applications/sentry_chassis -d ../build/applications/sentry_chassis
```

```sh
west twister -T skywalker_code/tests -p native_sim/native/64 --inline-logs --jobs 2 -O build/unit-tests
```

参考底盘走其现有工程和固件。第一阶段仅烧云台。 [项目测试说明](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/tests/README.md) · [参考底盘兼容配置](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/applications/sentry_gimbal/README.md)。

### 8.2 台架顺序与记录

1. 校准、静置并按现有流程解锁，确认 gyro 有效。先单轴低速正反向，核对编码器中心、安装方向、停机和反馈丢失处理。
2. 底盘静止做匀速、阶跃、松杆和换向。比较枪头原始/可行速度、大 Yaw 世界请求与 gyro 实测、小 Yaw 偏角与实测速度，记录最大偏角、回到 ±5°所需时间、稳态中心误差与实际加速度。
3. 在两侧安全余量内从不同偏角与速度接近边界，测制动距离与延迟，确认向内仍可恢复。再叠加小陀螺启停和不同方向，量化剩余关节速度/加速度预算。
4. 用不含发射的模拟视觉波形先做小幅低频，再逐步提高频率和加速度，比较低频与高频下两轴速度/加速度幅值、枪头角度误差、相位滞后、峰值电流和行程裕量。最后接真实视觉进行断流和手动接管验收。

现有 VOFA 的云台 I17 为物理中心偏角、I18 为载体 gyro、I38 为发送意图、I39 为对端 armed；Zephyr 底盘 I26 为收到的世界意图、I28 为航向误差、I29 为斜率后的关节目标、I9 为关节实际速度。两板同步记录，静止底盘时可直接比较世界和关节目标趋势；自转时先将关节速度加车体 gyro 转回世界坐标。新增诊断最少包含枪头原始/可行速度、大/小 Yaw 目标加速度、小 Yaw 实测速度、两侧剩余行程或制动裕量、实际请求时刻/序号和触限原因；更新通道表及 schema 标识，保留旧日志可解析。 [VOFA 通道表](https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/4dc29fb/docs/dev/大Yaw突发跟转诊断操作指南.md)。

本文交付基于两份源码的静态复核。各档真机加速度、制动余量、载体跟踪带宽及 3 rad/s 持续能力由实施后的台架数据确认。
