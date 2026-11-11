# SkyWalker 底盘前进方向偏角与小陀螺漂移：执行模型修复任务书

## 0. 执行范围与工作原则

**主仓库**：https://github.com/SHM-white/SkyWalker_General_Embedded_Code ；以 `dev` 分支为基线。分析时基线为 `dc859d00681421d54a36c8dbd9cfa1b1483071e6`（2026-11-11），执行前先重新拉取最新 `dev`，确认实际文件与逻辑，不能把行号或旧 commit 当作当前事实。

**参考仓库**：https://github.com/JJL0612/sentry_onlychassis ，尤其是 `Skywalker/Application/Src/app_robot.c`、`docs/大Yaw底盘陀螺仪补偿.md`、`tests/test_gimbal_translation.py`、`tests/test_big_yaw_gyro.py`。参考仓库的设计可以借鉴，但文档/模拟测试通过不等于实车确认，不要整段复制 C/FreeRTOS 实现。

**目标**：
1. 使遥控杆正前方的平移指令在大 Yaw 正前方坐标下保持正确，初始固定偏角可通过真实标定校正。
2. 小陀螺持续旋转、多圈、跨 DM-J4310 原始角量程边界后，平移方向不产生跳变、累计错误或软件坐标漂移。
3. 大 Yaw 在底盘旋转时尽量保持世界航向，补齐参考工程已有、当前 dev 缺少的航向 P + 角速度 PI；保留现有原生速度控制模式。
4. 不破坏双主控 UART v4 协议、四舵轮运动学、原有 motor API 语义及已可工作的手动控制通路。

**范围约束**：这是一项定向修 bug，不允许借机大规模重构架构，不允许新添多个“确认开关”、运行许可、回退层、静默替代坐标系或与问题无关的门控；已有必要的无效/过期数据处理予以复用。保留电机协议控制模式、CAN ID、轮组安装方向、硬件功率参数；不写入 J4310 驱动器零位。不修改任何已知零位为臆测值。先自测再交 PR，**不要直接合并 dev/main**。

## 1. 已定位的当前实现与问题

### 1.1 编码器平移坐标：确定的逻辑缺口

- `applications/sentry_chassis/src/main.cpp` 约第 125–136 行，取 `big_drive.snapshot().native_position_rad` 后调用 `referenceV4ChassisToBody()`。
- `include/robotics/chassis/reference_v4_motion.hpp` 约第 10–35 行，直接以 `theta = remainder(direction*(raw-forward_zero)/ratio, 2*pi)` 旋转云台坐标向量。
- `drivers/motor/motor.cpp` 中 `Motor::acceptDmFeedback()` 约第 687 行起，**已经存在**对 DM 反馈位置增量的 `±2*position_max_rad` 解回绕逻辑，但形成的是第一帧起点为 0 的 `feedback.position_rad`，且有 `position_reference_valid` / gap 语义；该值**不是**带电机原始编码器固定零位的绝对连续位置，不能简单替换 `native_position_rad`。
- J4310 配置 `position_max_rad=12.5`，原始反馈在 `[-12.5,+12.5]`，物理转过量程边界时原始值跳 `25 rad`，**25 不是 `2*pi` 的整数倍**，若直接使用 raw 会在每次跨界产生约 `|25-4*2*pi|=0.132741 rad ≈ 7.61°` 的方向跳变。多次同向跨界的误差可累积。这与缓慢、连续的方向漂移须分别诊断，不能混为一谈。

### 1.2 当前大 Yaw 世界航向没有完整闭环

- `include/robotics/chassis/reference_v4_motion.hpp` 中 `referenceV4WorldToJointRate()` 当前只有 `desired_world_rate - body_rate + 0.25*(desired_world_rate-head_rate)`；**没有**世界航向角 P 项与角速度积分项。
- `applications/sentry_chassis/src/main.cpp` 中 `BigYawMode::FollowCenter` 负责由目标世界角速度得到关节速度。
- `applications/sentry_gimbal/src/main.cpp` 中 `YawCenteringController`（小 Yaw 回中）生成 `BigYawRequest::target_rate_rad_s`，同时把云台板 BMI088 的 yaw/rate 发送给底盘。不要误把这个 `target_rate` 当成固定的摇杆直接速度；它承载云台回中/世界角速度意图。
- `lib/robotics/big_yaw_executor.cpp` 中原生速度分支已按 `target_joint_rate * motor_to_joint_ratio * direction` 映射到电机目标速度，并有限加速度。**补偿函数输出保持物理关节 rad/s**，禁止重复乘安装符号或传动比。
- `include/robotics/vehicle/motion_profile.hpp` 当前 `big_yaw_backend = BigYawBackend::NativeVelocity`；`ChassisMotionAdapter` 配置 `Frame::Body` 且 `compensation_enabled=false`。也就是说平移方向应由固定编码器参考决定，不能再额外叠加两颗六轴 IMU 的启动航向差或二次旋转。

### 1.3 零位来自参考车历史标定，未必仍适用

`include/robotics/vehicle/calibration.hpp`：

```cpp
inline constexpr MotorCalibration reference_big_yaw{6, 0x11, 1, -1, 10.0f, 10.0f, 0};
inline constexpr float reference_big_yaw_forward_zero_rad = -1.132792f;
```

`-1.132792f` 是参考工程 2026-10-28 在对齐车头后读取的原始反馈，不是此次现场测量。**先保留该值**，添加明确的现场复测说明；如需修正，由用户/现场人员在已知编码器分支、前方物理对齐后提供实际读数。不要把首次上电方向、遥控开始运行方向、两颗 IMU yaw 初值自动设为“车头零位”。

## 2. 第一阶段：修复 DM 原始编码器的连续绝对坐标

### 2.1 最小化数据设计

优先在 `Motor::DmRuntime` 增加与现有相对 `feedback.position_rad` **独立**的固定电机坐标跟踪：

```cpp
// 示意命名：可按项目风格优化
bool native_unwrap_initialized = false;
float last_native_raw_rad = 0;
std::int64_t native_wrap_count = 0;  // 或使用有明确越界保护的等效表示
// Snapshot 中另行暴露：
double native_unwrapped_position_rad = 0; // 电机原始坐标延拓：第一帧为原始角，不是 0
auto native_unwrapped_position_valid = false;
```

也可以从已有累计角和首次原始角推导，但**必须**严谨核对 `gap`、`reseedPosition()` 和 `position_reference_valid`：不允许直接复用一个可能以 0 起步、被用户重定基准或在反馈 gap 后失效的值。尽量保持原 `feedback.position_rad`、`reseedPosition`、Motor API 既有意义不变。

### 2.2 核心更新伪代码

```cpp
on_valid_fresh_dm_frame(raw, now):
    span = 2.0 * config.limits.position_max_rad;  // J4310 当前 25 rad
    if !native_unwrap_initialized:
        native_wrap_count = 0
        last_native_raw_rad = raw
        native_unwrap_initialized = true
    else:
        delta = double(raw) - double(last_native_raw_rad)
        if delta < -span/2: ++native_wrap_count  // +12.5 -> -12.5
        if delta > +span/2: --native_wrap_count  // -12.5 -> +12.5
        last_native_raw_rad = raw
    unwrapped = double(raw) + double(native_wrap_count) * span
    publish_snapshot(raw, unwrapped, valid=true) // 与 Motor 原有 snapshot 同锁提交
```

**注意边界**：
- 仅在已被现有 Motor 接收逻辑认可的有效、按序的新帧上更新；无效/旧序列/NaN/无穷/解码失败不得改变历史角。
- 双精度计算多圈累计与 `2*pi` 归一，最后将小范围 `theta` 转 `float`，避免长时间多圈运转时 `float` 累计精度下降。
- 停用/重使能不清零 `wrap_count`；真实板卡/电机重新启动、角度零点重置、很长时间无反馈可能造成无法判明跨越了多少个 25rad 周期，此时不能声称历史相位仍可靠。复用现有 stale/feedback 处理，定义清楚恢复策略并暴露最小诊断状态，不额外制造多个永久锁死状态。恢复若不可判明，明确提示需人工重新对齐/归零；不要悄悄捕获当前位置为零。
- 如 `decoded.position_rad` 的协议量程/电机实际参数与软件配置不一致，先确认硬件配置再继续；不要为了让测试通过调整 `position_max_rad`。

### 2.3 改造平移入口

`applications/sentry_chassis/src/main.cpp`：将传入 `referenceV4ChassisToBody` 的 `native_position_rad` 替换为新 `native_unwrapped_position_rad` 与匹配的有效位/反馈新鲜度。保持其余结构与调用顺序，不把云台/底盘 IMU 航向差重新用于平移。

`include/robotics/chassis/reference_v4_motion.hpp`：如需将角度输入从 `float` 升到 `double`，只做必要 API 调整（测试一起跟进）。

```cpp
const double mechanical_angle = direction * (absolute_native_unwrapped - fixed_raw_zero) / ratio;
const float theta = static_cast<float>(std::remainder(mechanical_angle, 2.0 * std::numbers::pi_v<double>));
vx_body = (cos(theta)*vx_gimbal - sin(theta)*vy_gimbal) * original_scale;
vy_body = (sin(theta)*vx_gimbal + cos(theta)*vy_gimbal) * original_scale;
```

如现有 C++ 标准不支持 `std::numbers`，使用仓库现有 π 常量即可，不因此调整编译标准。维持 `+X 前进、+Y 左、俯视逆时针为正` 的定义，保持小陀螺 `forward_scale=0.3`、`lateral_scale=0.4` 仅改变速度大小、不改变方向。旋转命令 `wz` 及 `SwerveKinematics` 不应被重复坐标转换。

### 2.4 现场零位标定（不自动瞎改）

- 云台物理正前方与车体正前方严格对齐；在已知编码器原始位置分支下读取 5–10 次稳定原始角 `raw`、反馈时间和错误码。
- 如果它与 `-1.132792` 不同，由现场记录确定要否更新 `reference_big_yaw_forward_zero_rad`；保留旧值和新的记录、单位、硬件零点状态。
- 静态旋转大 Yaw 相对底盘 +90°：正前推杆应转成车体 `+Y` 平移；-90° 应为 `-Y`（如果物理正方向约定不同，要先核验并统一 `direction` 的意义，而不是通过额外负号补偿）。
- 如果不能现场读取/确认，执行模型只能提供仪表/串口观测方式与操作步骤，不能宣称已标定或自行编造新零位。

## 3. 第二阶段：恢复大 Yaw 世界航向闭环（修复连续漂移）

**不要**直接将当前无状态的 `referenceV4WorldToJointRate()` 增添静态全局积分变量；推荐一个职责单一、可单测的 `BigYawWorldHeadingFollower`（或同等小类），状态由 `sentry_chassis` 持有，入口复用现有 `FollowCenter`。逻辑源于参考仓库 `RobotBigYawStabilizedRate()`，但按照当前 `BigYawExecutor` 的物理速度接口实现。

### 3.1 输入与状态

输入：`desired_world_rate_rad_s`（云台回中发来的目标世界角速度）、有效 `body_rate_rad_s`、`head_yaw_rad`、`head_rate_rad_s`、云台头部 IMU 时间戳/原始样本年龄、Boot ID、零位代次、当前本地时钟以及 `dt`。核实两颗 IMU 的坐标轴映射，当前底盘板输入直接用 `gyro.z`，云台角速度来自 `ImuHeadingReference` Euler 变化率；先保证同为物理逆时针正、单位 rad/s。

状态：`initialized`, `target_heading`, `previous_requested_rate`, `rate_integral`, `last_update_time`, `last_peer_boot_id`, `last_zero_generation`。不要重定义已有 UART v4 消息，不加另一套运行许可。

### 3.2 控制伪代码（与参考工程对应）

```cpp
step(input):
    // 继续复用已有有效性/50ms时效约束以及发送方Boot ID/接收方Boot ID/zero_generation
    // 若失效：停止该次执行、清目标和积分（按现有停机语义），不要继续积分旧yaw
    dt = measured_control_cycle_dt; // 正常约 2ms, 不对重复/倒退时间积分
    heading = wrapPi(head_yaw + head_rate * bounded_sample_age_s);
    if first_valid_enable || imu_boot_or_zero_generation_changed:
        target_heading = heading;
        rate_integral = 0;
        previous_requested_rate = 0;
    if became_zero(previous_requested_rate, desired_world_rate):
        target_heading = heading;     // 参考工程：松手后捕获当时航向
        rate_integral = 0;

    heading_error = clamp(wrapPi(target_heading + desired_world_rate*dt - heading), -PI/2, +PI/2);
    target_heading = wrapPi(heading + heading_error); // 控制目标最多超前90度
    world_rate = desired_world_rate + clamp(K_heading*heading_error, -1.5, 1.5);
    rate_error = world_rate - head_rate;
    feedforward = world_rate - body_rate;

    candidate_integral = rate_integral + Ki_rate * rate_error * dt;
    candidate_joint_rate = feedforward + Kp_rate * rate_error + candidate_integral;
    // 含feedforward的抗饱和：如果达到输出极限，只允许积分朝脱离饱和方向发展
    if !saturated(candidate_joint_rate) || unwinding(rate_error, candidate_joint_rate):
        rate_integral = clamp(candidate_integral, integration_bounds);
    joint_rate = clamp(feedforward + Kp_rate*rate_error + rate_integral, -JOINT_MAX, +JOINT_MAX);
    previous_requested_rate = desired_world_rate;
    return joint_rate; // 物理关节速度；不要在此乘direction/gear_ratio
```

**初始参数**：`K_heading=2.0`，航向修正限幅 `±1.5 rad/s`，`Kp_rate=0.25`，`Ki_rate=0.5`，用作参考起点，不宣称这些在新硬件上已经调通。输出约束由当前 `big_yaw_config.rate_abs_max_rad_s` 和 native acceleration 负责；对实际角速度饱和/斜率导致的积分累积应当采取抗饱和处理。若 `desired_world_rate` 来自回中死区附近，使用现有合适的零判断/滞回，避免每轮因浮点微噪声反复“松手捕获”。

### 3.3 接入细节

- `applications/sentry_chassis/src/main.cpp`：仅在现有 `BigYawMode::FollowCenter && !big_yaw_heading_lock_enabled` 分支内调用新 follower；与现有 `big_axis.update()` 的 enable/stop 生命周期一致；反馈中断/对端重启/零位代次切换后重新捕获。
- 必须检查 `rx.gimbal_imu` 的真实新鲜度、sender/receiver Boot ID 和 zero generation，**不能仅依据 `rx.gimbal_imu.valid` 位**；复用 `forwardedFresh` 和已有 heartbeat/session 逻辑，不额外增加无关门控。
- `big_axis` 电机 NativeVelocity 分支负责 `joint_rate * ratio * direction`、电机限幅/缓启动；新 follower 不直接发送 CAN 帧，不改变电机 CTRL_MODE=3，不把 MIT 航向锁定模式打开，也不同时叠加 `ImuHeadingLock`。
- 不调整小 Yaw `YawCenteringController` 的 kp、死区或四轮减速倍率来掩盖大 Yaw 朝向漂移；保留其语义，并用测试验证目标世界角速度的符号与零速航向保持。
- 区分**车体与云台之间的相对角**（大 Yaw 编码器控制平移参考）和**云台相对于世界的估计航向**（BMI088 负责航向保持）。不要互相替代。6轴 BMI088 的长期绝对 yaw 漂移没有外部约束无法根除；不得承诺永久无漂移。

## 4. 必需自动化回归测试

建议放到 `tests/sentry_control`，可按编译依赖增加轻量测试文件/测试 target，不复制另一套算法作为断言对象；至少调用真实业务函数或真实 Motor 接收/发布逻辑。参考 `JJL0612/sentry_onlychassis/tests/test_gimbal_translation.py`、`tests/test_big_yaw_gyro.py`。

**A. 编码器与坐标**：
1. 固定零位时 `raw=zero`，前推 `(+vx,0)`，输出应为 `(+vx,0)`（不 spin）；相对 ±π/2 时输出前/后/左/右符合符号约定；四舵轮收到的仍是 Body 坐标。
2. 真实解码 `+12.49 -> -12.49` 和 `-12.49 -> +12.49`，连贯的展开角应无 25rad 跳变；对应 `theta` 连续。模拟正反方向多次回绕/360°/720°/十几圈运行，避免每圈偏移。明确对比旧算法 7.61° 跳变与新算法不跳变。
3. `raw` 反馈非法、旧 sequence、NaN、过期、短断流/恢复：按现有保护处理，不破坏累计坐标、不重复积分跨界；长断流无法确认分支时不默默假设历史完整。
4. 停机/重新使能、`reseedPosition()`、电机 snapshot 接收函数回归；新的连续**原始**坐标不受原 `feedback.position_rad` 置零语义干扰；`native_position_rad` 仍保留原本 raw 含义。
5. `spin_forward_scale=.3`, `spin_lateral_scale=.4` 和纯旋转 `wz` 不被二次缩放；原有轮组输出、转向符号不变。

**B. 大 Yaw 航向闭环**：
1. 零速/正速/反速/正负底盘角速度的前馈、角度误差修正方向符合实际物理号。
2. 模拟 `body_rate` 有 0.03rad/s 偏置、速度增益误差、20ms 电机延迟，验证含积分的控制量可抵抗稳态误差，且无发散、明显震荡或输出方向翻转（模拟只作回归，不声称真实效果）。
3. IMU `±π` 角度跨界、连续旋转、云台回中正负切换、命令从非零到零捕获当前航向，不发生追赶旧目标。
4. 丢 IMU、时戳老化、重复样本、云台板重启、零位代次变化、使能状态变化时清理状态；恢复后以**当前**测量朝向重新捕获，不能沿旧零位快速转动。
5. 电机最后收到的正负号与单位转换**只做一次**；原生速度帧格式、命令限幅、native acceleration、原来与 MIT 后端互斥的逻辑不回归。

**C. 构建与现有测试**：

```bash
west build -b native_sim/native/64 tests/sentry_control -d build/test-sentry-direction --pristine
west build -b dm_mc02/stm32h723xx applications/sentry_chassis -d build/sentry-chassis --pristine
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d build/sentry-gimbal --pristine
```

对涉及的现有 motor regression、motor protocols 和 interboard 测试也要按仓库配置运行；若环境缺失明确报告未运行的具体命令和原因，不写“测试通过”。尽量先覆盖真实物理坐标变换的测试，避免只有编译检查。

## 5. 实车分步验收（执行模型只提供步骤，不能虚构结果）

1. **架空/低速，先禁止小陀螺**：校准真实正前方，正前推杆、左推杆；观察大 Yaw 原始编码器、解回绕位置、`theta`、四舵轮目标及运动方向；再人工相对转动 ±90° 测转向符号。
2. **架空旋转/小速度小陀螺**：在安全环境中持续旋转直至跨原始 `±12.5rad` 反馈边界，记录原始/连续编码器及前进方向是否跳变。观察日志变化是突然 ~7.6° 还是连续逐渐偏移。
3. **固定外部标记观察大 Yaw**：分别记录「大 Yaw 相对底盘角」「大 Yaw 相对地面标记朝向」和推杆后的实际移动方向；若真实大 Yaw 自身在世界中漂移，不能把它误诊成坐标转换漂移。
4. **启用航向闭环**：先低角速度正反旋，记录 `head_yaw, body_rate, head_rate, target_heading, heading_error, joint_rate, motor_feedback_rate`，看航向误差是否收敛。先观察再调 Kp/Ki；别一次调整多个旋钮。
5. **长时间回归/恢复**：旋转停止、切模式、断链/重连、重新使能、两板分别重启。确认重捕获逻辑不会带来意外旋转，原先手动控制不受影响。

## 6. 交付要求

1. 在最新 `dev` 上新建独立修复分支，尽量拆成有意义的 2–3 个 commit：`encoder-frame-unwrapping` / `big-yaw-heading-follow` / `regression-docs`。允许重新命名，但**不要每提交一步就 push**；本地自测全部完成后统一推送。
2. 开 PR 指向 `dev`，**不要自动合并**，在描述中列出变更路径、为什么这样改、风险、有效和未完成的测试、实车需要测哪些数据。
3. 在仓库 docs/dev 下补一份简洁的标定与排障文档：车头零位测法、raw 与 unwrapped 的语义、25rad 不等于 2π 的陷阱、航向闭环参数与图示、6轴 IMU 长期 yaw 漂移的边界；附 GitHub/VS Code 可定位的源码相对链接。
4. 最终总结按四段：**改了什么 / 为什么 / 影响 / 注意事项**，列出具体代码路径、测试命令及结果。若发现问题与上面推断不一致，以实读代码与测试证据为准，报告差异，做最小正确修改，避免生搬硬套方案。

**参考入口**：
- https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dev/applications/sentry_chassis/src/main.cpp
- https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dev/include/robotics/chassis/reference_v4_motion.hpp
- https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dev/drivers/motor/motor.cpp
- https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dev/include/drivers/motor/motor.hpp
- https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dev/include/robotics/vehicle/calibration.hpp
- https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dev/lib/robotics/big_yaw_executor.cpp
- https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/dev/applications/sentry_gimbal/src/main.cpp
- https://github.com/JJL0612/sentry_onlychassis/blob/main/Skywalker/Application/Src/app_robot.c
- https://github.com/JJL0612/sentry_onlychassis/blob/main/docs/大Yaw底盘陀螺仪补偿.md
