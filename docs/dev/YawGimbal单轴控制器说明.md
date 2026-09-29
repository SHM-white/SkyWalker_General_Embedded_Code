# YawGimbal 单轴控制器说明

> 历史源码解读：Axis 与 YawGimbal 现已合并为 GimbalAxis。当前接口与验证记录见 [GimbalAxis 单轴封装实施记录](GimbalAxis单轴封装实施记录.md)。

依据 `include/robotics/gimbal/yaw_gimbal.hpp` 与 `lib/robotics/yaw_gimbal.cpp` 的静态阅读。本任务只解释现有代码，无业务修改、构建或硬件验证。

## 核心分工

`YawGimbal` 将云台运动命令转换为每个控制周期的目标角度，再交给 `PositionMotor::update(target_angle_rad_, dt)` 执行位置控制。它保存目标角度，处理运动模式、目标变化速率、角度边界与部分状态检查。

调用关系：样例 `Axis` → `YawGimbal` → `PositionMotor` → 电机输出。`Axis` 聚合对象并维护样例的就绪流程；`YawGimbal` 决定本轴目标角度；`PositionMotor` 执行位置控制。CAN 输出提交仍由样例主循环调用。

## 运动模式

- `Rate`：把命令角速度限制在配置的最大角速度内，再计算 `目标角度 += 角速度 × dt`。例如目标为 1 rad、角速度为 0.5 rad/s、周期为 0.005 s，则下一目标为 1.0025 rad。
- `AbsoluteAngle`：向指定角度逐步移动目标，每周期目标角度变化不超过 `max_rate_rad_s × dt`。连续转轴按最短角度差更新目标；有限转角轴先限制命令角度。
- `Hold`：刚进入保持状态时以实际反馈角度建立目标，此后维持该目标。
- `Disabled`：返回 `-EACCES`，不会在此函数内直接调用电机禁用接口。

默认 `hold_on_zero_rate = true`，零角速度保持原目标；设为 `false` 时，零角速度会用当前反馈重新设置目标，不等于关闭电机输出。

## 配置和状态

`Continuous` 用于连续转动，要求 `PositionMotor` 使用 `AbsoluteNearest` 参考方式，并将目标角度按一圈取余。`Limited` 用于有限转角，要求 `DriverContinuous`，限制目标角度；实际角度超限时返回 `-ERANGE`。

`target_angle_rad_` 是目标角度，不是实时测量角度；`targetAngleRad()` 返回该目标。`previous_action_`、`previous_mode_` 用于识别进入保持状态；`initialized_`、`generation_` 用于首次更新或电机重新使能后，从反馈重新建立目标。

`drive_` 与 `axis_` 均为引用，指向外部创建的电机与位置控制对象，调用期间这些对象必须存活。`config_` 保存配置副本。该类自身没有额外同步，应由外部保证调用顺序。

## 接口与边界

- `validate()`：检查配置与位置参考方式匹配，成功返回 0，错误返回 `-EINVAL` 或 `-ENOTSUP`。
- `begin()`：验证配置后调用位置控制器 `configure()`；头文件要求在 `CanBus::start()` 后调用。
- `reset()`：重置位置控制器并从新鲜反馈建立目标，不使能电机。电机处于 Active 或 Enabling 时返回 `-EBUSY`。
- `update(command, action, dt)`：处理命令和安全动作，更新目标并调用位置控制器，返回其结果或本层错误。周期单位为秒，须满足 `0 < dt <= 0.02`；角度单位为 rad，角速度单位为 rad/s。
- `seed(snapshot)`：内部辅助函数，用反馈设置目标并记录使能代次。反馈过期返回 `-EAGAIN`，所需位置数据或参考缺失返回 `-ENODATA`。

`update()` 要求电机 Active 且允许输出；禁用状态返回 `-EACCES`，无效参数返回 `-EINVAL`。本层不检查遥控时间戳，也不代替外部急停、超时和组合启停逻辑。

## 与 pitch 的关系

样例给 yaw 和 pitch 分别创建一个 `YawGimbal`。虽然类名和命令字段使用 yaw 命名，它们在此处承载各自电机的单轴控制，两者都读取 `yaw_rate_rad_s` / `yaw_target_rad`。

## 阅读自检

- 能区分目标角度和实际反馈角度。
- 能用角速度乘周期算出下一周期的目标增量。
- 能区分保持原目标、进入 Hold 时锁定当前反馈、禁用电机三个动作。
- 能说明 `YawGimbal` 的目标生成与 `PositionMotor` 的位置控制分工。

上述内容未验证实机跟踪性能；目标变化速率限制不代表实际机械速度的硬性保证。本说明无实施步骤或上电操作，业务源码未修改。
