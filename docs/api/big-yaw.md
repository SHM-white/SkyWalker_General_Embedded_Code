# 双 Yaw 分配、大 Yaw 回中与独立速度环：接口与调用参考

源码核对基线：`dd3f673`（2026-11-15）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

正式云台由 `DualYawMotionAllocator` 生成小 Yaw 关节轨迹与大 Yaw 世界速率意图；底盘 `BigYawWorldHeadingFollower` 与 `BigYawExecutor` 完成世界航向闭环和独立速度内环，不需绝对零点或轮控恢复授权。

**接入状态：已有代码。** 分配器与大 Yaw 执行链已接入双主控应用；中央硬件安装、零位、行程和带载响应仍需现场确认。

## 职责与关联

云台分配器把头部世界速率拆成小 Yaw 关节轨迹与大 Yaw 世界意图；底盘航向跟随器把世界意图变成关节速率，再由独立速度内环执行，不需绝对零点或轮控恢复授权。

输入 / 依赖：[云台单轴与本地执行](gimbal.md)、[VelocityMotor / PositionMotor 硬件闭环](motor-control.md)、[双主控板间通信](interboard.md)

消费者：[双主控应用与执行器](application.md)

## 接口契约

### 1. `update()` / `validate()` / `reset()`

```cpp
DualYawAllocatorOutput DualYawMotionAllocator::update(const DualYawAllocatorInputs &, core::TimeUs now_us);
int validate() const;
void reset()
```

把头部世界速率意图拆成小 Yaw 关节轨迹与大 Yaw 世界意图。`head` 是经小 Yaw 行程/制动包络约束后仍可执行的头部世界速率，`big.rate_rad_s` 是发给底盘的持续分量；`joint_rate_rad_s`/`joint_position_rad` 已按 `joint_direction` 只换算一次到编码器坐标。视觉有效时由角度误差生成头部目标，否则使用 `head_rate_target_rad_s`。

**返回 / 输出：** enabled、head/big 速率计划、joint_rate_valid、joint_reseed_requested、outward_budget_*、brake_margin_*、big_saturated/travel_limited/tracking_limited、head_reference_* 与 reason。

**线程 / 时序：** 单一执行线程拥有状态；每周期最多调用一次，`now_us` 单调递增。

**错误 / 边界：** `validate()` 返回配置错误；输入失效或未授权时 `enabled=false`，不能把输出当作已执行。退化路径也给出 `joint_rate_valid` 并夹紧关节速率。

### 2. YawCenteringController（已作废）

`YawCenteringOutput YawCenteringController::update(const YawCenteringInputs &, core::TimeUs now_us)` 仅保留给历史样例和 `tests/robotics_contracts`。正式云台不再构造该对象；回中由 `dualYawAllocatorConfig()` 的 `center_*` 参数和分配器内部慢调节完成，不要按旧接口新增调用。

### 3. `BigYawExecutor()` / `begin()` / `update()` 等接口

```cpp
BigYawExecutor(Motor &, const VelocityMotor::Config &, const Config &);
int begin();
RunStatus update(const BigYawExecutionInputs &, core::TimeUs now_us);
BigYawFeedback feedback() const
```

消费 v4 request、实际 local_boot_id、peer_online、transport_ready 和输入撤销条件；只暂存，不 commit。

**返回 / 输出：** RunStatus / BigYawFeedback 值副本；actual_rate_rad_s 仅 valid 时使用。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

## 调用示例

### 大 Yaw 回中与独立速度环 周期

```cpp
const auto allocation = dual_yaw.update(alloc_inputs, now_us);
request.mode = allocation.enabled ? BigYawMode::FollowCenter : BigYawMode::Disabled;
request.target_rate_rad_s = allocation.enabled ? allocation.big.rate_rad_s : 0;
// 同时保留 source_sequence、来源/命令/权限原年龄；本周期不生成新授权。
endpoint.submitBigYaw(request);
```

应用周期片段，构造与实物配置以链接源码为准。分配器的 `joint_position_rad`/`joint_rate_rad_s` 进入 `GimbalExecutionInputs::yaw_trajectory` 或手动关节速率，`head_reference_*` 只给视觉姿态外环。

## 调用顺序

1. 静态构造与校验实物配置。
2. 保留原输入/测量时间及坐标身份。
3. 唯一执行 owner 持续 update；应用统一物理 CAN commit。
4. 真实输入停止才撤销；电机等待仅影响本轴计算。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `joint_center_rad` / `joint_min_rad` / `joint_max_rad` | 实测小 Yaw 机械中心与编码器坐标行程；不能用户零点替代。 |
| `joint_direction` | 编码器到头部相对世界的符号，只允许 +1/-1，且只应用一次。 |
| `head_*` / `big_*` | 头部速率、加速度、jerk 预算与大 Yaw 世界意图/跟踪增益。 |
| `center_*` / `brake_*` | 小 Yaw 回中慢调节与外向制动包络；正式值见 `dualYawAllocatorConfig()`。 |
| `v4` | 目标保留 boot/producer/age，无 resume_generation，发送不等待电机 ready。 |

- API 已存在不等于实物标定和闭环通过。
- 状态生产时间不能被读者/通信刷新。

## 正文与源码

- [executors.md](../modules/robotics/executors.md)
- [dual-controller.md](../applications/dual-controller.md)

- [dual_yaw_motion_allocator.hpp](../../include/robotics/gimbal/dual_yaw_motion_allocator.hpp)
- [big_yaw_executor.hpp](../../include/robotics/execution/big_yaw_executor.hpp)
- [yaw_centering.hpp](../../include/robotics/gimbal/yaw_centering.hpp)（仅历史/测试）
- [dual_yaw_motion_allocator.cpp](../../lib/robotics/dual_yaw_motion_allocator.cpp)
- [big_yaw_executor.cpp](../../lib/robotics/big_yaw_executor.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
