# 大 Yaw 回中与独立速度环：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

云台小 Yaw 中心外环产生速度请求；底盘大 Yaw 独立连续速度内环，不需绝对零点或轮控恢复授权。

**接入状态：接入 / 配置未完成。** 软件接口与整车装配已有；中央硬件/安装/测量确认未完成，不代表实机验收。

## 职责与关联

云台小 Yaw 中心外环产生速度请求；底盘大 Yaw 独立连续速度内环，不需绝对零点或轮控恢复授权。

输入 / 依赖：[云台单轴与本地执行](gimbal.md)、[VelocityMotor / PositionMotor 硬件闭环](motor-control.md)、[双主控板间通信](interboard.md)

消费者：[双主控应用与执行器](application.md)

## 接口契约

### 1. YawCenteringOutput YawCenteringController::update(const YawCenteringInputs &, core::TimeUs now_us); void reset()

```cpp
YawCenteringOutput YawCenteringController::update(const YawCenteringInputs &, core::TimeUs now_us); void reset()
```

使用独立标定中心、关节采样时刻、头部稳定与新鲜许可，产生带死区/迟滞/限速/斜坡的角速度。

**返回 / 输出：** enabled、velocity_rad_s、center_error_rad、stamp 与 reason。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

### 2. BigYawExecutor(Motor &, const VelocityMotor::Config &, const Config &); int begin(); RunStatus update(const BigYawExecutionInputs &, core::TimeUs now_us); BigYawFeedback feedback() const

```cpp
BigYawExecutor(Motor &, const VelocityMotor::Config &, const Config &); int begin(); RunStatus update(const BigYawExecutionInputs &, core::TimeUs now_us); BigYawFeedback feedback() const
```

消费 v4 request、实际 local_boot_id、peer_online、transport_ready 和输入撤销条件；只暂存，不 commit。

**返回 / 输出：** RunStatus / BigYawFeedback 值副本；actual_rate_rad_s 仅 valid 时使用。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

## 调用示例

### 大 Yaw 回中与独立速度环 周期

```cpp
const auto follow = centering.update(center_inputs, now_us);
request.mode = follow.enabled ? BigYawMode::FollowCenter : BigYawMode::Disabled;
request.target_rate_rad_s = follow.velocity_rad_s;
request.stamp = follow.stamp;
// 同时保留 source_sequence、来源/命令/权限原年龄。
endpoint.submitBigYaw(request);
```

应用周期片段，构造与实物配置以链接源码为准。

## 调用顺序

1. 静态构造与校验实物配置。
2. 保留原输入/测量时间及坐标身份。
3. 唯一执行 owner 持续 update；应用统一物理 CAN commit。
4. 真实输入停止才撤销；电机等待仅影响本轴计算。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `center_rad` | 独立实测小 Yaw 机械中心，不能用编码器零点替代。 |
| `v4` | 目标保留 boot/producer/age，无 resume_generation，发送不等待电机 ready。 |

- API 已存在不等于实物标定和闭环通过。
- 状态生产时间不能被读者/通信刷新。

## 正文与源码

- [executors.md](../modules/robotics/executors.md)
- [dual-controller.md](../applications/dual-controller.md)

- [yaw_centering.hpp](../../include/robotics/gimbal/yaw_centering.hpp)
- [big_yaw_executor.hpp](../../include/robotics/execution/big_yaw_executor.hpp)
- [yaw_centering.cpp](../../lib/robotics/yaw_centering.cpp)
- [big_yaw_executor.cpp](../../lib/robotics/big_yaw_executor.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
