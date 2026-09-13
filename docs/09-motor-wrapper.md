# 09 统一速度 / 位置电机封装

实现位置：`include/control/`、`lib/control/motor_control.cpp`、`dji_motor_backend.cpp`、`dm_motor_backend.cpp`。

封装目标是让同一套 C 控制内核面对 DJI 电流执行器和 DM MIT 力矩执行器，同时集中处理反馈、时序和故障恢复。

## 1. 组成

### `MotorBackend`

后端接口负责：

```text
describe → configure/pollPrepare → read → arm → write → flush → stop
```

`DjiMotorBackend` 内部拥有一个单电机 DJI Bus；`DmMotorBackend` 内部拥有一个单电机 DM Bus，可选在 CAN 建立后调用板级 `power_on` hook。两者都不能复制，实例要保持到应用结束。

### `MotorRuntime`

runtime 负责：

- effort 单位检查和后端能力匹配。
- 反馈 timestamp 新鲜度。
- 真实 dt 和控制周期范围。
- 超速、超温和非有限值保护。
- `poll()` 阶段的有限重试、反馈稳定窗口和恢复次数。
- `suspend()` / `stop()` 的安全撤销。

## 2. 生命周期

```text
configure()
  ↓
poll(now) ── 反馈无效/总线恢复中 ──→ Waiting / Recovering
  ↓ 新鲜反馈稳定
Ready
  ↓ resume()（必须有新命令）
Active
  ↓ update(target)
Active
  ├─ suspend(reason) → Waiting/Recovering
  ├─ 故障              → Recovering/Fault
  └─ clearEmergencyStop(released) → 允许后续恢复
```

状态对外用 `ExecutionState` 表示：`Waiting`、`Recovering`、`Ready`、`Active`、`EStopLatched`、`ConfigBlocked`。低层 driver 的 `State` 和 BusState 仍应单独查看。

`begin()` 是阻塞式便捷入口，会在最多约 3 秒内反复 poll 后 resume；需要和整机安全/权限协调时，应使用 `configure → poll → resume → update`，不要直接 `begin()`。

## 3. `VelocityMotor`

```cpp
control::VelocityMotor motor(backend, config);
motor.configure();
for (;;) {
    const auto now = k_uptime_get();
    if (motor.state() != control::ExecutionState::Active) {
        if (motor.poll(now) == 0 && permission)
            motor.resume();
    } else {
        motor.update(target_rad_s);
    }
}
```

`update()` 自己测 dt，读取反馈，调用 `control_motor_velocity_step()`，写入 effort 并 flush。返回负值时应认为本周期输出无效；telemetry 的 `valid` 也会变为 false。

## 4. `PositionMotor` 与参考系

| `PositionReference` | 目标意义 | 适用场景 |
|---|---|---|
| `StartupRelative` | `0` 是第一次有效 resume 的位置 | 普通相对位置 |
| `DriverContinuous` | 直接使用驱动连续坐标 | 有机械限位的多圈轴 |
| `AbsoluteNearest` | 以固定零点单圈角走最短路径 | GM6020 连续 yaw |

`AbsoluteNearest` 要求后端提供 `FeedbackAbsolutePosition`；DM 的 MIT wrapper 不应假装拥有固定零点单圈能力。位置 PID 还会在连续运行超过约 128 rad 后平移内部坐标，避免浮点和历史值无限增长。

## 5. 配置检查

`MotorSafety` 至少要配置：

```cpp
{
    .velocity_abs_max_rad_s = 12,
    .temperature_max_c = 0,       // 0 表示禁用温度保护
    .recovery_stable_ms = 30,
    .recovery_retry_ms = 100,
    .recovery_poll_ms = 5,
}
```

位置/速度环的 effort 上限必须不大于 backend 的 `MotorInfo.effort_limit`。温度保护大于 0 时，后端必须声明温度反馈能力。

## 6. 多轴注意事项

统一 wrapper 是单电机独占模型；多个 DJI 电机不能各建一个 wrapper 再共享同一个底层 CAN 而不做调度。多轴应用应：

- 直接创建一个原生 `dji::Bus` / `dm::Bus`，统一 attach、arm、命令和 flush；或
- 按当前 `applications/sentry_chassis/src/chassis_hardware.*` 的方式集中管理多个 motor device 和 Bus。

## 7. 推荐参考

- DJI 统一速度/位置：`samples/motor/dji_speed_control`、`dji_position_control`。
- DM MIT 统一速度/位置：`samples/motor/dm_mit_velocity_control`、`dm_mit_position_control`。
- 断电恢复：`samples/motor/recovery`。
- 云台策略：`samples/robotics/yaw_gimbal`、`include/robotics/gimbal/yaw_gimbal.hpp`。

## 8. 失败处理

常见返回值：

- `-EACCES`：未处于允许的生命周期，例如未 arm 就 update。
- `-EAGAIN`：反馈/总线恢复尚未完成，需要稍后 poll。
- `-ESTALE` / `-EHOSTDOWN`：反馈或驱动状态不可用。
- `-ERANGE`：dt、速度、温度或 effort 越界。
- `-EALREADY`：对象已 configure/begin，当前设计不支持重复初始化。

失败后不要继续使用旧 telemetry effort；先 `suspend()` / `stop()`，等待 `poll()` 恢复并在得到新鲜反馈、新命令后再 `resume()`。
