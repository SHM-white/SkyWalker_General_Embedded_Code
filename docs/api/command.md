# 命令仲裁与后台服务：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

按 Safe/Manual/Auto、输入新鲜度、参考系、人工接管和裁判许可发布完整机器人命令。

**接入状态：已有代码。** 注册来源与后台服务已实现；整车手动默认 Remote，裁判/视觉按 VEHICLE_* 阶段配置注册，不改变原输入年龄。

## 职责与关联

CommandArbiter 是同步策略核心；CommandManager 注册静态来源、启动来源并周期采样/仲裁，发布非消费快照。执行器负责本地电机反馈、使能与恢复。

输入 / 依赖：[来源适配与手动映射](command-sources.md)

消费者：[双主控应用与执行器](application.md)、[云台单轴与本地执行](gimbal.md)、[头部惯性云台适配](inertial.md)、[摩擦轮与拨盘发射执行](shooter.md)

## 接口契约

### 1. explicit CommandManager(const Config &config); int CommandManager::registerSource(ICommandSource &source)

```cpp
explicit CommandManager(const Config &config); int CommandManager::registerSource(ICommandSource &source)
```

构造策略服务并在启动前注册来源。一个 Operator 必须存在，Aim 可选，每个角色最多一个，总容量两个。

| 参数 | 含义与边界 |
| --- | --- |
| `config` | CommandArbiter::Config 的别名，包含限速、输入超时、视觉参考、许可策略 |
| `source` | 静态来源对象，role 在整个生命周期固定 |

**返回 / 输出：** registerSource：0 注册成功；不启动来源。

**线程 / 时序：** 同一个启动线程完成注册/绑定/start；所有公共调用只能在线程上下文。

**错误 / 边界：** -EWOULDBLOCK：ISR；-EBUSY：已尝试启动；-EINVAL：非法 role；-EEXIST：对象或角色重复；-ENOSPC：容量已满。注册顺序不代表优先级。

### 2. int CommandManager::bindPermissions(IPermissionSource &source)

```cpp
int CommandManager::bindPermissions(IPermissionSource &source)
```

绑定单独的裁判许可来源。许可是对目标的约束，不是第三个运动目标角色。

| 参数 | 含义与边界 |
| --- | --- |
| `source` | 长生命周期 IPermissionSource，例如 RefereePermissionSource |

**返回 / 输出：** 0：绑定成功。

**线程 / 时序：** 启动前同一个启动线程调用。

**错误 / 边界：** -EWOULDBLOCK：ISR；-EBUSY：已尝试 start；-EEXIST：已有许可来源。

### 3. int CommandManager::start()

```cpp
int CommandManager::start()
```

校验策略与必要来源，依次启动来源和许可源，最后创建后台 worker；一次启动尝试后不能重启对象。

**返回 / 输出：** 0：worker 已创建，尚不保证发布/输入在线；负 errno：配置或来源启动错误。

**线程 / 时序：** 线程上下文，只尝试一次；manager、来源、receiver、codec 与 DMA 均静态存活，即使启动中途失败。

**错误 / 边界：** -EWOULDBLOCK：ISR；-EALREADY：重复尝试；-EINVAL：配置错误或缺少 Operator；-ENODEV：策略要求裁判但未绑定；其他值透传来源 start。失败时发布 invalid/Disabled 决策。

### 4. int CommandManager::current(RobotCommand &out) const; int CommandManager::snapshot(CommandSnapshot &out) const

```cpp
int CommandManager::current(RobotCommand &out) const; int CommandManager::snapshot(CommandSnapshot &out) const
```

current 复制最终命令；snapshot 同时复制 observed、decision 与 remote/vision/permission 诊断，供执行和观测读者独立使用。

| 参数 | 含义与边界 |
| --- | --- |
| `out` | 由当前读者拥有的输出值 |

**返回 / 输出：** 0：复制当前值，可能仍是同一序号；-EAGAIN：尚未首次发布；错误不修改 out。

**线程 / 时序：** 多个线程可读，短锁保护；非消费、不刷新 stamp，ISR 返回 -EWOULDBLOCK。

**错误 / 边界：** 读取 0 不代表命令 Active；应检查模式、decision.error、原因位和 command.stamp。执行器继续强制过期，防止 worker 停滞时重用旧目标。

### 5. explicit CommandArbiter(const Config &config); int CommandArbiter::configError() const; CommandDecision CommandArbiter::update(const CommandInputs &inputs); void CommandArbiter::reset()

```cpp
explicit CommandArbiter(const Config &config); int CommandArbiter::configError() const; CommandDecision CommandArbiter::update(const CommandInputs &inputs); void CommandArbiter::reset()
```

同步入口适合应用自行采集/调度。update 生成 requested 候选与按裁判裁剪后的 command，保留最终采用视觉的完整 selected_vision；reset 清仲裁历史，不操作硬件。

| 参数 | 含义与边界 |
| --- | --- |
| `inputs.now_us` | 本次仲裁时刻，μs |
| `inputs.remote / vision / referee` | 保留来源原始 stamp 的完整值副本 |

**返回 / 输出：** configError：0 或 -EINVAL；update：完整 CommandDecision，error 表示错误，reasons 给出各机构的禁止/限幅原因。

**线程 / 时序：** 单写入者；update/reset 由调用者串行化，不适用于 ISR。

**错误 / 边界：** 时间倒退可产生 -ESTALE/ClockRegression；非法配置/输入产生 -EINVAL；来源离线、Safe、视觉缺失和许可撤销可正常返回 error=0 但命令 Disabled/Hold。

## 调用示例

### 真实后台服务：注册三源、读取同帧诊断

```cpp
#include <robotics/command/command_manager.hpp>
#include <robotics/command/receiver_sources.hpp>
using namespace skywalker;

// remote / vision / referee 需按各模块接口静态构造。
int startService(robotics::CommandManager &manager,
                 robotics::RemoteSource &remote_source,
                 robotics::VisionSource &vision_source,
                 robotics::RefereePermissionSource &permission_source) {
    int ret = manager.registerSource(remote_source);
    if (ret == 0) ret = manager.registerSource(vision_source);
    if (ret == 0) ret = manager.bindPermissions(permission_source);
    if (ret == 0) ret = manager.start();
    return ret;
}

void observeCommand(const robotics::CommandManager &manager) {
    robotics::CommandSnapshot frame{};
    if (manager.snapshot(frame) == 0) {
        const auto &command = frame.decision.command;
        const auto reasons = frame.decision.reasons();
        // command 是最终授权结果；requested 仅供诊断。
        (void)command; (void)reasons;
    }
}
```

完整静态对象与参数见 samples/robotics/command_manager/src/main.cpp 和 board_config.hpp；所有引用对象需要覆盖 worker 生命周期。

### 直接使用同步仲裁核心

```cpp
#include <core/clock.hpp>
#include <robotics/command/command_arbiter.hpp>
using namespace skywalker;

robotics::CommandDecision arbitrate(
    robotics::CommandArbiter &arbiter, robotics::CommandInputs inputs) {
    inputs.now_us = core::monotonicTimeUs();
    // 只设置本次仲裁时钟，不改 remote/vision/referee 的 stamp。
    return arbiter.update(inputs);
}
// 构造后先检查 arbiter.configError()；所有调用必须串行。
```

CommandManager 不接受 CommandInputs，也没有 update、configError、reset 或 stop；这些同步入口属于 CommandArbiter。

## 调用顺序

1. 静态构造 receiver/codec/source/manager；策略与参考系先确定。
2. 一个启动线程 registerSource(Operator)，按需注册 Aim，bindPermissions，然后 start 一次。
3. worker 按周期 sample → 原始时间有效性 → CommandArbiter::update → 发布整帧。
4. 执行线程 current/snapshot 后检查目标模式和命令年龄，交本地执行器。
5. 遥控 Safe、输入过期、参考不匹配或裁判许可撤销都会约束命令；本地恢复还需新的有效目标与恢复上下文。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `CONFIG_SKYWALKER_LIB_ROBOTICS / ROBOTICS_COMMAND / COMMAND_SERVICE` | 启用同步仲裁与后台注册服务；来源另外启用对应接收模块。 |
| `CONFIG_SKYWALKER_COMMAND_PERIOD_MS / PRIORITY / STACK_SIZE` | 默认周期 10 ms、优先级 5、栈 6144 字节；服务发布与执行控制周期分开。 |
| `新鲜度与参考` | input_timeout_ms=100、permission_timeout_ms=300、vision_timeout_us=100000，expected_vision_reference={1,1}。 |
| `模式与人工接管` | require_referee_for_motion=true、allow_auto=true；override_enter_norm=0.15、exit=0.05、释放安静期 200000 μs。应用没有视觉时设 allow_auto=false。 |
| `目标上限` | 默认底盘 vx/vy=3 m/s、wz=6 rad/s；云台 yaw=3/pitch=2 rad/s；视觉加速度上限 30/20 rad/s²；射速请求 5 Hz。 |

- 不要使用已移除的 GlobalSafetyManager/GimbalLocalSafety/ChassisLocalSafety；开发指南中的拟议接口不代表当前 API。
- decision.requested 是未按裁判裁剪的候选，电机只能消费 decision.command。
- 启动成功不代表输入上线；命令被禁止也可能 error=0，需看模式和原因位。
- Auto 进入或人工接管释放后需要新的视觉目标；旧视觉快照不能跨新自动上下文直接采用。
- shooting 命令类型已存在，但当前 sentry 应用没有完整发射执行链。

## 正文与源码

- [注册来源与线程契约](../modules/robotics/command-service.md)
- [命令与执行恢复](../modules/robotics/command-recovery.md)
- [三源台架](../../samples/robotics/command_manager/README.md)

- [后台服务 API](../../include/robotics/command/command_manager.hpp)
- [仲裁核心 API](../../include/robotics/command/command_arbiter.hpp)
- [决策与原因位](../../include/robotics/command/command_inputs.hpp)
- [后台发布实现](../../lib/robotics/command_manager.cpp)
- [RobotCommand 单位与字段](../../include/robotics/messages/command.hpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
