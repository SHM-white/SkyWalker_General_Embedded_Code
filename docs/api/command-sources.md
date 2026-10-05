# 来源适配与手动映射：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

把接收器快照接到固定 Operator/Aim 角色；裁判单独作为 Permission，保留原始时间与诊断。

**接入状态：已有代码。** 注册来源与后台服务已实现；整车手动默认 Remote，裁判/视觉按 VEHICLE_* 阶段配置注册，不改变原输入年龄。

## 职责与关联

ICommandSource/IPermissionSource 是扩展入口；内置 RemoteSource、VisionSource、RefereePermissionSource 适配接收器。ManualCommandMapper 把遥控值变成归一化操作意图，不输出电机电流。

输入 / 依赖：[DR16 遥控输入](remote.md)、[视觉链路与 AB 协议](vision.md)、[裁判许可与功率预算](referee.md)

消费者：[命令仲裁与后台服务](command.md)

## 接口契约

### 1. virtual SourceRole ICommandSource::role() const = 0; virtual int ICommandSource::start() = 0; virtual int ICommandSource::sample(SourceSample &out) = 0

```cpp
virtual SourceRole ICommandSource::role() const = 0; virtual int ICommandSource::start() = 0; virtual int ICommandSource::sample(SourceSample &out) = 0
```

输入扩展接口。role 固定为 Operator 或 Aim；SourceValue 是 RemoteState 或 Measurement<AimCommand> 的 variant，必须和角色相匹配。

| 参数 | 含义与边界 |
| --- | --- |
| `out` | 0 时写完整 value 与 diagnostics；value 即使离线/无效也应完整写出 |

**返回 / 输出：** start：0 已调度；sample：0 更新缓存、-EAGAIN 保留上次缓存、其他负值清空该来源有效值。

**线程 / 时序：** startup owner 只调用一次 start；只有 manager worker 调用 sample；采样应有界、不等待 I/O。

**错误 / 边界：** 角色/variant 不匹配由 manager 视为 -EINVAL；其他错误进入诊断，不能留旧值继续有效。

### 2. virtual int IPermissionSource::start() = 0; virtual int IPermissionSource::sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics) = 0

```cpp
virtual int IPermissionSource::start() = 0; virtual int IPermissionSource::sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics) = 0
```

单独的许可约束接口；不占据 Operator/Aim 来源槽。

| 参数 | 含义与边界 |
| --- | --- |
| `now_ms` | manager 的本机当前 ms，供接收轮询和过期判断 |
| `out` | 完整裁判快照，保留字段时间 |
| `diagnostics` | error、sample_error、state 与可用丢包计数 |

**返回 / 输出：** 0 更新、-EAGAIN 保留、其他负值使许可缓存失效。

**线程 / 时序：** start 单次；sample 只有 manager worker 所有。

**错误 / 边界：** 缺少必需许可来源时 manager.start=-ENODEV；合法离线数据可返回 0，由仲裁按原始 stamp 决定。

### 3. explicit RemoteSource(communication::RemoteReceiver &receiver); int RemoteSource::start(); int RemoteSource::sample(SourceSample &out)

```cpp
explicit RemoteSource(communication::RemoteReceiver &receiver); int RemoteSource::start(); int RemoteSource::sample(SourceSample &out)
```

固定 Operator；start 转发 receiver.start，sample 拷贝 RemoteState、UART 状态与 dropped 诊断。

| 参数 | 含义与边界 |
| --- | --- |
| `receiver` | 静态遥控接收器；不要先手动启动 |
| `out` | variant 中的 RemoteState 与同次接收诊断 |

**返回 / 输出：** start 透传；sample 把 receiver 的 -EAGAIN 转成保留且已执行读者侧过期的 0，其余错误透传。

**线程 / 时序：** 只给一个 manager 注册；receiver 与 source 持续存活。

**错误 / 边界：** 预先 start 接收器会导致 manager 再启动时 -EALREADY；dropped_available=true。

### 4. explicit VisionSource(communication::vision::VisionReceiver &receiver); int VisionSource::start(); int VisionSource::sample(SourceSample &out)

```cpp
explicit VisionSource(communication::vision::VisionReceiver &receiver); int VisionSource::start(); int VisionSource::sample(SourceSample &out)
```

固定 Aim；从 receiver.snapshot 提取原始 Measurement，不把读快照的时刻写到目标 stamp。

| 参数 | 含义与边界 |
| --- | --- |
| `receiver` | 静态视觉接收器及其 codec/DMA |
| `out` | variant 中的 Measurement<AimCommand> 与 UART/丢包诊断 |

**返回 / 输出：** start 透传；sample 返回 0，目标可以无效/过期，由仲裁器判断。

**线程 / 时序：** manager worker 采样；receiver 自己的 worker 接收字节。

**错误 / 边界：** UART error 放在 diagnostics.error；样本存在不代表视觉请求正在控制；dropped_available=true。

### 5. explicit RefereePermissionSource(communication::RefereeReceiver &receiver); int RefereePermissionSource::start(); int RefereePermissionSource::sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics)

```cpp
explicit RefereePermissionSource(communication::RefereeReceiver &receiver); int RefereePermissionSource::start(); int RefereePermissionSource::sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics)
```

start 为 0，真正 I/O 由 sample 调用 receiver.poll；裁判没有单独 worker。

| 参数 | 含义与边界 |
| --- | --- |
| `receiver` | 唯一裁判 UART owner |
| `now_ms` | 本机当前 ms |
| `out / diagnostics` | 裁判值与 receiver.error |

**返回 / 输出：** start/sample 当前均返回 0；在线、许可与接收错误在值/诊断中表达。

**线程 / 时序：** manager worker 独占 poll；应用不能并行轮询同一裁判 receiver。

**错误 / 边界：** RefereeReceiver 未公开丢块数，因此 dropped_available=false，不伪造为可用计数。

### 6. explicit ManualCommandMapper(const Config &config); int ManualCommandMapper::map(const RemoteState &remote, OperatorIntent &out) const

```cpp
explicit ManualCommandMapper(const Config &config); int ManualCommandMapper::map(const RemoteState &remote, OperatorIntent &out) const
```

将中心化遥控值变为 [-1,1] 意图。左开关 Down=Safe/Middle=Manual/Up=Auto；右开关 Up=KeyboardMouse，其余合法档=Remote。

| 参数 | 含义与边界 |
| --- | --- |
| `remote` | 合法遥控快照，带在线与 stamp |
| `out` | 模式、来源、归一化速度、摩擦轮/射击请求及原始 stamp |

**返回 / 输出：** 0：映射；离线输入映射成 Safe/零值；-EINVAL：配置或开关非法。

**线程 / 时序：** 纯值映射，无 I/O；后续 CommandArbiter 应用物理限速和许可。

**错误 / 边界：** mapper 不自行按当前时刻判断 stamp 年龄，仲裁器必须先检查新鲜度。默认 DR16 不解 wheel，底盘旋转请求因此可能为 0。

## 调用示例

### 用内置适配器连接现有接收器

```cpp
#include <robotics/command/receiver_sources.hpp>
#include <robotics/command/command_manager.hpp>
using namespace skywalker;

// 示例调用位置：接收器构造后，manager.start 之前。
int connectInputs(robotics::CommandManager &manager,
                  robotics::ICommandSource &operator_source,
                  robotics::ICommandSource &aim_source,
                  robotics::IPermissionSource &permissions) {
    int ret = manager.registerSource(operator_source);
    if (ret == 0) ret = manager.registerSource(aim_source);
    if (ret == 0) ret = manager.bindPermissions(permissions);
    return ret;
}
// 实际静态对象声明：
// robotics::RemoteSource operator_source(remote);
// robotics::VisionSource aim_source(vision);
// robotics::RefereePermissionSource permissions(referee);
```

source 对象及依赖长期存活，所有注册完成后统一 manager.start，不单独调用内置 source.sample。

### 解释遥控映射，而后交仲裁器限速

```cpp
#include <robotics/command/manual_command_mapper.hpp>
using namespace skywalker::robotics;

int inspectIntent(const RemoteState &remote, OperatorIntent &intent) {
    const ManualCommandMapper mapper({});
    return mapper.map(remote, intent);
}
// intent.chassis_vx_norm 为归一化值，不能直接送电机。
// CommandArbiter 会将它乘 max_chassis_vx_m_s 并施加裁判许可。
```

遥控 left_y→vx，-left_x→vy，wheel→wz，-right_x→yaw_rate，right_y→pitch_rate；键鼠 W/S、A/D 与鼠标按固定映射提供意图。

## 调用顺序

1. 先构造接收器，再构造内置 source 引用。
2. 固定来源 role，在同一启动线程注册并绑定许可。
3. 由 manager.start 启动来源；之后只有 manager worker sample。
4. 采样复制原始值与时间，按明确返回契约替换/保留/失效缓存。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `ManualCommandMapper::Config` | analog_deadband=0.03、channel_range=660、mouse_yaw_scale/mouse_pitch_scale=0.002。 |
| `来源编译条件` | RemoteSource 需 REMOTE_RECEIVER，VisionSource 需 VISION_RECEIVER，RefereePermissionSource 需 REFEREE 与 UART_TRANSPORT。 |
| `对象所有权` | source 只引用接收器；manager 只引用 source，不拥有它们，不负责释放。 |

- 不要在 sample 中无限等待串口或重新刷新旧值时间；后台周期不能因此停滞。
- SourceRole 只有 Operator/Aim；没有直接把所有消息源塞入同一优先级队列的接口。
- 注册多个 Operator 或 Aim 返回 -EEXIST；来源最多两个。
- 自定义来源不得发布与 role 不匹配的 variant；诊断的 unavailable 与数值 0 不同。

## 正文与源码

- [来源与后台服务说明](../modules/robotics/command-service.md)
- [来源样例](../../samples/robotics/command_manager/README.md)

- [来源契约与快照](../../include/robotics/command/command_source.hpp)
- [接收器适配器](../../include/robotics/command/receiver_sources.hpp)
- [适配实现](../../lib/robotics/receiver_sources.cpp)
- [手动映射](../../include/robotics/command/manual_command_mapper.hpp)
- [映射实现](../../lib/robotics/command.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
