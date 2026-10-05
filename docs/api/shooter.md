# 摩擦轮与拨盘发射执行：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

摩擦轮与拨盘独立持续目标；供弹业务检查热量/可信原点/摩擦稳定/头部状态；离散旧事件消费丢弃不重放。

**接入状态：接入 / 配置未完成。** 软件接口与整车装配已有；中央硬件/安装/测量确认未完成，不代表实机验收。

## 职责与关联

摩擦轮与拨盘独立持续目标；供弹业务检查热量/可信原点/摩擦稳定/头部状态；离散旧事件消费丢弃不重放。

输入 / 依赖：[命令仲裁与后台服务](command.md)、[云台单轴与本地执行](gimbal.md)、[VelocityMotor / PositionMotor 硬件闭环](motor-control.md)、[裁判许可与功率预算](referee.md)

消费者：[双主控应用与执行器](application.md)

## 接口契约

### 1. ShooterExecutor(Motor &left, Motor &right, Motor &dial, Group &friction, Group &feed, const VelocityMotor::Config &, const PositionMotor::Config &, const Config &); int begin()

```cpp
ShooterExecutor(Motor &left, Motor &right, Motor &dial, Group &friction, Group &feed, const VelocityMotor::Config &, const PositionMotor::Config &, const Config &); int begin()
```

注入三电机、两批量组与环参数；应用先 attach/start，不在执行器内 commit。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

### 2. ShooterStatus update(const ShooterExecutionInputs &, core::TimeUs now_us); ShooterStatus suspend(core::TimeUs, WaitReason, int error = 0)

```cpp
ShooterStatus update(const ShooterExecutionInputs &, core::TimeUs now_us); ShooterStatus suspend(core::TimeUs, WaitReason, int error = 0)
```

输入原始命令/事件、source_stamp、热量/许可、拨盘参考、云台状态、allow_feed 与急停；单发去重，忙碌/过期/恢复中旧事件丢弃。

**返回 / 输出：** friction/feed RunStatus、摩擦就绪、拨盘 busy/jammed、last_event_id、软件 shots 与 reserved_heat。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

## 调用示例

### 摩擦轮与拨盘发射执行 周期

```cpp
const auto status = shooter.update(inputs, now_us);
// 小云台/摩擦/拨盘共享 DJI CAN1，所有机构 stage 后统一 commit。
const auto published = dji_bus.commit();
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
| `真实供弹来源` | IShooterHeatSource、IDialHomeSource 默认 Pending 无效；不能将当前点伪造有载原点。 |
| `软件 shots` | 软件执行统计不是可信裁判弹数，不直接用作 AB 实测反馈。 |

- API 已存在不等于实物标定和闭环通过。
- 状态生产时间不能被读者/通信刷新。

## 正文与源码

- [executors.md](../modules/robotics/executors.md)
- [dual-controller.md](../applications/dual-controller.md)

- [shooter_executor.hpp](../../include/robotics/shooter/shooter_executor.hpp)
- [shooter_executor.cpp](../../lib/robotics/shooter_executor.cpp)
- [shooter_bench.hpp](../../samples/robotics/common/shooter_bench.hpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
