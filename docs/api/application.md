# 双主控应用与执行器：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

两个 sentry 入口共用 vehicle_bench：云台惯性双轴/发射/回中，底盘四舵轮/大 Yaw，各机构统一物理 CAN 发布。

**接入状态：接入 / 配置未完成。** 软件框架已有，connections_confirmed、imu_mounting_confirmed 等中央确认默认关闭；功率、热量、拨盘原点与视觉反馈测量待接。

## 职责与关联

公共运行时持有物理设备、长期对象、输入/通信/观测线程与执行周期；公开执行器保存目标和消费新鲜输入，Motor/CAN 独立恢复。

输入 / 依赖：[命令仲裁与后台服务](command.md)、[双主控板间通信](interboard.md)、[头部惯性云台适配](inertial.md)、[云台单轴与本地执行](gimbal.md)、[舵轮底盘与功率缩放](chassis.md)、[大 Yaw 回中与独立速度环](big-yaw.md)、[摩擦轮与拨盘发射执行](shooter.md)、[板级：MC02 / RoboMaster Type-C](boards.md)

消费者：由应用或样例直接装配。

## 接口契约

### 1. int samples::vehicle::run(IPowerMeasurementSource *power_source = nullptr, IShooterHeatSource *heat_source = nullptr, IDialHomeSource *home_source = nullptr)

```cpp
int samples::vehicle::run(IPowerMeasurementSource *power_source = nullptr, IShooterHeatSource *heat_source = nullptr, IDialHomeSource *home_source = nullptr)
```

共用整车入口按 VEHICLE_CHASSIS_ROLE 选择角色，持有硬件并进入唯一机构执行循环。

| 参数 | 含义与边界 |
| --- | --- |
| `power_source` | 真实功率源 |
| `heat_source` | 真实热量源 |
| `home_source` | 真实拨盘 home/index 原点源 |

**返回 / 输出：** 初始化结果或运行循环；默认 Pending 来源无效，不伪造测量。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

### 2. RunStatus::stamp / SnapshotCache<T>::publish / snapshot

```cpp
RunStatus::stamp / SnapshotCache<T>::publish / snapshot
```

状态由执行 owner 生产，通信/观测只复制；Endpoint 使用状态年龄撤销陈旧 ready/armed。

**返回 / 输出：** 生产时间和一致值副本；状态不是目标生产门禁。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 缺数据、争锁或过期分别处理，不能重盖时间。

## 调用示例

### 正式应用共用入口

```cpp
#include "../../../samples/robotics/common/vehicle_bench.hpp"
int main() {
    return skywalker::samples::vehicle::run();
}
```

两个正式应用的当前 main.cpp；阶段配置和资源所有权在公共运行时。

## 调用顺序

1. 中央 calibration 与 overlay 填实物参数；保持未完成确认为 false。
2. 静态构造 Motor/CanBus/输入来源/快照缓存。
3. attach/start 全部物理总线，再 begin 各执行器和命令服务。
4. 通信线程 1 ms poll；云台约 5 ms，底盘 2 ms 绝对节拍执行。
5. 各机构先暂存，统一每物理 CAN commit 一次；错误诊断不撤销其他机构目标。
6. 按手动、视觉观察、视觉执行、功率和有载发射逐级接入真实来源。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `calibration.hpp` | connections_confirmed/imu_mounting_confirmed/power_model_calibrated/shooter_constraints_confirmed 默认 false。 |
| `VEHICLE_*` | CHASSIS_ROLE、REFEREE、POWER_BUDGET、VISION_OBSERVE/EXECUTE、SHOOTING、LOCK_PITCH；默认手动台架，不等于比赛整车授权。 |

- 旧 applications/src/board_config 与私有 executor 已删除。
- 电机反馈等待不撤销用户运行意图，输入本身过期仍撤销。
- sample 物理遥控与 applications 键鼠/console 操作 profile 有区别。
- 底盘 CAN1/3/2 已全部用于电机，板间 CAN 后端需重新分配资源。

## 正文与源码

- [dual-controller.md](../applications/dual-controller.md)
- [executors.md](../modules/robotics/executors.md)
- [README.md](../../samples/robotics/vehicle_integration/README.md)

- [main.cpp](../../applications/sentry_gimbal/src/main.cpp)
- [main.cpp](../../applications/sentry_chassis/src/main.cpp)
- [vehicle_bench.hpp](../../samples/robotics/common/vehicle_bench.hpp)
- [vehicle_mc02.overlay](../../samples/robotics/common/vehicle_mc02.overlay)
- [calibration.hpp](../../include/robotics/vehicle/calibration.hpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
