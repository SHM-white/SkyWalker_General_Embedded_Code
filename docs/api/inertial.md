# 头部惯性云台适配：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

头部 IMU 与机械反馈将惯性目标转成双轴关节 Rate；两轴数学有效性分别返回，电机恢复不改业务目标。

**接入状态：接入 / 配置未完成。** 软件接口与整车装配已有；中央硬件/安装/测量确认未完成，不代表实机验收。

## 职责与关联

头部 IMU 与机械反馈将惯性目标转成双轴关节 Rate；两轴数学有效性分别返回，电机恢复不改业务目标。

输入 / 依赖：[IMU：独立采集、姿态与加热](imu.md)、[云台单轴与本地执行](gimbal.md)、[命令仲裁与后台服务](command.md)

消费者：[双主控应用与执行器](application.md)

## 接口契约

### 1. InertialGimbalAdapter(const Config &); InertialGimbalOutput update(const InertialGimbalInputs &, core::TimeUs now_us)

```cpp
InertialGimbalAdapter(const Config &); InertialGimbalOutput update(const InertialGimbalInputs &, core::TimeUs now_us)
```

输入头部 Snapshot、两轴 MotorSnapshot、command/source_stamp/prerequisites_ready，输出关节 command、原 stamp、两轴 output_valid、stabilization_valid 与误差。

**返回 / 输出：** 值副本；不写电机、不持有总线。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** IMU/参考/质量不足时相应数学输出无效；视觉参考须匹配头部 frame_id/epoch。

## 调用示例

### 头部惯性云台适配 周期

```cpp
const auto adapted = adapter.update(inputs, now_us);
mechanical_inputs.command = adapted.command;
mechanical_inputs.source_stamp = adapted.source_stamp;
mechanical_inputs.yaw_output_valid = adapted.yaw_output_valid;
mechanical_inputs.pitch_output_valid = adapted.pitch_output_valid;
const auto status = gimbal.update(mechanical_inputs, now_us);
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
| `InertialGimbalAdapter::Config` | 头部超时20ms、机械50ms、输入100ms；pitch_locked=true，方向/安装/姿态质量必须确认。 |

- API 已存在不等于实物标定和闭环通过。
- 状态生产时间不能被读者/通信刷新。

## 正文与源码

- [executors.md](../modules/robotics/executors.md)
- [dual-controller.md](../applications/dual-controller.md)

- [inertial_gimbal.hpp](../../include/robotics/gimbal/inertial_gimbal.hpp)
- [inertial_gimbal.cpp](../../lib/robotics/inertial_gimbal.cpp)
- [main.cpp](../../samples/robotics/inertial_gimbal/src/main.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
