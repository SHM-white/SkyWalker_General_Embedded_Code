# 双主控整车应用

本文按 `dd3f673`（2026-11-15）源码概述正式应用的装配；代码接入不表示已完成接线、运动、带载或故障验收。各板各自拥有 `main.cpp` 和控制循环，共享 [installed 配置](../../config/vehicle/installed/README.md) 与 `include/robotics/vehicle/` 的参数。

## 两板职责与资源

| 板卡 | 输入及资源 | 执行与通信 |
| --- | --- | --- |
| 云台 | UART5 遥控、CAN3 外置头部 IMU、板载 BMI088、USB CDC 视觉、USART1 导航 | CAN1 小 Yaw及发射电机，CAN2 Pitch；USART10 板间 v4；UART7 诊断 |
| 底盘 | USART1 板间 v4、板载 BMI088、USART10 裁判输入 | CAN1 四舵向、CAN2 四轮驱、CAN3 大 Yaw |

云台在配置允许时注册视觉与导航来源；头部 IMU 提供视觉姿态参考，板载 BMI088 提供载体航向和角速度。底盘把通过裁判帧校验的原始数据经板间转发给云台，云台再转发至导航。细节及输入新鲜度要求以 [云台 README](../../applications/sentry_gimbal/README.md)、[底盘 README](../../applications/sentry_chassis/README.md) 和各自源码为准。

## 控制数据流

```text
遥控 / 导航 / 视觉 → CommandManager → 云台轴控制、发射与底盘请求
外置头部 IMU → 视觉姿态参考；板载 BMI088 → 载体航向和角速度
双 Yaw 分配 → 小 Yaw 关节命令 + 0x0402 大 Yaw 世界速度请求
0x0101 云台系平移、0x0402 大 Yaw 请求、0x0404 载体航向
    → 板间 v4 → 底盘执行器 → 四舵轮与大 Yaw
底盘裁判 UART → 原始帧分片 → 板间 v4 → 云台 → 导航 UART
```

底盘使用大 Yaw 原生角与标定前向零位将云台系平移转换到车体系；原生角无效时平移清零。板间命令、生产时间和 boot 边界见 [v4 协议](../protocols/interboard-v4.md)。新应用的裁判转发扩展要求两板都具备对应实现，不能仅依据旧参考底盘的 v4 支持推断兼容。

## 授权和现场前提

遥控解锁、导航档掉线保持、巡航、视觉接管以及停止行为见[云台拨杆说明](../../applications/sentry_gimbal/README.md#拨杆与巡航模式)。各机构还受输入新鲜度、反馈、安装与裁判等条件约束。云台发射执行器目前不要求热量和发射许可；这不代表有载发射已安全验收。底盘实测功率闭环及标定仍待完成；接线、方向、机械行程、停止时限和组合性能均需现场确认。

## 构建与诊断

在仓库目录分别构建并烧录两个应用：

```sh
west build -b dm_mc02/stm32h723xx applications/sentry_gimbal -d ../build/applications/sentry_gimbal
west build -b dm_mc02/stm32h723xx applications/sentry_chassis -d ../build/applications/sentry_chassis
west flash -d ../build/applications/sentry_gimbal
west flash -d ../build/applications/sentry_chassis
```

`CONFIG_SKYWALKER_VEHICLE_DIAGNOSTICS` 控制是否编入诊断；当前共享配置在云台启用、底盘默认关闭。启用时通过 [diagnostics_config.hpp](../../include/robotics/vehicle/diagnostics_config.hpp) 选择 Log/Vofa。正式云台使用 [32 通道 schema 113](../dev/云台双轴响应VOFA通道说明.md)；底盘启用诊断时输出基础 19 通道加 30 项扩展（I19=109，共 49 通道），完整通道见 [底盘 README](../../applications/sentry_chassis/README.md)。UART7/DAPLink 接线仍需现场确认。

2026-10-25 的 27 项编译/链接记录属于旧基线，不覆盖本次源码。本次文档更新未重新构建、测试、烧录或实板验证。
