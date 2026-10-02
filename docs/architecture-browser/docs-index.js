'use strict';

// The Markdown files remain the text sources; this index only supplies navigation.
window.SKYWALKER_DOCS = [
  {
    "title": "SkyWalker 文档中心",
    "path": "docs/README.md",
    "group": "文档入口"
  },
  {
    "title": "双主控应用骨架",
    "path": "docs/applications/dual-controller.md",
    "group": "整机与联动"
  },
  {
    "title": "模块联动与调用路线",
    "path": "docs/applications/module-integration.md",
    "group": "整机与联动"
  },
  {
    "title": "架构与接口浏览器",
    "path": "docs/architecture-browser/README.md",
    "group": "文档入口"
  },
  {
    "title": "开发专题与状态记录",
    "path": "docs/dev/README.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "rm_typec 设备树 `spi-max-frequency` 提示诊断",
    "path": "docs/dev/rm_typec设备树SPI频率属性诊断指南.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "三源命令管理 sample 手工实施指南",
    "path": "docs/dev/三源命令管理sample手工实施指南.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "云台 GimbalAxis 与 Gimbal 分层重构方案",
    "path": "docs/dev/云台GimbalAxis与Gimbal分层重构方案.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "云台遥控样例安全策略阅读指南",
    "path": "docs/dev/云台遥控样例安全策略阅读指南.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "命令来源注册与后台仲裁服务实施指南",
    "path": "docs/dev/命令来源注册与后台仲裁服务实施指南.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "命令管理器调用接口简化实施指南",
    "path": "docs/dev/命令管理器调用接口简化实施指南.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "哨兵空闲巡航与搜索模式规划",
    "path": "docs/dev/哨兵空闲巡航与搜索模式规划.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "多品牌电机驱动重构架构与手工实施指南",
    "path": "docs/dev/多品牌电机驱动重构架构与手工实施指南.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "安全策略收敛与执行层自恢复实施指南",
    "path": "docs/dev/安全策略收敛与执行层自恢复实施指南.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "当前项目完成度分析与后续实施清单",
    "path": "docs/dev/当前项目完成度分析与后续实施清单.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "板间通信统一传输接口与三种后端实施指南",
    "path": "docs/dev/板间通信统一传输接口与三种后端实施指南.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "电机驱动审查复核与线程弹性实施指南",
    "path": "docs/dev/电机驱动审查复核与线程弹性实施指南.md",
    "group": "开发记录与待实施方案"
  },
  {
    "title": "架构与构建",
    "path": "docs/getting-started/architecture.md",
    "group": "工程与入门"
  },
  {
    "title": "03 板级支持",
    "path": "docs/getting-started/boards.md",
    "group": "工程与入门"
  },
  {
    "title": "01 快速开始",
    "path": "docs/getting-started/quickstart.md",
    "group": "工程与入门"
  },
  {
    "title": "10 样例索引",
    "path": "docs/getting-started/samples.md",
    "group": "工程与入门"
  },
  {
    "title": "11 调试与观测",
    "path": "docs/guides/debugging.md",
    "group": "调试与工作流"
  },
  {
    "title": "17 电机驱动工作链路、并发与调用示例",
    "path": "docs/guides/motor-workflow.md",
    "group": "调试与工作流"
  },
  {
    "title": "12 故障排查",
    "path": "docs/guides/troubleshooting.md",
    "group": "调试与工作流"
  },
  {
    "title": "STM32 UART DMA 的 nocache 报错：原因、修复与排查",
    "path": "docs/guides/uart-dma.md",
    "group": "调试与工作流"
  },
  {
    "title": "公共模块地图",
    "path": "docs/modules/README.md",
    "group": "模块与接口"
  },
  {
    "title": "封装模块调用示例",
    "path": "docs/modules/call-examples.md",
    "group": "模块与接口"
  },
  {
    "title": "13 通信层：UART、视觉、DR16、裁判与板间协议",
    "path": "docs/modules/communication/communication.md",
    "group": "模块与接口"
  },
  {
    "title": "板间 UART、RS485 与 CAN",
    "path": "docs/modules/communication/interboard-transports.md",
    "group": "模块与接口"
  },
  {
    "title": "18 视觉独立模块与 AB 协议",
    "path": "docs/modules/communication/vision.md",
    "group": "模块与接口"
  },
  {
    "title": "08 纯 C 控制算法库",
    "path": "docs/modules/control/algorithms.md",
    "group": "模块与接口"
  },
  {
    "title": "09 统一速度 / 位置电机控制",
    "path": "docs/modules/control/motor-control.md",
    "group": "模块与接口"
  },
  {
    "title": "06 IMU 独立源、滤波和温控",
    "path": "docs/modules/drivers/imu.md",
    "group": "模块与接口"
  },
  {
    "title": "07 Kalman 滤波与矩阵库",
    "path": "docs/modules/drivers/kalman-matrix.md",
    "group": "模块与接口"
  },
  {
    "title": "04 DJI CAN 电机驱动",
    "path": "docs/modules/drivers/motor-dji.md",
    "group": "模块与接口"
  },
  {
    "title": "05 达妙 DM CAN 电机驱动",
    "path": "docs/modules/drivers/motor-dm.md",
    "group": "模块与接口"
  },
  {
    "title": "命令仲裁与执行恢复",
    "path": "docs/modules/robotics/command-recovery.md",
    "group": "模块与接口"
  },
  {
    "title": "注册式命令来源与后台仲裁服务",
    "path": "docs/modules/robotics/command-service.md",
    "group": "模块与接口"
  },
  {
    "title": "机器人决策与执行模块",
    "path": "docs/modules/robotics/robotics.md",
    "group": "模块与接口"
  },
  {
    "title": "独立上机微型项目",
    "path": "samples/FRAMEWORK_SAMPLES.md",
    "group": "可构建样例"
  },
  {
    "title": "DR16 实机输入与 VOFA 回显",
    "path": "samples/communication/dr16/README.md",
    "group": "可构建样例"
  },
  {
    "title": "两板 UART / RS485 / CAN 联调",
    "path": "samples/communication/interboard/README.md",
    "group": "可构建样例"
  },
  {
    "title": "裁判 UART 实机输入",
    "path": "samples/communication/referee/README.md",
    "group": "可构建样例"
  },
  {
    "title": "视觉 AB 指令接收与 VOFA 回显",
    "path": "samples/communication/vision/README.md",
    "group": "可构建样例"
  },
  {
    "title": "板载与 RS485 外置 IMU 同时读取",
    "path": "samples/imu/dual_imu/README.md",
    "group": "可构建样例"
  },
  {
    "title": "DM-J4310 控制示例",
    "path": "samples/motor/DM_J4310_EXAMPLES.md",
    "group": "可构建样例"
  },
  {
    "title": "电机速度与位置闭环样例",
    "path": "samples/motor/MOTOR_CONTROL.md",
    "group": "可构建样例"
  },
  {
    "title": "多电机 CAN 拓扑台架样例",
    "path": "samples/motor/mixed_topology/README.md",
    "group": "可构建样例"
  },
  {
    "title": "单电机断电恢复",
    "path": "samples/motor/recovery/README.md",
    "group": "可构建样例"
  },
  {
    "title": "三源命令管理台架",
    "path": "samples/robotics/command_manager/README.md",
    "group": "可构建样例"
  },
  {
    "title": "遥控命令保险观察台架",
    "path": "samples/robotics/command_safety/README.md",
    "group": "可构建样例"
  },
  {
    "title": "遥控双轴电机样例",
    "path": "samples/robotics/gimbal_control/README.md",
    "group": "可构建样例"
  },
  {
    "title": "一套真实舵轮模块",
    "path": "samples/robotics/swerve/README.md",
    "group": "可构建样例"
  },
  {
    "title": "单轴 Yaw 实机调试",
    "path": "samples/robotics/yaw_gimbal/README.md",
    "group": "可构建样例"
  }
];
