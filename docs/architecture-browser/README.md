# 交互式架构浏览器

本地浏览器展示双主控数据流、模块关系和电机异步链路。用 docs/architecture-browser/run.sh 启动本地静态服务器，再打开 http://127.0.0.1:8000。页面没有 npm、CDN 或后台依赖。

架构图由 data.js 的模块基线、app.js 的交互逻辑和 architecture-refresh.js 的更新层组成。当前更新层显示注册式命令来源服务：RemoteSource / VisionSource / RefereePermissionSource → CommandManager worker → CommandArbiter → 非消费式 CommandSnapshot。全量 API、配置和应用状态以[文档中心](../README.md)及当前源码为准。

## 当前代码状态

- CommandManager 是来源注册和后台仲裁服务；同步策略核心是 CommandArbiter。启动前注册 Operator/Aim 来源并绑定许可源，之后消费者用 snapshot() 或 current() 读结果。
- samples/robotics/command_manager 已接入遥控、视觉和裁判来源；command_safety 只接入遥控。
- applications/sentry_gimbal 当前只注册 RemoteSource 和 RefereePermissionSource，没有注册 VisionSource，allow_auto=false。board_config 的 RefereeVersion 仍为 Unspecified，connections_configured=false。
- GimbalExecutor、ChassisExecutor 是应用内部执行封装。旧 GlobalSafetyManager、GimbalLocalSafety、ChassisLocalSafety 和 CommandRouter 类不属于当前源码接口。
- BMI088、DM-IMU-L1 RS485、ImuReceiver、VisionReceiver 和 AB 协议各自已实现；正式 sentry_gimbal 目前没有将 IMU 或视觉接入命令与姿态反馈主链。外设方向、时序和机械响应仍需实机核验。

交互视图用于解释模块关系，不能替代源文件核对、构建或硬件验收。点击节点查看的源码链接固定到 architecture-refresh.js 中的代码快照；更新源码接口后需同时更新该快照和本说明。

## 更新边界

data.js 与 app.js 保留原始图布局。architecture-refresh.js 提供后来加入的模块和当前命令链更新。若更新层与源码冲突，以源码和正式主题文档为准；调整交互场景时同步更新本文件中的当前状态说明。
