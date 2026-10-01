# 交互式架构与电机工作链路

> 命令与安全架构已迁移：[现行命令保险与局部恢复](../19-command-recovery.md)。下文旧全局/局部安全链路属于历史说明。

[本地页面](index.html) 展示双主控总览、模块目录、实现状态筛选与 [电机详细链路](index.html#motor-workflow)。本轮同步到 `main` 快照 `fcf2c7fbcda02ee1083b8da19c9e2b19f63b837c`，并补入视觉 AB 通信与独立 IMU 架构。页面是软件说明，不连接硬件，不代表实机遥测或机械安全验收。

## 打开

从仓库根目录执行：

```bash
bash docs/architecture-browser/run.sh
# 端口可选：bash docs/architecture-browser/run.sh 4174
```

打开 <http://127.0.0.1:4173/architecture-browser/#motor-workflow>，Ctrl+C 结束服务。脚本服务整个 `docs` 目录，因此详情中的 Markdown 链接也能访问；Markdown 可能由浏览器直接显示为文本或下载。

也可直接打开 `index.html`。页面无 npm 构建步骤、无 CDN、无后台；GitHub 文件页本身不会执行交互脚本。

## 双主控与感知视图

顶部总览保留三条原有场景，并新增“视觉 / IMU”：

- **正常控制**：DR16 → `CommandManager` → 本地小 Yaw / 板间底盘命令。
- **视觉 / IMU**：`ImuReceiver`、`VisionReceiver` 的已实现边界，以及 `sentry_gimbal` 仍缺的应用装配与 Manual/Auto 仲裁。
- **串口掉线**：底盘本地停输出，云台独立降级。
- **电机掉电恢复**：反馈恢复、generation、连续新命令与重新使能。

模块目录现包含 19 个节点。视觉和 IMU 被明确拆成“模块已实现、主应用未装配”，避免把协议/驱动完成误写成整车自瞄已经可用。

## 当前实现边界

### IMU

`Bmi088Imu`、`DmImuRs485Source`、`ImuState`、`ImuReceiver`、可选 `QuaternionEkf` / `ImuHeater` 已实现。板载 BMI088 与外置 DM-IMU-L1 可发布统一快照，但正式 `sentry_gimbal` 目前没有实例化 `ImuReceiver`，因此姿态尚未进入正式云台控制或视觉反馈链路。RS485 单位、安装方向、四元数方向、持续频率和温控仍需实机确认。

### 视觉

`VisionProtocol` / `VisionLink` / `VisionReceiver` 与 AB 协议已实现：115200 8N1，下行 29 B、上行 43 B。通信层只表达目标和反馈，不持有 IMU、不直接生成 `RobotCommand`。正式 `sentry_gimbal` 尚未实例化 Receiver，也没有完成 Manual/Auto 仲裁；`CommandManager` 当前入口仍是 `OperatorIntent`。

### 电机

统一多品牌核心继续使用 `Motor`、`CanBus`、`Group`、`PositionMotor` / `VelocityMotor`。六种详细场景覆盖目标发送、启动使能、反馈超时、停机竞态、CAN 恢复和锁存故障；`DjiChassisHardware` 使用 1/2 个共享 `CanBus` 管理八台 DJI 电机。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `index.html` | 双板总览、模块详情、电机链路与示例区域；装载 v4 增量脚本 |
| `data.js` | v3 模块、依赖、接口、源码路径与实现状态基线 |
| `app.js` | v3 场景讲解、目录筛选与详情面板 |
| `architecture-refresh.js` | v4 增量：视觉 / IMU 节点、感知场景、应用装配边界与更新基线源码链接 |
| `styles.css` | 总览与模块浏览器样式 |
| `motor-flow.js` / `motor-flow.css` | 电机六场景步骤链路、调用示例与交互 |
| `run.sh` | 仅监听本机的 docs 静态服务器 |

v4 详情中的源码链接固定到上述更新快照，避免文档随 `main` 后续移动而悄悄改变语义。下一次架构、接口或正式应用装配发生变化时，应更新 `architecture-refresh.js` 中的 `UPDATED_COMMIT`、对应节点描述/依赖与本 README；若改动规模继续扩大，再把增量内容折回 `data.js` / `app.js`。

## 检查

```bash
node --check docs/architecture-browser/data.js
node --check docs/architecture-browser/app.js
node --check docs/architecture-browser/architecture-refresh.js
node --check docs/architecture-browser/motor-flow.js
bash -n docs/architecture-browser/run.sh
```

浏览器检查重点：四个总览场景、19 个模块、状态筛选、视觉/IMU 模块详情、六种电机链路、示例复制，以及窄屏布局。页面只在主动点击播放时定时切换，并尊重系统减少动画偏好。

## 发布

既有在线入口：<https://skywalker-architecture-browser.docile-raven-1977.chatgpt.site/>。仓库改动不会自动同步该站点；发布时需一起部署本目录 HTML/CSS/JS 并保留原访问设置，不要提交凭据或访问名单。
