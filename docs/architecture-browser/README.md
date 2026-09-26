# 交互式架构与电机工作链路

[本地页面](index.html) 保留双主控总览、模块目录、搜索与实现状态筛选，新增 [电机详细链路](index.html#motor-workflow)。页面是软件说明，不连接硬件，不代表实机遥测或机械安全验收。

## 打开

从仓库任意目录执行脚本（以下命令以仓库根为当前目录）：

```bash
bash docs/architecture-browser/run.sh
# 端口可选：bash docs/architecture-browser/run.sh 4174
```

打开 <http://127.0.0.1:4173/architecture-browser/#motor-workflow>，Ctrl+C 结束服务。脚本服务整个 docs 目录，使完整 Markdown 文档的相对链接也能访问；Markdown 在浏览器中可能作为文本显示或下载。

也可直接打开 `index.html`，保留同目录所有 CSS/JS。复制按钮在不支持剪贴板 API 时选中代码供手工复制。页面无 npm 构建步骤、无 CDN、无后台。GitHub 文件页不会执行交互。

已有根目录 `run.sh` 若只服务 `docs/architecture-browser`，页面交互仍可用，但上一级 Markdown 链接不可访问；建议使用这里的新脚本。

## 电机视图

六种场景：目标与发送、启动与使能、反馈与超时、停机竞态、CAN 恢复、锁存故障。

每种场景包含：

- 应用、控制器、Motor、Group、发布区、候选、授权、CAN、回调、反馈、完成、恢复的模块调用图。
- 当前步骤的连线、执行线程、输入输出、代次和状态说明。
- 上一步、下一步、直接选择步骤、自动播放、暂停和重置；切换场景或隐藏页面会停止播放。
- DJI 同 CAN、跨 CAN 云台、DM/软件控制器、多线程协调的调用片段与复制按钮。

节点点击查看职责，不会修改真实参数或执行电机命令。状态和 sequence / generation 数字均为讲解示意值。

## 文件职责与基线

| 文件 | 职责 |
| --- | --- |
| `index.html` | 双板总览、模块详情、电机链路与示例区域 |
| `styles.css` / `app.js` / `data.js` | 原有架构浏览器样式、场景、目录与模块数据 |
| `motor-flow.css` / `motor-flow.js` | 电机图、六条步骤链路、调用示例与交互 |
| `run.sh` | 仅监听本机的 docs 静态服务器 |

双板总览原始快照为 `77b053875425827fe0e805f578c1131ee22a3315`，导入网页源版本为 `9a931981ef5c7aab4a1eec1bdae9204151f7c160`。本次电机模块、云台控制器与底盘电机适配器描述按当前工作区同步，标为“源码路径 · 当前工作区”，不把尚未发布的修改链接到旧提交。其他历史模块的源码链接仍固定到原基线。

新增链路与 [17 完整文字文档](../17-motor-workflow.md) 一起维护。已有代码、待实机、待实现分别表示代码存在、硬件链路仍需验收、契约/执行链尚未实现。线程数不设固定总上限；唯一写入方与共享 CAN 发布协调是必须说明的边界。

## 检查

```bash
node --check docs/architecture-browser/data.js
node --check docs/architecture-browser/app.js
node --check docs/architecture-browser/motor-flow.js
bash -n docs/architecture-browser/run.sh
```

浏览器检查桌面和 390 px 手机宽度：原有场景/模块/筛选，新六种链路全步骤、节点点击、播放/暂停/重置、四种示例和复制，无页面横向溢出或控制台错误。手机上模块纵向排列，以步骤说明表达跨模块调用，避免长连线遮挡内容。

页面只在主动点击播放时定时切换；尊重系统减少动画偏好，不自动播放。

## 发布

既有在线入口：<https://skywalker-architecture-browser.docile-raven-1977.chatgpt.site/>，其访问控制独立于本目录。本轮更新本地文件，不自动同步在线站点。后续发布需将本目录 HTML/CSS/JS 一起部署，保留原访问设置；不要上传凭据或访问名单。
