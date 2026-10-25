# 架构与接口浏览器

浏览器负责把已有文字文档和当前源码组织成可阅读的整车视图。`docs/getting-started`、`docs/modules`、`docs/applications`、`docs/guides` 和 `docs/dev` 目录继续保留；正文在页面内直接读取，源码也直接读取本地工作区。

## Github Pages 在线部署链接

[https://shm-white.github.io/SkyWalker_General_Embedded_Code/docs/architecture-browser/](https://shm-white.github.io/SkyWalker_General_Embedded_Code/docs/architecture-browser/)

## 启动

在仓库根目录运行：

```sh
bash docs/architecture-browser/run.sh
```

打开 [http://127.0.0.1:4173/docs/architecture-browser/#overview](http://127.0.0.1:4173/docs/architecture-browser/#overview)。脚本启动 Python 文件服务器，并按请求扫描工作区 Markdown，按 Ctrl+C 停止。自定义端口可运行 `bash docs/architecture-browser/run.sh 8000`；从任意工作目录运行该脚本，都以仓库根目录提供文件。

页面无需 npm、CDN 或额外服务；扫描器仅使用 Python 标准库。使用服务器打开，才能通过 `fetch` 读取 Markdown 和源码；直接双击 `index.html` 不适合文件阅读。旧服务器若仍在以 `docs/` 为根目录运行，先停止，再用新脚本启动。

## 怎么阅读

| 入口                     | 解决的问题                                                                            |
| ------------------------ | ------------------------------------------------------------------------------------- |
| 整车架构 → 当前实际接入 | 现在源码真正连通了什么？两板如何分工？哪些配置仍阻断输出？                            |
| 整车架构 → 最终上车蓝图 | 视觉、IMU、双轴、底盘、发射和观测最终应如何装配？哪些部分仍待实现？                   |
| 模块关系                 | 谁依赖谁？点击节点高亮直接上游和消费者，并进入详细接口。                              |
| 接口与示例               | 21 个逻辑模块的职责、主要 API、参数、返回、错误、线程边界、调用顺序、配置和真实用法。 |
| 端到端调用链             | 遥控、视觉、板间底盘、IMU 和异常恢复，每一步由谁推进？                                |
| Markdown 文档            | 保留已有目录结构，内嵌阅读模块正文、指南、开发记录及样例 README。                     |
| 变更来历                 | Git 中何时加入增量覆盖层，注明的目的是什么？                                          |

顶部搜索支持模块说明、Markdown 接口签名、参数、示例、配置，以及文档标题和路径；接口检索文本由扫描器从 `docs/api` 自动提取，不需手工维护。按 `/` 聚焦搜索框；示例提供复制按钮。模块页的右侧目录可以直接跳到接口或示例，页面路由可收藏和分享给使用同一份本地服务器的人。

示例分为具体工程中的周期片段和带构造上下文的使用示意，适用范围写在代码块下方。真实设备、PID、机械参数及完整线程入口仍以链接到的应用或样例源码为依据。

## 状态怎么理解

- **已有代码**：当前仓库存在这项模块能力或调用路径。它不代表已完成真实接线、带载验证或整车验收。
- **接入 / 配置未完成**：基础模块已实现，但正式应用尚未装配，或板级、协议、参考、功率等配置仍有缺口。
- **目标 / 待实现**：最终蓝图中的建议扩展，当前没有对应的完整应用链路。

总览的箭头表示运行时数据方向；模块关系图的箭头表示“被依赖模块 → 使用者”，两种关系有各自说明。总览中未连通的感知支路保持独立，蓝图中的未完成连接以虚线标注。

## 当前源码与发布基线

本轮应用与实车文档对齐固件 `48984ed90898726e8a9d451ac07bdc88c4663f11`（2026-10-25）。Markdown 与源码读取指向本地或部署的同份快照；本轮只修改仓库文档及浏览器文档数据，没有发布在线站点。GitHub Pages 内容需等相应分支部署后才更新，不能把本地修改当成已上线。

当前两个 sentry 入口分别拥有 `main.cpp` 装配：云台是物理 RC、CAN3 头 IMU、惯性双轴与回中；底盘是四舵轮与独立大 Yaw。它们共享 [installed 配置](../../config/vehicle/installed/README.md) 和 `include/robotics/vehicle/` 纯参数，不再共用 `vehicle_bench.hpp::run()`。21 个固定上车工程见[入口表](../../samples/bringup/README.md)，历史宏组合 sample 仍保留用于理解演进。

电机继续采用持续目标、逐轴反馈与独立恢复；状态生产 stamp 不因通信重发续期，板间固定 v4，完整线协议见[统一契约](../protocols/interboard-v4.md)。当前诊断候选为 UART7 经 DAPLink 单口选择 Log/Vofa；云台板间 USART10、底盘板间 USART1。

中央连接、IMU 安装和发射约束标志沿用 true，功率模型为 false；确认 bool 不能代替现场记录。上一轮共 27 个工程编译、链接通过，未运行测试、烧录或实板验证。当前最终应用不启动视觉或发射，不启用实测功率闭环；真实热量/发射许可、拨盘参考与视觉会话仍有输入缺口。协议车体实测速度/功率摘要尚未填充。

详情见[双主控正文](../applications/dual-controller.md)、[公开执行器](../modules/robotics/executors.md)、[诊断 API](../api/telemetry.md)和[上车说明](../../samples/bringup/README.md)。在线私有站点原地址保留为 [历史部署入口](https://skywalker-architecture-browser.docile-raven-1977.chatgpt.site/)，本轮未向它发布。

## 为什么重建

`32f07b9`（2026-09-30 03:30:05，北京时间）新增 `architecture-refresh.js`，`d860163`（03:30:47）在 HTML 中启用它，`a5054de`（03:31:07）说明目的是补视觉与独立 IMU。之后 `cedab2a` 继续追加命令服务覆盖，旧基础节点和后续描述同时保留，页面靠运行时替换维持当前说明。Git 记录没有说明要替代 Markdown 目录。

新版移除旧刷新层与独立电机渲染脚本，用一个渲染入口、关系图摘要和独立 Markdown 接口正文提供完整页面。电机链路的接口、生命周期、调用示例仍在 DJI、DM、电机控制器、恢复场景及[电机工作流正文](../guides/motor-workflow.md)中。

## 维护文件

| 文件                            | 维护内容                                                      |
| ------------------------------- | ------------------------------------------------------------- |
| `index.html` / `styles.css` | 页面骨架、导航、排版及图形外观                                |
| `app.js`                      | 路由、页面渲染、关系高亮、搜索、示例复制、Markdown / 源码读取 |
| `data.js`                     | 模块摘要、分类、状态、依赖图布局和 Markdown 参考路径          |
| `architecture-data.js`        | Git 历史、当前调用路径、缺口、上车蓝图、场景与启动顺序        |
| `vehicle-diagram.js`          | 当前与目标主图的节点、物理分区、显式数据连线及标签            |
| `workspace_docs.py`          | 自动扫描 Markdown、提取标题与分组；本地动态索引和部署生成共用 |
| `run.sh`                      | 以仓库根目录启动文件服务器和动态扫描端点                       |

修改公开接口时更新 `docs/api/*.md` 和模块主题正文；签名、参数、返回、时序、错误、示例、生命周期和配置只在 Markdown 中维护。`data.js` 保留导航摘要、依赖和实际接入状态，不再保存接口 / 示例数组。全部模块详情复用 Markdown 阅读器，编辑器和 GitHub 可直接阅读；浏览器的代码块仍支持复制。每轮更新按[文档同步清单](../maintenance.md)执行。`depends` 使用已有模块 ID，供依赖图和上下游文字导航共用。

修改实际应用装配时同步当前链路、缺口和主图；蓝图仍待实现的连接不得标成已接入。主图中的节点使用仓库相对源码路径关联模块，不固定到历史提交。

## 自动发现 Markdown

无需维护文档列表。`run.sh` 以仓库根目录作为工作区，每次进入 Markdown 文档页、搜索或点击“刷新目录”时，页面请求 `docs-index.json`，服务器重新递归扫描 `.md` 文件（扩展名不区分大小写）。尚未提交的新增文件也会出现，删除、改名和标题修改在下次扫描后生效；读取正文仍直接请求原文件。

标题来自第一个 Markdown 标题（支持 ATX 和 Setext，跳过围栏代码块），无标题时使用文件名。已知文档目录按入门、模块、应用、指南、开发记录分组，样例和其他目录按路径分组；根 README、旧迁移说明和自建目录也自动加入。隐藏文件/目录、`AGENTS.md`、符号链接、`build`/`build-*`、`dist`、`_site`、`node_modules`、`target`、`vendor`、`__pycache__`、`venv`、`env`、`coverage`、`htmlcov` 被排除，避免把依赖、内部指令或生成产物加入目录。

GitHub Pages 的发布流程在仓库归档后自动生成索引。在线静态站点展示的是该次部署的 Markdown 快照，无法直接扫描访问者的本地磁盘；新增或删除文档在下一次部署后生效。其他静态部署在打包前运行（`_site` 是准备好的部署根目录）：

```sh
python3 docs/architecture-browser/workspace_docs.py \
  --root _site --output _site/docs/architecture-browser/docs-index.json
```

普通 `python3 -m http.server` 不提供动态扫描，需要先生成同路径 JSON；使用 `run.sh` 不需要生成或提交它。生成文件已被 Git 忽略。目录加载失败时页面提供重试提示，并保留上次成功加载的结果；全局搜索和总览数量使用同一份目录。

阅读器处理标题、段落、链接、代码块、表格、列表、引用和任务项，不执行 Markdown 内的 HTML 或脚本；原始文件入口始终保留。正文中的 Mermaid 显示源定义，交互架构图由本浏览器直接绘制。
