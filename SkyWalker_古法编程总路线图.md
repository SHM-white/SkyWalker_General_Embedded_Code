# SkyWalker 古法编程总路线图

> 这是整个项目的“先看我”文档。
>
> 目标：把现有五份专题手册串成一条不会迷路的施工主线。你不用按创建时间依次读完所有文档；每到一个阶段，只打开该阶段指定的章节。
>
> 本文只整理路线、依赖、阶段出口和阅读索引，没有修改任何业务源码。

## 0. 你现在到底该做什么

当前只做下面四件事，其他文档先关掉：

1. 打开《SkyWalker 多型号电机驱动古法实施手册》第 4、6、7 节。
2. 修正 `motor.hpp` 的三个函数指针检查。
3. 把 DJI 公共反馈和 TxGroup 从 M3508 专用改成 DJI 通用，但保持现有 M3508 行为不变。
4. 重新构建并实测现有 M3508：先只收反馈，再发全 0，最后才发很小的 raw 电流。

在 M3508 回归通过前，不写 M2006、不写 GM6020、不写 PID、不写舵轮、不写裁判系统。

如果你已经在真实硬件上完成过上述四件事，并且能拿出构建记录、反馈日志和全 0 停机结果，就从本文“阶段 2：电机公共层”开始逐项核对；不要只凭“以前好像转过”跳阶段。

## 1. 当前仓库实际处于什么位置

只读核查当前源码后，状态如下：

| 模块 | 当前状态 | 还缺什么 |
|---|---|---|
| M3508/C620 | 已有 Zephyr device、CAN RX、raw 电流接口、单组 TxGroup 和 sample 骨架 | 公共 API 修错、通用化、纯协议测试、硬件回归和安全停机验证 |
| 公共 motor API | 已有 `State/Feedback/Api` | `disable/readFeedback/getState` 错查 `enable` 指针；统一输出单位仍需明确 |
| DJI 协议层 | 已有 M3508 反馈解码和四槽打包 | 改成不带 M3508 型号名的公共 codec |
| DJI TxGroup | 能绑定 M3508 | 内部硬编码 M3508 namespace，不能绑定 M2006/GM6020 |
| M2006 | 未实现 | binding、driver、sample、测试 |
| GM6020 | 未实现 | 默认电流模式、分组 ID、driver、sample、测试 |
| DM-J4310 | 未实现 | 独立协议族、MIT codec、特殊帧、driver、sample、测试 |
| PID | 已有最小位置式 PID | reset、非法 dt/finite 检查、测量微分、滤波、anti-windup、结果遥测 |
| 应用 | `application/` 三个入口仍为空 | 等底层稳定后再拆 chassis/gimbal 两个 application |
| 舵轮 | 未实现 | 数学、单模块控制、四模块控制、状态机 |
| 双板通信 | 未实现 | codec、framing、freshness、enable token |
| 大小 yaw | 未实现 | 两轴独立闭环、坐标关系、唯一协调器、卸载 |
| 裁判系统 | 未实现 | RX parser、当前赛季 decoder、store、TX scheduler、UI |

所以你现在处于：

```text
M3508 骨架已有
    ↓
必须先做“基线回归 + 公共层收口”
    ↓
才能开始新增电机型号
```

## 2. 五份专题文档分别是干什么的

### 2.1 《SkyWalker 电机驱动从零重写实施手册》

定位：M3508 基线的历史施工手册。

现在怎么用：

- 不再从第一页完整照抄，因为其中大部分骨架已经存在于源码。
- M3508 构建、设备树、RX callback、TxGroup 或 sample 看不懂时，再查对应章节。
- 它是“为什么当前代码长这样”的说明书，不是当前总主线。

### 2.2 《SkyWalker 多型号电机驱动古法实施手册》

定位：当前第一主线文档。

它负责：

- 修公共 motor API。
- DJI 公共协议和通用 TxGroup。
- M2006、GM6020、DM-J4310。
- PID、串级控制、前馈、调参和单电机闭环。

从现在直到单电机闭环完成，它是你主要阅读的文档。

### 2.3 《SkyWalker 舵轮解算与大小 Yaw 双板协同古法实施框架》

定位：第二主线文档。

只有在以下前提全部满足后再进入：

- M3508/M2006/GM6020 等实际使用电机能独立安全收发。
- PID 基础能力补齐。
- 一只 M3508 速度环稳定。
- 一只 GM6020 位置—速度—电流串级稳定。

它负责舵轮数学、单模块、四模块、双板协议和大小 yaw。

### 2.4 《SkyWalker 对 sp_middleware 的可复用性审计与迁移指南》

定位：参考字典，不是施工主线。

用法：

- 写舵轮最短转向时查第 4 节。
- 写通用关节控制器时查第 5 节。
- 写 PID/前馈时查第 6～7 节。
- 写角度展开、滤波时查第 11 节。
- 写板间流解析时查第 10 节。

不要按第 0 节一路读到最后，也不要复制参考仓库函数体。

### 2.5 《SkyWalker 裁判系统架构古法实施框架》

定位：独立支线，后期接入主线。

先实现 RX parser、decoder 和 store；等底盘能基本运动后，再用裁判数据做功率策略；UI、图传和自定义客户端最后做。

## 3. 全项目唯一推荐顺序

```text
阶段 0  参数、安全和测试环境
   ↓
阶段 1  回归现有 M3508 基线
   ↓
阶段 2  收口 motor API、DJI codec、通用 TxGroup
   ↓
阶段 3  M2006 → GM6020 电流模式 → DM-J4310
   ↓
阶段 4  重构基础 PID 与轨迹/角度小工具
   ↓
阶段 5  单电机闭环：驱动轮速度、舵向串级、yaw 单轴
   ↓
阶段 6  纯舵轮数学：逆解、最短转向、缩放、正解
   ↓
阶段 7  一只舵轮模块 → 四只模块 → 底盘状态机
   ↓
阶段 8  双板协议和两个 application
   ↓
阶段 9  大 yaw、小 yaw 独立闭环 → 协调卸载
   ↓
阶段 10 裁判 RX/store → 功率策略
   ↓
阶段 11 UI、多机通信、前馈增强、打滑诊断等后期功能
```

这是一条串行主线。一个人做项目时，不建议“电机写一半、顺手写 UI、再回来调大小 yaw”。每次只让一个阶段处于进行中。

## 4. 阶段 0：先填参数和安全表

### 目标

把会决定代码结构的硬件事实写清楚，避免用临时负号和魔法常量调车。

### 需要记录

```text
M3508 数量、ID、CAN 总线、安装方向：___
M2006 数量、ID、CAN 总线、安装方向：___
GM6020 数量、ID、CAN 总线、零点、软/硬限位：___
DM-J4310 数量、Motor ID、Master ID、模式、CAN 总线：___
四个舵轮模块 x/y 位置：___
驱动轮半径：___
各电机真实传动比：___
大 yaw 型号、零点、方向、限位：___
小 yaw 型号、零点、方向、限位：___
两块板的 UART/CAN/CAN-FD 分配：___
IMU 位于哪块板、安装在哪一级：___
裁判常规链路 UART：___
图传链路 UART：___
```

### 安全前提

- 真电机第一次上电必须架空或拆除负载。
- 先发全 0 并确认停机路径，再发小命令。
- 急停/断电方式必须伸手可及。
- DJI 和达妙如果存在 CAN ID 冲突，放不同总线。
- 不用手抓正在闭环的舵向、yaw 或驱动轮。

### 出口条件

- [ ] 表中所有当前实际使用项已经填写。
- [ ] 能明确说出每种电机在哪条 CAN 上。
- [ ] 能明确说出急停和软件失联后的零输出路径。

## 5. 阶段 1：回归现有 M3508 基线

### 本阶段只看

- 《电机驱动从零重写实施手册》第 11～16 节。
- 《多型号电机驱动古法实施手册》第 3～4 节。

### 由你修改的文件

```text
include/drivers/motor/motor.hpp
samples/motor/dji_m3508_driver/src/main.cpp
必要时检查：
drivers/motor/dji/dji_m3508.cpp
drivers/motor/dji/dji_protocol.cpp
drivers/motor/dji/dji_tx_group.cpp
```

### 实施顺序

1. 修正 `disable()` 检查 `api->disable`。
2. 修正 `readFeedback()` 检查 `api->read_feedback`。
3. 修正 `getState()` 检查 `api->get_state`。
4. sample 第一轮把目标 raw 改为 0，只验证反馈和分组发送。
5. 确认 encoder/rpm/current/temperature 字节顺序。
6. 确认拔 CAN 后按 heartbeat timeout 变 Offline。
7. 再使用很小 raw 值确认方向。
8. 最后绑定第二台 M3508，确认两个槽位不会互相覆盖。

### 出口条件

- [ ] sample 能稳定构建和烧录。
- [ ] raw=0 时发送帧的八个数据字节全为零。
- [ ] 电机反馈 ID、command ID、slot 与实物一致。
- [ ] 一台和两台 M3508 都能正确收反馈。
- [ ] 小命令方向符合记录表。
- [ ] `clear()` 后下一帧确实发送零，而不是只清本地变量后停止发送。
- [ ] 断 CAN、发送失败和正常停机行为已经记录。

任何一项失败，都留在本阶段。

## 6. 阶段 2：收口电机公共层

### 为什么必须先做

如果直接复制 `dji_m3508.cpp` 写 M2006 和 GM6020，之后会出现三套反馈结构、三套分组逻辑和三个互相不兼容的 TxGroup。

### 本阶段只看

- 《多型号电机驱动古法实施手册》第 6～7、12.1～12.2、19 节。
- 《sp_middleware 可复用性审计与迁移指南》第 9 节只作风险核对。

### 由你修改/新增的文件

```text
include/drivers/motor/dji_protocol.hpp
drivers/motor/dji/dji_protocol.cpp
include/drivers/motor/dji_tx_group.hpp
drivers/motor/dji/dji_tx_group.cpp
include/drivers/motor/dji_m3508.hpp
drivers/motor/dji/dji_m3508.cpp
tests/motor/dji_protocol/            后续由你创建
```

### 只做四件事

1. 把 `M3508Feedback` 抽成协议级 `DjiFeedbackRaw`，保留 M3508 兼容别名或包装。
2. 把 `buildGroupCurrentFrame()` 改成不假设物理单位的 `buildGroupCommandFrame()`。
3. 给 TxGroup 定义 `GroupCommandSource`/adapter，使它不调用 M3508 namespace。
4. 给解码和四槽打包写固定向量测试。

### 出口条件

- [ ] DJI codec 不引用任何具体电机 namespace。
- [ ] TxGroup 不引用 M3508 namespace。
- [ ] 重复 slot、错误 command ID、错误 CAN device 都返回明确错误。
- [ ] 未绑定槽位发送零。
- [ ] M3508 行为和阶段 1 完全一致。
- [ ] 纯协议测试不需要真实电机。

## 7. 阶段 3：依次增加电机型号

一次只写一个型号，每个型号都是：

```text
协议/配置
  → RX only
  → 全 0 TX
  → 小命令
  → 离线/失能
  → 两台或分组边界
  → 该型号完成
```

### 3A：M2006/C610

先看《多型号电机驱动古法实施手册》第 8、11～15 节。

原因：它与 M3508 最接近，最适合验证公共 DJI 层是否真的通用。

关键区别：

- raw 电流范围 `+-10000`。
- C610 反馈第 6/7 字节不是有效温度。
- 默认减速比与 M3508 不同。

出口：单台和同组多台 M2006 收发、清零、离线全部通过。

### 3B：GM6020 电流模式

再看《多型号电机驱动古法实施手册》第 9 节。

关键点：

- 本项目默认电流模式。
- ID 1～4 使用 `0x1FE`，5～7 使用 `0x2FE`。
- raw `+-16384` 约对应 `+-3A`。
- 反馈 ID 是 `0x204 + motor_id`。
- 不要把电流模式和电压模式混入同一隐式 API。

出口：一只 GM6020 在极小安全电流下方向正确，能可靠全 0，分组发送正确。

### 3C：DM-J4310

最后看《多型号电机驱动古法实施手册》第 10、12.3～12.4 节。

原因：它不是 DJI 四槽协议，不能用“改几个 ID”的方式实现。

第一版只做：

- MIT 完整帧 codec。
- 反馈 codec。
- enable、disable、clear error/设零中实际需要的特殊帧。
- 零命令、使能、反馈、失能。

先不做参数读写大全，也不同时开放 MIT、位置速度和速度三种控制路径。

如果当前第一台舵轮车完全不用 DM-J4310，可以在 3B 通过后把 3C 标成“有明确使用者前暂缓”，继续阶段 4；但不能宣称 DM 驱动已经完成。

### 阶段 3 总出口

- [ ] 每个实际使用型号都有独立 sample。
- [ ] 每个 sample 都验证 RX、全 0、极小命令、失能和超时。
- [ ] 所有协议纯函数都有黄金向量。
- [ ] 电机驱动层没有 PID。
- [ ] DJI 和 DM 的总线/ID 冲突表已确认。

## 8. 阶段 4：重构 PID 和控制数学基础

### 本阶段只看

- 《多型号电机驱动古法实施手册》第 20～30、35、38 节。
- 《sp_middleware 可复用性审计与迁移指南》第 5～7、11 节。

### 先后顺序

1. `pid_reset()`。
2. 指针、finite 和 `dt` 范围检查。
3. deadband 改为 effective error，不能提前吞掉 feedforward。
4. D 项改成测量微分，加入按 `tau/dt` 定义的一阶低通。
5. 条件积分 anti-windup。
6. 总输出限幅和可选变化率限制。
7. 返回/记录 P、I、D、FF、未饱和输出、饱和输出。
8. 新增角度最短误差和连续角展开。
9. 新增目标限速/限加速度，先不做复杂二阶跟踪微分器。

### 由你修改/新增的建议文件

```text
include/drivers/pid/pid.h
drivers/pid/pid.c
tests/pid/

include/control/math/angle.hpp
include/control/math/filter.hpp
include/control/trajectory.hpp
lib/control/math/angle.cpp
lib/control/math/filter.cpp
lib/control/trajectory.cpp
tests/control_math/
```

### 出口条件

- [ ] `dt<=0`、过大 dt、NaN/Inf 不会产生危险输出。
- [ ] reset 后积分和微分历史清零。
- [ ] setpoint 阶跃没有明显 D kick。
- [ ] 饱和时积分不会继续朝错误方向积累。
- [ ] deadband 内 feedforward 仍按明确策略工作。
- [ ] 每个闭环实例有独立状态，不能共享一个 `pid_data`。
- [ ] 角度跨 `+-pi` 的测试通过。

## 9. 阶段 5：先完成三条单轴闭环

不要直接做“万能关节控制器”。按下面顺序做三个可观察的小闭环。

### 5A：M3508 速度 PI

```text
目标转速
  → 限速/斜坡
  → 速度 PI
  → raw 电流限幅
  → TxGroup
```

- 从架空单电机开始。
- 先 P，后 I，不用 D。
- 500Hz 左右只是建议起点，实际使用测得 dt。
- 断 CAN、dt 异常、send 失败都清零并 reset。

### 5B：GM6020 舵向串级

```text
连续角目标
  → 位置 P/PD
  → 有限速度目标
  → 速度 PI
  → 电流 A
  → GM6020 raw
```

- 第一版不加惯量前馈。
- 先确认方向和零点，再闭环。
- 软限位外允许往安全方向退回。
- 位置反馈、速度反馈失效时输出零。

### 5C：大/小 yaw 各自的单轴闭环

如果 yaw 也是 GM6020，可复用 5B 的控制结构，但每个关节有独立参数、限位和状态。

此处只证明单轴能稳定，不做大小 yaw 协调。

### 出口条件

- [ ] M3508 速度阶跃、斜坡和停机可解释。
- [ ] GM6020 能追踪几个小角度，不撞限位。
- [ ] 每个 yaw 能独立捕获当前角后平稳使能。
- [ ] 所有闭环都有 P/I/D/FF/饱和遥测。
- [ ] 断反馈后在定义时间内清零。

## 10. 阶段 6：只写纯舵轮数学

### 本阶段只看

- 《舵轮解算与大小 Yaw 双板协同古法实施框架》第 6～9、23 节。
- 《sp_middleware 可复用性审计与迁移指南》第 4 节。

### 实施顺序

1. 坐标约定和 `ModuleGeometry[4]`。
2. 单模块速度向量。
3. 四模块逆解。
4. 零速保持上一拍舵角。
5. 连续角的 180°等价解和驱动反转。
6. 把“驱动反转”和“舵角余弦降速”分成两个可观察步骤。
7. 四轮共同轮速缩放。
8. 正运动学，优先通用最小二乘。

### 文档冲突的最终选择

《舵轮框架》中的“90°翻转优化”是第一版概念；具体实现采用《sp_middleware 审计》第 4 节的改进版：

- 输出连续舵角，不跳回单圈。
- 分别输出 `drive_reversed` 和 `alignment_scale`。
- 零速用阈值，不精确比较浮点零。

### 出口条件

- [ ] 纯 `+vx/+vy/+wz` 黄金向量通过。
- [ ] 100°差值选择反向驱动和 80°转向。
- [ ] 零速保持上一目标。
- [ ] 一轮超速时四轮同比例缩放。
- [ ] 逆解结果送入正解可还原底盘速度。
- [ ] 测试不依赖 CAN、Zephyr device 或真实电机。

## 11. 阶段 7：一只舵轮，再到四只

### 7A：单模块

只接一只 M3508 + 一只 GM6020：

1. GM6020 追踪几个低速小角度。
2. 舵角误差较大时 M3508 输出为零。
3. 舵角逐渐对齐后再放开很小驱动速度。
4. 验证 180°等价解会反转 M3508 目标。
5. 验证停机保持舵角还是进入 X-lock，由明确模式决定。

### 7B：四模块

1. 逐个标定零点和安装方向。
2. 四轮架空，只发纯 `+vx`。
3. 再测纯 `+vy`。
4. 再测纯 `+wz`。
5. 最后测组合命令。
6. 加入底盘 Disabled/Ready/Running/Fault 状态机。
7. 加入正运动学和状态发布。

### 出口条件

- [ ] 单模块不会在未对齐时强行驱动。
- [ ] 四模块没有公式里的临时修正负号；方向都来自配置。
- [ ] 任一电机离线时进入定义的降级/停机状态。
- [ ] 同组 CAN 命令每周期只统一发送一次。
- [ ] 最大轮速限制不改变期望运动方向。

## 12. 阶段 8：双板协议和两个 application

### 本阶段只看

- 《舵轮解算与大小 Yaw 双板协同古法实施框架》第 15～21 节。
- 《sp_middleware 可复用性审计与迁移指南》第 10 节。

### 先拆应用

```text
application/chassis/
application/gimbal/
```

两块板分别构建固件，不让一个空 `application/main.c` 最后变成万能主程序。

### 协议实施顺序

1. 纯 byte codec。
2. header/version/type/length/sequence/timestamp/flags/CRC。
3. heartbeat。
4. sequence 和 timeout。
5. 拆包、粘包、噪声和 CRC 错误重同步测试。
6. 底盘命令和状态摘要，但电机仍强制零。
7. 重启后旧 enable token 失效。
8. 最后允许显式 enable 打开输出。

### 出口条件

- [ ] 不发送 packed struct 或裸 float ABI。
- [ ] 两板任意一边重启都不会恢复旧运动命令。
- [ ] link stale 后进入安全状态。
- [ ] parser 能从坏帧后找回下一帧。
- [ ] 控制闭环仍在电机所在板本地运行。

## 13. 阶段 9：大小 yaw 协调

### 本阶段只看

- 《舵轮解算与大小 Yaw 双板协同古法实施框架》第 13～17、24 节。
- 《sp_middleware 可复用性审计与迁移指南》第 7～8 节。

### 顺序

1. 大 yaw 本地单轴闭环回归。
2. 小 yaw 本地/世界角闭环回归。
3. 标定两个角的正方向、中心和软/硬限位。
4. 第一版使用同轴标量角关系。
5. 协调器只放在云台板。
6. 先做 Independent。
7. 再做小 yaw 偏离中心时的大 yaw 低频卸载。
8. 测通信断线和一轴故障。
9. 最后才加速度/加速度前馈。
10. 只有非共轴三维误差确实需要时，再做双 IMU 四元数换系。

### 出口条件

- [ ] 小 yaw 快、大 yaw 慢，职责明确。
- [ ] 两块板没有各自计算一套协调目标。
- [ ] 小 yaw 接近限位时大 yaw 能卸载，但不互相追赶。
- [ ] 底盘旋转时最终世界 yaw 能保持。
- [ ] link stale 时两轴进入明确模式。

## 14. 阶段 10：裁判系统 RX 和功率策略

### 为什么排在这里

裁判 parser 可以更早独立开发，但对于一个人完成整车，先让底盘低功率、架空、可控地运动更容易判断问题。裁判数据参与控制前，底盘状态机、共同缩放和安全停机必须已经存在。

### 本阶段只看

- 《裁判系统架构古法实施框架》第 0～10、13～18 节。

### 第一版只做 RX

1. 固定当前赛季官方协议版本。
2. byte codec 和官方 CRC 黄金向量。
3. 流式 FrameParser。
4. `CommandSpec` 表。
5. 优先实现 `0x0201/0x0202/0x0001` decoder。
6. `RefereeStore` 保存 value/valid/stamp/generation。
7. 只接 115200 常规链路观察数据。
8. 拔线验证 freshness 失效。

### 再接功率策略

```text
裁判功率限制/缓冲能量
  + 超级电容反馈
  + 底盘命令需求
  → 独立 ChassisPowerPolicy
  → 四轮共同缩放/电流预算
```

parser 不直接修改电机输出。

### 出口条件

- [ ] 使用当前官方长度，不使用参考仓库混合版本 packed 类型。
- [ ] UART 支持半帧、粘包、坏 CRC 后重同步。
- [ ] 周期状态、锁存状态和事件生命周期分开。
- [ ] 过期裁判数据不会被 999 等默认值伪装成有效。
- [ ] 数据过期时底盘平滑退到安全功率上限。

## 15. 阶段 11：最后才做的功能

按收益和风险排序：

1. 基于已规划速度/加速度的 `kS/kV/kA` 前馈。
2. 裁判 TX scheduler。
3. UI scene/diff 和 1/2/5/7 批量。
4. 多机通信、自定义控制器、自定义客户端。
5. 超级电容功率融合。
6. slip telemetry；先观察，后决定是否控制。
7. 三维 quaternion frame math。

当前不要做：

- 模糊 PID。
- 一上来就做整车万能控制类。
- 把 `sp_middleware` 作为 submodule 编译。
- 把 HAL CAN/UART 驱动移入 Zephyr。
- 为“以后也许用”一次性实现所有裁判 cmd。
- 未有独立传感依据时让 slip detector 接管驱动力。

## 16. 文档内容有冲突时听谁的

优先级：

```text
当赛季/当前型号官方文档
  > 实车安全约束和实测参数
  > 本总路线图的阶段依赖
  > 对应专题实施手册
  > sp_middleware/旧仓库参考实现
```

具体裁决：

| 问题 | 最终采用 |
|---|---|
| 电机 CAN ID、量程、字节定义 | `motor_docs/` 和当前版本厂商官方手册 |
| GM6020 默认模式 | 电流模式；电压模式显式可选，不混用 |
| 舵轮翻转 | 连续角两候选；反转和 alignment scale 分开 |
| yaw 第一版换系 | 近似同轴时先用标量关系，不先搬大 Gimbal 类 |
| PID D 项 | 优先测量微分/显式反馈速度，不直接对 error 阶跃求导 |
| 前馈加速度 | 来自轨迹规划，不直接差分遥控阶跃 |
| 板间/UART | 流式 parser + 明确字节序，不 memcpy packed float struct |
| 裁判协议 | 当前赛季官方协议，参考仓库类型头只做反例和目录索引 |
| C++ | 保持 C++14；不依赖参考仓库的 C++17 写法 |

## 17. 每天开工时的最小流程

不要每天重新思考整个机器人。只做：

1. 在本文找到当前阶段。
2. 只打开该阶段列出的专题章节。
3. 从该阶段 checklist 选一个最小未完成项。
4. 先写/更新纯测试，再写对应实现。
5. 构建通过后才上硬件。
6. 硬件先全 0，再小命令。
7. 保存日志、参数和预期/实际结果。
8. checklist 没全过，不进入下一阶段。

建议一次提交只做一个可回滚目标，例如：

```text
fix motor API dispatch guards
generalize DJI feedback codec
generalize DJI TxGroup source adapter
add M2006 RX-only driver
add M2006 zero-output sample
add GM6020 current-mode codec
add PID reset and dt validation
add swerve inverse golden tests
```

## 18. 建议由你运行的构建/测试命令

本文没有运行这些命令，因为它们会生成 workspace build 产物。

现有 M3508 sample：

```bash
west build \
  -b dm_mc02/stm32h723xx \
  samples/motor/dji_m3508_driver \
  -d build/dji_m3508_driver \
  -p always \
  -- -DBOARD_ROOT=$PWD
```

烧录前先再次确认 sample 中的 raw 命令为 0：

```bash
west flash -d build/dji_m3508_driver
```

以后创建测试目录后：

```bash
west twister -T tests/motor -v
west twister -T tests/pid -v
west twister -T tests/control_math -v
west twister -T tests/swerve_kinematics -v
west twister -T tests/board_link -v
west twister -T tests/referee -v
```

命令如果因当前 Zephyr revision、board qualifier 或测试结构变化而失败，先检查构建参数，不修改协议/控制算法去迁就构建命令。

## 19. 总进度板

一次只勾当前阶段。括号中是依赖关系。

- [ ] 阶段 0：参数与安全表
- [ ] 阶段 1：M3508 基线回归（依赖 0）
- [ ] 阶段 2：motor/DJI 公共层（依赖 1）
- [ ] 阶段 3A：M2006（依赖 2）
- [ ] 阶段 3B：GM6020 电流模式（依赖 2）
- [ ] 阶段 3C：DM-J4310（依赖 2；无当前使用者可暂缓）
- [ ] 阶段 4：PID 与控制数学基础（依赖 2；闭环前必须完成）
- [ ] 阶段 5A：M3508 速度 PI（依赖 1/2/4，不依赖未使用的 DM 驱动）
- [ ] 阶段 5B：GM6020 舵向串级（依赖 3B/4）
- [ ] 阶段 5C：大/小 yaw 单轴闭环（依赖实际型号驱动/4）
- [ ] 阶段 6：纯舵轮数学（依赖 4）
- [ ] 阶段 7A：单舵轮模块（依赖 5A/5B/6）
- [ ] 阶段 7B：四舵轮底盘（依赖 7A）
- [ ] 阶段 8：双 application 与板间协议（依赖 7B；纯 codec 可提前测试）
- [ ] 阶段 9：大小 yaw 协调（依赖 5C/8）
- [ ] 阶段 10A：裁判 RX/store（可独立开发，接控制前依赖 7B）
- [ ] 阶段 10B：裁判功率策略（依赖 10A/7B）
- [ ] 阶段 11：UI/前馈/多机通信/增强功能（依赖对应基础链路稳定）

## 20. 当前阶段的“下一张工单”

标题：`M3508 基线回归前的公共 API 修正`

范围只包含：

```text
include/drivers/motor/motor.hpp
samples/motor/dji_m3508_driver/src/main.cpp
```

完成内容：

1. 修三个错误的函数指针检查。
2. 把 sample 首次上电命令改成 raw 0。
3. 构建、烧录并记录 M3508 反馈。
4. 验证 Offline timeout。
5. 确认全 0 后，再进行极小 raw 方向测试。

验收证据：

```text
构建成功日志：___
raw=0 的 CAN 数据：___
encoder/rpm/current/temp 日志：___
拔线后 Offline 用时：___ ms
小命令值和实际方向：___
clear 后零帧证据：___
```

这张工单完成后，回到本文阶段 2；不要临时跳去写裁判 UI 或大小 yaw。

## 21. 尚未验证的假设

- 尚不知道现有 M3508 driver 是否已经在当前硬件和当前源码版本上完整回归。
- 尚不知道第一台整车是否实际使用 DM-J4310；这决定阶段 3C 是否可以暂缓。
- 尚不知道大 yaw、小 yaw 的具体电机型号、传动比、限位和 IMU 安装位置。
- 尚不知道双板最终使用 Classic CAN、CAN-FD 还是 UART。
- 尚不知道参赛赛项、兵种和比赛前最终采用的裁判协议版本。
- 尚不知道四舵轮几何、轮径、零点和安装符号的实测值。

这些未知项不会阻止你完成当前 M3508 基线回归，但会阻止对应后续阶段通过出口检查。
