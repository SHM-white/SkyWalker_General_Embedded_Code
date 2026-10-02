# 多电机 CAN 拓扑台架样例

本样例展示一条 CAN 上独立电机组、共享 DM 反馈 ID、跨 CAN 联动组及跨 CAN 故障隔离。电机型号、ID、限幅和 Group 关系只在 `src/main.cpp` 配置；`app.overlay` 不创建 motor 设备节点。默认编译 DJI 同帧模式。

| 构建模式 | 物理连接 | 组关系 | 重点观察 |
| --- | --- | --- | --- |
| `dji_shared_frame`（默认） | CAN1：M3508 ID1、M2006 ID2 | 各自一个 Group | 两个电机均使用 TX 0x200 的不同槽位；一台失联时另一台仍可运行 |
| `dm_shared_master` | CAN1：DM J4310 MIT ID1、ID2，Master ID 均为 0x11 | 各自一个 Group | RX 0x011 只需一条 CAN 过滤器，按反馈 data[0] 低 4 位分发 |
| `cross_can_group` | CAN1：M3508 ID1；CAN2：DM J4310 MIT ID1、Master 0x11 | 两台同属一个 Group | 任一电机或 CAN 故障均撤销两台的软件输出许可 |
| `cross_can_isolation` | CAN1：M3508 ID1 + M2006 ID2；CAN2：DM J4310 MIT ID1 + ID2、Master 0x11 | M3508 ID1 与 DM ID1 联动；M2006 ID2 和 DM ID2 各自独立 | CAN1 故障时 CAN2 独立组继续，CAN2 故障时 CAN1 独立组继续；联动组两轴均停 |

每次构建只选择一种模式；`cross_can_isolation` 需要四台电机，其余模式只需两台。MC02 的 CAN1/CAN2 均为 1 Mbps。DM 模式会在所用 CAN 完成 `start()` 后打开 XT30_1，并等待 1.5 秒；启动流程只执行驱动安全准备，不自动使能或发送运动目标。电机仍可能在失能后自由转动，先可靠支撑机构并准备物理断电。

## 构建

在仓库根目录运行，各模式使用不同构建目录：

```sh
west build -b dm_mc02/stm32h723xx samples/motor/mixed_topology -d build/mixed_dji
west build -b dm_mc02/stm32h723xx samples/motor/mixed_topology -d build/mixed_dm -- -DMIXED_TOPOLOGY=dm_shared_master
west build -b dm_mc02/stm32h723xx samples/motor/mixed_topology -d build/mixed_cross -- -DMIXED_TOPOLOGY=cross_can_group
west build -b dm_mc02/stm32h723xx samples/motor/mixed_topology -d build/mixed_isolation -- -DMIXED_TOPOLOGY=cross_can_isolation
```

改同一构建目录的模式时加 `-p always`，以免 CMake 缓存旧选择。每个固件上机前分别刷写。控制台为板载 USART10、115200 baud。

## 上机前配置

`src/main.cpp` 顶部的 `m3508Id1()`、`dmMit(id)`、M2006 配置和 `kDjiTestCurrentA`／`kDmTestTorqueNm` 都是台架示例值。按实物确认 CAN 口、型号、ID、减速比、允许电流/力矩、DM 持久化 MIT 模式及 PMAX/VMAX/TMAX。尤其 DM 的编码范围必须与调试助手中的设备设置一致。示例请求只有 +0.05 A 或 +0.02 N·m，但方向仍需实际核对。缺少第二台同型号电机时，只构建相应模式；不要把未接的电机从配置中临时删掉后声称已验证共享路由。

遥控器使用板级 `remote-uart`，MC02 为 UART5/PD2，100000 baud、8E1、RX DMA；接收机与主控共地。启用第五通道拨轮，不使用遥控键鼠或终端输入。控制台和 VOFA 仍可用于观测。

上电保持左、右开关都在 Down，四个摇杆轴和拨轮归中至少 0.5 秒，再将左开关拨到 Middle 解锁。左 Down 随时停机，左 Up 在本台架不启用 Auto。失联超过 100 ms、执行器运行中故障或失能后，必须重新完成安全档归中与解锁。清除故障：先完成双 Down 归中 0.5 秒，保持左 Down，将右开关拨到 Up 且全部通道归中 1 秒；清除只发生一次，不会自动使能。

遥控目标保留接收帧的原始时间戳和序号。重复读取快照不会续期，也不需要手动以控制周期重复发送字母。硬件接线、校准和现有电流/力矩保护仍是运行前提。

上电先等各组 `ready=1`。解锁后使用保持式通道启停，通道正向超过归一化 0.6 请求使能，回到 0.3 以下停止；0.3..0.6 为滞回区，避免边界抖动。偏转量不改变既有 +0.05 A / +0.02 N·m 固定目标。

| 模式 | 左纵杆 | 右纵杆 | 拨轮 |
| --- | --- | --- | --- |
| 两种共享总线独立组 | 第一组 | 第二组 | 不使用 |
| `cross_can_group` | linked 联动组 | 不使用 | 不使用 |
| `cross_can_isolation` | linked 联动组 | CAN1 独立组 | CAN2 独立组 |

任意组可同时保持运行，不需要选择菜单。每次 enable 仍检查新鲜反馈、转速小于 10 rad/s，以及可用温度低于 60°C（DM 包括 MOS/转子温度）。enable 返回成功仅表示请求已接受；Active 后才写低幅目标。

单组失能或失败只重置该通道的请求门，必须先回中再推起；其他健康组保持原有输出。已锁存的 Fault 仍需上述明确清除手势，未自动清除硬故障。清除不重新使能。全局 Safe、断流或输入诊断超时停止所有组。

普通构建 `CONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=0`，不提供运行时故障菜单。独立诊断构建：

```sh
west build -b dm_mc02/stm32h723xx samples/motor/mixed_topology -d ../build/mixed_topology_rc_diagnostic -- -DEXTRA_CONF_FILE=diagnostic.conf
```

`diagnostic.conf` 选择 1（暂停命令生产）。支持场景：1 输入暂停、2 执行暂停。也可为同一诊断构建设置 `-DCONFIG_SAMPLE_DIAGNOSTIC_SCENARIO=N` 选择支持的编号。连续 Active 3 秒后仅触发一次，暂停持续 1.5 秒；实时遥控停机与清故障始终优先，诊断不伪造新的来源时间戳。停机后重新解锁；诊断计时器不会跟随被暂停的执行器停住。

## 应看到的隔离行为

- DJI 模式：两台均 Active 时，抓包的 0x200 帧前两个槽位分别对应 ID1/ID2。只断开一台的反馈且另一台仍可正常 ACK 时，故障台槽位归零，健康组继续输出。若 CAN 控制器 bus-off，则两台都停。
- DM 模式：两台使用同一 Master 0x011，但各自反馈时间和状态独立；一台反馈超时只影响该组。驱动以反馈数据的低 4 位识别 ID1/ID2。CAN 控制器故障仍影响本 CAN 的两台。
- 跨 CAN 模式：两台全部 ready 才能接受左纵杆运行请求。断开 CAN1 电机反馈或 CAN2 电机反馈时，共同 Group 撤销两台许可；另一条 CAN 的安全动作不需要等待业务线程转发。
- 跨 CAN 隔离模式：四台都先用对应保持式通道使能。在 CAN1 制造 bus-off 时，CAN1 两台和 CAN2 联动成员撤销许可，CAN2 独立 DM ID2 继续；CAN2 bus-off 时则由 CAN1 独立 M2006 ID2 继续。单电机仅反馈掉线时，应只停其所属 Group，另一同 CAN 独立组继续。

每 500 ms 日志记录本地时间、各 Group 代次、Motor 状态与使能代次、停机进度、反馈时间，以及每条 CAN 最近的 commit 序号和 TX 完成结果。用抓包核对同帧槽值与安全帧；TX 完成只证明总线发送，不能当作 DM 电机执行确认。

本轮未在此环境刷写或连接实物。故障注入时应区分“单电机反馈消失”和“整条 CAN 失去 ACK／bus-off”，两者故障范围不同。
