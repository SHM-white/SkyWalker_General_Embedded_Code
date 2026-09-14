# 独立上机微型项目

这些 samples 都是可以直接编译、刷入达妙 MC02、接真实外设运行的小固件，不是单元测试。每个目录有自己的 CMakeLists、prj.conf、overlay、main、配置和使用说明；不引用其他 sample 或 application 的文件。正式库 `lib/communication`、`lib/robotics` 和 `lib/control` 才是复用边界。

| 项目 | 真实硬件/观察内容 | 操作 |
| --- | --- | --- |
| [DR16](communication/dr16/README.md) | 接遥控 UART，观察通道、拨杆、鼠标键盘、掉线 | 遥控器实际操作 |
| [裁判](communication/referee/README.md) | 接裁判 User 串口，观察 CRC、许可、额度、缓冲和过期 | 插拔真实串口/改变供电许可 |
| [板间](communication/interboard/README.md) | 两块板互接 UART，观察在线、boot/generation、命令超时 | 控制台 `p` 暂停命令生产，`g` 更改恢复上下文 |
| [命令与安全](robotics/command_safety/README.md) | 真实 DR16 -> 意图 -> 安全 -> 机器人命令；不接电机 | RC 拨杆；控制台 `!` 急停、`r` 复位 |
| [电机恢复](motor/recovery/README.md) | 单 DJI 或 DM MIT 电机，主控存活时单独断电恢复 | `e` 运行，空格暂停，`!` 急停，`r` 复位 |
| [小 Yaw](robotics/yaw_gimbal/README.md) | GM6020 小云台，Hold/Rate/AbsoluteAngle | `e/a/d/h/0/1`，空格暂停，`!` 急停，`r` 复位 |
| [单舵轮](robotics/swerve/README.md) | 一套 GM6020 舵向 + M3508 驱动，共享 CAN | `w/s/a/d/q` 选择平移/旋转，`e` 使能 |

控制台沿用板定义的 USART10 / 115200 baud。新电机 samples 启动后不自动使能，需要在控制台输入 `e`。`r` 只解除锁存，还需重新 `e`；普通暂停或真实掉电不需要急停复位。恢复 sample 在已启用且仅电机供电中断时自动重新准备并从零斜坡执行当前台架目标。

通用构建/刷写方式（替换 sample 路径与 build 目录）：

```sh
west build -b dm_mc02 samples/communication/dr16 -d build/bench_dr16
west flash -d build/bench_dr16
```

现有达妙 samples 的 `dm_sample_support.*` 已收回各项目私有目录，`dm_common` 共享库已移除。两个 MIT wrapper samples 只保留本地电源 hook，其余旧模式样例保留本地会话辅助代码。旧示例有自身的自动轨迹/电源设置，使用前阅读其 main；它们与新样例的控制台使能流程不同。

DJI 速度/位置、DM MIT 速度/位置四个旧 wrapper samples 已改为常驻 configure/poll/resume/update，保留原控制参数和观测通道，临时故障不退出 main。
