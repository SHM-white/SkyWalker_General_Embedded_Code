# 模块联动与调用路线

本文按当前源码串起遥控双轴云台样例和双主控应用。公共模块负责收发、仲裁和控制计算；应用持有设备对象、线程和执行恢复逻辑。

## 公共对象的调用顺序

| 阶段 | 当前对象 | 调用方责任 |
|---|---|---|
| 输入 | RemoteReceiver、RefereeReceiver、VisionReceiver | 以 source adapter 保留原始 stamp |
| 采样与仲裁 | RemoteSource、VisionSource、RefereePermissionSource、CommandManager、CommandArbiter | start 前注册；唯一 worker 周期采样 |
| 输出快照 | CommandSnapshot、RobotCommand | 非消费式读取；消费者自行检查命令时间 |
| 子系统计算 | GimbalAxis、SwerveChassis、PositionMotor | 有效反馈、实际 dt_s 和明确授权 |
| 执行与传输 | Motor、Group、CanBus、InterBoardEndpoint | 应用负责使能、撤销、恢复和总线提交 |

CommandManager 是后台服务，CommandArbiter 是同步算法。当前接口没有 GlobalSafetyManager 或本地 SafetyManager 类。公共决策负责来源、策略与权限；本地执行器还要检查目标机构的反馈和恢复条件。

## 遥控到双轴云台台架

samples/robotics/gimbal_control 使用 RemoteReceiver 内部 worker 和一个控制线程，直接消费遥控值；它演示 GimbalAxis、Group 和异品牌/跨 CAN 提交，不走三源 CommandManager 服务。

~~~text
DR16 UART → RemoteReceiver → 遥控快照 + 原始时间
                                │
                  新鲜度 / 拨杆 / 急停门控
                                │
                    GimbalAxis × 2
                                │
                     PositionMotor × 2
                                │
              Group + 每条物理 CAN 一次 commit
~~~

初始化依次构造静态 Receiver/DMA、Motor、Group、CanBus 与各 GimbalAxis；启动接收器，attach 全部 Group 成员，再 start 每条物理 CAN 并调用各轴 begin。反馈与参考 ready 后 reset 控制器，由新授权请求 Group::enable；只有 active 后才周期 update 并 commit。若同一 CAN 控制两个轴，一次 commit；跨 CAN 则每条总线分别提交。

完整遥控门控和恢复代码见 samples/robotics/gimbal_control/src/main.cpp 与[封装模块调用示例](../modules/call-examples.md)。

## sentry_gimbal 命令服务

当前整机云台应用在启动线程注册 RemoteSource，绑定 RefereePermissionSource，再启动 CommandManager。命令 worker 通过 CommandArbiter 发布 CommandSnapshot。linkTask 读取 snapshot，提交底盘命令和裁判状态到 InterBoardEndpoint；gimbalTask 读取 current() 并交给 GimbalExecutor。

~~~cpp
int ret = commands.registerSource(remote_source);
if (ret == 0) ret = commands.bindPermissions(permission_source);
if (ret == 0) ret = commands.start();

// link task
CommandSnapshot frame{};
if (commands.snapshot(frame) == 0) {
    link.submit(frame.decision.command.chassis);
    link.setReferee(frame.observed.referee);
}
link.poll(k_uptime_get());

// gimbal task
RobotCommand command{};
if (commands.current(command) == 0)
    executor.update(command.gimbal, core::monotonicTimeUs());
~~~

当前 sentry_gimbal 没有注册 VisionSource，设置 allow_auto=false；裁判 profile 默认 Unspecified、硬件连接门禁关闭。三源台架 samples/robotics/command_manager 则注册 RemoteSource、VisionSource 和 RefereePermissionSource。

## sentry_chassis 执行链

底盘 linkTask 独占 InterBoardEndpoint::poll()。ChassisExecutor 在控制线程读取 endpoint snapshot，检查 heartbeat、命令原始时间、接收方 boot ID、resume generation 和功率约束；准备完成后调用 SwerveChassis 和 DjiChassisHardware 更新八台 Motor。停机和恢复由本地执行器推进，不依赖远端下一帧及时送达。

~~~text
InterBoardEndpoint → ChassisExecutor local checks
  → SwerveChassis → DjiChassisHardware
  → Motor / Group / CanBus → status returned through endpoint
~~~

线程与配置细节见[双主控应用](dual-controller.md)，命令来源合同见[后台仲裁服务](../modules/robotics/command-service.md)。

## 当前样例边界

| 要验证 | 工程 |
|---|---|
| DR16 / 裁判 / 板间协议 | samples/communication/dr16、referee、interboard |
| CommandManager 三源服务 | samples/robotics/command_manager |
| 命令输入恢复策略 | samples/robotics/command_safety |
| 单轴或异品牌双轴 | samples/robotics/yaw_gimbal、gimbal_control |
| 舵轮算法与硬件映射 | samples/robotics/swerve、applications/sentry_chassis |

应用仍需按真实负载、供电、接线和机械结构核对输出方向、限幅、失联响应与停机行为。
