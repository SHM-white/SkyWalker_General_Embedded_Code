# 模块联动与调用路线

本文对照当前 samples 与 applications，串起两条常用路径：遥控器到双轴云台，以及云台板到舵轮底盘。公共模块完成数据采集、解析、决策和控制计算；应用负责线程安排、板级对象和本地恢复策略。

## 共同调用约定

| 阶段 | 主要对象 | 调用方要负责 |
|---|---|---|
| 输入采集 | RemoteReceiver、RefereeReceiver、VisionReceiver、InterBoardEndpoint | 保留原始 stamp，检查有效期和 receiver 状态 |
| 命令决策 | CommandManager | 单一所有者线程，填好 CommandInputs.now_us |
| 轴/底盘计算 | GimbalAxis、SwerveChassis、PositionMotor、VelocityMotor | 提供真实 dt_s 和有效反馈 |
| 电机输出 | Motor、Group、CanBus | 显式使能，周期末提交，每个物理 CAN 一个 CanBus |

CommandManager 不替代本地执行器检查。GimbalAxis 和 SwerveChassis 也不创建线程、不自动 enable、不提交 CAN。任何应用仍须定义何时撤销输出、如何清故障以及哪些新输入允许恢复。

## 遥控到双轴云台

独立验证入口为 samples/robotics/gimbal_control。样例中的 RemoteReceiver 负责 UART 和 DR16 解码；一个控制线程读取快照、检查拨杆和时间戳，GimbalAxis 将授权角速度转换为 PositionMotor 目标，Group 负责两轴共同使能与停机。

~~~text
DR16 UART → RemoteReceiver → RemoteState + stamp
                                 │
                       freshness / switch / estop
                                 │
                   GimbalAxis × 2 → PositionMotor × 2
                                 │
                    Motor staged command values
                                 │
             one CanBus.commit() per physical CAN bus
~~~

初始化顺序为：静态构造 DMA 与接收器，启动 RemoteReceiver；静态构造 Motor、Group、CanBus 和 GimbalAxis；先 attach 所有组成员，再 start 每条物理 CAN；调用每轴 begin()；待反馈和参考准备完成后 reset 控制器，再由新的明确授权请求 Group::enable()。

控制周期只在 Group active 后更新目标并提交：

~~~cpp
const int yaw_result = yaw.updateRate(yaw_rate_rad_s, dt_s);
const int pitch_result =
    yaw_result == 0 ? pitch.updateRate(pitch_rate_rad_s, dt_s) : 0;

int commit_result = 0;
if (yaw_result == 0 && pitch_result == 0)
    commit_result = yaw_bus.commit().error;
if (yaw_result == 0 && pitch_result == 0 && commit_result == 0 && split_buses)
    commit_result = pitch_bus.commit().error;

if (yaw_result < 0 || pitch_result < 0 || commit_result < 0)
    group.disable();
~~~

同一 CAN 的两轴一次 commit；跨 CAN 各提交一次。提交只表示命令批次已发布，不表示两条总线同时完成发送。完整门控、重置和遥测见 samples/robotics/gimbal_control/src/main.cpp。

## 双主控命令与执行

当前整机应用由 sentry_gimbal 的命令线程调用 CommandManager。它读取 RemoteReceiver、RefereeReceiver 发布的值和底盘对端状态，输出 RobotCommand。CommandRouter 把云台命令交给本地 GimbalExecutor，并把底盘命令提交到 InterBoardEndpoint。

~~~cpp
robotics::CommandInputs inputs{};
inputs.now_us = core::monotonicTimeUs();
inputs.remote = rc_snapshot.remote;
inputs.referee = referee_snapshot;
const auto decision = manager.update(inputs);

if (decision.error == 0) {
    router.route(decision.command); // 本地 GimbalCommand + InterBoardEndpoint::submit
} else {
    // 记录错误和 reasons，并让执行器收到 Disabled/无效命令。
}
~~~

sentry_gimbal 的 link 线程单独拥有 InterBoardEndpoint::poll()。底盘端由另一个 InterBoardEndpoint 接收快照；ChassisExecutor 检查 peer heartbeat、命令时间戳、接收方 boot ID、resume generation 和功率约束，再调用 SwerveChassis 与 DjiChassisHardware。该板在本地撤销电机输出，不依赖云台板及时发送停机包。

~~~text
sentry_gimbal
  Remote / Referee / peer snapshots
       → CommandManager → CommandRouter
       → GimbalExecutor → GimbalAxis → Motor / CanBus
       → InterBoardEndpoint.submit(ChassisCommand)

sentry_chassis
  InterBoardEndpoint.poll() → endpoint snapshot
       → ChassisExecutor freshness / boot / generation / power checks
       → SwerveChassis → DjiChassisHardware
       → Motor / Group / CanBus
~~~

模块调用代码见[封装模块调用示例](../modules/call-examples.md)，应用线程和配置细节见[双主控应用](dual-controller.md)。

## 验证入口与范围

| 想验证 | 样例 |
|---|---|
| DR16 / 裁判 / 板间协议 | samples/communication/dr16、referee、interboard |
| 多输入命令仲裁 | samples/robotics/command_manager |
| 安全输入变化 | samples/robotics/command_safety |
| 单轴或异品牌双轴 | samples/robotics/yaw_gimbal、gimbal_control |
| 舵轮运动学和执行映射 | samples/robotics/swerve、applications/sentry_chassis |

台架样例和日志只能显示软件行为。实机还需按目标电机、供电、接线、负载和机械停机方式确认方向、限幅、失联响应与恢复策略。
