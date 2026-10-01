# 注册式命令来源与后台仲裁服务

当前命令链分成同步策略核心和后台服务两层：

~~~text
RemoteSource / VisionSource ─┐
                              ├─ CommandManager worker
RefereePermissionSource ──────┘       │
                                      ▼
                           CommandArbiter::update()
                                      │
                         CommandSnapshot publication
                           ┌──────────┴──────────┐
                           ▼                     ▼
                      current()              snapshot()
~~~

CommandArbiter 接收一次 CommandInputs 并同步生成 CommandDecision。CommandManager 负责注册输入源和许可源、启动来源、按配置周期调用仲裁器，并发布非消费式结果快照。CommandManager 本身没有 update、configError、reset 或 stop/restart 接口。

## 来源接口与内置适配器

ICommandSource 表示一个输入来源，role 固定为 Operator 或 Aim。一个 CommandManager 最多注册两个输入来源，每个角色至多一个；必须注册 Operator，Aim 来源可选。IPermissionSource 单独提供裁判许可，不与运动目标竞争。

内置适配器位于 include/robotics/command/receiver_sources.hpp：

- RemoteSource 把 RemoteReceiver 快照适配为 Operator 消息。
- VisionSource 把 VisionReceiver 的 Aim Measurement 适配为 Aim 消息。
- RefereePermissionSource 在后台 worker 调用 RefereeReceiver::poll() 并提供裁判许可。

适配器传递原始时间戳。消息拷贝不能刷新来源有效期。来源 sample 返回 -EAGAIN 时管理器保留上次缓存；其他错误会让对应缓存失效，并把错误放入诊断字段。RefereeReceiver 不公开串口丢块计数，因此裁判诊断不会伪造该计数。

## 初始化与非消费式读取

注册、绑定和启动只能由一个启动线程在 start 前完成。manager、source、receiver、协议对象和 DMA 缓冲区都需要静态或等同的长期生命周期。start 依次启动来源和许可源，然后创建后台仲裁线程；返回 0 只表示线程已创建，不代表输入在线或已经发布首个仲裁结果。

~~~cpp
#include <robotics/command/receiver_sources.hpp>
#include <robotics/command/command_manager.hpp>

using namespace skywalker;
using namespace skywalker::robotics;

static communication::AsyncUart::DmaBuffers remote_dma __nocache;
static communication::AsyncUart::DmaBuffers referee_dma __nocache;
static communication::RemoteReceiver remote(
    board_config::remote_uart, remote_dma, {});
static communication::RefereeReceiver referee(
    board_config::referee_uart, referee_dma,
    communication::RefereeVersion::Rm2026V1_3);

static RemoteSource operator_source(remote);
static RefereePermissionSource permission_source(referee);
static CommandManager manager(board_config::command_policy);

int startCommandService() {
    int ret = manager.registerSource(operator_source);
    if (ret == 0) ret = manager.bindPermissions(permission_source);
    if (ret == 0) ret = manager.start();
    return ret;
}

void telemetryTask() {
    CommandSnapshot frame{};
    for (;;) {
        if (manager.snapshot(frame) == 0) {
            telemetry.emit(frame);
            // frame 中的 observed、decision 和来源诊断属于同一次发布。
        }
        k_sleep(K_MSEC(10));
    }
}

void gimbalTask() {
    RobotCommand command{};
    for (;;) {
        if (manager.current(command) == 0) {
            // 交给本地执行器；读取不会推进命令时间戳。
        }
        k_sleep(K_MSEC(5));
    }
}
~~~

需要自动瞄准时，另行静态构造 VisionReceiver、AbProtocol 和 VisionSource，在 manager.start() 前 registerSource(vision_source)。注册顺序不代表优先级；仲裁规则由 CommandArbiter 配置决定。没有组装视觉来源时应设置 allow_auto=false。

snapshot() 提供同一时刻的观测输入、最终决策和来源诊断；current() 只复制最终 RobotCommand。读取不消费结果，也不会更新时间戳或保证收到新序号。首次发布前返回 -EAGAIN；start 失败会发布无效的 Disabled 结果，已启动来源仍须保持存活。所有接口只能在线程上下文使用，不能在 ISR 调用。

## 同步仲裁核心

需要自行管理采集线程和周期的程序可直接使用 CommandArbiter：

~~~cpp
CommandArbiter arbiter(config);
if (arbiter.configError() < 0) {
    // 阻止启动并记录配置错误。
}

CommandInputs input{};
input.now_us = core::monotonicTimeUs();
input.remote = remote_state;     // 保留遥控源的 stamp
input.vision = aim_measurement;  // 保留视觉源的 stamp
input.referee = referee_state;   // 保留权限字段的 stamp

const CommandDecision decision = arbiter.update(input);
if (decision.error == 0) {
    // 使用本次 decision.command；错误时不要转发上一次命令。
}
~~~

CommandArbiter 是单写入者对象；调用者负责序列化 update 与 reset。其配置类型由 CommandManager::Config 作为别名提供，但两类对象职责不同。

## 当前接入状态与配置

- samples/robotics/command_manager 注册 RemoteSource、VisionSource 和 RefereePermissionSource，三路都进入 CommandManager worker；sample README 记录当前台架策略和接线。
- samples/robotics/command_safety 只注册 RemoteSource，使用同一后台服务观察 Safe/Manual 和遥控断流恢复。
- applications/sentry_gimbal 当前只注册 RemoteSource 并绑定裁判许可源，没有注册 VisionSource；allow_auto=false。board_config 的裁判版本仍为 Unspecified，真实应用在配置前不会获得有效许可。
- 应用的 GimbalExecutor 与 ChassisExecutor 负责本地反馈、执行条件、使能和故障恢复；CommandManager 不操作电机或 CAN。

需要启用 SKYWALKER_ROBOTICS_COMMAND 与 SKYWALKER_COMMAND_SERVICE，并打开相应通信来源。遥控来源还需要 SKYWALKER_REMOTE_RECEIVER；裁判来源需要 SKYWALKER_REFEREE 与 SKYWALKER_UART_TRANSPORT；视觉来源需要 SKYWALKER_VISION_RECEIVER 与 AB 协议。周期、优先级和栈由 SKYWALKER_COMMAND_PERIOD_MS、SKYWALKER_COMMAND_PRIORITY、SKYWALKER_COMMAND_STACK_SIZE 配置，默认周期为 10 ms。

相关入口：[机器人模块](robotics.md)、[双主控应用](../../applications/dual-controller.md)、[三源命令管理样例](../../../samples/robotics/command_manager/README.md)。
