# 封装模块调用示例

本页展示公共封装对象的实际调用顺序。代码聚焦模块 API；型号、控制器参数、设备树 alias 和错误日志应结合链接样例配置。模块与其引用对象、UART DMA 缓冲、设备回调上下文都应保持足够长的生命周期。

## 1. Motor、Group、CanBus

初始化构造合法 Motor 配置，attach 全部成员，再 start 物理总线。Group 仅批量启停，成员不需要共同在线。

~~~cpp
if (run_requested) {
    const int enabled = axes.enable();
    const int first = left.setCurrent(left_current_a);
    const int second = right.setCurrent(right_current_a);
    // 分别记录调用错误，继续处理其他成员。
} else {
    axes.disable();
}
const auto publication = bus.commit();
const auto counts = axes.status();
~~~

enable 返回 0 表示意图保存，setter 返回 0 表示合法目标接受，不表示机械已运动。离线/故障/恢复不拒绝目标，底层独立重试。明确停止取消旧命令；总线恢复不撤销输入意图。

## 2. VelocityMotor 与 PositionMotor

attach/start 后 configure，然后持续 update，不使用 ready/preflight 或 active 门控。

~~~cpp
if (run_requested) {
    (void)drive.enable();
    const int accepted = axis.update(target_rad_s, dt_s);
} else {
    (void)drive.disable();
}
const auto sent = bus.commit();
const auto telemetry = axis.telemetry();
// telemetry.target_sequence / output_valid / issue 分别观察目标与实际计算。
~~~

普通离线等待返回 0。恢复首周期自动从反馈初始化并提交带执行版本的零输出。有限异常周期跳过积分，不撤销用户意图。位置参考模式见[控制器文档](control/motor-control.md)。

## 3. ImuReceiver

ImuReceiver 持有 ImuSource 的采集责任，可选同时拥有 ImuHeater 的更新责任。start 返回 0 只表示 worker 已启动；数据有效性看 fresh_mask，初始化与服务错误看 status。

~~~cpp
#include <drivers/imu/imu_receiver.hpp>

static imu::ImuReceiver receiver(
    source, {.poll_interval_us = 500, .priority = 5}, &heater);

int startImu() {
    return receiver.start();
}

void consumeImu() {
    const auto sample = receiver.snapshot(); // 不触发采样或 I/O
    if (sample.fresh_mask & imu::Orientation) {
        const auto orientation = sample.sample.orientation.value;
        // 把四元数和其原始 stamp 一起交给消费端。
    }
    const auto status = receiver.status();
    // init_error / service_error / heater_error 与测量快照分别读取。
}
~~~

source、estimator、heater 和 DMA storage 必须比 receiver 活得更久。启动一个 Receiver 后，不要从其他线程再次调用对应 source.init/service 或 heater.update。完整 BMI088 + EKF + 温控和 DM RS485 构造见 samples/imu/dual_imu/src/main.cpp 与 [IMU 文档](drivers/imu.md)。

## 4. RemoteReceiver

RemoteReceiver 内部拥有 AsyncUart 与 RemoteService；start 后由内部 worker 读取 UART。每个消费者保留自己的零初始化 Snapshot，snapshot 争锁失败时保留旧副本，但仍按时间令过期遥控离线。

~~~cpp
static communication::AsyncUart::DmaBuffers remote_dma __nocache;
static communication::RemoteReceiver remote(
    DEVICE_DT_GET(DT_ALIAS(remote_uart)), remote_dma, {});
static communication::RemoteReceiver::Snapshot remote_snapshot{};

int startRemote() {
    return remote.start();
}

void controlTick() {
    (void)remote.snapshot(remote_snapshot);
    const auto now_ms = static_cast<std::uint64_t>(k_uptime_get());
    if (remote_snapshot.remote.online &&
        robotics::isFresh(remote_snapshot.remote.stamp, now_ms, 100)) {
        // 消费本帧输入，并保留来源时间戳。
    } else {
        // 撤销依赖遥控的命令；不要把旧帧重新盖成当前时间。
    }
}
~~~

receiver 与 DMA 缓冲必须静态存活；同一 UART 不能再交给另一个 owner。更多参数和返回值见[通信模块文档](communication/communication.md)。

## 5. RefereeReceiver

RefereeReceiver 由调用线程轮询，内部封装 AsyncUart 的 service、初始化重试和 RefereeService 快照。要为其独占 UART 提供静态 DMA buffers；协议版本必须显式选择。

~~~cpp
static communication::AsyncUart::DmaBuffers referee_dma __nocache;
static communication::RefereeReceiver referee(
    DEVICE_DT_GET(DT_ALIAS(referee_uart)), referee_dma,
    communication::RefereeVersion::Rm2026V1_3);

void refereeTick() {
    const auto state = referee.poll(k_uptime_get());
    if (state.online) {
        // 使用权限和功率字段时，再按各自 stamp 检查新鲜度。
    }
    if (referee.error() < 0) {
        // 记录 UART / service 错误；下一轮 poll 按内部退避继续尝试。
    }
}
~~~

不要重复创建 AsyncUart 去读取同一设备。完整裁判字段和协议边界见[通信模块文档](communication/communication.md)。

## 6. InterBoardEndpoint

InterBoardEndpoint 将可选择的传输后端、板间 V1 编解码、peer 会话和线程安全值拷贝封装在一起。恰有一个通信线程调用 poll；生产者可通过短锁接口提交发送值，消费者取得 Snapshot 副本。UART、RS485 和 CAN 的初始化配置见[三种传输方式](communication/interboard-transports.md)。

~~~cpp
#include <communication/interboard/configured_interboard_transport.hpp>
#include <communication/interboard/interboard_endpoint.hpp>

static communication::AsyncUart::DmaBuffers link_dma __nocache;
static communication::ConfiguredInterBoardTransport transport(
    {.kind = communication::InterBoardTransportKind::Uart,
     .uart = DEVICE_DT_GET(DT_ALIAS(interboard_uart))}, &link_dma);
static communication::InterBoardEndpoint link(
    transport,
    {robotics::BoardRole::GimbalController, 100, 100});

void linkTask() {
    for (;;) {
        link.poll(k_uptime_get());
        k_sleep(K_MSEC(1));
    }
}

void commandTask(const robotics::ChassisCommand &command,
                 const robotics::RefereeState &referee_state,
                 const robotics::RunStatus &status) {
    link.setReferee(referee_state);
    link.setStatus(status);
    link.submit(command);
}

communication::InterBoardEndpoint::Snapshot readPeer() {
    return link.snapshot();
}
~~~

底盘端把角色改为 ChassisController。端点按配置的命令与心跳超时发布 online/error；应用仍需检查 command、constraint、feedback 各自的时间戳和接收方 boot 与原始输入年龄。对象和 DMA 在所有线程期间保持存活。双主控使用细节见[模块联动](../applications/module-integration.md)。

## 7. VisionReceiver

VisionReceiver 把 UART worker、VisionLink 和指定协议实现组合起来。protocol、receiver 与 DMA 需要长期存活；start 只代表 worker 创建，snapshot 中的 aim_fresh 才反映视觉命令是否新鲜。

~~~cpp
namespace vision = skywalker::communication::vision;

static communication::AsyncUart::DmaBuffers vision_dma __nocache;
static vision::AbProtocol protocol({.command_reference = {1, 1}});
static vision::VisionReceiver receiver(
    DEVICE_DT_GET(DT_ALIAS(vision_uart)), vision_dma, protocol, {});
static vision::VisionReceiver::Snapshot vision_snapshot{};

int startVision() {
    return receiver.start();
}

void visionTick() {
    vision_snapshot = receiver.snapshot();
    if (vision_snapshot.link.aim_fresh) {
        const auto aim = vision_snapshot.link.aim.value;
        // 转交带有原始时间戳的 aim；模块不执行云台或射击。
    }
}
~~~

若启用反馈上行，先把带独立时间戳和 reference 的有效 Feedback 交给 setFeedback；不符合协议字段、有效期或时间偏差时，编码会返回错误，不会填造数据。具体 AB 帧语义见[视觉协议](communication/vision.md)。

## 8. 命令服务与同步仲裁

当前 CommandManager 不提供 update(inputs)。它在 start() 后启动后台 worker，通过注册的 ICommandSource 和 IPermissionSource 采样并调用 CommandArbiter；执行/遥测线程读取 snapshot() 或 current()。具体注册代码、生命周期和错误语义见[命令来源与后台仲裁服务](robotics/command-service.md)。

若应用自行采集并调度，可直接使用 CommandArbiter::update(CommandInputs)。CommandInputs、CommandDecision 和 CommandArbiter 的同步调用入口也在上述页面。

## 9. GimbalAxis

GimbalAxis 负责单轴机械目标与范围；调用方管理运行意图和 CAN 发布。本轴没有反馈不阻断其他轴。

~~~cpp
if (run_requested) {
    (void)group.enable();
    const int yaw_result = yaw.updateRate(yaw_rate_rad_s, dt_s);
    const int pitch_result = pitch.updateRate(pitch_rate_rad_s, dt_s);
} else {
    group.disable();
}
const auto yaw_publish = yaw_bus.commit();
if (split_buses) {
    const auto pitch_publish = pitch_bus.commit();
}
~~~

poll 只观测/准备配置允许的可信参考，不产生启动许可。业务 Hold 可显式 reset 获取本轴位置；协议恢复不调用该操作重新定义目标。

## 10. SwerveChassis 与 ChassisPowerLimiter

SwerveChassis 是纯计算侧封装，反馈由硬件适配器组装。调用者先 validate，再持续传入命令、逐轴有效反馈和测得 dt_s，不等待全部设备上线。

~~~cpp
robotics::SwerveChassis chassis(chassis_config);
int ret = chassis.validate();

robotics::ChassisOutput output{};
if (ret == 0)
    ret = chassis.step(command, feedback, dt_s, output);

robotics::ChassisPowerLimiter limiter{};
robotics::ChassisPowerDecision power{};
if (ret == 0)
    ret = limiter.step(
        {measured_power_w, referee_limit_w, buffer_energy_j},
        dt_s, power);
if (ret == 0) {
    // 硬件层将 output 中的 effort 乘以 power.effort_scale，
    // 然后暂存 Motor 命令并提交各物理 CAN。
}
~~~

reset 失败、feedback 过期或周期异常时应撤销对应输出域。ChassisPowerLimiter 是台架启发式，需真实功率标定，不是比赛限功率合规认证。参考 applications/sentry_chassis/src/chassis_executor.cpp 和 samples/robotics/swerve/src/main.cpp。
