# 封装模块调用示例

本页展示公共封装对象的实际调用顺序。代码聚焦模块 API；型号、控制器参数、设备树 alias 和错误日志应结合链接样例配置。模块与其引用对象、UART DMA 缓冲、设备回调上下文都应保持足够长的生命周期。

## 1. Motor、Group、CanBus

一条物理 CAN 只创建一个 CanBus。先构造电机与机械故障域 Group，再 attach 所有端点并 start 总线。控制器或 setter 暂存目标；每个物理 CAN 的发布者在完成本周期全部写入后调用一次 commit。

~~~cpp
#include <drivers/motor/can_bus.hpp>
#include <drivers/motor/group.hpp>

using namespace skywalker;

static motor::Motor yaw{motor::dji::m3508({
    .id = 1, .current_limit_a = 0.3f, .gear_ratio = 19.2032f,
    .timing = {20, 20, 30, 100},
})};
static motor::Motor pitch{motor::dji::m2006({
    .id = 2, .current_limit_a = 0.3f, .gear_ratio = 36.0f,
    .timing = {20, 20, 30, 100},
})};
static motor::Group axes{yaw, pitch};
static motor::CanBus bus{DEVICE_DT_GET(DT_NODELABEL(can1))};

int initialize() {
    int ret = bus.attach(yaw, pitch);
    if (ret == 0) ret = bus.start();
    return ret;
}

int updateAuthorizedTargets(float yaw_a, float pitch_a) {
    if (!axes.active())
        return -EAGAIN;
    int ret = yaw.setCurrent(yaw_a);
    if (ret == 0) ret = pitch.setCurrent(pitch_a);
    if (ret == 0) ret = bus.commit().error;
    if (ret < 0) axes.disable();
    return ret;
}

// 仅在新的、明确的使能请求中调用；不在每个控制周期重复请求。
int requestEnable() {
    return axes.ready() ? axes.enable() : -EAGAIN;
}
~~~

Group 成员不能通过 Motor 单独 enable/disable/clearFault。disable 会先关闭软件输出许可，安全帧由 CAN I/O 线程异步发送。详见 [DJI](drivers/motor/motor-dji.md)、[达妙](drivers/motor/motor-dm.md) 与[完整电机链路](../guides/motor-workflow.md)。

## 2. VelocityMotor 与 PositionMotor

控制器绑定唯一 Motor 命令生产者。调用顺序为 attach/start、configure、等待 drive.ready、reset、显式 enable、等待 active；每周期先 update，再 commit。update 不会替应用提交 CAN。

~~~cpp
static motor::Motor drive{motor::dji::gm6020({
    .id = 4, .current_limit_a = 3.0f, .encoder_zero_ticks = 0,
    .current_mode_confirmed = true, .timing = {20, 20, 20, 100},
})};
static motor::CanBus bus{DEVICE_DT_GET(DT_NODELABEL(can1))};

// makeVelocityConfig 包含经台架整定的 control_motor_velocity_config、
// Ampere effort 单位，以及速度/温度安全限值；参照链接样例定义。
static control::VelocityMotor axis{drive, makeVelocityConfig()};

int configureAxis() {
    int ret = bus.attach(drive);
    if (ret == 0) ret = bus.start();
    if (ret == 0) ret = axis.configure();
    return ret;
}

int velocityTick(float target_rad_s, float measured_dt_s) {
    if (!drive.active())
        return -EAGAIN;
    int ret = axis.update(target_rad_s, measured_dt_s);
    if (ret == 0) ret = bus.commit().error;
    if (ret < 0) (void)drive.disable();
    const auto telemetry = axis.telemetry(); // 保留 valid/error 与 effort_unit
    return ret;
}
~~~

对位置控制器，将 Config 改为 PositionMotor::Config，设定 PositionReference，并调用 axis.update(target_position_rad, dt_s)。reset 必须在失能、反馈新鲜且参考有效时完成。loop 配置不可用默认空值代替，effort_abs_max 不得超过电机硬件限幅。配置和参考模式见[电机控制器文档](control/motor-control.md)，运行样例见 samples/motor/dji_speed_control 与 dji_position_control。

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

InterBoardEndpoint 将 UART、板间 V1 编解码、peer 会话和线程安全值拷贝封装在一起。恰有一个通信线程调用 poll；生产者可通过短锁接口提交发送值，消费者取得 Snapshot 副本。

~~~cpp
static communication::AsyncUart::DmaBuffers link_dma __nocache;
static communication::InterBoardEndpoint link(
    DEVICE_DT_GET(DT_ALIAS(interboard_uart)), link_dma,
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

底盘端把角色改为 ChassisController。端点按配置的命令与心跳超时发布 online/error；应用仍需检查 command、constraint、feedback 各自的时间戳和接收方 boot/generation。对象和 DMA 在所有线程期间保持存活。双主控使用细节见[模块联动](../applications/module-integration.md)。

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

GimbalAxis 封装单轴位置参考与运动转换，但由调用方管理 Motor 和 CanBus 生命周期。初始化后持续 poll；业务许可满足、反馈健康、ready_for_enable 为真且收到新命令后，reset 并显式 enable；active 时 update，再 commit。

~~~cpp
robotics::GimbalAxis axis(
    drive, position_motor_config, gimbal_axis_config);

int ret = bus.attach(drive);
if (ret == 0) ret = bus.start();
if (ret == 0) ret = axis.begin();

const auto readiness = axis.poll(now_ms);
if (authorized && readiness.feedback_healthy &&
    readiness.ready_for_enable && drive.ready()) {
    ret = axis.reset();
    if (ret == 0) ret = drive.enable();
}
if (drive.active()) {
    ret = axis.update(
        {robotics::GimbalMode::Rate, 0.0f, requested_rate_rad_s},
        robotics::SafetyAction::Active, dt_s);
    if (ret == 0) ret = bus.commit().error;
    if (ret < 0) (void)drive.disable();
}
~~~

同 Group 的电机需改用 Group 的 enable/disable/clearFault 管理生命周期。GimbalAxis 不会自动停机、清故障或提交另一条 CAN。位置 reference、限位和完整双轴样例见[机器人模块文档](robotics/robotics.md)和 samples/robotics/gimbal_control。

## 10. SwerveChassis 与 ChassisPowerLimiter

SwerveChassis 是纯计算侧封装，反馈由硬件适配器组装。调用者先 validate/reset，再按周期传入命令、反馈和测得 dt_s。

~~~cpp
robotics::SwerveChassis chassis(chassis_config);
int ret = chassis.validate();
if (ret == 0) ret = chassis.reset(feedback);

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
