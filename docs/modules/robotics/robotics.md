# 机器人决策与执行模块

机器人层位于 include/robotics 与 lib/robotics。它消费通信层发布的带时间戳消息，生成带物理单位的命令，或把已授权的目标转换为控制器输入。板级引脚、电机 ID、总线启停和最终恢复策略由应用配置和执行器所有者负责。

## 当前数据流

~~~text
Remote / Referee / Vision / InterBoard snapshots
                 │
                 ▼
CommandInputs → CommandManager → CommandDecision / RobotCommand
                                      │
                     ┌────────────────┴────────────────┐
                     ▼                                 ▼
              local GimbalCommand                ChassisCommand
                     │                                 │
                     ▼                                 ▼
          GimbalAxis / PositionMotor        SwerveChassis / chassis hardware
                     │                                 │
                     └────────── Motor / CanBus ──────┘
~~~

旧的 GlobalSafetyManager、GimbalLocalSafety、ChassisLocalSafety 和专用输入/决策消息已从当前接口移除。现在由 CommandManager 按输入新鲜度、操作档位、裁判权限与视觉状态生成决策；正式应用的执行器封装各自在本地检查反馈、命令时限、恢复上下文和硬件配置。电机驱动仍保留 Motor、Group 与 CanBus 的输出许可和停机机制。

各接口的完整调用样例集中在[封装模块调用示例](../call-examples.md)。运行中的样例源码是配置和线程细节的依据。

## CommandManager：输入快照到机器人命令

CommandManager 由单一命令线程持有。CommandInputs.now_us 是本轮仲裁时间；遥控、视觉和裁判消息各自保留原始时间戳。update() 返回值拥有自己的 RobotCommand，不要把一次旧结果重新发布成新输入。

~~~cpp
#include <core/clock.hpp>
#include <robotics/command/command_manager.hpp>

using namespace skywalker;
using namespace skywalker::robotics;

CommandManager::Config config{};
config.require_referee_for_motion = true;
config.allow_auto = false; // 没有组装 VisionReceiver 时关闭自动瞄准
config.input_timeout_ms = 100;
CommandManager manager(config);
if (manager.configError() < 0) {
    // 配置错误时阻止命令进入执行层，并记录错误。
}

communication::RemoteReceiver::Snapshot remote{};
communication::vision::VisionReceiver::Snapshot vision{};
RefereeState referee{};

void commandTick() {
    CommandInputs inputs{};
    inputs.now_us = core::monotonicTimeUs();
    inputs.remote = remote.remote;
    inputs.vision = vision.link.aim;
    inputs.referee = referee;

    const CommandDecision decision = manager.update(inputs);
    if (decision.error < 0) {
        // 记录 decision.error / decision.reasons()；不要沿用上一轮的命令。
        return;
    }
    const RobotCommand &command = decision.command;
    // 交给本地云台执行器；底盘部分交给板间发送端。
}
~~~

应用需先从 RemoteReceiver、VisionReceiver、RefereeReceiver 或 InterBoardEndpoint 读取独立快照，再把时间戳原样带入 CommandInputs。无效数据应保留无效状态，不以拷贝时刻刷新 stamp。完整三源台架入口见 samples/robotics/command_manager/src/main.cpp。

## GimbalAxis：单轴参考准备与控制

GimbalAxis 封装 PositionMotor 和云台轴坐标处理，但不创建 Motor/CanBus 线程，也不替调用方使能、停机或提交 CAN。调用线程依次完成总线 attach/start、begin、反馈准备、reset、显式 enable、update 和 commit。

~~~cpp
#include <drivers/motor/can_bus.hpp>
#include <robotics/gimbal/gimbal_axis.hpp>

using namespace skywalker;

static motor::Motor yaw{board_config::yawHardware()};
static motor::CanBus yaw_bus{board_config::yaw_can};
static robotics::GimbalAxis axis{
    yaw, board_config::yawMotorConfig(), board_config::yaw};

int initializeAxis() {
    int ret = yaw_bus.attach(yaw);
    if (ret == 0) ret = yaw_bus.start();
    if (ret == 0) ret = axis.begin();
    return ret;
}

int authorizedTick(float requested_rate_rad_s, float dt_s, std::uint64_t now_ms) {
    const auto readiness = axis.poll(now_ms);
    if (!readiness.feedback_healthy)
        return readiness.error < 0 ? readiness.error : -EAGAIN;

    if (!yaw.active()) {
        if (!readiness.ready_for_enable)
            return -EAGAIN;
        int ret = axis.reset();
        if (ret == 0) ret = yaw.enable(); // 请求已受理不表示已 Active。
        return ret;
    }

    int ret = axis.updateRate(requested_rate_rad_s, dt_s);
    if (ret == 0) ret = yaw_bus.commit().error;
    if (ret < 0) (void)yaw.disable();
    return ret;
}
~~~

样例中的 board_config 函数提供硬件型号和控制环配置；这里强调运行顺序。实际应用还需在独立状态逻辑中处理新命令时间、急停、组级停机、驱动状态和重试。多轴联动见 samples/robotics/gimbal_control/src/main.cpp；执行器可进一步把这套完整恢复流程封装成 GimbalExecutor。

## SwerveChassis：四模块运动学与控制计算

SwerveChassis 是无设备 I/O 的算法封装。应用每周期提供底盘命令、四个模块反馈和真实 dt_s，取得四个模块目标；它本身不会读取 Motor、使能 Group 或调用 CanBus.commit()。

~~~cpp
robotics::SwerveChassis::Config config = board_config::chassisConfig();
robotics::SwerveChassis chassis(config);

int ret = chassis.validate();
if (ret == 0) ret = chassis.reset(feedback); // 使用四模块当前可信反馈
if (ret < 0) {
    // 配置或参考不可用，阻止执行。
}

robotics::ChassisCommand command{};
command.mode = robotics::ChassisMode::BodyVelocity;
command.vx_m_s = 0.4f;
command.vy_m_s = 0.0f;
command.wz_rad_s = 0.2f;

robotics::ChassisOutput output{};
ret = chassis.step(command, feedback, dt_s, output);
if (ret == 0) {
    // 将 output 四轮目标交给硬件适配器，再由适配器写 Motor 并提交总线。
}
~~~

模块顺序为 FL、FR、RL、RR；坐标 +x 向前、+y 向左、+wz 逆时针。运动学限幅、舵向优化和电机控制参数由 SwerveChassis::Config 配置。samples/robotics/swerve 展示一个悬空模块台架，applications/sentry_chassis 展示四舵四驱的执行器适配与恢复。

## 功率预算封装

ChassisPowerLimiter 使用测得功率、裁判功率上限和 buffer energy 计算 effort_scale。它是台架启发式，不是竞赛功率合规证明。只有功率数据新鲜且 power model 已按真实机构标定时，应用才应采用其输出。

~~~cpp
robotics::ChassisPowerLimiter limiter{};
int ret = limiter.reset();

robotics::ChassisPowerDecision decision{};
if (ret == 0) {
    ret = limiter.step(
        {measured_power_w, power_limit_w, buffer_energy_j},
        dt_s, decision);
}
if (ret == 0) {
    // 由硬件执行器将输出 effort 乘以 decision.effort_scale。
}
~~~

## 消息、恢复与样例边界

- CommandDecision.command 是本轮执行候选；requested 用于诊断源选择和限幅前请求，telemetry 主通道应记录最终 command。
- Down 档和新鲜度丢失会撤销依赖该来源的目标；视觉失联时 Auto 云台进入 Hold 语义，裁判许可裁剪由 CommandManager 配置决定。
- GimbalAxis 负责单轴控制与参考准备；GimbalExecutor 是 sentry_gimbal 应用私有封装，管理初始化重试、反馈恢复、显式使能和 CAN 提交。
- SwerveChassis 负责运动学和每模块控制计算；ChassisExecutor 与 DjiChassisHardware 是 sentry_chassis 应用私有封装，负责本地恢复、八台电机快照、输出和总线提交。
- 独立验证入口见 samples/robotics/command_manager、command_safety、yaw_gimbal、gimbal_control 与 swerve。
