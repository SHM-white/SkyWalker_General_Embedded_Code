# 遥控器右摇杆 → 小 yaw / pitch 双轴测试

## 目标与当前状态

用户已确认仅控制小 yaw 和 pitch，大 yaw 不参与。本指南只分析和指导修改；业务代码未修改，未运行构建或硬件测试。

当前 `samples/robotics/gimbal_control/src/main.cpp` 来自完整哨兵应用，有遥控、裁判、板间通信、整车命令、安全决策、单轴执行五个任务。实际只有一个 `PositionMotor` 和一个 `YawGimbal`，没有 pitch 电机。`GimbalCommand` 虽有 pitch 字段，`lib/robotics/yaw_gimbal.cpp` 只读取 yaw 字段；因此仅删除多余任务不能得到双轴控制。

硬件仍待确认：两轴实际电机型号、CAN 总线和 ID、减速比、零点、限位、方向。现有 overlay 的 GM6020 CAN1 ID1 是禁用的模板，不能视为真实接线。下述路径沿用 MC02 板示例；硬件不是 MC02 时需改板名和引脚。

## 最小链路：两个任务即可

```text
接收机 → AsyncUart → RemoteService → Latest<RemoteState>
                                         ↓
                        gimbalTask：有效性检查 + 右摇杆映射
                             ↓                       ↓
                      小 yaw 单轴控制器       pitch 单轴控制器
                             ↓                       ↓
                      PositionMotor          PositionMotor
                             ↓                       ↓
                      对应 MotorBackend → CAN → 两根轴
```

保留 `remoteTask`（1 ms）和 `gimbalTask`（5 ms）。控制任务直接读取遥控快照，在同一线程内完成两轴更新，不再传递整车命令。电机闭环仍使用已有库，不重写 CAN 协议或 PID。

右摇杆表示角速度：横向控制小 yaw，纵向控制 pitch；回中后保持目标角度。拨杆退出使能或遥控超时则撤销输出。这两种行为要区分：回中保持仍然有力矩。

## 文件级保留与删除清单

| 文件/模块 | 操作 |
|---|---|
| `src/main.cpp` 的 `remoteTask`、`remote_state` | 保留；UART 初始化错误应明确记录，未初始化成功不进入有效接收状态 |
| `src/latest.hpp` | 保留跨线程快照；`main.cpp` 改为直接包含它 |
| `src/main.cpp` 的 `gimbalTask` | 改成两轴实例，加入本地遥控映射和共同使能条件 |
| `src/main.cpp` 的 `refereeTask`、`linkTask`、`commandTask` | 删除定义和对应 `K_THREAD_DEFINE` |
| `referee_state`、`peer_heartbeat`、`chassis_feedback`、`local_command`、`remote_command` | 删除 |
| `GimbalStatus`、`gimbal_status`、`age()`、`permission()` | 删除；改为控制任务直接打印两轴状态 |
| `reset_generation` | 删除跨任务原子通知；急停复位在控制任务内处理 |
| `ManualCommandMapper`、`CommandManager`、`GlobalSafetyManager` | 本测试不实例化；共享库源码保留 |
| `GimbalLocalSafety` | 最简方案可去掉类实例，以本地门控替代；必须保留失联停机、反馈检查与急停锁存语义 |
| `src/command_router.hpp/.cpp` | 从应用引用与构建中移除，可手动删除这两个应用文件 |
| `src/board_config.hpp` | 只保留遥控、两轴配置、超时和本地急停接口 |
| `app.overlay` | 只保留本测试需要的遥控资源、CAN、两个电机节点与别名 |
| `CMakeLists.txt` | `target_sources(app PRIVATE src/main.cpp)` |

移除 `main.cpp` 中裁判、板间通信、随机数、整车 mapper/manager、安全决策和 router 的 include。原有 `<limits>` 等标准头按剩余使用情况删除。新增映射用到的 `<algorithm>`、`<cmath>`，并直接包含 `<robotics/messages/remote.hpp>`、`"latest.hpp"`。不要依赖被删除头文件间接带入类型。

## 推荐手改顺序

### 1. 先确定两轴配置

在 `src/board_config.hpp` 中分别定义：

- `yaw_motor`、`pitch_motor` 设备指针；每轴真实后端类型。
- `yaw`、`pitch` 两份 `YawGimbalConfig`。
- `yawMotorConfig()`、`pitchMotorConfig()` 两份 `PositionMotor::Config`。
- `command_timeout_ms`、每轴方向符号和最大角速度。
- `connections_configured`、`emergencyStopRequested()`、`takeEmergencyResetRequest()`。

删掉 `require_referee_for_motion`、`referee_version`、裁判/板间 UART、裁判许可超时和底盘超时。只有接线、零点、限位和单位确认后才将 `connections_configured` 设为 true。原急停函数返回固定 false，是占位实现，不能宣称已有物理急停。

每轴只构造实际使用的一种后端：DJI 或 DM；不要继续为同一轴同时构造两个候选后端。两轴型号不同则保留两种后端支持。

pitch 通常有机械限位，使用 `YawTopology::Limited`，对应 `PositionReference::DriverContinuous`。小 yaw 如果有线束或机械约束也应 Limited。只有确认可连续旋转才沿用 `Continuous + AbsoluteNearest`。

Limited 的上下限必须与驱动反馈使用同一零点和坐标分支，不能把“上电位置”自动当成机械零点。角度单位为 rad，角速度为 rad/s，角度换算为 `度 × π / 180`。两轴电流/力矩限制、减速比和 PID 分开配置；DM 的 NewtonMeter 与 DJI 的 Ampere 不可混用。现有 yaw 参数不能直接作为 pitch 已调好参数。

自检：两份设备配置独立；同一 CAN 总线上的实际电机 ID 不冲突；限位对应真实装配坐标。

### 2. 删掉整车业务，改成本地双轴执行

保留 `remoteTask` 中 DMA 缓冲区的静态生命周期和 `__nocache`，保留 `uart.service()`、有预算的读取循环、溢出后的 `discardPartial()`、`snapshot()`。不要直接把一段 UART 数据当成完整遥控帧。

直接映射 `RemoteState::analog.right_x/right_y`。可参考 `lib/robotics/command.cpp` 的归一化：

```cpp
// 放在 main.cpp 匿名命名空间；输入为解码后已经去中心值的通道量。
float normalizeStick(std::int16_t raw) {
    const float x = std::clamp(float(raw) / 660.0f, -1.0f, 1.0f);
    constexpr float deadband = 0.03f;
    return std::fabs(x) <= deadband ? 0.0f
        : std::copysign((std::fabs(x) - deadband) / (1.0f - deadband), x);
}
```

660 和 0.03 沿用仓库现有 mapper，仍需观察实际遥控通道范围。现有映射方向为 yaw 负号、pitch 正号，最终按实物确定：

```cpp
GimbalCommand yaw_command{}, pitch_command{};
yaw_command.mode = pitch_command.mode = GimbalMode::Rate;
yaw_command.source = pitch_command.source = ControlSource::Remote;
yaw_command.stamp = pitch_command.stamp = remote.stamp;
yaw_command.yaw_rate_rad_s = -normalizeStick(remote.analog.right_x) * yaw_max_rate;
// 复用 YawGimbal 时，pitch 轴也必须填它实际读取的 yaw_rate_rad_s！
pitch_command.yaw_rate_rad_s = normalizeStick(remote.analog.right_y) * pitch_max_rate;
```

这里是两个相互独立的单轴控制器：`YawGimbal yaw_axis(yaw_motor, yaw_cfg)` 和 `YawGimbal pitch_axis(pitch_motor, pitch_cfg)`。类名虽为 yaw，其 Limited 模式的角度积分、限位和电机接口可供本测试 pitch 使用。不要只填 `pitch_rate_rad_s`，那样它不会动作。此方案不提供 IMU 稳定、世界坐标控制或两轴动力学补偿。

不再使用 mapper 后，右拨杆不切换键鼠输入；右摇杆始终是控制源。建议保留原有左拨杆 Middle 才允许输出的规则，其他位置停机。

以下为 `gimbalTask` 的实现顺序伪代码，需用实际轴配置补齐，不能整段当成可编译替换代码：

```text
创建两份 backend → PositionMotor → YawGimbal；对象生命周期覆盖整个任务
分别 begin，记录配置返回值；任一失败，两轴不允许输出
循环每 5 ms：
    读取 now，计算 dt 秒；读取遥控快照
    检查配置、物理急停，处理明确的本地复位请求
    对非 Active 且未急停的轴调用 poll(now)
    每轴恢复 generation 变化时，记录 ready_ms
    遥控允许 = 快照有效且 online，原始 stamp 未超时，左拨杆为 Middle
    两轴允许 = 两轴均 Ready/Active，反馈未超时，dt 在 (0, 0.02] 内
    恢复允许 = 有晚于两轴本轮 ready_ms 的遥控帧
    推荐：掉线/故障后先拨回非使能档，再拨 Middle，避免自动重启
    急停时：对两轴 suspend(EmergencyStop)，锁存直到明确复位
    其他不允许时：暂停仍处于 Active 的轴，原因按实际选择
    允许时：构造两份 Rate 命令，分别 update(command, Active, dt)
    任一 update 返回负数：记录错误并暂停两轴，不继续另一轴驱动
    每秒打印遥控新鲜度、两轴状态、目标/反馈和错误码
```

等待状态不要每周期无条件调用 `suspend()`，以免破坏恢复进度。急停复位要求急停输入已释放，并对两个轴都调用 `clearEmergencyStop(true)`；退出急停不应等同于立即使能。

`Latest::get/put` 使用非阻塞互斥锁，可能返回 `-EAGAIN`。读取失败时可使用之前快照，但必须继续检查原时间戳；首次快照默认无效。不能每个控制周期把旧遥控数据盖上 `now`，否则会掩盖失联。

反馈超时按每轴实际驱动家族分别选择 DJI/DM 配置。`PositionMotor::update()` 内部会重新取反馈并检查运行周期，不能因为外部检查上一轮 telemetry 就忽略它的返回值。恢复时从当前测量重新播种目标，不能沿用故障前积分角度。

注意：原 `GimbalLocalSafety` 对某些命令过期情况输出 Hold；若精简为本地门控，本指南明确选择遥控失联 Disable。保留该类时也要显式处理这一差异，不能误把失联当回中。

自检：代码中只有两个线程定义；没有裁判或对端心跳作为使能前提；pitch 对应独立电机对象；无效遥控不能推进目标。

### 3. 精简设备树和构建配置

`app.overlay` 删除本测试专用的 `interboard-uart` 别名、usart1 波特率覆盖、rng 启用段和裁判注释。保留/核对继承自板级 DTS 的 `remote-uart`。当前 MC02 板定义使用 UART5 PD2 RX、100000、8E1 和 RX DMA；这是代码配置，真实接线仍需核对。

添加 `pitch-motor` 别名及真实电机节点。只有实际型号也是 GM6020 电流模式时才参考现有 yaw 节点，逐项修改 ID、CAN、减速比、零点、电流限值和 status；否则参考对应驱动绑定。不要仅复制节点改名字，也不要未经确认启用 `current-loop-confirmed`。

`CMakeLists.txt` 移除 `src/command_router.cpp`。项目名和日志名可改为 `gimbal_rc_test`，仅影响辨识。

`prj.conf` 保留 C++、内核栈、日志、串口异步、CAN、电机驱动与控制库，明确写出：

```ini
CONFIG_SKYWALKER_LIB_COMMUNICATION=y
CONFIG_SKYWALKER_REMOTE_DR16=y
CONFIG_SKYWALKER_UART_TRANSPORT=y
CONFIG_SKYWALKER_REFEREE=n
CONFIG_SKYWALKER_INTERBOARD=n
CONFIG_SKYWALKER_LIB_MOTOR_CONTROL=y
CONFIG_SKYWALKER_LIB_ROBOTICS=y
CONFIG_SKYWALKER_ROBOTICS_GIMBAL=y
CONFIG_SKYWALKER_ROBOTICS_COMMAND=n
CONFIG_SKYWALKER_ROBOTICS_SAFETY=n
CONFIG_SKYWALKER_ROBOTICS_SWERVE=n
```

这是对现有配置的精简片段，不是完整 prj.conf。按硬件保留 `CONFIG_SKYWALKER_MOTOR_DJI` / `CONFIG_SKYWALKER_MOTOR_DM` 至少一种，以及对应反馈/命令超时。移除应用为板间 boot ID 打开的 `CONFIG_ENTROPY_GENERATOR=y`。若仍使用 `GimbalLocalSafety`，则 SAFETY 必须为 y。

上述通信和整车选项多项默认 y，只删 include 不会停止编译它们，必须显式设 n。不要删除仓库公共库。

## 关键接口速查

| 接口 | 语义与边界 |
|---|---|
| `Latest<T>::get(T&) / put(const T&)` | 任务上下文调用；成功 0，锁竞争 `-EAGAIN`；get 失败保持调用方原值 |
| `YawGimbal::begin()` | 校验并配置，不等待上电、不使能；参数错误 `-EINVAL`、坐标模式不支持 `-ENOTSUP`，或下层错误 |
| `poll(uint64_t now_ms)` | 推进等待/恢复，成功 0；负数时不进入输出路径；恢复代次变化时重建目标基准 |
| `update(const GimbalCommand&, SafetyAction, float dt_s)` | Ready 时首次使能周期零输出；Active 时角速度积分并执行位置闭环；dt 非法 `-EINVAL`，未初始化 `-EAGAIN`，Limited 实际位置越界 `-ERANGE`；亦会透传电机错误 |
| `suspend(PauseReason)` | 清目标初始化状态并委托电机暂停；返回下层结果，急停原因会进入锁存流程 |
| `clearEmergencyStop(bool released)` | 只在实际释放并收到复位请求时使用，检查返回值；不替代后续新命令和恢复检查 |

电机对象仅由控制任务访问；接收任务只发布遥控快照。回调所属的 UART 和 DMA 对象必须持续存活，沿用原 static 写法。

## 用户手动验证

完成每一节的自检后，在仓库工作环境运行：

```sh
west build -b dm_mc02 samples/robotics/gimbal_control -d build/gimbal_rc_test
west flash -d build/gimbal_rc_test
```

这里没有执行这些命令。刷写前确认连接的是目标板。

上电顺序：先机械支撑 pitch、防止撤销力矩后下坠；拨杆置非使能档；先给控制板/接收机上电，确认右摇杆原始数据与方向；再接电机电源，观察两个轴反馈；低速度、小行程使能测试。保留可立即切断电机动力的手段。

- 横向仅小 yaw 动，纵向仅 pitch 动，回中保持当前目标。
- 拨杆退出使能后两轴停止输出；关闭遥控后在配置超时内撤销输出。
- 模拟反馈丢失或单轴报错，两轴均停；恢复后不追逐旧目标。
- 若不动，依次查 `connections_configured`、电机节点 status、begin 返回值、遥控 stamp/拨杆、反馈时间戳和两轴状态。
- 若 pitch 不动，先查是否错误填写了 `pitch_rate_rad_s`；若一使能就报越界，查 DriverContinuous 坐标、零点及机械限位。
- 若 pitch 回中仍下沉，不要先扩大输出；检查力矩上限、PID、负载和重力补偿需求。复用 yaw 控制器本身不保证带载 pitch 性能。

最终检查：只剩遥控接收与双轴控制任务；两轴真实配置齐全；裁判/板间/底盘依赖消失；保留反馈超时、失联停机、急停锁存、恢复边界与机械限位；构建和实机结果由用户执行确认。
