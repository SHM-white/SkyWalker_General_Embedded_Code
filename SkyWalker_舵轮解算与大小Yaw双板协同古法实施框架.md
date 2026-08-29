# SkyWalker 舵轮解算与大小 Yaw 双板协同古法实施框架

> 目标：在多型号电机驱动和 PID 可用之后，由你亲手搭出四舵轮底盘、双板通信，以及大 yaw/小 yaw 协同控制的第一版框架。
>
> 本文只给架构、接口、伪代码、实施顺序和验收方法，没有修改任何业务源码。

## 0. 先说结论

不要把旧仓库的 `chassis_ctrl.c` 整份搬过来。

新项目建议拆成六块：

```text
纯数学层
  ├─ 坐标变换
  ├─ 舵轮逆运动学
  ├─ 舵轮正运动学
  └─ 90°翻转优化

单轮控制层
  ├─ 驱动轮速度 PI
  ├─ 舵向位置 P/PD
  ├─ 舵向速度 PI
  └─ GM6020 电流输出适配

底盘控制层
  ├─ 命令限幅与斜坡
  ├─ 四轮目标生成
  ├─ 对齐保护
  ├─ 电机分组发送
  └─ 故障状态机

大小 yaw 协调层
  ├─ 世界/底盘/大 yaw/小 yaw 坐标关系
  ├─ 小 yaw 快速稳像
  ├─ 大 yaw 低频跟随与卸载
  └─ 软限位、硬限位和降级

双板协议层
  ├─ 逻辑消息
  ├─ 明确字节序的编解码
  ├─ sequence/timestamp/valid/fault
  └─ CAN-FD、Classic CAN 或 UART 传输适配

双应用层
  ├─ chassis board：舵轮 + 大 yaw
  └─ gimbal board：小 yaw + IMU + 目标 + 协调器
```

最重要的边界：

- 电机闭环必须留在连接该电机的板上。
- 跨板只传目标、测量摘要和故障状态。
- 舵轮解算不认识 Zephyr device、CAN ID 或 PID。
- 大小 yaw 只有一个协调器，不能两块板各算一套目标互相打架。
- GM6020 在本项目中默认走电流控制：`0x1FE`/`0x2FE`、raw `±16384`，约对应 `±3 A`。

## 1. 本文假设与必须实测的量

为了先把框架搭起来，本文采用以下假设：

1. 底盘是四模块矩形舵轮。
2. 每个模块由一个 M3508 驱动轮和一个 GM6020 舵向电机构成。
3. GM6020 使用电流控制，不使用电压控制作为默认路径。
4. 底盘板连接四个驱动电机、四个舵向电机和大 yaw 电机。
5. 云台板连接小 yaw 电机、上层 IMU、遥控/视觉输入。
6. 大 yaw 与小 yaw 的旋转轴近似同轴。
7. 上层 IMU 安装在小 yaw 之后，能测到最终云台世界 yaw。
8. 两块板都有单调递增的本地时间，但第一版不要求时钟完全同步。
9. 工程继续使用 C++14。

以下信息目前不能从仓库确定，写代码前要填表：

| 参数 | 你要填写的实测值 |
|---|---|
| 前后轮中心距 | ___ m |
| 左右轮中心距 | ___ m |
| 驱动轮有效半径 | ___ m |
| M3508 实际减速比 | ___ |
| 四个驱动电机安装方向 | `+1/-1`：___ |
| 四个 GM6020 安装方向 | `+1/-1`：___ |
| 四个舵向零点 raw encoder | ___ |
| 大 yaw 电机型号及减速比 | ___ |
| 小 yaw 电机型号及减速比 | ___ |
| 大 yaw 可连续旋转还是有线缆限位 | ___ |
| 小 yaw 软限位/硬限位 | ___ rad |
| 两板物理链路 | CAN / CAN-FD / UART：___ |
| 每块板可用 CAN 控制器数量 | ___ |
| IMU 实际安装在哪一级 | ___ |

如果假设 4～7 与机械实物不一致，保留本文模块边界，但必须重画第 10 节的角度关系，不能硬套公式。

## 2. 已核查的旧仓库

参考仓库：

- `https://gitee.com/esmorang/Roam-the-Heavens-and-Ride-the-Wind.git`
- 本次核查提交：`60a0843e40271a5855044a15e53bda51fd868464`

重点文件：

- `Sentinel_rob/H7_Chassis/Users/Application/Control/app_control.c`
- `Sentinel_rob/H7_Chassis/Users/Application/Communication/app_communication.c`
- `Sentinel_rob/H7_Chassis/Users/Application/Init/app_init.c`
- `Sentinel_rob/H7_Chassis/Users/Device/Motor/DJI/dev_dji_motor.c`
- `Sentinel_rob/H7_Chassis/Users/Config/config.c`
- `Sentinel_rob/H7_Chassis/Users/Type/type.h`
- `Sentinel_Robot/Chassis/Users/Application/chassis_ctrl.c`

旧仓库已经做过：

- 四模块舵轮逆运动学。
- 舵向目标超过 90°时，将目标转 180°并反转驱动轮速度。
- M3508 速度环。
- GM6020 位置—速度串级，最终输出电流。
- 2 ms 控制周期和 20 ms 板间通信周期。
- GM6020 电流 raw 与安培换算。
- 通过 UART 传底盘期望速度和实际速度。

## 3. 旧代码哪些能参考，哪些不能照搬

### 3.1 可以参考的思路

- 单轮速度向量使用：

```text
v_ix = vx - wz * y_i
v_iy = vy + wz * x_i
```

- 使用 `hypot` 求轮速，`atan2` 求舵角。
- 舵角旋转超过 90°时反转驱动速度。
- GM6020 采用位置外环、速度内环、电流输出。
- 控制和通信使用不同周期。
- 将四个电机命令放入对应 DJI 分组帧后统一发送。

### 3.2 不能照搬的部分

#### 所有职责都塞在一个应用文件

旧代码把解算、PID、CAN 打包、功率估算和任务调度放在一起。这样任何一个符号错误都会扩散到整条链路。

新项目必须让纯数学函数可以在不连接电机、不启动 Zephyr 的情况下测试。

#### 四个轮子的公式和方向被手写展开

旧代码分别写四遍公式，并用若干负号修正电机方向。轮序一变就很难检查。

新项目使用 `ModuleGeometry[4]` 描述每个模块的位置、零点和方向，循环计算。

#### 缺少轮速整体缩放

某个轮子超过最大速度时，不能只截断该轮，否则合成运动方向会改变。应该按同一个比例缩放四个轮速。

#### 零速时会重新计算角度

`atan2(0, 0)` 没有有效方向。直接算通常得到 0，四个舵轮会在停车时突然回零。

第一版策略：命令速度低于阈值时保持上一拍舵角；X 锁是单独模式，不是零速默认行为。

#### 正运动学公式不应复制

旧代码的正运动学直接手写长表达式，单位中没有清晰体现轮径、减速比和模块几何。新项目用矩阵最小二乘或经过推导的对称底盘公式。

#### 裸 `packed struct + float` 通信

旧仓库直接把 packed C 结构体映射到 UART buffer。它依赖：

- 两端 float 格式一致。
- 两端字节序一致。
- 编译器布局一致。
- 一次 UART 回调正好收到完整帧。

新项目要逐字段编码，加入版本、长度、序号、源时间、有效位和 CRC。UART 必须支持粘包、拆包和重新同步。

#### 没有完整失联行为

旧代码收到命令后直接写全局状态，没有记录最后接收时间。通信断掉后可能继续保持最后一次速度。

新项目所有命令都必须有 age，超时后先斜坡归零，再清零电机电流并 reset PID。

#### `24V × 转矩电流绝对值` 不是可靠底盘功率

GM6020/M3508 反馈中的转矩电流不等于电池母线电流。这个值最多做粗略相对指标，不能当裁判系统功率控制依据。

## 4. 当前仓库距离目标还缺什么

当前项目的真实状态：

- `application/CMakeLists.txt`、`application/prj.conf` 和 `application/src/main.c` 还是空文件。
- 业务驱动目前只有可运行的 M3508 骨架。
- `motor_api.cpp` 和 `motor_can_router.cpp` 还是空占位。
- 当前 `TxGroup` 内部硬编码调用 M3508 namespace，不能直接发送 GM6020。
- M2006、GM6020、DM4310 驱动仍需要你按电机实施手册完成。
- PID 仍需要按前一份手册增加 reset、dt 检查、测量微分和抗饱和。
- 没有舵轮数学库。
- 没有底盘控制器。
- 没有板间协议。
- 没有大小 yaw 协调器。
- IMU 内部计算了 `YawTotal`，但当前公开到 `imu_data.angle[2]` 的仍是单圈 yaw；使用者必须按所需语义明确取单圈还是连续 yaw。

因此不要从“写底盘 main”开始。先完成纯算法，再接一只模块。

## 5. 推荐目录框架

下面是你后续亲手创建的建议结构：

```text
include/
├── control/
│   ├── chassis_types.hpp
│   ├── swerve_kinematics.hpp
│   ├── swerve_module_controller.hpp
│   ├── chassis_controller.hpp
│   └── yaw_coordinator.hpp
└── lib/
    └── board_link/
        ├── board_link_messages.hpp
        └── board_link_codec.hpp

lib/
├── control/
│   ├── CMakeLists.txt
│   ├── swerve_kinematics.cpp
│   ├── swerve_module_controller.cpp
│   ├── chassis_controller.cpp
│   └── yaw_coordinator.cpp
└── board_link/
    ├── CMakeLists.txt
    ├── board_link_codec.cpp
    ├── board_link_can.cpp
    └── board_link_uart.cpp

application/
├── chassis/
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── boards/
│   │   └── dm_mc02.overlay
│   └── src/
│       ├── main.cpp
│       ├── chassis_control_task.cpp
│       └── chassis_link_task.cpp
└── gimbal/
    ├── CMakeLists.txt
    ├── prj.conf
    ├── boards/
    │   └── dm_mc02.overlay
    └── src/
        ├── main.cpp
        ├── gimbal_control_task.cpp
        └── gimbal_link_task.cpp

tests/
├── swerve_kinematics/
├── yaw_coordinator/
└── board_link_codec/
```

为什么把两个固件分成两个 application：

- 两块板的设备树不同。
- 电机实例和 CAN 总线不同。
- 故障策略不同。
- 编译出来就是两个明确的固件，不靠运行时猜“我是哪块板”。

第一版不要急着引入 sysbuild。先让两个 application 可以分别 build、flash 和独立运行。

## 6. 公共数据类型

文件：`include/control/chassis_types.hpp`

### 6.1 坐标约定

全项目统一：

- `+x`：机器人前方。
- `+y`：机器人左方。
- `+z`：机器人上方。
- `+yaw`、`+wz`：从上往下看逆时针。
- 位置：m。
- 线速度：m/s。
- 角度：rad。
- 角速度：rad/s。
- 控制周期：s。

不允许某一层突然使用角度、rpm 或毫米。

### 6.2 类型建议

```cpp
enum class MotionFrame : std::uint8_t
{
    Chassis = 0,
    World,
    Gimbal,
};

struct ChassisTwist
{
    float vx_m_s = 0.0f;
    float vy_m_s = 0.0f;
    float wz_rad_s = 0.0f;
};

struct ModuleGeometry
{
    float x_m = 0.0f;
    float y_m = 0.0f;
    float wheel_radius_m = 0.0f;
    float drive_gear_ratio = 1.0f;
    float steer_zero_offset_rad = 0.0f;
    std::int8_t drive_sign = 1;
    std::int8_t steer_sign = 1;
};

struct SwerveModuleFeedback
{
    float drive_velocity_rad_s = 0.0f;
    float steer_position_rad = 0.0f;
    float steer_velocity_rad_s = 0.0f;
    std::uint64_t timestamp_ms = 0;
    bool drive_valid = false;
    bool steer_valid = false;
};

struct SwerveModuleTarget
{
    float wheel_speed_m_s = 0.0f;
    float drive_velocity_rad_s = 0.0f;
    float steer_position_rad = 0.0f;
};
```

`steer_position_rad` 的契约必须写清：

- 它是去掉机械零点、应用安装方向后的模块角。
- 供控制使用时最好是连续角，而不是每圈跳回 `[-π, π)` 的角。
- raw encoder、绝对电机角和模块相对角不能共用同一个变量名。

## 7. 舵轮逆运动学

文件：

- `include/control/swerve_kinematics.hpp`
- `lib/control/swerve_kinematics.cpp`

### 7.1 单模块公式

模块中心相对底盘中心的位置为 `(x_i, y_i)`。

底盘命令为 `(vx, vy, wz)`。

模块接地点速度：

```text
v_ix = vx - wz * y_i
v_iy = vy + wz * x_i

wheel_speed_i = sqrt(v_ix² + v_iy²)
steer_angle_i = atan2(v_iy, v_ix)
```

这个公式已经包含平移和旋转，不要再根据“左前轮”“右后轮”手工写四套正负号。

### 7.2 推荐接口

```cpp
struct SwerveKinematicsConfig
{
    ModuleGeometry modules[4];
    float max_wheel_speed_m_s = 0.0f;
    float stop_speed_threshold_m_s = 0.0f;
};

struct SwerveKinematicsState
{
    float last_steer_target_rad[4] = {};
    bool initialized[4] = {};
};

int swerve_inverse(
    const SwerveKinematicsConfig &config,
    const ChassisTwist &command,
    const float current_steer_rad[4],
    SwerveKinematicsState &state,
    SwerveModuleTarget out[4]);
```

行为契约：

- 指针或数组由调用方保证有效。
- 所有输入必须 finite。
- 轮径、减速比、最大速度必须大于 0。
- 非法输入返回 `-EINVAL`，同时把四个输出速度清零。
- 几何或限幅非法返回 `-ERANGE`。
- 零速时保持上一拍舵向目标。
- 成功返回 0。
- 函数不分配内存、不加锁、不读取设备、不发送 CAN。

### 7.3 轮速整体缩放

先算出四轮速度，再找绝对值最大项：

```text
peak = max(abs(wheel_speed[0..3]))

if peak > max_wheel_speed:
    scale = max_wheel_speed / peak
    wheel_speed[i] *= scale
```

四轮必须使用同一个 `scale`，这样速度矢量比例不变。

### 7.4 90°翻转优化

```cpp
float delta = wrap_pi(
    desired_angle_rad - current_angle_rad);

if (delta > HALF_PI) {
    delta -= PI;
    desired_speed_m_s = -desired_speed_m_s;
} else if (delta < -HALF_PI) {
    delta += PI;
    desired_speed_m_s = -desired_speed_m_s;
}

const float continuous_target =
    current_angle_rad + delta;
```

注意最后返回 `current + delta`，不要立刻把目标重新压回 `[0, 2π)`。连续目标能避免在 0/2π 边界突然绕一整圈。

### 7.5 停车角度策略

当每个模块的目标平移速度和旋转速度都很小：

- Normal Stop：保持上一拍舵角，驱动速度归零。
- X Lock：四轮指向“X”形，作为明确的独立模式。
- Boot：尚无有效舵角时，不允许记住未初始化角度。

不要用 `atan2(0, 0)` 决定停车方向。

### 7.6 驱动转子目标速度

```text
wheel_omega = wheel_speed_m_s / wheel_radius_m
motor_omega = wheel_omega * drive_gear_ratio * drive_sign
```

`motor_omega` 单位为 rad/s，可直接交给 M3508 速度环。

## 8. 坐标变换

遥控/导航可能给世界坐标速度，逆解算需要底盘坐标速度。

若底盘世界 yaw 为 `psi_chassis`：

```text
vx_body =  cos(psi_chassis) * vx_world
         + sin(psi_chassis) * vy_world

vy_body = -sin(psi_chassis) * vx_world
         + cos(psi_chassis) * vy_world
```

建议接口：

```cpp
int transform_twist_to_chassis(
    MotionFrame source_frame,
    const ChassisTwist &input,
    float chassis_yaw_world_rad,
    float gimbal_yaw_world_rad,
    ChassisTwist &output);
```

边界：

- `wz` 本身绕同一个 z 轴旋转，不因二维坐标旋转改变符号。
- yaw 反馈无效时，World/Gimbal 模式返回 `-EAGAIN`。
- Chassis 模式不依赖 IMU。
- 一定先写纯平移测试，再接遥控器通道。

## 9. 正运动学

每个模块已知轮速 `s_i` 和舵角 `alpha_i`：

```text
v_ix = s_i * cos(alpha_i)
v_iy = s_i * sin(alpha_i)
```

每个模块提供两条方程：

```text
v_ix = vx - wz * y_i
v_iy = vy + wz * x_i
```

把四个模块组成 8 条方程：

```text
b = A * [vx, vy, wz]^T
```

使用最小二乘：

```text
twist = inverse(A^T A) * A^T * b
```

`A` 只由模块位置决定，可以初始化时预计算。某个模块故障时，可用剩余模块重新构造方程；第一版若不实现降阶解算，就直接标记里程计无效。

不要先照搬旧仓库的长公式。先让逆解算的目标重新送入正解算，检查能否恢复原始 `vx/vy/wz`。

## 10. 单个舵轮模块控制

文件：

- `include/control/swerve_module_controller.hpp`
- `lib/control/swerve_module_controller.cpp`

### 10.1 控制链

```text
steer angle target
        │
        ▼
位置 P/PD
        │ steer speed target
        ▼
速度 PI
        │ current A 或 current_raw
        ▼
GM6020 电流模式

drive speed target
        │
        ▼
速度 PI + kS/kV
        │ current_raw
        ▼
M3508/C620
```

第一版推荐：

- 舵向位置环：P，必要时少量 D。
- 舵向速度环：PI。
- 驱动速度环：PI。
- 不要三个环同时从随便抄来的 PID 参数开始。

### 10.2 模块控制器接口

```cpp
struct SwerveModuleControlConfig
{
    pid_config drive_speed_pid;
    pid_config steer_position_pid;
    pid_config steer_speed_pid;

    float max_drive_current_raw = 0.0f;
    float max_steer_current_a = 0.0f;
    float max_steer_speed_rad_s = 0.0f;
    float align_full_drive_error_rad = 0.0f;
    float align_zero_drive_error_rad = 0.0f;
};

struct SwerveModuleControlState
{
    pid_data drive_speed_state;
    pid_data steer_position_state;
    pid_data steer_speed_state;
};

struct SwerveModuleEffort
{
    std::int16_t drive_current_raw = 0;
    std::int16_t steer_current_raw = 0;
};

int swerve_module_step(
    const SwerveModuleControlConfig &config,
    SwerveModuleControlState &state,
    const SwerveModuleTarget &target,
    const SwerveModuleFeedback &feedback,
    float dt_s,
    SwerveModuleEffort &out);

void swerve_module_reset(
    SwerveModuleControlState &state,
    const SwerveModuleFeedback &feedback);
```

### 10.3 对齐保护

舵还没有转到目标方向时，不应让驱动轮全速刮地。

可以对驱动速度乘：

```text
alignment_scale = max(0, cos(steer_error))
drive_target *= alignment_scale
```

再加硬门限：

- 误差小于 `align_full_drive_error`：正常输出。
- 中间区域：平滑缩小驱动速度。
- 大于 `align_zero_drive_error`：驱动电流为 0。

启动阶段四个舵轮都对齐后，底盘状态机才进入 Active。

## 11. GM6020 电流控制在舵向环里的位置

DJI GM6020 v1.4 的电流模式：

| 电机 ID | 反馈 ID | 电流命令 ID | slot |
|---:|---:|---:|---:|
| 1 | `0x205` | `0x1FE` | 0 |
| 2 | `0x206` | `0x1FE` | 1 |
| 3 | `0x207` | `0x1FE` | 2 |
| 4 | `0x208` | `0x1FE` | 3 |
| 5 | `0x209` | `0x2FE` | 0 |
| 6 | `0x20A` | `0x2FE` | 1 |
| 7 | `0x20B` | `0x2FE` | 2 |

raw 范围：

```text
-16384 ... 0 ... +16384
约对应
-3 A ... 0 ... +3 A
```

建议控制器内部用安培，最终适配时转换：

```cpp
float safe_a = clampf(
    requested_a,
    -mechanical_safe_current_a,
    mechanical_safe_current_a);

float raw_f = safe_a * 16384.0f / 3.0f;

std::int16_t raw =
    static_cast<std::int16_t>(
        std::lround(raw_f));
```

`mechanical_safe_current_a` 必须小于等于 3 A，并从很小值开始测试。

不要：

- 把 `0x1FF` 当成电流帧；它属于电压模式。
- 把 `±30000` 当成电流 raw 上限。
- 把 GM6020 的 `±16384` 与 M3508 的 `±16384` 当成相同安培比例。
- 用一个 TxGroup 同时混入不同 command ID 的实例。

## 12. 底盘控制器

文件：

- `include/control/chassis_controller.hpp`
- `lib/control/chassis_controller.cpp`

### 12.1 输入输出

```cpp
enum class ChassisMode : std::uint8_t
{
    Disabled = 0,
    Normal,
    XLock,
};

struct ChassisCommand
{
    ChassisMode mode = ChassisMode::Disabled;
    MotionFrame frame = MotionFrame::Chassis;
    ChassisTwist twist;
    std::uint32_t sequence = 0;
    std::uint64_t timestamp_ms = 0;
};

struct ChassisFeedback
{
    ChassisTwist measured_twist;
    std::uint32_t fault_bits = 0;
    std::uint64_t timestamp_ms = 0;
    bool valid = false;
};
```

### 12.2 一拍执行顺序

```text
1. 复制一份最新命令快照
2. 检查 sequence、timestamp、finite、mode
3. 检查八个电机反馈 freshness
4. 将 World/Gimbal 命令变换到底盘坐标
5. 对 vx/vy/wz 做限幅和斜坡
6. 运行 swerve_inverse
7. 对四个模块运行 swerve_module_step
8. 写入四个 M3508 raw current
9. 写入四个 GM6020 raw current
10. 每个 TxGroup 只 send 一次
11. CAN 任一发送失败：本拍转安全输出并记录 fault
12. 用反馈运行正运动学，发布状态快照
```

不要在 CAN RX callback、UART callback 或 `k_timer` callback 里做这些计算。callback 只更新时间戳、写缓存、投递信号量或消息队列。

## 13. 大小 yaw 的坐标关系

先定义：

- `psi_c`：底盘相对世界的 yaw。
- `theta_b`：大 yaw 相对底盘的角度。
- `theta_s`：小 yaw 相对大 yaw 的角度。
- `psi_g`：最上层云台相对世界的 yaw。
- `s_b`、`s_s`：安装方向，取 `+1` 或 `-1`。
- `psi_zero`：机械安装零偏。

若三层旋转轴同轴：

```text
psi_g =
    psi_c
    + s_b * theta_b
    + s_s * theta_s
    + psi_zero
```

角速度关系：

```text
omega_g =
    omega_c
    + s_b * omega_b
    + s_s * omega_s
```

上层 IMU若直接测得 `psi_g`，世界角闭环优先使用 IMU，而不是三个编码器相加。编码器和上式主要用于：

- 检查机械关系。
- 计算小 yaw 离中心还有多远。
- 通信失效时降级。
- 检测方向或零点错误。

### 13.1 为什么必须同时保留单圈角和连续角

- 世界 yaw 误差通常用 `wrap_pi(target - measurement)`。
- 电缆管理、累计转圈和轨迹规划可能需要连续 yaw。
- 小 yaw 有机械限位时必须使用相对关节角，不能只看世界 yaw。

建议每个 yaw 状态明确包含：

```cpp
struct YawJointState
{
    float relative_angle_rad = 0.0f;
    float continuous_angle_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    std::uint64_t timestamp_ms = 0;
    bool valid = false;
};
```

## 14. 大小 yaw 怎么配合

推荐“快小 yaw + 慢大 yaw”的主从卸载方案：

- 小 yaw：用上层 IMU做最终世界 yaw 快速闭环，负责高频误差和瞄准精度。
- 大 yaw：低频跟随，把小 yaw 慢慢拉回中心，负责大范围运动。
- 协调器：只运行在云台板，因为视觉目标、上层 IMU和小 yaw 状态都在这里。
- 底盘板：只接收大 yaw 目标，并在本板完成大 yaw 电机闭环。

### 14.1 不要直接固定比例分角度

下面这种第一眼很简单：

```text
big_target   = 0.7 * total_target
small_target = 0.3 * total_target
```

它在角度跨 ±π、通信延迟、关节限位和底盘旋转时都会出问题。

更稳妥的是：

1. 小 yaw 始终控制最终世界 yaw。
2. 大 yaw 根据小 yaw 相对中心的偏移慢慢跟随。
3. 大 yaw 动作造成的剩余扰动，由小 yaw 世界角闭环消除。

### 14.2 卸载算法

令小 yaw 中心为 `theta_s_center`。

```text
center_error =
    theta_s_relative - theta_s_center

center_term =
    k_center * deadzone(
        center_error,
        center_deadband)
```

若给定了最终云台世界角速度 `omega_g_ref`，底盘当前角速度为 `omega_c`：

```text
required_relative_rate =
    omega_g_ref - omega_c

big_rate_feedforward =
    lowpass(required_relative_rate)

big_rate_ref =
    clamp(
        big_rate_feedforward + center_term,
        -big_rate_max,
        +big_rate_max)

small_rate_feedforward =
    required_relative_rate - big_rate_ref
```

所有符号还要乘各自安装方向；第一次上电必须逐轴验证正方向。

如果第一版还没有可靠的 `omega_g_ref`：

- 先把 `big_rate_feedforward` 设为 0。
- 只用 `center_term` 让大 yaw 慢慢回中。
- 小 yaw 继续独立做世界角闭环。

这样功能少，但容易确认谁在跟谁。

### 14.3 推荐接口

文件：

- `include/control/yaw_coordinator.hpp`
- `lib/control/yaw_coordinator.cpp`

```cpp
enum class YawCoordinationMode : std::uint8_t
{
    Disabled = 0,
    Independent,
    FollowAndRecenter,
};

struct YawCoordinatorConfig
{
    float small_center_rad = 0.0f;
    float small_center_deadband_rad = 0.0f;
    float small_soft_limit_rad = 0.0f;
    float small_hard_limit_rad = 0.0f;
    float big_rate_max_rad_s = 0.0f;
    float big_accel_max_rad_s2 = 0.0f;
    float center_kp = 0.0f;
    float split_filter_tau_s = 0.0f;
};

struct YawCoordinatorInput
{
    YawCoordinationMode mode =
        YawCoordinationMode::Disabled;

    float upper_yaw_target_world_rad = 0.0f;
    float upper_yaw_rate_ref_rad_s = 0.0f;
    float upper_yaw_world_rad = 0.0f;
    float chassis_yaw_rate_rad_s = 0.0f;
    YawJointState big_yaw;
    YawJointState small_yaw;
    float dt_s = 0.0f;
};

struct YawCoordinatorOutput
{
    float big_yaw_rate_ref_rad_s = 0.0f;
    float small_yaw_world_target_rad = 0.0f;
    float small_yaw_rate_ff_rad_s = 0.0f;
    std::uint32_t status_bits = 0;
};

int yaw_coordinator_step(
    const YawCoordinatorConfig &config,
    const YawCoordinatorInput &input,
    YawCoordinatorOutput &output);
```

行为：

- 输入无效、dt 非法或状态过期：返回 `-EAGAIN`，大 yaw rate 输出 0。
- Disabled：两个协调输出为安全值。
- Independent：不做卸载，大 yaw 目标为 0，小 yaw 独立跟踪。
- FollowAndRecenter：运行低频跟随和中心卸载。
- 小 yaw 超过 soft limit：提高回中优先级，同时限制上层目标速度。
- 小 yaw达到 hard limit：禁止继续向限位外运动，设置 fault。
- 大 yaw 达到自身限位：停止向限位外运动，小 yaw若仍有余量则降级承接。
- 所有输出经过 rate 和 acceleration limit。

### 14.4 防止两个 yaw 互相打架

必须满足：

- 云台板的协调器是唯一大 yaw 目标来源。
- 底盘板不再根据自己的角度另算一套“自动回中”。
- 底盘板只做大 yaw 本地位置/速度/电流闭环和安全限幅。
- 小 yaw 的最终世界角闭环不跨板。
- 切换模式时 reset 相关 PID，并使用无突变目标初始化。

## 15. 双板职责

### 15.1 底盘板

拥有：

- 四个 M3508 驱动电机。
- 四个 GM6020 舵向电机。
- 大 yaw 电机。
- 底盘 IMU或至少 `gyro_z`，若硬件存在。
- 舵轮逆解算、模块 PID、正解算。
- 大 yaw 本地闭环。
- 电机和通信失联安全状态机。

接收：

- 底盘 `vx/vy/wz` 命令。
- 命令所属坐标系。
- 大 yaw rate/position 目标。
- 模式和 enable token。

发送：

- 大 yaw 角度/速度/有效位。
- 底盘 yaw/gyro_z。
- 正解算速度。
- 电机在线位图、故障位图。
- 最近有效命令 sequence。

### 15.2 云台板

拥有：

- 小 yaw 电机。
- 上层 IMU。
- 视觉和遥控输入。
- 小 yaw 世界角闭环。
- 大小 yaw 协调器。
- 最终底盘命令生成。

接收：

- 大 yaw 实际状态。
- 底盘 yaw/gyro_z。
- 底盘故障和 online 状态。

发送：

- 底盘运动命令。
- 大 yaw 目标。
- 控制模式、enable token 和 sequence。

## 16. 板间逻辑消息

文件：

- `include/lib/board_link/board_link_messages.hpp`
- `include/lib/board_link/board_link_codec.hpp`
- `lib/board_link/board_link_codec.cpp`

### 16.1 不把 transport 写进业务结构

先定义逻辑消息：

```cpp
struct LinkHeader
{
    std::uint8_t protocol_version = 1;
    std::uint8_t message_type = 0;
    std::uint16_t sequence = 0;
    std::uint32_t source_time_ms = 0;
};

struct ChassisCommandMessage
{
    LinkHeader header;
    ChassisCommand command;
    std::uint32_t enable_token = 0;
};

struct BigYawCommandMessage
{
    LinkHeader header;
    float rate_ref_rad_s = 0.0f;
    float position_ref_rad = 0.0f;
    std::uint32_t valid_bits = 0;
};

struct ChassisStateMessage
{
    LinkHeader header;
    ChassisFeedback chassis;
    YawJointState big_yaw;
    float chassis_yaw_world_rad = 0.0f;
    float chassis_yaw_rate_rad_s = 0.0f;
    std::uint32_t online_bits = 0;
    std::uint32_t fault_bits = 0;
};
```

然后由 CAN-FD、Classic CAN 或 UART adapter 决定怎么拆成帧。

### 16.2 编解码接口

```cpp
int encode_chassis_command(
    const ChassisCommandMessage &message,
    std::uint8_t *buffer,
    std::size_t capacity,
    std::size_t &written);

int decode_chassis_command(
    const std::uint8_t *buffer,
    std::size_t length,
    ChassisCommandMessage &message);
```

其他消息提供同形接口。

返回值：

- 成功：0。
- buffer 太小：`-ENOSPC`。
- 长度不对：`-EMSGSIZE`。
- 版本不支持：`-EPROTONOSUPPORT`。
- CRC 错误：`-EBADMSG`。
- 字段不是 finite 或超过协议范围：`-ERANGE`。

### 16.3 逐字段编码

不要：

```cpp
memcpy(buffer, &message, sizeof(message));
```

要明确写：

- 整数的大小端。
- float 是使用 IEEE754 bytes，还是先缩放为定点整数。
- 每个字段的 offset 和长度。
- CRC 覆盖范围。

第一版更推荐固定点：

- 线速度：`int16_t`，单位 1 mm/s。
- 角速度：`int16_t`，单位 1 mrad/s。
- 单圈角度：`int16_t`，单位 1 mrad。
- 连续角：`int32_t`，单位 1 mrad。
- 时间：`uint32_t`，单位 ms。

这样日志和抓包更容易看懂，也不会直接依赖两端 float ABI。

### 16.4 UART framing

如果使用 UART，建议帧格式：

```text
SOF 0xA5
protocol_version u8
message_type u8
payload_length u16
sequence u16
source_time_ms u32
payload N bytes
crc16 u16
```

UART RX 必须有状态机：

1. 搜 SOF。
2. 收固定头。
3. 检查 version 和 length 上限。
4. 等完整 payload + CRC。
5. 校验。
6. 成功后投递完整消息。
7. 失败时丢一个字节重新找 SOF。

不要假设一次 UART callback 就是一帧。

### 16.5 CAN/CAN-FD

- CAN 自带链路层 CRC，但 sequence、timestamp 和 valid bits 仍然必须保留。
- Classic CAN 只有 8 字节，逻辑消息要拆成多个固定 ID 的小帧。
- CAN-FD 可放更完整的状态，但仍要定义 payload version。
- 控制命令使用固定周期覆盖发送，不依赖“只变化时才发”。
- callback 解码成功后写入消息队列或带锁快照，不直接改 PID 状态。

## 17. 通信 freshness 与安全行为

建议起始阈值，不是最终硬编码参数：

| 条件 | 建议起点 | 行为 |
|---|---:|---|
| 底盘命令 age | > 30 ms | 标记 stale，开始将速度目标斜坡拉到 0 |
| 底盘命令 age | > 100 ms | 四个驱动电流清零，模块 PID reset |
| 大 yaw 命令 age | > 30 ms | 大 yaw rate 目标斜坡到 0 |
| 大 yaw 命令 age | > 100 ms | 大 yaw电流清零或进入经机械确认的安全保持模式 |
| 任一电机反馈 age | > 10 ms | 对应模块立即禁止驱动 |
| 两板状态 age | > 100 ms | 大小 yaw 协同退出，进入 Independent/Disabled |

阈值必须：

- 大于正常周期和抖动。
- 小于电机驱动 100 ms 在线心跳，不能等驱动已经 Offline 才停。
- 经过拔线试验确认。

`enable_token` 用于防止旧命令在重启后误恢复：

1. 上层进入 enable 时生成新 token。
2. 每条控制消息携带 token。
3. 底盘板只接受当前会话 token。
4. 任一板重启后必须重新走 enable handshake。

## 18. CAN 总线规划

四个 M3508 和四个 GM6020 都可能约 1 kHz 反馈。

粗算一帧 Classic CAN 数据帧连同仲裁、CRC、ACK、填充和间隔约百余 bit：

```text
8 motors * 1000 frames/s * about 120 bit
≈ 960 kbit/s
```

这还没算控制帧、错误重发和板间通信，不能把八台电机和板间通信全塞进一条 1 Mbps CAN。

推荐起始分配：

| 总线 | 设备 |
|---|---|
| chassis CAN-A | 四个 M3508：反馈 `0x201..0x204`，命令 `0x200` |
| chassis CAN-B | 四个 GM6020：反馈 `0x205..0x208`，电流命令 `0x1FE` |
| chassis CAN-C 或 UART | 双板通信；必要时再规划大 yaw |
| gimbal local CAN | 小 yaw 与云台其他电机 |

如果底盘板只有两路 CAN：

- 两路优先留给两组高频电机。
- 双板通信优先使用独立 UART、CAN-FD 或硬件允许的其他链路。
- 不要未经实测就把板间帧塞进已经接近满载的电机 CAN。

大 yaw 的电机型号和 ID 目前未知。在它确定前，不要最终冻结总线表。

## 19. Zephyr 线程与数据所有权

建议频率：

| 任务 | 建议起点 | 说明 |
|---|---:|---|
| CAN RX callback | 事件驱动 | 只解码和提交反馈 |
| chassis motor control | 500 Hz | 四模块 PID 与分组发送 |
| swerve command/kinematics | 250～500 Hz | 第一版可与 motor control 同线程 |
| small yaw control | 500～1000 Hz | 与 IMU有效采样率匹配 |
| big yaw local control | 500 Hz | 在底盘板 |
| board link TX/RX | 100～200 Hz | 命令和状态 |
| health monitor | 100 Hz | freshness/fault/state |
| telemetry/log | 20～50 Hz | 不阻塞控制线程 |

推荐使用：

- `k_timer` 只 `k_sem_give()`。
- 高优先级控制线程等待 semaphore 后计算。
- 通信 RX 使用 `k_msgq` 或双缓冲快照。
- 配置由初始化线程写完后只读。
- PID state 只允许所属控制线程写。

不要：

- 在 UART callback 中跑 yaw coordinator。
- 在 CAN callback 中调用 PID。
- 控制线程直接高频 `printk`。
- 两个线程同时写同一个 `ChassisCommand`。

## 20. 两块板的主循环伪代码

### 20.1 底盘板

```cpp
for (;;) {
    wait_control_tick();

    const std::uint64_t now_ms = monotonic_ms();
    const float dt_s = compute_dt_seconds();

    ChassisCommand command{};
    BigYawCommandMessage big_yaw_command{};
    link_get_latest_commands(
        command,
        big_yaw_command);

    read_all_local_motor_feedback();
    update_health(now_ms);

    if (!safe_to_control(now_ms)) {
        ramp_or_zero_all_effort();
        reset_all_control_states();
        send_all_zero_groups();
        publish_fault_state();
        continue;
    }

    chassis_controller_step(
        command,
        dt_s);

    big_yaw_local_controller_step(
        big_yaw_command,
        dt_s);

    send_each_dji_group_once();
    publish_chassis_snapshot();
}
```

### 20.2 云台板

```cpp
for (;;) {
    wait_gimbal_control_tick();

    const std::uint64_t now_ms = monotonic_ms();
    const float dt_s = compute_dt_seconds();

    update_imu();
    read_small_yaw_feedback();
    link_get_latest_chassis_state();
    update_operator_and_vision_target();

    if (!local_feedback_is_safe(now_ms)) {
        zero_small_yaw_effort();
        send_disabled_commands();
        reset_yaw_states();
        continue;
    }

    YawCoordinatorOutput coordinated{};
    yaw_coordinator_step(
        yaw_config,
        build_yaw_input(),
        coordinated);

    small_yaw_world_controller_step(
        coordinated.small_yaw_world_target_rad,
        coordinated.small_yaw_rate_ff_rad_s,
        dt_s);

    publish_big_yaw_and_chassis_commands(
        coordinated);
    send_small_yaw_motor_group();
}
```

## 21. 状态机

底盘板建议：

```text
Boot
  │ 所有设备 ready
  ▼
WaitFeedback
  │ 八电机反馈有效
  ▼
Aligning
  │ 舵向对齐，驱动轮保持 0
  ▼
Ready
  │ 收到新的 enable token
  ▼
Active
  │
  ├─ command stale ──> Stopping ──> Ready
  ├─ 单模块故障 ────> Degraded 或 Fault
  └─ 严重故障 ──────> Fault
```

第一版不要做“三轮降级继续跑”。任一关键舵向反馈丢失，就让四个驱动轮停下；确认基础安全后再研究降级运动学。

Yaw 建议状态：

```text
Disabled
  ▼
Homing/ZeroCheck
  ▼
Independent
  ▼
FollowAndRecenter
  │
  ├─ link stale ─────> Independent
  ├─ small soft limit -> RecenterPriority
  └─ hard limit/fault -> Disabled
```

## 22. 文件级实施顺序

### 阶段 A：把底层前提补齐

1. 按多型号电机手册修 `motor.hpp` 的函数指针检查。
2. 完成 M2006 驱动。
3. 完成 GM6020 电流模式驱动和 `0x1FE`/`0x2FE` 分组发送。
4. 完成大、小 yaw 实际使用的电机驱动。
5. 改造通用 TxGroup，不能硬编码 M3508 namespace。
6. 按 PID 手册补 reset、dt 和抗饱和。

自检：每种电机单独 sample 能收反馈、发全 0、发很小安全命令并可靠清零。

### 阶段 B：纯数学

1. 新建 `chassis_types.hpp`。
2. 写 `wrap_pi()`。
3. 写单模块速度向量。
4. 写四模块逆解算。
5. 写共同轮速缩放。
6. 写 90°优化。
7. 写零速保持。
8. 写正运动学。

自检：不用 Zephyr device，不接 CAN，测试全部通过。

### 阶段 C：单模块

1. 只连接一组 M3508 + GM6020。
2. 只让 GM6020 舵向到几个小角度。
3. 加位置—速度—电流串级。
4. 舵向误差大时保持 M3508 电流为 0。
5. 舵向对齐后才给 M3508 很小速度。
6. 验证 90°优化真的会反转轮速。

### 阶段 D：四模块

1. 校准四个舵向零点。
2. 填四个 module geometry 和方向。
3. 只测纯 `+vx`。
4. 测纯 `+vy`。
5. 测纯 `+wz`。
6. 测组合命令。
7. 加轮速整体缩放。
8. 加正运动学和状态发布。

### 阶段 E：双板协议

1. 先只发 heartbeat 和 sequence。
2. 做断线 timeout。
3. 发底盘三轴命令，但底盘电机仍强制 0。
4. 回传底盘状态。
5. 做错误 CRC、错误 version、乱序和重启测试。
6. 最后才允许 enable token 打开电机。

### 阶段 F：大小 yaw

1. 大 yaw 单独本地闭环。
2. 小 yaw 单独世界角闭环。
3. 校准 `theta_b/theta_s` 正方向与中心。
4. Independent 模式联调。
5. 只加中心卸载，不加速度前馈。
6. 再加 `required_relative_rate` 的低频分配。
7. 测底盘静止。
8. 测底盘慢速旋转。
9. 测通信断开。
10. 最后才测快速跟踪和视觉目标。

## 23. 纯算法测试向量

### 23.1 逆运动学

设四模块：

```text
FR = (+L, -W)
FL = (+L, +W)
RL = (-L, +W)
RR = (-L, -W)
```

#### 纯前进

输入：

```text
vx=1, vy=0, wz=0
```

预期：

- 四轮速度绝对值相同。
- 四轮角度为 0。
- 安装方向只影响最终电机速度符号，不影响模块物理目标。

#### 纯左移

输入：

```text
vx=0, vy=1, wz=0
```

预期：

- 四轮速度绝对值相同。
- 四轮角度为 `π/2`。

#### 纯逆时针旋转

输入：

```text
vx=0, vy=0, wz=1
```

预期模块向量：

```text
FR = (+W, +L)
FL = (-W, +L)
RL = (-W, -L)
RR = (+W, -L)
```

速度都为 `sqrt(L² + W²)`。

### 23.2 90°优化

- current=179°，target=-179°：只走约 2°，不反转速度。
- current=170°，target=-10°：舵向目标可保持在 170°附近，驱动速度反号。
- current=0°，target=90°：明确边界策略，不能一会翻一会不翻；建议加少量 hysteresis。

### 23.3 零速

上一拍目标 45°，下一拍 `vx=vy=wz=0`：

- 轮速为 0。
- 舵角仍是 45°。
- XLock 模式才改变角度。

### 23.4 正解闭环

随机生成多组 `vx/vy/wz`：

1. 逆解算出四模块目标。
2. 不加噪声地送入正解算。
3. 恢复值应在 float 容差内等于输入。

## 24. 大小 yaw 测试向量

### 24.1 静止居中

- `theta_s = center`。
- `omega_g_ref = 0`。
- `omega_c = 0`。

预期大 yaw rate 为 0。

### 24.2 小 yaw 偏正

- `theta_s` 超过正 deadband。
- 其他速度为 0。

预期大 yaw向能减小小 yaw偏差的方向缓慢运动。若偏差变大，立即停机并修正方向符号。

### 24.3 底盘旋转、云台世界角保持

- `omega_g_ref = 0`。
- `omega_c > 0`。

预期大小 yaw 的合成相对速度趋向 `-omega_c`，最终 `psi_g` 保持。

### 24.4 通信断开

拔掉板间链路：

- 大 yaw目标在规定时间内斜坡归零。
- 底盘速度归零。
- 小 yaw不再使用过期的大 yaw 状态。
- fault bits 和 stale 状态能被日志看到。

### 24.5 小 yaw 限位

逐渐把小 yaw推向 soft limit：

- 大 yaw回中权重提高。
- 不出现目标跳变。

到 hard limit：

- 禁止继续向外。
- 上报 fault。
- 不靠 PID 饱和硬顶机构。

## 25. 硬件上电顺序

1. 四个驱动轮全部架空。
2. 机械旁准备物理急停/断电。
3. 两块板先只跑通信和反馈，不发非零电流。
4. 核对每个 CAN ID，没有重复实例和槽位。
5. 核对 GM6020 实际发的是 `0x1FE`/`0x2FE`。
6. 用手转每个舵，确认角度和速度方向。
7. 给每个舵很小电流，逐一确认正方向。
8. 完成零点标定。
9. 单舵位置环，不开驱动轮。
10. 单模块对齐后给极小驱动速度。
11. 四模块只做纯前进。
12. 再做横移和旋转。
13. 板间命令先在 Disabled 下观察。
14. 大小 yaw 先 Independent。
15. 最后开启 FollowAndRecenter。

出现下列任一情况立即清零并断电：

- 舵向误差变大而非变小。
- 电机反馈时间戳停止更新。
- 电流持续饱和。
- 机构撞软/硬限位。
- CAN error counter 快速上升。
- 双板 sequence 停止或倒退。
- 小 yaw回中时大 yaw反向扩大偏差。

## 26. 建议构建与测试命令

下面命令由你在完成对应文件后运行；本文没有替你执行：

```bash
west build -p always \
  -b dm_mc02/stm32h723xx \
  application/chassis \
  -d build/chassis

west build -p always \
  -b dm_mc02/stm32h723xx \
  application/gimbal \
  -d build/gimbal
```

若为纯算法建立 Zephyr ztest：

```bash
west twister \
  -T tests/swerve_kinematics \
  -p native_sim

west twister \
  -T tests/yaw_coordinator \
  -p native_sim

west twister \
  -T tests/board_link_codec \
  -p native_sim
```

实际 board qualifier 以你当前 workspace 中能成功构建 M3508 sample 的写法为准。

## 27. 第一版完成标准

### 电机与总线

- [ ] M3508、GM6020 和 yaw 电机都能独立收反馈。
- [ ] GM6020 默认使用 current mode。
- [ ] ID 1～4 使用 `0x1FE`，ID 5～7 使用 `0x2FE`。
- [ ] GM6020 raw `±16384` 与约 `±3 A` 的换算独立于 M3508。
- [ ] 每个分组每控制拍只发一次。
- [ ] CAN 总线负载留有余量。

### 舵轮数学

- [ ] 全部使用 x前、y左、z上和 SI 单位。
- [ ] 模块几何通过数组配置，不手写四套公式。
- [ ] 有共同轮速缩放。
- [ ] 有 90°优化。
- [ ] 零速保持上一拍舵角。
- [ ] 正解和逆解能互相验证。

### 模块控制

- [ ] 四个舵向零点已实测记录。
- [ ] 每个电机方向已单独验证。
- [ ] 舵向使用位置—速度—电流串级。
- [ ] 舵未对齐时驱动轮不输出。
- [ ] 每个 PID 有独立 state。
- [ ] 反馈 stale 会 reset 并清零。

### 大小 yaw

- [ ] 坐标关系和安装符号已在实物上验证。
- [ ] 小 yaw可独立稳定世界角。
- [ ] 大 yaw可独立跟踪本地目标。
- [ ] 协调器只存在于云台板。
- [ ] 中心 deadband、soft limit、hard limit 已设置。
- [ ] 大 yaw低频卸载不会与小 yaw打架。
- [ ] 底盘旋转时世界 yaw保持测试通过。

### 双板

- [ ] 没有裸 memcpy packed float struct。
- [ ] 协议有 version、sequence、timestamp、valid 和 fault。
- [ ] UART 能处理粘包/拆包，或 CAN 分帧有完整性规则。
- [ ] 重启后旧 enable token 不能重新启动电机。
- [ ] 断线后底盘、大 yaw和小 yaw均进入定义好的安全状态。

## 28. 你现在真正应该先写什么

按下面五个小目标开工：

1. 完成 GM6020 电流模式驱动，确认 `0x1FE`、`±16384` 和全 0 停机。
2. 单独写 `swerve_inverse()`，只做纯数学测试。
3. 接一组 M3508 + GM6020，完成单模块对齐保护。
4. 将两块板的 heartbeat、sequence 和 timeout 跑通，暂时不允许 enable。
5. 让大 yaw、小 yaw分别独立闭环，再写最简单的 `center_term` 卸载。

在这五步全部可测之前，不要写“万能底盘云台总控制类”，也不要一次性调八个底盘电机和两个 yaw。

## 29. 资料

- 旧项目参考仓库：<https://gitee.com/esmorang/Roam-the-Heavens-and-Ride-the-Wind>
- 本地 GM6020 旧版手册：`motor_docs/GM6020_User_Guide.pdf`
- DJI GM6020 v1.4 中文手册：<https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E20231013.pdf>
- 电机驱动与 PID：`SkyWalker_多型号电机驱动古法实施手册.md`
