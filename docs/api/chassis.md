# 舵轮底盘与功率缩放：接口与调用参考

源码基线：`main@99a97c9`（2026-10-05）。本文维护为独立 Markdown，可在编辑器或 GitHub 直接阅读；在线浏览器读取同一正文。完整源码、硬件配置和未列出的接口以文末链接为准。

四轮运动学与八轴独立有效性；公开 SwerveHardware/ChassisExecutor 持续暂存，应用统一 commit。

**接入状态：已有代码。** 单舵轮最新提交记录基本功能调通；四轮/整车软件入口已有，真实几何与功率仍待标定。

## 职责与关联

SwerveKinematics 分目标，SwerveModule 独立转向/轮驱计算与翻转迟滞，SwerveHardware 映射八电机；功率执行使用可信预算与测量。

输入 / 依赖：[PID、前馈、斜坡与角度工具](pid.md)、[DJI 电机与共享 CAN 总线](motor-dji.md)、[双主控板间通信](interboard.md)

消费者：[双主控应用与执行器](application.md)

## 接口契约

### 1. int SwerveChassis::validate() const; int reset(const ChassisFeedback &); int step(const ChassisCommand &, const ChassisFeedback &, float dt_s, ChassisOutput &out)

```cpp
int SwerveChassis::validate() const; int reset(const ChassisFeedback &); int step(const ChassisCommand &, const ChassisFeedback &, float dt_s, ChassisOutput &out)
```

持续解算，不等待全轮反馈；ModuleFeedback.steer_valid/drive_valid 与各轴 generation 独立决定 effort 有效性。

| 参数 | 含义与边界 |
| --- | --- |
| `command` | vx/vy m/s、wz rad/s；+x 前、+y 左、+wz 逆时针 |
| `feedback` | 四模块 FL/FR/RL/RR；舵绝对角/速度，轮驱速度 |

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；本轴缺反馈以本轴 output_valid=false 表达。

### 2. int SwerveModule::step(const ModuleTarget &, const ModuleFeedback &, float dt_s, ModuleOutput &out)

```cpp
int SwerveModule::step(const ModuleTarget &, const ModuleFeedback &, float dt_s, ModuleOutput &out)
```

最短转向、翻转迟滞、转向斜坡、Hold/Coast；无对齐门控或 cos 缩速，舵向和轮驱分别恢复。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

### 3. int SwerveHardware::begin(); int read(ChassisFeedback &); int stage(const ChassisOutput &, float steer_scale, float drive_scale); void suspend()

```cpp
int SwerveHardware::begin(); int read(ChassisFeedback &); int stage(const ChassisOutput &, float steer_scale, float drive_scale); void suspend()
```

应用注入八台 Motor 和批量 Group；stage 独立携带计算 enable generation，不 commit 总线。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

### 4. ChassisExecutor(SwerveHardware &, const SwerveChassis::Config &, const Config &); int begin(); RunStatus update(const ChassisExecutionInputs &, core::TimeUs now_us)

```cpp
ChassisExecutor(SwerveHardware &, const SwerveChassis::Config &, const Config &); int begin(); RunStatus update(const ChassisExecutionInputs &, core::TimeUs now_us)
```

注入硬件，不绑定 Endpoint；消费原输入、权限、预算和真实功率。

| 参数 | 含义与边界 |
| --- | --- |
| `inputs` | command/source_stamp/permission/power_budget/measured_power 与运行/急停条件 |

**返回 / 输出：** RunStatus 与输出/测量观察入口，电机等待不撤销其他轴目标。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。

### 5. int ChassisPowerLimiter::reset(); int step(const ChassisPowerInput &, float dt_s, ChassisPowerDecision &out)

```cpp
int ChassisPowerLimiter::reset(); int step(const ChassisPowerInput &, float dt_s, ChassisPowerDecision &out)
```

标定后以 measured_power_w、预算 W、缓冲 J 计算 [0,1] 缩放；预算不可冒充测量。

**返回 / 输出：** 0 为接受/配置成功；实际状态单独观测。

**线程 / 时序：** 单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。

**错误 / 边界：** 数值/周期非法 -EINVAL；模型未标定或测量过期由执行器阻断对应功率模式。

## 调用示例

### 四轮与大 Yaw 分别推进

```cpp
const auto wheels = chassis.update(chassis_inputs, now_us);
const auto yaw = big_yaw.update(yaw_inputs, now_us);
const auto steer_tx = steer_bus.commit();
const auto wheel_tx = wheel_bus.commit();
const auto yaw_tx = yaw_bus.commit();
// 每条物理 CAN 一个 owner；一个失败不漏提交另外两条。
```

应用周期片段，构造与实物配置以链接源码为准。

## 调用顺序

1. 核对 FL/FR/RL/RR、方向、单圈零位、半径、减速比。
2. 应用 attach/start 各总线，hardware/executor.begin 校验。
3. 持续解算各轴目标；只用有效测量计算对应 effort。
4. 功率模式保留预算与测量时效；未标定台架使用独立电流上限。
5. 统一提交每物理 CAN；输入停止才明确批量撤销。

## 配置与使用边界

| 配置项 | 作用与前提 |
| --- | --- |
| `SwerveModule::Config` | wheel_radius_m、两套控制环、steer_target_rate、翻转进入/退出阈值、IdleBehavior。 |
| `ChassisExecutor::Config` | 真实功率/预算年龄、模型确认、estimated 回退、舵/驱独立台架缩放与电流上限。 |

- ModuleFeedback 没有旧 steer_continuous_rad，舵向绝对角局部展开。
- 单轮经验速度比例/等效半径不是整车实测几何。
- mode=Disabled 的运动学目标不替代 Motor.disable。
- 电机独立恢复，Group 不形成故障传播。

## 正文与源码

- [executors.md](../modules/robotics/executors.md)
- [README.md](../../samples/robotics/swerve/README.md)
- [README.md](../../samples/robotics/four_swerve/README.md)

- [swerve_types.hpp](../../include/robotics/swerve/swerve_types.hpp)
- [swerve_module.hpp](../../include/robotics/swerve/swerve_module.hpp)
- [swerve_chassis.hpp](../../include/robotics/swerve/swerve_chassis.hpp)
- [swerve_hardware.hpp](../../include/robotics/chassis/swerve_hardware.hpp)
- [chassis_executor.hpp](../../include/robotics/chassis/chassis_executor.hpp)
- [chassis_executor.cpp](../../lib/robotics/chassis_executor.cpp)

[全部接口参考](README.md) · [文档同步清单](../maintenance.md)
