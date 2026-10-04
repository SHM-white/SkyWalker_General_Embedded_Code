# MotorSession 首批实现与接入说明

2026-10-05。用户已对本次实施禁用古法编程模式，本文对应已写入源码的第一批实现。

## 已迁移范围

- `include/control/motor_session.hpp`、`lib/control/motor_session.cpp`：静态成员绑定、初始化、准备参考、复位、使能、运行、停止与故障诊断；不创建新线程，不动态分配内存。
- `samples/robotics/swerve/src/dual_speed.cpp`：应用只准备遥控输入与两路目标，再调用 `session.step()`。保留 GM6020 ID3 / CAN1、M3508 ID3 / CAN2、原 PID 参数及限制。
- `samples/motor/dji_speed_control/src/main.cpp`、`samples/motor/dji_position_control/src/main.cpp`：单电机使用同一生命周期；保留原硬件配置、目标波形、VOFA 通道与一次性自动启动行为。两个示例的电机 ID、CAN 口仍按各自源文件配置，并未统一成双电机台架接线。
- `samples/robotics/common/rc_controls.hpp`：合法拨杆启动动作新增递增 `start_event_id`；原手势与其他使用方行为保留。
- 位置/速度控制器新增 `preflight()` 和带原因的遥测。普通 `configure()` 仍保留原错误停机行为；Session 使用 `ReportOnly`，在任一控制器失败时统一停止整组。底层权限、代际、超时与协议保护继续独立工作。

这版适用于一个 Session 独占一条或多条 CAN 总线；同一 Session 的多台电机可以在同一 CAN 上。初始化拒绝已有外部组成员或已启动的总线。配置失败应修改配置并重新启动应用，不能对已经部分初始化的对象重试 `configure()`。

共享总线多组并发、四舵轮、云台、发射器与整车执行器尚未迁移。它们需要方案中的分组发布接口，不能直接将多个 Session 放在同一 `CanBus` 上。

## 接入方式

```cpp
static control::MotorSession::Member members[] = {
    {"steer", &steer, &can1, &position_axis,
        control::ReferencePolicy::CaptureOnExplicitStart},
    {"drive", &drive, &can2, &velocity_axis,
        control::ReferencePolicy::NotRequired},
};
static control::MotorSession session(members);
// 不再另外创建 Group、attach/start 总线、configure/reset 控制器或 enable 电机。
int error = session.configure();
```

`step(input, now_us)` 由一个应用线程周期调用；它不等待反馈或 TX。对象与成员必须活到后台 I/O 线程结束，示例采用静态存储期。`status()` 通过短锁拷贝，可跨线程读取。目标数组只在本次 `step()` 内使用，允许局部数组。

`Input` 包含：

| 字段 | 含义 |
| --- | --- |
| `enabled` | 允许运行；false 撤销整组输出 |
| `start_sequence` | 每次新的明确启动动作递增；保持同一值不会在故障后自动重试 |
| `acknowledge_sequence` | 每次明确确认故障递增；只在停止且输入源新鲜时执行 |
| `source` | 输入源原始时间戳与序号，单位微秒；读取缓存不能修改其接收时间 |
| `target_stamp` | 本次目标实际生成时间与序号，单位微秒 |
| `targets` | 与 members 顺序一致；速度轴为 rad/s，位置轴为 rad |

Session 的四种状态是 Stopped、Starting、Running、Blocked。Starting 内部依次等待稳定反馈、准备参考并复位、等待准备完成之后的新源输入与新目标、发起使能。默认整个准备过程最多 3 秒，使能等待最多 1 秒；各电机原有更短的独立驱动超时仍生效。Running 默认控制周期上限 20 ms，源输入和目标新鲜度上限均为 100 ms；控制器原有 dt 范围也继续生效。

源、目标与 `now_us` 必须使用同一个单调时钟。序号用于启动/确认事件的单次消费；源和目标的新鲜度按原始时间戳判断。Session 不提供原始通信包排序，应由接收器处理乱序。启动前或故障时提交的启动序号会被消费；需要再次启动时必须发出新事件。

所有控制器计算成功后才发布各总线的目标。某个成员失败时，先停止整组，不发布本轮其余目标。各总线 `commit()` 只确认发布，不能保证跨总线原子到达；后台工作线程继续检查代际与权限。上一轮诊断用的逐帧阻塞等待已从双电机示例移除。

## 参考策略

| 策略 | 用途与行为 |
| --- | --- |
| NotRequired | 速度轴不需要位置参考 |
| CaptureOnExplicitStart | 仅用于 StartupRelative 位置轴；明确启动时重新播种相对零点，随后 reset 捕获中心 |
| UseCalibratedAbsolute | 仅用于 AbsoluteNearest；利用驱动按已有标定得到的绝对角重新播种连续坐标，不自行标定编码器零点 |
| RequireExternalReference | Session 不播种参考；外部必须先完成参考建立，缺失时阻止启动 |

双电机台架每次明确启动都会重新捕获当前位置为摆动中心；CAN 恢复本身不会建立新的相对零点或触发启动。

## 诊断与操作

`current_blocker` 是此刻的阻碍；`last_stop` 是上一次异常停机首因，重新启动后仍保留。`stop_event` 仅在新的异常停机时递增，正常拨杆停止不覆盖历史故障。

`can_start` 表示硬件/测量条件和输入新鲜度允许尝试启动，不代表已经收到新的启动手势。准备、使能和正常运行时该值为 false。Blocked 中若条件恢复，`current_blocker` 可清空且 `can_start=true`，此时仍需要新的 Start。

状态或阻碍变化时打印 `SESSION state=... blocker=... member=... bus=... op=... err=... next=...`。异常停机打印 `SESSION STOP`，例如 `reason=can_bus_off`、`temperature_limit`、`speed_limit`、`position_reference_lost`、`invalid_control_period`，同时保留原始错误码。准备等待超时通过 `timeout=1` 标明，并保留当时阻碍原因。

传输故障关联到本次组故障的 CAN 恢复记录时，`last_stop.recovery` 保存恢复前控制器状态、TEC/REC、CAN 统计及最后 TX 的帧 ID、用途、序号、错误和时间。`SESSION CAN evidence` 打印关键字段。最后一帧不一定就是失败帧，要结合 `tx_valid`、`tx_err`、用途与时间判断；不能只凭帧 ID 推定因果。后续恢复和安全帧不会覆盖已保存的这次 Session 停机证据。

`stops[]` 提供每个成员当前的停止请求代际和进度，TX 完成、驱动确认均不表示机械上已经静止。双电机每秒摘要还会显示当前阻碍、历史停机和停止进度。

双电机原有操作：两拨杆 Down、摇杆居中保持 500 ms，然后左拨杆 Middle 启动；左 Down 停止。出现需要确认的驱动故障时，先两拨杆 Down 居中建立基线，再保持左 Down、右 Up 且居中 1000 ms 确认；最后回到启动手势。通讯故障恢复后也必须重新发出启动动作，不能保持 Middle 自动重启。

## 编译与现场边界

最终仅做编译检查，不运行测试、不烧录、不启动电机。本批构建目录分别为 `../build/swerve_session`、`../build/dji_speed_session`、`../build/dji_position_session`，板型为 `dm_mc02/stm32h723xx`。

```bash
/home/shm-white/skywalker_ws/.venv/bin/west build -b dm_mc02/stm32h723xx samples/robotics/swerve -d ../build/swerve_session
/home/shm-white/skywalker_ws/.venv/bin/west build -b dm_mc02/stm32h723xx samples/motor/dji_speed_control -d ../build/dji_speed_session
/home/shm-white/skywalker_ws/.venv/bin/west build -b dm_mc02/stm32h723xx samples/motor/dji_position_control -d ../build/dji_position_session
```

本次软件收敛不能证明原来启动约一秒后的 Bus-off 已消失。尚需使用新固件的串口记录确认现场行为，再继续共享总线和整车迁移。
