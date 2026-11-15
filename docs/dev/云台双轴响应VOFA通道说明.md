# 云台双轴响应 VOFA 通道说明

2026-11-15。适用于 `applications/sentry_gimbal`，布局号 **113**，固定 **32 个 float 通道 I0～I31**。原 schema 112 的 49 通道已从正式云台输出路径移除，包括启动首帧；底盘和其他默认布局调用者继续使用原格式。本轮只改诊断输出，不调整控制器、前馈增益、MPC 参数或机械限位。

## 连接与固件识别

- 使用云台板原诊断 UART，115200 baud、8N1、JustFloat；输出周期 20 ms，约 50 Hz，控制周期仍为 5 ms。
- 每帧 32×4+4=132 字节；名义线路占用约 57.3%，原 49 通道帧约为 86.8%。序列化和排队不阻塞控制线程，UART 仍由低优先级线程发送。
- **I0 必须恒为 113**。如果仍是 I0=1、I19=112，读到的是旧固件格式。VOFA 工程中原来的曲线和名称可能保留，需要删除 I32～I48，并按本表重新绑定 I0～I31。
- 本轮使用正式云台构建目录 `build/sentry-gimbal`，不要与 `build/sentry_gimbal` 或其他应用目录的旧镜像混淆。烧录对应 `build/sentry-gimbal/zephyr/zephyr.elf` 后重新连接串口。

全部源码及说明完成后使用一次编译检查：

```bash
/home/shm-white/skywalker_ws/.venv/bin/west build -d build/sentry-gimbal
```

编译结果以任务回复为准。本轮不运行测试、烧录或设备运动。

## 固定通道表

角度统一 rad，速度统一 rad/s。世界角正 Pitch 低头；电机关节量沿用编码器坐标，不能直接与世界角相减。Yaw 努力为 A，Pitch 努力为 N·m。

| 通道 | 建议名称 | 含义 |
|---|---|---|
| I0 | Schema | 恒为 113 |
| I1 | SeqLow | 诊断帧尝试序号低16位 |
| I2 | SeqHigh | 诊断帧尝试序号高16位 |
| I3 | TimeLow | 本控制周期 MCU 时间 ms 的低16位 |
| I4 | TimeHigh | 本控制周期 MCU 时间 ms 的高16位 |
| I5 | DropLow | 诊断队列/输出累计丢弃数低16位 |
| I6 | DropHigh | 诊断队列/输出累计丢弃数高16位 |
| I7 | StateBits | 本帧命令来源、模式、输出和参考有效性 |
| I8 | WindowError | 自上次成功入队以来最近一次非零错误；无错误为0 |
| I9 | YawWorldRef | 最终送入头部 Yaw 外环的世界角，包含分配器参考选择 |
| I10 | YawWorldMeasured | 头部 IMU 实测世界 Yaw |
| I11 | PitchWorldRef | 按本地报文年龄外推后的视觉 Pitch 世界目标 |
| I12 | PitchWorldMeasured | 头部 IMU 实测世界 Pitch |
| I13 | YawJointRef | 小 Yaw 轴实际位置参考 |
| I14 | YawJointMeasured | 小 Yaw 关节实测位置 |
| I15 | YawJointRefVelocity | 小 Yaw 本周期实际接受的参考速度 |
| I16 | YawVelocityRequest | 小 Yaw 位置校正加前馈、合成限幅后的速度请求 |
| I17 | YawVelocitySlew | 小 Yaw 内环经过参考斜率限制后的速度 |
| I18 | YawVelocityMeasured | 小 Yaw 电机反馈速度 |
| I19 | YawEffortA | 小 Yaw 控制器最终电流请求 |
| I20 | PitchJointRef | Pitch 轴实际位置参考 |
| I21 | PitchJointMeasured | Pitch 关节实测位置 |
| I22 | PitchJointRefVelocity | Pitch 本周期实际接受的参考速度 |
| I23 | PitchVelocityRequest | Pitch 位置校正加前馈、合成限幅后的速度请求 |
| I24 | PitchVelocitySlew | Pitch 内环经过参考斜率限制后的速度 |
| I25 | PitchVelocityMeasured | Pitch 电机反馈速度 |
| I26 | PitchEffortNm | Pitch 控制器最终力矩请求 |
| I27 | CarrierYawMeasured | 分配器实际使用的载体世界 Yaw 速度 |
| I28 | CarrierYawEstimated | 视觉适配器实际使用的载体 Yaw 速度估计 |
| I29 | BigYawSubmitted | 最近一次提交给板间端点的门控后大 Yaw 世界速度意图 |
| I30 | WindowMaxDtUs | 自上次成功入队以来最大控制间隔，us |
| I31 | WindowEventBits | 自上次成功入队以来所有控制周期事件的按位 OR |

序号、时间、丢弃数都按 `low + high × 65536` 还原。MCU 时间和视觉 PC 时间没有同步，不能仅按两个 CSV 的行号或秒数拼接来计算跨设备延迟。

测量失效、未实际输出或参考未接受时，对应曲线输出 NaN；静止且有效时才输出0。I9/I11 仅在视觉有效时输出，手动或保持时为 NaN。I15/I22 是传给位置控制层的参考速度，增益为0时也可能有效；它们不表示额外速度前馈一定非零。

I13/I20、速度请求和努力输出均来自本周期实际有效的轴输出。执行器漏周期时不把上周期遥测伪装成本周期执行。I14/I21 和 I18/I25 使用本控制周期取得的电机反馈快照；各传感器仍可能有各自采样时刻。

I29 按板间请求提交时刻更新，通常每10 ms更新一次，期间保持最近值；尚未提交时为 NaN，Disabled 请求为0。它是本机已提交意图，不能代表远端已经收到、执行或达到该速度。

## I7 当前状态位

| 位 | 含义 |
|---|---|
| bit0～3 | `ControlSource`：0=None、1=Remote、2=KeyboardMouse、3=Vision、4=Autonomous、5=LocalBehavior |
| bit4～6 | `GimbalMode`：0=Disabled、1=Hold、2=Rate、3=AbsoluteAngle |
| bit7 / bit8 | 本周期 Yaw / Pitch 输出有效 |
| bit9 / bit10 | 本周期应用请求 Yaw / Pitch Rate 参考桥接 |
| bit11 / bit12 | 本周期 Yaw / Pitch 参考速度实际接受 |
| bit13 | `vision_active` |
| bit14 | 头部姿态可用 |
| bit15 / bit16 | 小 Yaw / Pitch 位置反馈有效 |
| bit17 | 大 Yaw 通道满足本机发送准备条件 |

读取某一位可先把浮点数转整数，再做 `(value >> bit) & 1`。源/模式与参考接受状态分开看：视觉被选中不代表轴已经成功输出，恢复播种周期不接受前馈也是正常情况。

## I31 窗口事件位

每对位均为 Yaw 在前、Pitch 在后。

| 位 | 含义 |
|---|---|
| bit0 / bit1 | 合成速度请求被限幅 |
| bit2 / bit3 | 内环斜率后速度与合成请求不同，容差为 `1e-5×(1+abs(request))` |
| bit4 / bit5 | 位置 PID 输出饱和 |
| bit6 / bit7 | 速度调节器饱和，或控制器努力请求触及配置努力上限 |
| bit8 / bit9 | 轴机械限位或超前角保护投影 |
| bit10 / bit11 | 轴参考重建，或 Rate 参考建立周期没有积分 |
| bit12 / bit13 | 本周期该轴无有效输出，包括 Disabled、等待或漏周期 |
| bit14 / bit15 | 该轴控制错误或对应 CAN 总线提交错误 |
| bit16 | Yaw 分配器行程限制或外环速率被实时预算夹紧 |
| bit17 | 来源、视觉 Rate 会话或头部世界参考变化 |
| bit18 | 控制周期无效：首次、超过20 ms或执行器按 Cycle 返回 |
| bit19 | 载体姿态不可用，或已选视觉所需的头部姿态不可用 |
| bit20 | 分配器载体跟踪监督正在收缩预算 |
| bit21 | 大 Yaw 意图饱和 |

事件在每个5 ms控制周期累积，成功入队后才清零；被20 ms节流跳过或队列拒绝时保留。若发生丢包，下一成功入队帧可能覆盖更长窗口，I5/I6 和序号可见异常。

bit6/7 和 I19/I26 覆盖本机控制器层，不宣称拿到了驱动内部最终限幅或实际已发送努力；电机电流/力矩实测没有混入这两个请求通道。启动、关闭或正常恢复时 bit10～13 可能置位，应结合 I7 判断，不把所有非零事件都视为故障。

## 推荐先打开的曲线

1. 世界响应：Yaw I9/I10，Pitch I11/I12。
2. 关节位置：Yaw I13/I14，Pitch I20/I21。
3. 速度链：Yaw I15/I16/I17/I18，Pitch I22/I23/I24/I25。
4. 努力：Yaw I19、Pitch I26 分图，避免混淆 A 与 N·m。
5. 需要追踪双 Yaw 配合时再显示 I27/I28/I29；元数据保留采集，I7/I31 用位图解释。

若参考位置正常而 I17/I24 落后，先定位内环斜率；若 I17/I24 正常而 I18/I25 落后，关注速度内环和努力限制。若 I13/I20 自身跳变，先查参考恢复或生成过程。50 Hz瞬时曲线不能重建全部200 Hz尖峰，窗口事件只能证明发生过，不能给出每次事件的完整波形。

## 本轮 tmp 记录的只读摘要

按 `feedback_valid=plan_valid=control=1` 且 `phase_s>=3` 取段，执行误差为 `MPC-RX`，单位度；未把上位机记录当作 MCU 内部诊断。

| 记录 | 工况：全宽×全高、频率 | Yaw / Pitch 执行 RMSE |
|---|---|---|
| `response_1794665671532.csv`、`1794665673109.csv`、`1794665676239.csv` | 6°×3°、1 Hz | 约1.94～1.96° / 1.012° |
| `response_1794731793135.csv` | 15.2°×10.1°、0.7 Hz | 约3.764° / 2.310° |
| `response_1794731851542.csv`、`1794731855113.csv`、`1794731858135.csv` | 4.3°×2.8°、1.7 Hz | 约4.65～4.78° / 1.05° |

最新三份1.7 Hz文件起点同为 `t_s≈159.8554`，终点逐次延长，属于重叠记录；其 Yaw 平均 `MPC-RX` 约为 -3.81～-3.95°，需要用新 I9/I10、I13/I14 和 I27～I29 区分世界参考、关节执行与大小 Yaw 配合。它们不能当作三次独立重复，也不能与尺寸不同的0.7 Hz记录直接比较优劣。

这些文件没有 MCU 内部量和完整固件参数身份，不能单凭 RMSE 确定应调整哪一层。本轮没有据此修改 PID、前馈或 MPC 参数；下一次同步保存上位机 CSV 与 schema 113 的 VOFA 记录。
