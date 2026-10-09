# 板载与 RS485 外置 IMU 同时读取

两个 ImuReceiver 实例分别管理 Bmi088Imu 和 DmImuRs485Source 的后台采集。样例调用 start() 启动，然后用 snapshot() 读取两套测量，只显式定义一条 VOFA 输出线程。板载侧启用实例化四元数 EKF、加速度低通、静止零偏估计和独立 PWM 温控；外置侧直接采用设备输出四元数，不重复滤波，也不融合两套姿态。

## 接口与调度

样例中的 onboard/external 是 ImuReceiver，onboard_source/external_source 是底层数据源。采集线程、初始化、轮询和温控推进都由 ImuReceiver 管理，snapshot 本身只读取，不触发采集。

板载接收器每轮休眠 500µs、优先级 5，外置接收器每轮休眠 1ms、优先级 6；配置位于 src/board_config.hpp。开启 CONFIG_SKYWALKER_IMU_RECEIVER，每个接收器默认栈为 8192 字节。板载源仍保留自身采样限速，两个实例独立调度。温控作为板载接收器的可选依赖，VOFA 从 onboard.status().heater_duty 读取占空比。

start 返回值仅表示线程启动结果；初始化结果和温控诊断用 status() 查看，测量是否有效用 snapshot().fresh_mask 判断。实例和依赖均为静态存储；运行期间不能销毁，应用不要再直接调用 source.init/service 或 heater.init/update/disable。

## 硬件与参数

| 功能 | 样例配置 |
|---|---|
| 板载 IMU | MC02 板载 BMI088，SPI2 上 bmi08x_accel / bmi08x_gyro |
| 加热 | TIM3 CH4 / PB1，PWM 周期 20ms，目标 50℃、测量上限 65℃ |
| 外置 IMU | DM-IMU-L1，DT_ALIAS(rs485_2) → 重映射 USART2，PD4 DE、PD5 TX、PD6 RX |
| 外置串口 | 2 Mbit/s、8N1，固件 2.0.3.0 请求/应答协议、从机 ID=1 |
| VOFA | 主控 USB CDC，JustFloat；样例 overlay 覆盖 telemetry-uart alias |
| 日志 | USART10，保持板级 console 设置 |

USART2 的 RX DMA 使用 DMAMUX1 channel 5/request 43，TX 使用 channel 7/request 44，与板载 SPI2、USART1、UART5 的通道分开。驱动只发送数据读取请求，不发送配置、归零或校准指令。串口 alias 已在板 DTS 定义，样例直接使用 rs485-2，不把板级另一个名为 usart2 的 alias（实际指向 USART10）误当成外置 IMU 口。

新固件 RS485 不支持主动上报。驱动使用固定 24 字节 A5/5A 帧，按 RID 3（四元数）、1（角速度）、0（加速度）依次轮询，每次只等待一个请求；正常应答后间隔至少 1ms 再发送下一请求。应答超时默认 50ms，最多重试两次，随后继续下一类数据。USB/旧主动串行协议的 55 AA 解析器单独保留，不用于当前 RS485 源。应答与请求没有事务序号，不能完全区分同一 RID 的迟到应答；RS485 收发器应在发送期间抑制本地回显，不能把全零请求当作测量回复。

## 上电与运行

1. 先用厂家上位机确认外置模块版本和保存参数：本驱动对应新手册 V1.3 的固件 2.0.3.0，RS485 接口、从机 ID 1、2 Mbit/s；保存，断电重上电确认。若实物保存了其他波特率，需同步修改 app.overlay。上位机 USB 默认的 921600 不能当作 RS485 速率。主控 init 不执行归零、校准、切换接口或写寄存器。
2. 断电接线：外置 IMU 接板卡 485-2 的 A/B、供电与公共参考地。手册供电范围 5—28V，按板卡接口实际供电核对。不要把 RS485 差分线接成 TTL UART。此样例不需要电机。
3. 编译烧录后，板子先静置。板载 EKF 需要连续 100 次近静止样本初始化；运动时继续等待，orientation 不会假装有效。板载热控默认开始调到 50℃。外置模块温控由厂家软件独立配置；若开启，新固件普通测量输出要等待目标温度 ±0.5℃ 内连续稳定 10 秒，期间不能保证已有有效测量。
4. VOFA 观察温度、加热 duty 和 freshness；温度有效期 200ms，过期、非有限值或达到 65℃时撤销 PWM。PWM 撤销失败会保留错误诊断；软件检查不能代替硬件实测。
5. 缓慢分别绕 X/Y/Z 正向转动，确认安装方向、角速度和四元数方向。拔掉外置模块后，外置字段在 20ms 后无效，板载采样与温控继续工作。

~~~bash
west build -p always -b dm_mc02/stm32h723xx samples/imu/dual_imu -d build/dual_imu
west flash -d build/dual_imu
~~~

板载传感器与温控硬件从默认板级 DTS 的 accel0、gyro0、imu-heater alias 获取；样例 overlay 不再定义加热节点。串口参数集中在 app.overlay，三处安装变换与参考 ID 从 `include/robotics/vehicle/calibration.hpp` 读取；单位缩放、目标温度与接收器调度参数在 src/board_config.hpp。两块板的 IMU alias 已统一，但此双 IMU 样例的外置 RS485 与 DMA 仍绑定 MC02。开启 CONFIG_FPU 和 CONFIG_FPU_SHARING，避免用软浮点运行高频 EKF。

## VOFA 通道

100Hz、16 个 float，一帧 68 字节，通过主控 USB CDC 输出。

| 通道 | 含义/单位 |
|---|---|
| 0、1、2 | 板载 roll、pitch、yaw，rad |
| 3、4、5 | 板载 gyro X/Y/Z，rad/s |
| 6、7、8 | 外置 roll、pitch、yaw，rad |
| 9、10、11 | 外置 gyro X/Y/Z，rad/s |
| 12 | 板载温度，℃ |
| 13 | 加热 duty，0—1 |
| 14、15 | 板载/外置 fresh_mask：Accel=1、Gyro=2、Orientation=4、Temperature=8 |

没有有效姿态或字段已过期时，对应图表数据输出 NaN，不能把初始化单位四元数误认成真实姿态。板载全部字段新鲜时 mask=15，外置首版没有温度输出能力，全部基础字段新鲜时 mask=7。外置质量为 Unknown，表示模块没有公开内部 EKF 收敛标志。

默认云台板配置使用中央 `carrier_reference`（frame_id=2）及 `carrier_sensor_to_body`，对应大 Yaw 载体；头部外置使用 `head_reference`（frame_id=3）及 `head_sensor_to_body`，对应随 Pitch 运动的头部。底盘板配置使用 `chassis_reference`（frame_id=1）及 `chassis_sensor_to_body`，对应底盘车体。frame_id 标识安装位置，epoch 标识该位置的参考会话；三者 yaw 各自有初始化参考，不能据此认定零点相同。中央默认安装旋转是单位四元数，只表示传感器自身坐标。TODO(标定)：记录并更新三处安装变换、参考会话、正方向和原始数据年龄，验收后再将中央 `imu_mounting_confirmed` 设为 true。该确认开关不会阻止此观测样例采集，也不会授权电机输出。外置单位缩放默认为 1，姿态方向默认为 sensor→world，仍须用实物确认。

## 三个实际安装位置的观测

同一入口包含两种配置。底盘板只启动板载接收器和温控；云台板同时启动载体板载与头部外置 IMU。底盘配置的 VOFA 外置通道为 NaN，fresh_mask=0。

```sh
west build -p always -b dm_mc02/stm32h723xx samples/imu/dual_imu \
  -d build/imu_chassis -- -DEXTRA_CONF_FILE=chassis.conf
west build -p always -b dm_mc02/stm32h723xx samples/imu/dual_imu \
  -d build/imu_gimbal -- -DEXTRA_CONF_FILE=gimbal.conf
```

每 200 ms 的 console 观测行同时记录安装位置、`frame_id/epoch`、质量、fresh_mask、姿态与角速度各自的原始序号、生产时间与年龄，以及初始化/传输错误。无有效时间或未来时间时年龄为 UINT64_MAX。VOFA 失败不阻止 console 观测；16 个原 VOFA 通道的顺序保持不变。

断开电机动力，用人工缓慢转动各级：

| 操作 | 底盘板载 | 载体板载 | 头部外置 |
|---|---|---|---|
| 转整个底盘，关节相对位置固定 | 变化 | 变化 | 变化 |
| 底盘固定，只转大 Yaw | 不变 | yaw 变化 | yaw 变化 |
| 载体固定，只转小 Yaw | 不变 | 不变 | yaw 变化 |
| 小 Yaw 固定，只转 Pitch | 不变 | 不变 | pitch 变化 |

这里的 yaw/pitch 指安装变换标定后的机械轴；初始默认变换未标定时应先记录三轴实际符号，再校准。姿态质量为 Unknown 的外置设备需要实物确认，不能直接作为自动惯性使能依据。断开头部 IMU后只有头部字段过期；此样例不包含电机控制、大小 Yaw 回中或姿态融合。已知参考变化应由 source 的采集所有者更新 epoch，不要从观测线程调用归零函数。

每秒增加一行 `dm_imu_rs485` 诊断：`tx` 为提交成功的请求数，`rx_bytes` 为接收字节数，`replies` 为匹配的结构/数值有效应答数（含非零设备应答码），`timeouts` 为应答超时数，`invalid` 和 `partial_timeouts` 为解析异常，`response` 为最近匹配应答的原始设备应答码。没有接收字节时优先核对实物速率、地址、接线和温控；接收字节增长而 replies 不增长时核对帧格式。计数累计，错误诊断保留最近一次错误，不因后续成功自动清零。

字段分别盖接收时间，收到 accel 不会给旧 quat 续期。新 RS485 帧不含 CRC，不应套用 USB CRC。已知外置归零/校准后，拥有 source 的线程调用 resetReference() 更新本地 epoch 并让旧请求应答窗口过期；协议无法自动识别全部静默重启。当前协议适配仍需目标固件实机确认。

CAN 只在 include/drivers/imu/dm_imu_can.hpp 保留抽象接口，没有接收器、解码实现或启用开关。
