# 06 IMU 独立源、滤波和温控

正式接口位于 include/drivers/imu/imu.hpp、imu_types.hpp。旧 imu.h/imu.c、skywalker,imu 聚合设备和 SKYWALKER_DRIVER_IMU 已移除，不提供兼容 wrapper。底层 Zephyr BMI08x sensor 驱动继续使用；姿态 EKF 不依赖通用线性 Kalman 或 Matrix 库；线性 Kalman 也已改为普通算法实例。

## 数据流和类型

~~~text
BMI088 sensor → Bmi088Imu → 可选 QuaternionEkf → ImuState → Snapshot
DM RS485 → AsyncUart → DmImuParser → 单位/安装变换 → ImuState → Snapshot
有效板载温度 → 独立 ImuHeater → PWM
~~~

各实例拥有自己的状态，任意源均不绑定 base/head、云台角色或应用。温控不属于 ImuSource；外置设备的内部恒温不由 MCU PWM 控制。

Snapshot 包含 Sample、State、capabilities、fresh_mask 和 Diagnostics：

| 数据 | 单位/语义 |
|---|---|
| accel_m_s2 | 传感器测得的加速度，包括静止时重力对应的比力；m/s² |
| gyro_rad_s | 输出坐标 B 内角速度；rad/s |
| orientation | Hamilton wxyz，q_WB 将 B 中向量转到局部参考 W |
| temperature_c | 摄氏温度，仅支持时声明 Temperature |
| reference | frame_id 和本地 epoch；参考重置后旧姿态失效 |
| attitude_quality | Unavailable、Unknown、Initializing、Tracking、Degraded |

capabilities 是实现支持能力；stamp.valid 表示最后一份测量通过校验；fresh_mask 是本次读取时仍未过期的字段。三者不能混用。Running 仅表示接收/采样已初始化。外置质量 Unknown 表示设备未报告 EKF 收敛状态，不等于无有效数值。

每字段有独立的 time_us 和本地 sequence。失败不改旧时间，读取快照不递增序号，收到 accel 不刷新 quat。Fault 时 fresh_mask=0。diagnostics.last_error 保留最近错误，计数用于排查，不因任意成功帧自动抹掉。

## 接口与线程

推荐使用 `include/drivers/imu/imu_receiver.hpp` 中的 ImuReceiver 管理后台采集：

~~~cpp
// 静态/长生命周期实例；source、estimator、heater 和 DMA 同样需要长期存活。
ImuReceiver receiver(source, {.poll_interval_us = 500, .priority = 5}, &heater);
// 外置源不需要 heater，省略第三个参数即可。
const int ret = receiver.start();

// 任意读线程，无 I/O：
const auto snapshot = receiver.snapshot();
if (snapshot.fresh_mask & skywalker::imu::Orientation) {
    const auto q = snapshot.sample.orientation.value;
}
const auto status = receiver.status();
~~~

每个 ImuReceiver 拥有一条采集线程，负责一次初始化、周期调用 source.service，以及可选的 heater.update。滤波仍在 Bmi088Imu 内部执行。start 返回 0 只表示线程已启动；初始化结果查看 status.init_complete、init_error 和 heater_init_error，数据是否可用查看 snapshot.fresh_mask。重复 start 返回 -EALREADY；目前不提供 stop/restart，运行期间不能销毁实例或依赖对象。

status 提供最近一次非 -EAGAIN 的 service_error，以及 heater_error、heater_disable_error 和 heater_duty；它与测量快照分别读取，不承诺跨接口原子一致。snapshot 直接委托 source 读取，不新增样本缓存；每次读取都会重新计算 freshness。初始化失败后线程仍调用 service，让 RS485 后端自行恢复，不重复注册回调。温控初始化失败不阻止采集；温控成功初始化后，即使本轮没有新测量也检查温度，源故障或温度失效时尝试关断 PWM。

配置中的 poll_interval_us 是每次 service 后的休眠时间，不保证精确采样周期；priority 为抢占式线程优先级。每个实例拥有 CONFIG_SKYWALKER_IMU_RX_STACK_SIZE 字节配置的栈（默认 8192），使用硬件浮点共享时声明 K_FP_REGS。

需要自行调度时仍可直接使用 ImuSource 的 init/service/snapshot。构造不做 I/O；init 不等于已经产生有效姿态。service 不等待下一个周期，但底层 SPI 传输本身可能阻塞。一个底层 sensor 或 UART 必须有唯一所有者；使用 ImuReceiver 后，应用不能再次调用该源的 init/service 或该 heater 的 init/update/disable。快照在短 spinlock 内复制，在锁外取当前时钟重新计算 freshness；锁内不读外设、滤波或解码。

ImuState 是具体发布核心，接收 Update.updated_mask 标识本次成功字段，校验后原子提交。一项非法则该 Update 整体拒绝；后端可把独立读取结果分次提交。倒退时间拒绝，初始单位四元数保持 invalid。

time_us 使用 MCU 同一单调时钟；微秒单位不保证微秒精度。当前 UART chunk 只有毫秒接收时间，不能把稍后的解码时刻当作传感器采样时刻。

## 板载 BMI088 与 EKF

Bmi088Imu 显式接收 accel/gyro device 指针、Config 和可选 QuaternionEkf 指针。没有估计器时只发布原始字段；启用估计器时还需开启 SKYWALKER_ATTITUDE_EKF。安装旋转 sensor_to_body 为 q_BS，算法在 S 内估计：

~~~text
accel_B = R(q_BS) accel_S
gyro_B = R(q_BS) gyro_S
q_WB = q_WS conjugate(q_BS)
~~~

加速度和角速度分别检查 fetch/get 返回值，读取失败不能用旧缓存冒充新样本。锁定版本的 BMI08x DIE_TEMP 会直接读取温度寄存器，因此温度可独立读取并标记时间；这里与“所有 channel_get 只读缓存”的泛化假设不同。

QuaternionEkf 用固定数组持有四元数、4×4 协方差、低通和静止零偏状态。初始化要求连续 100 次近静止采样，以重力估计 roll/pitch、局部 yaw=0；长期 yaw 没有绝对观测。重力模长/新息异常时保留 Degraded 的 gyro 预测；协方差用 Joseph 形式更新。长采样间隔、数值异常重新初始化，Bmi088Imu 随 generation 变化递增 reference.epoch 并使旧姿态失效。

默认按 1250µs 限速轮询，温度周期 10ms；实际速率取决于线程调度和 SPI。分别读取的 accel/gyro 不宣称硬件同步，也不宣称有 DRDY 去重。两次成功输入的时间差不得超过 max_input_skew_us。

## 温控

MC02 与 C 板默认 DTS 均提供 accel0、gyro0、imu-heater alias。样例分别通过 DEVICE_DT_GET(DT_ALIAS(accel0/gyro0)) 获取传感器，通过 PWM_DT_SPEC_GET(DT_ALIAS(imu_heater)) 获取温控 PWM；温控节点不再由样例 overlay 重复声明。MC02 使用 TIM3 CH4/PB1，C 板使用 TIM10 CH1/PF6，周期均为 20ms。目标温度和 PID 仍由应用配置。

ImuHeater 显式传入 pwm_dt_spec 与温控配置。init 写零占空比，update 才开始输出；按 duty=0—1 的 PID 控制，不复用旧 API 中以纳秒为 PID 输出的参数。

双 IMU 样例使用 TIM3 CH4/PB1、周期 20ms、目标 50℃，测量达到 65℃时撤销输出。温度过期、非法、时间倒退或 PID/PWM 错误均尝试关 PWM，关闭失败保存在 disable_error。温控失败不会阻止 gyro 读取；ImuReceiver 会持续调用 update；自行调度时需由所有者线程完成，软件不是独立硬件热保护。

## DM-IMU-L1 RS485

DmImuRs485Source 只接收主动模式，不自动配置、归零、校准或加热。先用厂家上位机设置并保存，实物重上电确认。

- 固定头 55 AA，ID，type 01/02/03/04；三轴帧 19 字节，四元数帧 23 字节，float 小端，尾 0A。
- 四类数据全开共 80 字节，但按四个独立帧解析。
- CRC 初值 FFFF、0x1021 表、左移 1 位更新；从帧头覆盖到数据末尾，CRC 低字节先发。19 字节算法已有手册实帧依据。
- Euler 仅校验/消耗，不覆盖权威四元数。首版没有温度能力。
- UART overflow 丢弃半包，保留原测量时间；部分初始化失败保留对象，由 service 重试。对象、DMA 缓冲和设备必须覆盖回调寿命。
- Config 可指定 ID、安装旋转、单位缩放和设备四元数方向。默认 SI 缩放 1、sensor→world，实物还须确认。
- resetReference() 只在源所有者线程调用，用于已知设备归零/重启；它不会发送任何命令，也不能自动发现全部远端静默重启。

CAN 只保留 DmImuCanSource 抽象接口与 Config，未实现接收、映射或寄存器事务，不提供 CAN 开关；当前可用外置实现是主动流式 RS485。

## 配置与样例

| 配置 | 作用 |
|---|---|
| SKYWALKER_IMU | CPP 接口与具体状态核心，不强制总线/PWM |
| SKYWALKER_IMU_RECEIVER | 每实例后台采集线程，可选驱动 heater；不强制 PWM |
| SKYWALKER_IMU_BMI088 | SENSOR/BMI08X 采样 |
| SKYWALKER_ATTITUDE_EKF | 实例化算法，固定数组，无 device/DSP 注册依赖 |
| SKYWALKER_IMU_HEATER | 可选 PWM 和控制算法 |
| SKYWALKER_IMU_DM_PROTOCOL | 主动串行流式解码 |
| SKYWALKER_IMU_DM_RS485 | 协议加 UART_TRANSPORT 后端 |

[双 IMU 样例](../../../samples/imu/dual_imu/README.md) 同时启用板载采样/滤波/温控和 485-2 外置模块，并给出 VOFA 通道。samples/imu_test 作为单板载入口已迁移新 API，输出 roll/pitch/yaw、温度、duty、fresh_mask 六通道。算法/协议/过期及模拟 PWM 的运行检查见 [主流程验收](../../../tests/vision_imu/README.md)。

没有进行实物温控、SPI、RS485 或姿态方向验收。目标固件 23 字节四元数、单位、方向和实际可持续频率应上板核对。
\n\n更多封装对象的初始化和周期调用见[封装模块调用示例](../../../call-examples.md)。\n