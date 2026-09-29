# 18 视觉独立模块与 AB 协议

视觉模块由 VisionProtocol、VisionLink、VisionReceiver 组成，位于 include/communication/vision 和 lib/communication。公共数据只依赖 core 值类型；不持有 IMU、不包含机器人命令、不执行电机或射击。

## 使用

~~~cpp
static communication::AsyncUart::DmaBuffers dma __nocache;
communication::vision::AbProtocol protocol({.command_reference={1,1}});
communication::vision::VisionReceiver receiver(uart, dma, protocol, {});
const int ret = receiver.start();
const auto snapshot = receiver.snapshot();
if (snapshot.link.aim_fresh) {
    const auto command = snapshot.link.aim.value;
    // 交给未来调用者；本模块只表达 control_requested/fire_requested。
}
~~~

对象、protocol 和 DMA 必须在回调存在期间持续存活。start 返回 0 表示线程创建成功，实际 UART 初始化由 snapshot.state/uart_error 报告。start 只允许一次，无停止/析构后重新注册能力。

VisionLink 可脱离 UART 单独使用：init 后，单一所有者调用 processRxBytes/discardPartial/encodeFeedback；其他线程可以 setFeedback 和 snapshot。构造不做 I/O。零长度输入周期推进半包超时；快照读取时按当前时钟判断已接受命令是否过期，即使接收线程卡住仍会过期。

命令 sequence 是本地接受计数，不是远端采样序号。接收块 timestamp_ms 换算成 time_us 后仍只有毫秒粒度。mode=0 是合法停止消息，会清空目标并覆盖旧 active；停止来包只使 aim_fresh=false，消费者不能只看遗留的 control_requested。

## 已确认的线协议

2026-09-30 按用户指定的 [gimbal.hpp](https://github.com/SHM-white/RM2026-AutoAim/blob/main/io/gimbal/gimbal.hpp)、[gimbal.cpp](https://github.com/SHM-white/RM2026-AutoAim/blob/main/io/gimbal/gimbal.cpp)、[CRC 实现](https://github.com/SHM-white/RM2026-AutoAim/blob/main/tools/crc.cpp) 核对。这里的 AB 是用户本次选定的真实协议，不是旧 MCU API 兼容层。无导航、裁判混包功能。

通用规则：115200 baud、8N1；float 为小端 IEEE 754 float32；整数小端；CRC 初值 FFFF，右移反射多项式 8408，无末尾异或，覆盖从 AB 开始到 CRC 前的所有字节，CRC 低字节先发。该 CRC 与 DM-IMU 的左移 1 位算法完全不同；视觉实现不链接 referee.cpp。

下行 VisionToGimbal 固定 29 字节：

| 偏移 | 类型 | 含义 |
|---|---|---|
| 0—1 | byte[2] | 41 42，即 AB |
| 2 | uint8 | 0 不控制；1 控制但不请求开火；2 控制并请求开火 |
| 3、7、11 | float32 | yaw、yaw_vel、yaw_acc |
| 15、19、23 | float32 | pitch、pitch_vel、pitch_acc |
| 27—28 | uint16 | 前 27 字节的 CRC |

角度 rad、角速度 rad/s、角加速度 rad/s²；yaw/pitch 按 ZYX Euler 的绝对角约定解释。active 命令拒绝非有限值；未知 mode 拒绝。示例 mode=2、yaw=0.25、yaw_vel=0.5、yaw_acc=1、pitch=-0.2、pitch_vel=-0.3、pitch_acc=0.25：

~~~text
41 42 02 00 00 80 3E 00 00 00 3F 00 00 80 3F CD CC 4C BE 9A 99 99 BE 00 00 80 3E 39 77
~~~

上行 GimbalToVision 固定 43 字节：

| 偏移 | 类型 | 含义 |
|---|---|---|
| 0—1 | byte[2] | AB |
| 2 | uint8 | 0 Idle；1 AutoAim；2 SmallBuff；3 BigBuff |
| 3、7、11、15 | float32 | 四元数 wxyz |
| 19、23 | float32 | yaw、yaw_vel |
| 27、31 | float32 | pitch、pitch_vel |
| 35 | float32 | bullet_speed，m/s |
| 39—40 | uint16 | bullet_count，累计次数，按 16 位值传输 |
| 41—42 | uint16 | 前 41 字节 CRC |

Feedback 以有效四元数、body-frame gyro 和带独立时间戳的弹速/计数输入。yaw/pitch 由四元数推导；gyro 转成 ZYX Euler 角速度，不把 gyro.z/gyro.y 在任意姿态下直接当 yaw_vel/pitch_vel。pitch 接近 ±π/2 时返回 -ERANGE。

上位机当前读线程实际用收到的 yaw/pitch 重建 roll=0 的四元数，直接使用 q 字段的代码被注释；因此不能以“上行含 q”推断它已使用完整三轴姿态。MCU 编码保持 q 和 Euler 来源一致。

## 有效性、参考与反馈发送

AB 帧没有 valid bits、frame_id、epoch 或会话序号。公开接口保留这些字段，不能伪装它们存在于线上：

- AbProtocol::Config.command_reference 是双方约定的固定本地参考。上行 Feedback.reference 必须与该配置相同，否则 -ESTALE；远端参考改变需要显式协调新配置，当前接收器没有热切换会话 API。
- setFeedback 接收值副本，保留各字段原始测量时间，不在复制时刷新有效期。
- encodeFeedback 检查姿态/角速度时间差和各字段有效期；AB 任一必要字段无效/过期时返回 -ENODATA，不发送填造值。未初始化 -EACCES、参数错误 -EINVAL、容量不足 -EMSGSIZE。
- 默认 feedback_period_us=0，只接收。设为正数后接收线程定时编码并发送；忙时跳过，下一周期取最新值，无历史反馈队列。
- fire_requested 是电平请求。latest 快照不保证保存“每条指令开一发”的离散事件，不提供射击执行语义。

core 单位为 m/s²、rad/s、rad；四元数为 Hamilton wxyz，表示 B→W。body-frame gyro 与 Euler 角导数不同。未来组装者把 IMU 快照复制成 Feedback 时，应按 fresh_mask 清除对应 valid，并检查参考一致性。

## 实现边界与验收

启用 SKYWALKER_VISION 编译核心，SKYWALKER_VISION_AB 增加具体协议，SKYWALKER_VISION_RECEIVER 增加依赖 UART_TRANSPORT 的线程端点。关闭 REFEREE/REMOTE_DR16/INTERBOARD 仍可独立构建。

[视觉回显样例](../samples/communication/vision/README.md) 提供接线、VOFA 通道和发送器；[主流程验收](../tests/vision_imu/README.md) 覆盖流式收包、坏帧恢复、停止请求、过期和反馈编码。尚未实物验证 UART、上位机运行配置和坐标约定。
