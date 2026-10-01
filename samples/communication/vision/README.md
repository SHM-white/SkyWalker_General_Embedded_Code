# 视觉 AB 指令接收与 VOFA 回显

独立接收线程由 VisionReceiver 管理，内部使用 AsyncUart、VisionLink 和 AbProtocol。此样例只读取视觉下行指令并回显，没有 IMU、机器人控制、射击执行或虚构的姿态反馈。

## 接线和启动

仅支持 dm_mc02/stm32h723xx。本样例 overlay 明确选择 UART7 接视觉、USART1 接 VOFA；与当前板级 telemetry-uart=USART1 一致。

| 用途 | 设备/引脚 | 设置 |
|---|---|---|
| 视觉输入 | UART7，PE7 RX；PE8 TX 已配置但本样例不发送 | 115200，8N1，DMA RX/TX |
| VOFA 输出 | USART1，PA9 TX、PA10 RX | 115200，8N1，JustFloat |
| console 日志 | USART10，PE2 RX、PE3 TX | 板级日志配置 |

视觉端 TX 接 PE7，VOFA USB 转串口适配器 RX 接 PA9，两端与板卡共地并匹配电平。两个端口不得共享设备回调。

~~~bash
west build -p always -b dm_mc02/stm32h723xx samples/communication/vision -d build/vision
west flash -d build/vision
~~~

console 应出现 Vision receiver start=0 和 VOFA init=0。start 成功只表示线程已创建，UART 初始化结果还要看 state/uart_error；失败后保持静态对象，线程每 100ms 尝试恢复。

## 发送指令

协议来自用户指定的 [RM2026-AutoAim gimbal.hpp](https://github.com/SHM-white/RM2026-AutoAim/blob/main/io/gimbal/gimbal.hpp)、[gimbal.cpp](https://github.com/SHM-white/RM2026-AutoAim/blob/main/io/gimbal/gimbal.cpp) 和 [crc.cpp](https://github.com/SHM-white/RM2026-AutoAim/blob/main/tools/crc.cpp)，核对日期 2026-09-30。完整偏移、反馈和参考约定见 [视觉协议说明](../../../docs/modules/communication/vision.md)。

下行固定 29 字节：AB、mode、六个小端 float32、CRC16。mode=0 停止、1 控制但不射击、2 控制并请求射击；角度用 rad、角速度 rad/s、角加速度 rad/s²。CRC 覆盖前 27 字节，初值 FFFF、反射多项式 8408、无末尾异或，CRC 低字节先发。

提供一个独立发送器，不需要启动完整自瞄程序：

~~~bash
python3 -m pip install pyserial
python3 samples/communication/vision/send_command.py --port /dev/ttyUSB0 --yaw 0.25 --pitch -0.2
python3 samples/communication/vision/send_command.py --port /dev/ttyUSB0 --mode 0 --count 1
~~~

只查看待发帧可以加 --hex-only，无需 pyserial。默认 50Hz、100 帧、mode=1。115200 下不要直接改为 500Hz：29×500×10=145000 bit/s，已超过线路容量。

原上位机 Gimbal 构造函数等待第一份反馈姿态；直接运行它可能在发送控制前等待。这个接收样例没有真实姿态来源，故不发送假的单位四元数解锁；用上述发送器联调，或由真实反馈生产者调用 receiver.setFeedback() 后另行启用 feedback_period_us。

## VOFA 通道

VOFA+ 选 115200、JustFloat，约 100Hz 输出 16 通道：

| 通道 | 含义 |
|---|---|
| 0 | aim_fresh：最近 100ms 是否有合法命令 |
| 1、2 | control_requested、fire_requested |
| 3、4、5 | yaw、yaw_vel、yaw_acc |
| 6、7、8 | pitch、pitch_vel、pitch_acc |
| 9 | 本地有效命令 sequence |
| 10 | UART 接收 chunk 数，不是帧数 |
| 11、12 | CRC 错误、非法帧累计数 |
| 13、14 | 连续性重置次数、transport 丢块数 |
| 15 | 接收线程状态：0 未启动、1 启动中、2 Running、3 初始化/接收失败 |

停止发送约 100ms 后通道 0 变为 0，其余命令字段保留最后值；消费者必须检查 freshness，不能只看 control_requested。收到 mode=0 会立即覆盖旧 active 命令并清空目标/射击请求。计数转 float 后超过 2²⁴ 不再逐个精确表示。

接收按流处理拆包、粘包、噪声和 CRC 失败；半包超时 20ms，overflow 时丢弃半包。协议没有传感器参考、设备序号或会话字段，配置中的 command_reference={1,1} 只是本地约定，不能用于自动识别远端重启。

构建与主流程模拟通过不等于实物串口、线缆和上位机版本已经验收。实机应检查接收计数、CRC 错误以及断连后过期。
