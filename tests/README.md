# 单元测试与 CI 验证

本仓库的 **Zephyr Twister + native_sim/native/64** 是唯一要求运行的单元测试环境。PR 和 Push 更新测试或生产源码时由 GitHub Actions 自动执行；不依赖开发者安装 Zephyr SDK 或连接实体主控板。

| 测试入口 | 覆盖范围 |
| --- | --- |
| [algorithms](algorithms/README.md) | PID、前馈、限幅、角度、矩阵/卡尔曼/EKF、舵轮运动学、功率限制 |
| [command_manager](command_manager/README.md) | 手动映射、输入过期、视觉仲裁、权限拒绝、执行命令安全退化 |
| [command_service](command_service/README.md) | CommandManager 生命周期、线程缓存、多个消费者与输入超时 |
| [execution](execution/README.md) | 当前 RunStatus、SnapshotCache、Heartbeat/Endpoint、源生产时间戳；删除已经不存在的 RecoveryGate 老接口测试 |
| [motor/regression](motor/regression/README.md) | Motor/Group/CanBus 公共接口、失效默认态、命令合法性与修订原子性 |
| [motor_protocols](motor_protocols/README.md) | 新增 DJI / DM 原生 CAN 报文编解码与拒绝非法输入 |
| [protocols](protocols/README.md) | 新增 DR16、裁判系统、板间协议、Endian/时间戳/重放等可独立测试协议 |
| [robotics_contracts](robotics_contracts/README.md) | 新增 YawCenteringController、状态反馈与权限撤销 |
| [vision_imu](vision_imu/README.md) | 视觉 AB、DM-IMU 协议、IMU 状态、姿态估计与温控数据流 |

**执行命令**（Actions 使用同一入口，并自动发现将来新增的子目录）：

```sh
west twister -T skywalker_code/tests -p native_sim/native/64 --inline-logs --jobs 2 -O build/unit-tests
```

本次重构的原则：
1. 测试生产实现，而不是复制算法到测试中；外部输入使用固定的 golden vector，避免编码器和解码器一起出错造成假阳性。
2. 校验边界、无效帧和状态撤销时**原输出不被部分写入**；校验原生产时间戳不因重传或读缓存续命。
3. 对硬件驱动分层：CAN 报文 codec 在宿主机直接编译实际生产 `.cpp`，电机组公共接口测试不启动真实 CAN 控制器；真实 UART、CAN 控制器、GPIO/PWM 物理行为应另做 HIL 测试，不用 `native_sim` 的通过率冒充板级可靠性。
4. 保留已有有价值的算法 golden vector、噪声/数值边界、实时状态机回归及计时场景；还移除了依赖已删除私有成员的电机旧测试；CAN 时序回归需专用假件重新覆盖。

## 仍需要板级或后续替身测试的部分

电机发送/停止竞态、CAN 恢复、`drivers/imu/bmi088_imu.cpp`、`drivers/imu/dm_imu_rs485.cpp`、`lib/communication/async_uart.cpp`、`lib/communication/*interboard_transport.cpp`、`lib/communication/remote_receiver.cpp`、`lib/communication/vision_receiver.cpp`、实体 CAN/UART/PWM 以及机器人应用装配需要硬件或 Zephyr 驱动替身。**本次 PR 不声称已实现这些模块的单元测试覆盖或真实板级验证**。纯逻辑和数据契约必须先通过 Actions。

Actions 会在 Workflow Summary 输出逐套通过数和算法耗时，上传 Twister JSON/XML、构建及 handler 日志（14 天）。CI 失败时先看单项测试及对应编译日志；不要仅根据样例固件能编译就判定测试通过。
