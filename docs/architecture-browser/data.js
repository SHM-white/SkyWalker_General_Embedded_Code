'use strict';
// Navigation and graph metadata only. Interface details live in docs/api/*.md.
window.SKYWALKER_CATEGORIES = [
  "工程与板级",
  "驱动与感知",
  "控制算法",
  "通信与输入",
  "机器人与仲裁",
  "调试与观测"
];
window.SKYWALKER_GRAPH_COLUMNS = [
  {
    "title": "设备与测量",
    "ids": [
      "boards",
      "imu",
      "kalman",
      "uart"
    ]
  },
  {
    "title": "协议与输入",
    "ids": [
      "remote",
      "referee",
      "vision",
      "interboard"
    ]
  },
  {
    "title": "决策与编排",
    "ids": [
      "command-sources",
      "command",
      "application",
      "telemetry"
    ]
  },
  {
    "title": "机构执行",
    "ids": [
      "gimbal",
      "inertial",
      "chassis",
      "big-yaw",
      "shooter"
    ]
  },
  {
    "title": "算法与电机",
    "ids": [
      "pid",
      "motor-control",
      "motor-dji",
      "motor-dm"
    ]
  }
];
window.SKYWALKER_MODULES = [
  {
    "id": "application",
    "title": "双主控应用与执行器",
    "category": "机器人与仲裁",
    "status": "partial",
    "summary": "两个 sentry 入口共用 vehicle_bench：云台惯性双轴/发射/回中，底盘四舵轮/大 Yaw，各机构统一物理 CAN 发布。",
    "responsibility": "公共运行时持有物理设备、长期对象、输入/通信/观测线程与执行周期；公开执行器保存目标和消费新鲜输入，Motor/CAN 独立恢复。",
    "statusNote": "软件框架已有，connections_confirmed、imu_mounting_confirmed 等中央确认默认关闭；功率、热量、拨盘原点与视觉反馈测量待接。",
    "depends": [
      "command",
      "interboard",
      "inertial",
      "gimbal",
      "chassis",
      "big-yaw",
      "shooter",
      "boards"
    ],
    "reference": "docs/api/application.md"
  },
  {
    "id": "motor-dji",
    "title": "DJI 电机与共享 CAN 总线",
    "category": "驱动与感知",
    "status": "ready",
    "summary": "统一 Motor/CanBus 持续接收目标，设备独立自动恢复；effort 单位 A。",
    "responsibility": "Motor 保存运行意图、最新命令和真实反馈；CanBus 处理物理 I/O、限频协议重试和独立恢复；Group 只做显式批量操作。",
    "statusNote": "源码与台架入口已有；模式、ID、方向、范围和机械参数按实物确认。设备掉线不撤销其他轴。",
    "depends": [
      "boards"
    ],
    "reference": "docs/api/motor-dji.md"
  },
  {
    "id": "motor-dm",
    "title": "达妙电机：MIT / 速度 / 位置速度",
    "category": "驱动与感知",
    "status": "ready",
    "summary": "统一 Motor/CanBus 持续接收目标，设备独立自动恢复；effort 单位 N·m。",
    "responsibility": "Motor 保存运行意图、最新命令和真实反馈；CanBus 处理物理 I/O、限频协议重试和独立恢复；Group 只做显式批量操作。",
    "statusNote": "源码与台架入口已有；模式、ID、方向、范围和机械参数按实物确认。设备掉线不撤销其他轴。",
    "depends": [
      "boards"
    ],
    "reference": "docs/api/motor-dm.md"
  },
  {
    "id": "imu",
    "title": "IMU：独立采集、姿态与加热",
    "category": "驱动与感知",
    "status": "partial",
    "summary": "BMI088 和达妙 RS485 共用带时间戳的快照，不同来源独立采集与独立诊断。",
    "responsibility": "ImuSource 负责某种真实输入；ImuState 按字段发布测量与新鲜度；ImuReceiver 独占采集循环并可管理 ImuHeater。BMI088 可挂 QuaternionEkf 估计姿态；RS485 直接使用设备主动上报的四元数。",
    "statusNote": "板载与外置统一采集已实现，整车云台装配头部外置 IMU 与惯性适配；imu_mounting_confirmed 默认 false，姿态质量与参考仍须实测。",
    "depends": [
      "boards",
      "kalman",
      "pid",
      "uart"
    ],
    "reference": "docs/api/imu.md"
  },
  {
    "id": "kalman",
    "title": "姿态 EKF 与通用 Kalman / Matrix",
    "category": "控制算法",
    "status": "ready",
    "summary": "明确两条独立路线：BMI088 四元数 EKF；CMSIS-DSP 线性 Kalman 和矩阵工具。",
    "responsibility": "QuaternionEkf维护每实例四元数、协方差、零偏和静止初始化。KalmanFilter提供通用线性预测/校正，调用者提供持久缓冲并显式Init，两者没有调用依赖。",
    "statusNote": "独立QuaternionEkf已接BMI088输入与dual_imu样例；通用Kalman与Matrix为可选数学工具，不在当前BMI088姿态链中。整车仍需明确yaw外部参考、源选择和降级策略，EKF本身不提供绝对航向或双IMU融合。",
    "depends": [],
    "reference": "docs/api/kalman.md"
  },
  {
    "id": "pid",
    "title": "PID、前馈、斜坡与角度工具",
    "category": "控制算法",
    "status": "ready",
    "summary": "无I/O的纯控制计算，明确单位、采样间隔、积分限制与失败语义。",
    "responsibility": "PID算反馈校正，前馈按参考运动估算需要的输出；斜坡限制目标变化率，角度工具解决±π跳变；C速度/位置环把这些算子串起来。硬件权限、反馈时效和CAN由上层封装处理。",
    "statusNote": "纯算法已由MotorControl、IMU加热与台架样例复用；可接入上车控制。增益和限幅仍是设备/机构相关参数，现有样例数值不代表整车调参完成。",
    "depends": [],
    "reference": "docs/api/pid.md"
  },
  {
    "id": "motor-control",
    "title": "VelocityMotor / PositionMotor 硬件闭环",
    "category": "控制算法",
    "status": "ready",
    "summary": "VelocityMotor / PositionMotor 保存最新目标，各轴独立反馈计算与自动重置历史。",
    "responsibility": "纯 C 控制算法 + producer 身份与计算版本；不管理运行许可，不创建线程，不发送 CAN。",
    "statusNote": "现行 configure/update/telemetry 契约已实现，MotorSession、MotorSafety、ControlFailurePolicy 和 preflight 已移除。",
    "depends": [
      "motor-dji",
      "motor-dm",
      "pid"
    ],
    "reference": "docs/api/motor-control.md"
  },
  {
    "id": "boards",
    "title": "板级：MC02 / RoboMaster Type-C",
    "category": "工程与板级",
    "status": "ready",
    "summary": "把物理引脚、外设、alias、时钟、DMA与电源连接交给Zephyr，业务参数保留在应用配置。",
    "responsibility": "DTS描述物理设备与连接，overlay选择应用实际使用的设备，Kconfig决定哪些实现进入构建，board_config.hpp提供电机ID/限幅/机械方向/链路身份。它们是四个不同层次。",
    "statusNote": "两套板级存在；正式整车实物参数集中 calibration.hpp，connections_confirmed 默认 false；样例可另有 board_config。",
    "depends": [],
    "reference": "docs/api/boards.md"
  },
  {
    "id": "uart",
    "title": "异步 UART 与 DMA",
    "category": "通信与输入",
    "status": "ready",
    "summary": "把 UART 回调产生的字节复制进有界队列，业务线程按接收时刻解析；同一设备只有一个所有者。",
    "responsibility": "AsyncUart 负责双 RX 缓冲、整批 TX 复制、断流代次与重试。它不识别 DR16、裁判、视觉或板间消息，也不判断业务在线。",
    "statusNote": "已用于遥控、裁判、视觉和板间 UART/RS485；实际设备、DMA 通道与 nocache 区域仍由板级配置确定。",
    "depends": [
      "boards"
    ],
    "reference": "docs/api/uart.md"
  },
  {
    "id": "remote",
    "title": "DR16 遥控输入",
    "category": "通信与输入",
    "status": "ready",
    "summary": "从 18 字节 DR16 帧发布摇杆、开关、鼠标和键盘快照；脱机时保持数据但撤销 online。",
    "responsibility": "Dr16Decoder 解码单帧，RemoteService 负责流式对齐和超时，RemoteReceiver 拥有 UART worker。遥控输入如何变成运动模式由命令层决定。",
    "statusNote": "DR16 receiver、独立样例和命令来源适配器已实现；sentry_gimbal 已接入 RemoteSource，硬件接线门禁仍需配置。",
    "depends": [
      "uart"
    ],
    "reference": "docs/api/remote.md"
  },
  {
    "id": "referee",
    "title": "裁判许可与功率预算",
    "category": "通信与输入",
    "status": "ready",
    "summary": "校验版本化裁判帧，分别记录机构输出许可、功率限额和缓冲能量的有效期。",
    "responsibility": "RefereeParser 负责 CRC 与已支持命令，RefereeService 计算在线状态，RefereeReceiver 用 poll 推进 UART。裁判许可约束运动命令，不能与遥控或视觉竞争控制来源。",
    "statusNote": "当前实现 profile 为 Rm2026V1_3，识别 0x0201/0x0202；sentry_gimbal 的板级配置仍是 Unspecified，需要明确配置后才有有效许可。",
    "depends": [
      "uart"
    ],
    "reference": "docs/api/referee.md"
  },
  {
    "id": "interboard",
    "title": "双主控板间通信",
    "category": "通信与输入",
    "status": "ready",
    "summary": "统一 v4 字节契约，七类消息、boot/生产序号与原年龄，UART/RS485/CAN 三后端。",
    "responsibility": "Transport 管物理批次；Endpoint 唯一通信线程推进收发，其他线程复制输入/状态，不根据电机 ready 决定目标发送。",
    "statusNote": "v4 已实现，旧版本直接拒绝；恢复授权 generation 删除。真实两板应使用同版固件，整车默认 USART1 UART。",
    "depends": [
      "uart",
      "boards"
    ],
    "reference": "docs/api/interboard.md"
  },
  {
    "id": "vision",
    "title": "视觉链路与 AB 协议",
    "category": "通信与输入",
    "status": "ready",
    "summary": "接收独立的 yaw/pitch 目标请求，提供带参考系与有效期的值；反馈由应用复制姿态等实测数据。",
    "responsibility": "AbProtocol 定义 29 字节下行/43 字节上行线协议；VisionLink 校验参考、时间与反馈；VisionReceiver 驱动独占 UART。模块不持有 IMU，不仲裁机器人命令，不操作电机和发射机构。",
    "statusNote": "AB 接收已接入整车可选观察/执行阶段；自动执行要求头部参考匹配。弹速/弹数来源与显式视觉会话待补，反馈 TX 默认关闭。",
    "depends": [
      "uart"
    ],
    "reference": "docs/api/vision.md"
  },
  {
    "id": "command",
    "title": "命令仲裁与后台服务",
    "category": "机器人与仲裁",
    "status": "ready",
    "summary": "按 Safe/Manual/Auto、输入新鲜度、参考系、人工接管和裁判许可发布完整机器人命令。",
    "responsibility": "CommandArbiter 是同步策略核心；CommandManager 注册静态来源、启动来源并周期采样/仲裁，发布非消费快照。执行器负责本地电机反馈、使能与恢复。",
    "statusNote": "注册来源与后台服务已实现；整车手动默认 Remote，裁判/视觉按 VEHICLE_* 阶段配置注册，不改变原输入年龄。",
    "depends": [
      "command-sources"
    ],
    "reference": "docs/api/command.md"
  },
  {
    "id": "command-sources",
    "title": "来源适配与手动映射",
    "category": "机器人与仲裁",
    "status": "ready",
    "summary": "把接收器快照接到固定 Operator/Aim 角色；裁判单独作为 Permission，保留原始时间与诊断。",
    "responsibility": "ICommandSource/IPermissionSource 是扩展入口；内置 RemoteSource、VisionSource、RefereePermissionSource 适配接收器。ManualCommandMapper 把遥控值变成归一化操作意图，不输出电机电流。",
    "statusNote": "注册来源与后台服务已实现；整车手动默认 Remote，裁判/视觉按 VEHICLE_* 阶段配置注册，不改变原输入年龄。",
    "depends": [
      "remote",
      "vision",
      "referee"
    ],
    "reference": "docs/api/command-sources.md"
  },
  {
    "id": "gimbal",
    "title": "云台单轴与本地执行",
    "category": "机器人与仲裁",
    "status": "ready",
    "summary": "机械单轴保存目标；公开双轴 GimbalExecutor 独立计算两轴并由应用统一提交总线。",
    "responsibility": "GimbalAxis 管机械范围与目标，PositionMotor 管本轴反馈计算；GimbalExecutor 管新鲜输入/许可和双轴调用，不持有 CAN。",
    "statusNote": "正式云台已装配双轴与惯性适配框架，真实接线与安装确认默认关闭。",
    "depends": [
      "motor-control",
      "command"
    ],
    "reference": "docs/api/gimbal.md"
  },
  {
    "id": "chassis",
    "title": "舵轮底盘与功率缩放",
    "category": "机器人与仲裁",
    "status": "ready",
    "summary": "四轮运动学与八轴独立有效性；公开 SwerveHardware/ChassisExecutor 持续暂存，应用统一 commit。",
    "responsibility": "SwerveKinematics 分目标，SwerveModule 独立转向/轮驱计算与翻转迟滞，SwerveHardware 映射八电机；功率执行使用可信预算与测量。",
    "statusNote": "单舵轮最新提交记录基本功能调通；四轮/整车软件入口已有，真实几何与功率仍待标定。",
    "depends": [
      "pid",
      "motor-dji",
      "interboard"
    ],
    "reference": "docs/api/chassis.md"
  },
  {
    "id": "telemetry",
    "title": "VOFA、日志与一致快照",
    "category": "调试与观测",
    "status": "ready",
    "summary": "从同一帧快照解释来源、目标、许可、反馈和错误；JustFloat 有界排队，主机离线不阻塞控制。",
    "responsibility": "VOFA 有界 JustFloat/调参行；Latest 与 SnapshotCache 交换值副本，应用观察线程不参与目标授权。",
    "statusNote": "单舵轮 16 通道，M2006 速度环 USB CDC 12 通道与反馈/CAN 前后快照诊断；详细通道见各样例 README。",
    "depends": [],
    "reference": "docs/api/telemetry.md"
  },
  {
    "id": "inertial",
    "title": "头部惯性云台适配",
    "category": "机器人与仲裁",
    "status": "partial",
    "summary": "头部 IMU 与机械反馈将惯性目标转成双轴关节 Rate；两轴数学有效性分别返回，电机恢复不改业务目标。",
    "responsibility": "头部 IMU 与机械反馈将惯性目标转成双轴关节 Rate；两轴数学有效性分别返回，电机恢复不改业务目标。",
    "statusNote": "软件接口与整车装配已有；中央硬件/安装/测量确认未完成，不代表实机验收。",
    "depends": [
      "imu",
      "gimbal",
      "command"
    ],
    "reference": "docs/api/inertial.md"
  },
  {
    "id": "big-yaw",
    "title": "大 Yaw 回中与独立速度环",
    "category": "机器人与仲裁",
    "status": "partial",
    "summary": "云台小 Yaw 中心外环产生速度请求；底盘大 Yaw 独立连续速度内环，不需绝对零点或轮控恢复授权。",
    "responsibility": "云台小 Yaw 中心外环产生速度请求；底盘大 Yaw 独立连续速度内环，不需绝对零点或轮控恢复授权。",
    "statusNote": "软件接口与整车装配已有；中央硬件/安装/测量确认未完成，不代表实机验收。",
    "depends": [
      "gimbal",
      "motor-control",
      "interboard"
    ],
    "reference": "docs/api/big-yaw.md"
  },
  {
    "id": "shooter",
    "title": "摩擦轮与拨盘发射执行",
    "category": "机器人与仲裁",
    "status": "partial",
    "summary": "摩擦轮与拨盘独立持续目标；供弹业务检查热量/可信原点/摩擦稳定/头部状态；离散旧事件消费丢弃不重放。",
    "responsibility": "摩擦轮与拨盘独立持续目标；供弹业务检查热量/可信原点/摩擦稳定/头部状态；离散旧事件消费丢弃不重放。",
    "statusNote": "软件接口与整车装配已有；中央硬件/安装/测量确认未完成，不代表实机验收。",
    "depends": [
      "command",
      "gimbal",
      "motor-control",
      "referee"
    ],
    "reference": "docs/api/shooter.md"
  }
];
