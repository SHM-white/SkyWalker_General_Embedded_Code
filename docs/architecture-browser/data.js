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
    "summary": "两个 sentry 工程各自独立装配：云台手动惯性双轴/回中发送，底盘四舵轮/大 Yaw 执行，共用实车配置和普通模块 API。",
    "responsibility": "每个 main 持有长期对象、初始化顺序、执行循环和物理 CAN 提交；固定上车工程共用 installed overlay/conf 与纯配置 profile，不共享整车 run。",
    "statusNote": "27 个相关工程编译完成，未实板验证；连接/IMU/发射确认现值 true、功率模型 false。当前最终应用不装配视觉/裁判/发射或功率闭环。",
    "depends": [
      "command",
      "interboard",
      "inertial",
      "gimbal",
      "chassis",
      "big-yaw",
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
    "summary": "BMI088、达妙 RS485 与 CAN 来源共用带时间戳的快照，独立采集与诊断。",
    "responsibility": "ImuSource 负责某种真实输入；ImuState 按字段发布测量与新鲜度；ImuReceiver 独占采集循环并可管理 ImuHeater。BMI088 可挂 QuaternionEkf 估计姿态；RS485 通过请求应答读取设备四元数；CAN 支持主动上报和被动轮询。",
    "statusNote": "当前云台使用专用 CAN3 主动四元数上报(从机 1/反馈 0x11)与惯性适配；imu_mounting_confirmed 现值 true，安装/质量/参考仍待实车记录。",
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
    "responsibility": "DTS 描述物理设备；installed overlay/conf 固定板级资源和驱动，calibration/profile 提供标定与纯配置。各独立工程自行装配。",
    "statusNote": "新上车入口和最终应用共用 config/vehicle/installed；云台 USART10 板间/CAN3 IMU、底盘 USART1 板间，两板 UART7 诊断。DAPLink 接线尚未实查。",
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
    "statusNote": "sentry_gimbal 使用 UART5 RemoteSource 与真实安全档归中解锁；底盘经板间接收原输入授权，恢复不能伪造永久运行。",
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
    "statusNote": "Rm2026V1_3 库支持 0x0201/0x0202；当前两个最终应用不创建裁判接收器。真实许可、测量及电气适配仍需独立接入。",
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
    "responsibility": "Transport 管物理批次；Endpoint 由唯一 owner 推进，当前应用 main 调用 poll；其他接口交换值副本，目标发送不等待电机 ready。",
    "statusNote": "v4 已实现并有独立字节契约；当前云台板间 USART10、底盘 USART1，460800。boot、原始年龄与状态生产时效分别检查。",
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
    "statusNote": "AB 库与历史样例保留；当前最终应用未创建视觉来源，新共享配置不提供 vision-uart，UART7 专用诊断。参考会话和弹速/弹数仍需补齐。",
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
    "statusNote": "当前固定手动云台只注册 RemoteSource，真实 RC 解锁接入外部门控；视觉/裁判不是额外 conf 开启的当前应用能力。",
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
    "statusNote": "RemoteSource、VisionSource 与 RefereePermissionSource API 存在；当前最终云台只注册真实 RemoteSource，快照读取不刷新原输入年龄。",
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
    "statusNote": "最终云台独立装配双轴与惯性适配，单轴/双轴有固定上车工程；中央连接/安装现值 true，不代表实板已通过。",
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
    "statusNote": "独立单模块/四模块/跨板工程与最终底盘已编译；当前最终应用为中央低电流手动机械策略，未启用实测功率闭环。",
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
    "summary": "同一 DAPLink UART7 通过一个代码变量选择日志或 VOFA JustFloat，队列拥塞丢观测并计数。",
    "responsibility": "Diagnostics 接收状态与事件；专用日志后台统一 LOG/printk，Vofa 模式丢弃文本；固定队列与唯一发送 worker 不参与电机授权。",
    "statusNote": "diagnostics_config.hpp 选择 Log/Vofa 后重编译烧录；console/shell/banner 关闭。16×192 字节有界队列，通道/单位见 samples/bringup/README.md。",
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
    "statusNote": "CAN3 头部 IMU 与独立惯性上车入口/最终装配已实现并编译；安装变换和实车性能仍需确认。",
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
    "statusNote": "独立请求/执行/双 Yaw 工程与最终装配已有；执行在底盘 CAN3，线上为关节 rad/s，保留当前反号适配；未实板验证。",
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
    "statusNote": "库 API 与历史样例已有，当前最终云台未创建发射电机或执行器；真实热量/许可/可信归位仍待接入，有载发射未完成。",
    "depends": [
      "command",
      "gimbal",
      "motor-control",
      "referee"
    ],
    "reference": "docs/api/shooter.md"
  }
];
