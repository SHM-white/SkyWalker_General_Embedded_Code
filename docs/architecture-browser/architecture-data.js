'use strict';

// Repository facts come from the application entry points and public interfaces.
// ready means source code exists; partial means assembly/configuration is incomplete;
// planned means the depicted integration or behavior is a proposed next step.
window.SKYWALKER_ARCHITECTURE = {
  history: [
    {
      hash: 'fc75055', date: '2026-09-22 19:15:29 +08:00',
      title: '首次加入双主控交互式架构浏览器',
      detail: '提交题为 docs: add interactive dual-controller architecture browser。index.html、styles.css、data.js、app.js 直接维护；原 README 说明浏览器是人工维护的架构快照，展示双板、模块关系与场景。Markdown 文档原本继续作为详细说明，没有被浏览器替代。'
    },
    {
      hash: '4937c12', date: '2026-09-27 01:30:17 +08:00',
      title: '随多品牌电机重构增加电机详细链路',
      detail: '该提交更新基础 data.js/app.js/index.html，并新增 motor-flow.js、motor-flow.css 和 run.sh。文档逐渐变成双板总览加独立电机交互视图；底层 Markdown 模块目录仍存在。'
    },
    {
      hash: '32f07b9', date: '2026-09-30 03:30:05 +08:00',
      title: '新增 architecture-refresh.js 增量覆盖层',
      detail: '提交题为 docs: add architecture browser v4 refresh，只新增 111 行刷新脚本。它继续使用基础 data.js/app.js 的全局对象，追加 IMU、视觉节点，并替换模块描述、场景和页面文字。'
    },
    {
      hash: 'd860163', date: '2026-09-30 03:30:47 +08:00',
      title: 'HTML 启用覆盖层：改成当前叠加维护方式的时间点',
      detail: '提交题为 docs: load architecture browser refresh。index.html 唯一变化是在 app.js 后、motor-flow.js 前加入 architecture-refresh.js，因此这是覆盖层实际进入浏览器执行的提交。'
    },
    {
      hash: 'a5054de', date: '2026-09-30 03:31:07 +08:00',
      title: 'README 记录改动目的：同步视觉与独立 IMU',
      detail: '提交说明刷新用于补入视觉 AB 通信与独立 IMU 架构，明确 data.js/app.js 仍为 v3 基线、刷新脚本为 v4 增量；并写明改动扩大时应将增量折回基础文件。Git 记录能够证明这个同步目的，无法证明某个作者另有取代 Markdown 的意图。'
    },
    {
      hash: 'cedab2a', date: '2026-10-02 00:38:53 +08:00',
      title: '在覆盖层继续追加新命令服务说明',
      detail: '提交题为 docs: add command service documentation for registered command sources and arbitration service。新增注册来源、CommandManager worker、CommandArbiter 与本地执行器描述，并覆盖旧 GlobalSafetyManager/CommandRouter 的名称。旧基线和多次覆盖并存，需要读者区分多代接口；本次重建将当前事实与目标蓝图放入明确的数据结构。'
    },
    {
      hash: '65a3de9', date: '2026-10-02 01:13:51 +08:00',
      title: '板间通信已扩展为 UART、RS485、CAN 三后端',
      detail: '当前应用通过 ConfiguredInterBoardTransport 选择后端，默认 UART。这个新接入能力在新版架构中明确展示，并保留 Endpoint 的业务协议、boot ID、恢复代次与原始消息有效期语义。'
    }
  ],

  current: {
    summary: '当前正式应用是“手动单 Yaw 云台 + 双板四舵轮底盘”的软件骨架。遥控、裁判权限、命令后台服务、跨板传输和本地执行恢复已有调用链；两应用的硬件连接门禁都默认关闭。IMU、视觉和三源仲裁各自有实现/样例，正式 sentry_gimbal 尚未装配自瞄。图中的已有代码不代表已经实车运行。',
    lanes: [
      {
        title: '云台主控：遥控输入到单 Yaw 电机',
        steps: [
          { title: 'DR16 接收器', moduleId: 'remote', status: 'ready', note: 'RemoteReceiver worker 收字节并发布 RemoteState，保留输入时间与 online 状态。' },
          { title: '注册命令与权限来源', moduleId: 'command-sources', status: 'ready', note: 'RemoteSource + RefereePermissionSource；后者在命令 worker 中 poll 裁判接收器。' },
          { title: '后台采样与命令仲裁', moduleId: 'command', status: 'ready', note: 'CommandManager worker 调用 CommandArbiter，发布 CommandSnapshot/RobotCommand；默认仲裁周期 10 ms。' },
          { title: '本地执行与恢复', moduleId: 'application', status: 'partial', note: 'gimbalTask 每 5 ms 调 GimbalExecutor；检查命令年龄、反馈、参考、控制周期和显式使能。connections_configured=false。' },
          { title: '单 Yaw 云台轴', moduleId: 'gimbal', status: 'ready', note: 'GimbalAxis 执行 Disabled/Hold/Rate/AbsoluteAngle；该应用只调用 yaw，没有执行 pitch。' },
          { title: '位置环与速度环', moduleId: 'motor-control', status: 'ready', note: 'PositionMotor 组合位置/速度控制；使用实际 dt_s，输出的 effort 单位跟随电机能力。' },
          { title: 'DJI GM6020 / CAN1', moduleId: 'motor-dji', status: 'partial', note: '当前工厂选择 GM6020 ID 1；CAN attach/start 后周期 commit。零点与电流模式仍为模板配置。' }
        ]
      },
      {
        title: '双板分工：底盘命令到八台电机，再回传执行状态',
        steps: [
          { title: '云台 linkTask 消费快照', moduleId: 'application', status: 'ready', note: 'snapshot → link.submit(chassis) + setReferee；独立于 gimbalTask，不消费或刷新命令时间。' },
          { title: '板间业务端点与物理链路', moduleId: 'interboard', status: 'ready', note: '两端 linkTask 约 1 ms poll；UART 默认 460800，另有 RS485/CAN 实现。10 ms 最早发送机会、20 ms 心跳机会。' },
          { title: '底盘本地授权与恢复', moduleId: 'application', status: 'partial', note: 'ChassisExecutor 检查 heartbeat、command、receiver_boot_id、resume_generation、功率预算；正式连接和功率标定门禁关闭。' },
          { title: '四舵轮运动学与控制', moduleId: 'chassis', status: 'ready', note: 'SwerveChassis：机体 vx/vy/wz → 四个舵向和轮速目标；ChassisPowerLimiter 可缩放 effort。' },
          { title: '八电机硬件适配', moduleId: 'motor-dji', status: 'ready', note: 'DjiChassisHardware 管 4×GM6020 + 4×M3508、一个 Group 和最多两条 CanBus；每条物理 CAN 分别 commit。' },
          { title: '底盘状态回到云台端', moduleId: 'interboard', status: 'partial', note: 'RunStatus → heartbeat + ChassisFeedbackSummary；当前回传 ready/armed/reason/sequence，没有有效实测机体速度/功率字段。' }
        ]
      },
      {
        title: '独立模块已有代码，正式应用尚未连通的感知分支',
        steps: [
          { title: '板载与外置 IMU', moduleId: 'imu', status: 'partial', note: 'BMI088 + 可选 QuaternionEkf/ImuHeater；DM-IMU-L1 主动 RS485；统一 ImuReceiver/Snapshot。正式应用无实例。' },
          { title: '视觉 AB 收发', moduleId: 'vision', status: 'partial', note: 'VisionReceiver 已有 29 B 目标下行、43 B 反馈上行；正式应用无实例，默认反馈周期为 0。' },
          { title: '三源自动仲裁', moduleId: 'command-sources', status: 'partial', note: 'VisionSource 已实现，samples/robotics/command_manager 演示 Operator + Aim + Permission；正式应用未注册 Aim。' },
          { title: '双轴与惯性执行装配', moduleId: 'gimbal', status: 'planned', note: '双轴云台样例已有，但正式应用仅单 Yaw；视觉惯性角目标与机械轴参考之间的适配仍待完成。' },
          { title: '行为搜索、射击和导航', moduleId: 'application', status: 'planned', note: '搜索只有设计文档；ShooterCommand 有类型与仲裁请求，无射击执行器；机体里程计/自主导航未装配。' }
        ]
      }
    ],
    gaps: [
      {
        title: '实际接线与电机配置尚未开放',
        detail: '两份 board_config.hpp 的 connections_configured=false；GM6020 的 current_mode_confirmed=false，encoder_zero_ticks=0 为模板值。CAN 设备、ID、方向、零点、限幅和真实机构必须逐项配置，当前默认会报告 Configuration 阻塞。',
        source: 'applications/sentry_gimbal/src/board_config.hpp'
      },
      {
        title: '正式云台只有 Yaw，尚无视觉与 IMU',
        detail: 'sentry_gimbal/main.cpp 只注册 RemoteSource，command_policy.allow_auto=false；没有 VisionReceiver、VisionSource 或 ImuReceiver。GimbalExecutor 只执行 yaw 字段。协议模块、双轴样例或 Auto 仲裁算法的存在，不能等同于整车自瞄已完成。',
        source: 'applications/sentry_gimbal/src/main.cpp'
      },
      {
        title: '裁判协议与串口仍需真实板级绑定',
        detail: 'require_referee_for_motion=true；referee_version=Unspecified，app.overlay 的 referee-uart alias 仍是注释。需要按选定裁判 profile 配置接收 UART/DMA，使输出许可和功率字段真正有效。',
        source: 'applications/sentry_gimbal/app.overlay'
      },
      {
        title: '底盘功率门禁与模型标定尚未完成',
        detail: 'require_power_budget=true 且 power_model_calibrated=false；当前估计式使用 idle_power_w + Σ|current_a|×power_per_abs_amp_w，系数默认均为 0。未标定不能把它当作实测功率。bench_effort_scale=0.15 也不是比赛功率保证。',
        source: 'applications/sentry_chassis/src/board_config.hpp'
      },
      {
        title: '状态摘要尚无真实运动反馈与状态生产年龄',
        detail: 'wireFeedback 仅映射执行状态，valid_fields 保持 0，vx/vy/wz/power 未从测量填充。RunStatus 没有生产时间；通信线程反复编码快照不能证明执行线程仍在更新。最终应分别声明执行状态年龄、机体反馈有效位和测量时间。',
        source: 'include/robotics/execution/run_status.hpp'
      },
      {
        title: '物理急停、自动行为与发射执行需要整车接入',
        detail: '操作者 Safe 和失联撤销已有软件路径，但正式应用仍需绑定真实急停与恢复策略；搜索规划仅有设计记录。ShooterCommand 能表达 Ready/FireContinuous 等请求，但两应用没有消费 shooter 的执行线程，也没有摩擦轮、拨弹、热量或卡弹闭环。',
        source: 'include/robotics/messages/command.hpp'
      },
      {
        title: '资源选择必须避免 UART/CAN 所有权冲突',
        detail: '板间 RS485 默认 USART2；如果外置 DM IMU 也选该端口，须改用另一独占端口或更换板间后端。CAN 板间后端默认独占 CAN3，不能与 motor::CanBus 共用同一控制器。各接收器需独立 nocache DMA，资源模板不等于已确认接线。',
        source: 'docs/modules/communication/interboard-transports.md'
      }
    ]
  },

  target: {
    summary: '建议最终上车采用“两块实时主控 + 视觉计算机”的分工：云台主控统一接入操作者、视觉、姿态和裁判，形成有权限、有时效的命令；云台与发射在本地执行，底盘主控独立完成四舵轮、功率与恢复。两板双向交换命令、权限、状态和可信反馈。虚线/待接入部分是目标蓝图，当前没有完整实现。实际是否需要大小 Yaw、发射、导航，应按机械与任务配置。',
    lanes: [
      {
        title: '感知与人工输入：字节或测量先变成可信快照',
        steps: [
          { title: 'DR16 / 键鼠', moduleId: 'remote', status: 'ready', note: '保留手动/自动/Safe 意图与原始接收时间；人工可随时接管。' },
          { title: '裁判输出许可与功率预算', moduleId: 'referee', status: 'partial', note: '按当前比赛协议 profile 绑定真实串口；各权限/功率测量分别过期。' },
          { title: 'IMU 采集、EKF 与安装变换', moduleId: 'imu', status: 'partial', note: '选择板载 BMI088 或外置 RS485，明确 B→W 四元数、fresh_mask、参考 epoch；双 IMU 必须显式说明各自角色。' },
          { title: '视觉计算机 ↔ VisionReceiver', moduleId: 'vision', status: 'partial', note: '下行目标与上行姿态/弹速/计数双向闭环；需要与上位机约定参考、坐标系和恢复会话。' }
        ]
      },
      {
        title: '云台主控：所有来源经过同一命令与权限边界',
        steps: [
          { title: 'Operator / Aim / Permission 适配', moduleId: 'command-sources', status: 'partial', note: '注册 RemoteSource、VisionSource、RefereePermissionSource；不要在转发时修改源 stamp。' },
          { title: '模式仲裁与命令快照', moduleId: 'command', status: 'partial', note: '已有 Manual/Auto、视觉失效 Hold、手动接管与权限否决；正式应用启用 Auto 前完成参考和本地控制适配。' },
          { title: '跟踪 / 丢目标 / 搜索行为', moduleId: 'application', status: 'planned', note: '在行为层生成目标，加入丢目标延迟与重获确认；搜索不下沉到 PID/驱动，也不绕过命令权限。' },
          { title: '命令与反馈消费者', moduleId: 'application', status: 'partial', note: '本地执行线程、板间 linkTask 和观测线程分别消费值副本；执行仍检查消息年龄。' }
        ]
      },
      {
        title: '云台执行：完整双轴，必要时增加大小 Yaw 协调',
        steps: [
          { title: '姿态目标 → 机械轴目标适配', moduleId: 'gimbal', status: 'planned', note: '依据安装关系和机构反馈转换惯性目标；Yaw/Pitch 不得直接把世界角当编码器角。确认 Continuous/Limited 拓扑与零位。' },
          { title: '双轴或多轴执行与恢复', moduleId: 'application', status: 'planned', note: '把 Pitch 装入正式执行器；有大小 Yaw 时再增加协调器与分配策略。两轴分别处理反馈、参考、限位与新命令。' },
          { title: 'GimbalAxis + PositionMotor', moduleId: 'gimbal', status: 'partial', note: '复用已有单轴控制；同机械故障域用 Group，共享物理 CAN 的提交由单一所有者协调。' },
          { title: 'DJI / DM 电机输出', moduleId: 'motor-dji', status: 'partial', note: '按实物选择驱动；DJI effort=A，DM MIT effort=N·m。DM 当前没有固定零绝对角，不直接套连续 Yaw 假设。' },
          { title: '真实姿态与轴状态反馈', moduleId: 'imu', status: 'planned', note: '反馈线程从 IMU/轴测量组装 Vision Feedback，保留各字段原始时间，参考重建时使旧目标失效。' }
        ]
      },
      {
        title: '底盘主控：独立执行、功率约束、反馈闭环',
        steps: [
          { title: '双向板间业务契约', moduleId: 'interboard', status: 'ready', note: '选 UART/RS485/CAN 之一；云台下发命令/约束，底盘回传 heartbeat/状态；保留 boot、generation、来源年龄。' },
          { title: 'ChassisExecutor 本地门控', moduleId: 'application', status: 'partial', note: '不依赖远端及时送停机消息；反馈、权限、链路、上下文或周期异常时本地撤销输出。' },
          { title: 'Swerve + 标定后的功率限制', moduleId: 'chassis', status: 'partial', note: '真实轮径、轮位、方向、零点、速度限幅与负载参数；预算与模型新鲜有效后缩放输出。' },
          { title: '4 舵向 + 4 驱动电机', moduleId: 'motor-dji', status: 'partial', note: 'Group 统一启停；一条或两条真实 CAN 均有唯一 CanBus，反馈和输出均按 SI 单位。' },
          { title: '机体速度 / 功率 / 里程计', moduleId: 'chassis', status: 'planned', note: '由真实轮速、舵角与可选姿态估计 vx/vy/wz，可信后才设置有效位；若需要导航，另接定位与路径目标生成。' }
        ]
      },
      {
        title: '整车闭环：按任务装配发射、急停和观测',
        steps: [
          { title: '发射执行子系统', moduleId: 'application', status: 'planned', note: 'ShooterCommand → 摩擦轮/拨弹执行器；增加热量、弹速、累计计数、卡弹恢复与许可门控。离散单发需要事件序号/确认。' },
          { title: '物理急停与恢复授权', moduleId: 'boards', status: 'planned', note: '按实物接 GPIO/动力切断；故障撤销输出，恢复从新反馈、新参考、新命令开始，不沿用断线前目标。' },
          { title: '状态、年龄与资源观测', moduleId: 'telemetry', status: 'partial', note: '记录输入年龄、仲裁理由、执行状态年龄、generation、CAN/UART 错误、控制周期与功率；VOFA 可用于台架观察。' }
        ]
      }
    ],
    requirements: [
      { title: '先固定真实机械结构', detail: '明确单/双 Yaw、Pitch、舵轮位置和发射机构。连续轴与有限位轴使用不同参考和限位策略；大小 Yaw 协调是需要机构支持的目标能力。' },
      { title: '所有命令有来源、单位与年龄', detail: '角度 rad，角速度 rad/s，机体速度 m/s；MessageStamp 使用 ms，core::Measurement 使用 us，控制周期使用 s。源消息时间在适配/转发/快照读取中不刷新。' },
      { title: '惯性目标与机械轴参考有明确适配', detail: '定义世界 W、机体 B、传感器 S、各电机零位以及旋转方向。BMI088 yaw 为局部参考，没有绝对 yaw 观测；IMU epoch 或电机 reference_generation 变化后重建目标。' },
      { title: '控制层与决策层各自承担门控', detail: 'CommandArbiter 处理输入和输出许可，本地执行器处理真实反馈、参考、周期和功率。底盘能在对端断线时独立停输出；通信在线不能代替执行任务正常。' },
      { title: '清晰的启动、失能、恢复顺序', detail: '启动服务不等于设备已准备；反馈稳定和参考有效后才 reset/enable。恢复要求属于新 boot/generation 的新命令，Enabling 需要跨周期推进，失能请求及时取消。' },
      { title: '物理外设只有一个所有者', detail: '同一 CAN 控制器不同时交给 Motor CanBus 与板间 CAN 后端；IMU/视觉/遥控/VOFA/板间串口不争用同一 UART。每实例 DMA、对象和回调依赖长期存活。' },
      { title: '发射请求与实际射击反馈分开', detail: 'fire_requested 是电平。单发必须有事件号、去重和结果；实际弹速/计数来自可信来源。缺失热量、许可、有效瞄准或机构状态时不得生成可执行射击输出。' },
      { title: '功率与机体反馈由真实数据支撑', detail: '标定功率模型或接入测量，预算过期时撤销输出；底盘目标速度不能冒充实测速度。ChassisFeedbackSummary 只有字段可信、新鲜时才置 valid_fields。' },
      { title: 'AB 反馈不能填造缺失字段', detail: '现协议没有线上 valid bit/frame_id/epoch。必须约定固定参考；上行所需姿态、角速度、弹速、计数的测量任一无效/过期，当前编码器拒绝发送，应用需完成全部反馈来源装配。' },
      { title: '交付分为三个可理解的里程碑', detail: '第一步：实车手动单 Yaw + 四舵轮安全启停恢复；第二步：正式双轴 + 姿态/视觉双向闭环与手动接管；第三步：按任务接搜索、发射、定位/导航。每步保留当前状态与剩余接入项。' }
    ]
  },

  scenarios: [
    {
      id: 'manual-gimbal', title: '遥控如何让云台转起来',
      summary: '当前正式 sentry_gimbal 的单 Yaw 调用链。运动还受真实配置和裁判许可门禁约束；pitch 意图虽存在于消息，应用没有消费。',
      steps: [
        { title: 'DR16 字节到遥控快照', moduleId: 'remote', detail: 'RemoteReceiver 内部 worker 通过 AsyncUart 接收 DMA 字节，DR16 解码后发布通道、拨杆、键鼠与原始 MessageStamp；读者不会把旧数据变新。' },
        { title: '采样遥控与裁判权限', moduleId: 'command-sources', detail: 'main 先 registerSource(remote_source)、bindPermissions(permission_source)、start()。RemoteSource.sample 复制遥控，RefereePermissionSource.sample 在仲裁 worker 中 poll 裁判。' },
        { title: '人工意图变成有单位的命令', moduleId: 'command', detail: 'ManualCommandMapper 将拨杆映射为 Safe/Manual/Auto，归一化通道经 CommandArbiter 转成机体 m/s、云台 rad/s。输入失联/Safe 或裁判否决生成 Disabled；当前 allow_auto=false。' },
        { title: '控制线程读取副本', moduleId: 'application', detail: 'gimbalTask 约每 5 ms 调 current(command)，再 executor.update(command.gimbal, monotonicTimeUs())。current 非消费式；不同读线程可能取得不同 sequence。' },
        { title: '轴准备与显式使能', moduleId: 'gimbal', detail: 'GimbalExecutor 检查连接配置、命令时效与反馈。GimbalAxis.poll 准备参考；新命令必须晚于 ready_since_ms。先 reset，再 Motor.enable，等待 Enabling 结束后才执行闭环。' },
        { title: '控制计算并发布 CAN 输出', moduleId: 'motor-control', detail: 'Active 时 GimbalAxis.update 驱动 PositionMotor；位置环/速度环输出 effort，暂存到 Motor，随后 CanBus.commit。提交接受不等于电机已经执行。' },
        { title: '电机反馈形成下一周期闭环', moduleId: 'motor-dji', detail: 'CAN 回调与总线 worker 更新 MotorSnapshot；下一周期读取角度、速度、状态与反馈年龄。反馈断流、参考变化或周期超 20 ms 时执行器撤销输出。' }
      ],
      sources: [
        { label: '正式云台应用入口', path: 'applications/sentry_gimbal/src/main.cpp' },
        { label: '单 Yaw 执行与恢复', path: 'applications/sentry_gimbal/src/gimbal_executor.cpp' },
        { label: '命令仲裁实现', path: 'lib/robotics/command.cpp' },
        { label: '双轴遥控样例（独立台架）', path: 'samples/robotics/gimbal_control/src/main.cpp' }
      ]
    },
    {
      id: 'vision-auto', title: '视觉目标如何进入自动瞄准',
      summary: '协议、VisionSource 与 Auto 仲裁已有实现/台架；以下完整上车链路仍需正式应用装配。不能据此认为当前 sentry_gimbal 已可自瞄。',
      steps: [
        { title: '视觉机发送 AB 目标', moduleId: 'vision', detail: 'VisionReceiver 解析 115200 8N1 的 29 B 帧：控制/开火请求、yaw/pitch 角度、速度、加速度。CRC/有限值校验后发布 AimMeasurement；mode=0 清空有效目标。' },
        { title: '将视觉注册为 Aim 来源', moduleId: 'command-sources', detail: '目标装配中增加 VisionReceiver 与 VisionSource，在 CommandManager.start 前注册。VisionSource 直接保留 receiver 的测量 stamp；正式应用当前缺少此步骤。' },
        { title: '进入 Auto 等待新目标', moduleId: 'command', detail: '操作者必须在线且主动选 Auto，allow_auto=true。仲裁等待进入 Auto 后的新 sequence/time，检查 vision_timeout_us、control_requested、OrientationReference 与有限值；无可信目标时输出 Hold。' },
        { title: '手动接管和权限否决', moduleId: 'command', detail: '遥控偏转超过阈值时人工接管；静默一段时间后要求新的视觉目标再回 Auto。裁判权限独立否决 chassis/gimbal/shooter，视觉目标不能绕过。' },
        { title: '姿态目标转换为机械轴目标', moduleId: 'gimbal', detail: '待接入：用 IMU 参考、轴角和安装关系，把视觉惯性目标适配到实际 yaw/pitch 执行参考。当前 GimbalExecutor 只控制 yaw，缺正式 Pitch 与惯性适配。' },
        { title: '闭环执行并回传真实姿态', moduleId: 'imu', detail: '待接入：双轴执行器按本地反馈/限位输出；独立反馈组装线程把新鲜 IMU 姿态、gyro 与可信弹速/计数交给 VisionReceiver.setFeedback，视觉机得到与目标相同参考的反馈。' },
        { title: '目标丢失与参考重建', moduleId: 'application', detail: '视觉过期/停止时不沿用旧目标；IMU reference.epoch 变化后失效旧姿态与目标。搜索行为目前只有规划，丢目标不应暗中被当成可开火状态。' }
      ],
      sources: [
        { label: '三源命令服务样例', path: 'samples/robotics/command_manager/README.md' },
        { label: '视觉协议与反馈合同', path: 'docs/modules/communication/vision.md' },
        { label: 'Auto 仲裁实现', path: 'lib/robotics/command.cpp' },
        { label: '当前正式云台配置', path: 'applications/sentry_gimbal/src/board_config.hpp' }
      ]
    },
    {
      id: 'interboard-chassis', title: '云台板的命令如何驱动底盘板',
      summary: '当前双板已有完整软件路线：最终底盘命令和裁判约束向下，底盘 ready/generation/状态向上；真实硬件和功率标定仍待完成。',
      steps: [
        { title: '云台板取得最终命令', moduleId: 'command', detail: 'linkTask 调 CommandManager.snapshot，取 frame.decision.command.chassis 与 frame.observed.referee；被权限否决后的 Disabled 也保留在该路线中。' },
        { title: '端点生成业务帧', moduleId: 'interboard', detail: 'submit/setReferee 只复制值。唯一通信线程 poll 推进心跳、控制、约束；云台命令绑定对端 boot ID 和恢复 generation，上下文变化后等待新 producer sequence。' },
        { title: '所选物理后端传递完整数据', moduleId: 'interboard', detail: '默认 UART 直接发送 V1 字节流；RS485 使用 Coordinator/Responder 请求应答；CAN 在独占控制器上分片完整批次。溢出、缺片和 CRC 错误会丢半包，不能交付部分控制指令。' },
        { title: '底盘端点发布快照', moduleId: 'interboard', detail: '底盘 linkTask poll 接收并校验，snapshot 包含 heartbeat、RemoteChassisControl、ChassisConstraint 与上下文。online 仅表示心跳新鲜，控制命令和功率仍分别检查。' },
        { title: '本地执行器检查恢复边界', moduleId: 'application', detail: 'ChassisExecutor 检查 100 ms heartbeat/command、receiver_boot_id、resume_generation、准备后的新命令与功率。对端重启、预算失效或上下文不匹配立即 suspend。' },
        { title: '机体速度分配到四舵轮', moduleId: 'chassis', detail: 'SwerveChassis 将 vx/vy/wz 分配为每轮舵角/轮速目标，以实际反馈和 dt 算控制输出；有已标定新鲜功率模型时 ChassisPowerLimiter 缩放 effort。' },
        { title: '八电机输出与状态回传', moduleId: 'motor-dji', detail: 'DjiChassisHardware.apply 设置八个电流，并对每条 CanBus commit。RunStatus 经 setStatus 反向成为 heartbeat 与 ChassisFeedbackSummary；当前没有真实机体速度/功率反馈有效位。' }
      ],
      sources: [
        { label: '云台 linkTask', path: 'applications/sentry_gimbal/src/main.cpp' },
        { label: '板间业务端点', path: 'lib/communication/interboard_endpoint.cpp' },
        { label: '底盘执行器', path: 'applications/sentry_chassis/src/chassis_executor.cpp' },
        { label: '八电机硬件适配', path: 'applications/sentry_chassis/src/chassis_hardware.cpp' }
      ]
    },
    {
      id: 'imu-feedback', title: 'IMU 姿态如何进入视觉反馈与控制',
      summary: '独立 IMU 采集与视觉反馈 API 已有实现，正式应用的两者联动待接入。四元数、字段年龄与参考必须一起传递。',
      steps: [
        { title: '选择独占采集源', moduleId: 'imu', detail: '板载 Bmi088Imu 读取 accel/gyro，可选 QuaternionEkf 与 heater；外置 DmImuRs485Source 解主动帧。每源一个采集所有者，UART/SPI/DMA 不与其他模块争用。' },
        { title: '采集与可选 EKF', moduleId: 'kalman', detail: 'ImuReceiver.start 仅启动 worker；source.service 更新成功测量。板载 EKF 需近静止初始化，重力约束 roll/pitch、yaw 为局部参考；温控有独立有效性与错误状态。' },
        { title: '读取带字段时效的 Snapshot', moduleId: 'imu', detail: 'snapshot 包含各测量 stamp、capabilities、fresh_mask、姿态质量及 reference(frame_id, epoch)。收到 gyro 不刷新 orientation；Running 也不等于姿态新鲜。' },
        { title: '应用组装姿态反馈', moduleId: 'application', detail: '待接入：复制新鲜 orientation 与 body-frame gyro，并保留原始时间。所用 W/B 安装参考须与视觉 command_reference 一致；无有效字段就清 valid，不能填默认单位四元数。' },
        { title: '补齐弹速与计数来源', moduleId: 'application', detail: 'AB 上行还需要 bullet_speed_m_s 与 bullet_count 的可信测量。它们不来自 IMU，也不能为了发帧重新标记为新；目前正式应用没有完整来源装配。' },
        { title: '编码并发送反馈', moduleId: 'vision', detail: 'setFeedback 复制值，feedback_period_us>0 才周期发送。VisionLink 检查各字段时效、姿态/gyro 时间差和参考；缺失 -ENODATA、参考不符 -ESTALE、Euler 奇异 -ERANGE。' },
        { title: '参考变化后重建控制', moduleId: 'gimbal', detail: '待接入：EKF 重初始化/设备归零改变 epoch 后，应用清旧反馈/视觉目标并协调新参考；实际云台轴再从新反馈重建机械参考与新目标。' }
      ],
      sources: [
        { label: 'IMU 接口与参考合同', path: 'docs/modules/drivers/imu.md' },
        { label: '后台采集 API', path: 'include/drivers/imu/imu_receiver.hpp' },
        { label: '视觉反馈数据类型', path: 'include/communication/vision/vision_types.hpp' },
        { label: '双 IMU 台架', path: 'samples/imu/dual_imu/README.md' }
      ]
    },
    {
      id: 'stop-recovery', title: '掉线、掉电、停机后如何恢复',
      summary: '当前执行器和电机层已有撤销/恢复流程。恢复沿新反馈、新参考、新上下文、新命令推进；真实物理急停与完整恢复授权还要按实车接入。',
      steps: [
        { title: '命令层撤销运动', moduleId: 'command', detail: '遥控失联/Safe、输入过期或裁判拒绝时 CommandArbiter 生成 Disabled。应用读取副本还会检查 stamp，快照停更不能继续作为新授权。' },
        { title: '执行层立即撤销输出', moduleId: 'application', detail: '云台命令无效先 suspend；底盘在 requested、power 或上下文不满足时先 suspend。撤销不等待反馈重建、重试截止时间或远端额外停机帧。' },
        { title: '识别可恢复与阻塞故障', moduleId: 'motor-dji', detail: 'UnexpectedDisabled/EnableTimeout 是应用允许尝试恢复的暂态故障；其他驱动故障或永久配置错误保持 Blocked，不由周期循环自动放行。' },
        { title: '等待新鲜稳定反馈和重建参考', moduleId: 'gimbal', detail: '轴 poll 或 hardware.pollRecovery 等待反馈与参考；底盘先 reseedPosition、等全部电机收到新反馈并稳定 30 ms，再 reset Swerve 控制状态。' },
        { title: '恢复代次改变，旧命令失效', moduleId: 'interboard', detail: '底盘准备后递增 generation；端点通过 heartbeat 报告。云台见对端 boot/generation 改变后建立 baseline，必须观察更新 producer sequence 再发送；底盘检查 receiver_boot_id 与 generation。' },
        { title: '接收准备后的新命令并使能', moduleId: 'application', detail: '云台要求 command.timestamp_ms 晚于轴 ready 边界；底盘要求 command 晚于 ready_ms 且上下文一致。reset→enable/arm→Enabling→Active；在握手中出现撤销仍及时失能。' },
        { title: '恢复后再次满足闭环与周期条件', moduleId: 'motor-control', detail: 'Active 期间使用真实 dt，反馈、限位、功率和周期持续检查；超出本地 20 ms 周期上限或总线提交错误可再次撤销。硬件急停不能仅由软件 Safe 模式代替。' }
      ],
      sources: [
        { label: '云台恢复实现', path: 'applications/sentry_gimbal/src/gimbal_executor.cpp' },
        { label: '底盘恢复实现', path: 'applications/sentry_chassis/src/chassis_executor.cpp' },
        { label: '底盘反馈/参考准备', path: 'applications/sentry_chassis/src/chassis_hardware.cpp' },
        { label: '命令与恢复合同', path: 'docs/modules/robotics/command-recovery.md' }
      ]
    }
  ],

  startup: [
    { title: '1 · 确认板级资源与真实机械参数', detail: '先填两应用 board_config.hpp 和 overlay：板间两端角色/后端/速率，独占 UART/CAN/DMA，电机 ID/模式/零点/方向，轮径/轮位和限幅。connections_configured 只在这些参数成立后开放。' },
    { title: '2 · 构造长期存活对象与存储', detail: 'Receiver、Source、Protocol、Estimator、Heater、Endpoint、Motor、CanBus、Group 和 DMA 必须覆盖 worker/callback 生命周期。接收 DMA 放 __nocache，每外设只有一个所有者。构造本身通常不做 I/O。' },
    { title: '3 · 注册来源并启动采样服务', detail: 'CommandManager.start 前注册 Operator、可选 Aim 并绑定 Permission。当前正式应用只注册 RemoteSource。ImuReceiver/RemoteReceiver/VisionReceiver start=0 仅表示 worker 启动，须继续观察初始化状态与数据新鲜度。' },
    { title: '4 · 通信与执行线程持续推进', detail: '当前两应用 K_THREAD_DEFINE 的 link/control 线程为自动启动；linkTask 约 1 ms poll，控制约 5 ms update。即使没有命令或正在恢复也保持 poll；尚未发布快照/未配置设备时保持失能。' },
    { title: '5 · 总线准备与反馈参考重建', detail: '执行器先校验配置、attach 电机、start 总线，再 begin/poll 轴或初始化硬件。等待新鲜反馈、正确参考和稳定窗口后 reset 控制器；启动成功不等于已可使能。' },
    { title: '6 · 新授权跨过准备边界后使能', detail: '命令必须更新、权限有效、板间 boot/generation 一致、底盘功率预算与模型可用。先 enable/arm，等待 Active 再更新/commit；发生撤销时随时取消使能握手。' },
    { title: '7 · 按里程碑装配完整上车功能', detail: '先完成手动云台/底盘与本地停机恢复；再装正式 Pitch、IMU/视觉反馈与惯性目标适配；最后按任务加搜索、发射、定位/导航。记录哪些是已有代码、已装配或仍待上机验证。' }
  ],

  sources: [
    { label: 'Markdown 文档总目录', path: 'docs/README.md' },
    { label: '公共模块地图', path: 'docs/modules/README.md' },
    { label: '双主控应用与线程分工', path: 'docs/applications/dual-controller.md' },
    { label: '模块联动与调用路线', path: 'docs/applications/module-integration.md' },
    { label: '封装模块调用示例', path: 'docs/modules/call-examples.md' },
    { label: '命令来源与后台仲裁服务', path: 'docs/modules/robotics/command-service.md' },
    { label: '云台正式入口', path: 'applications/sentry_gimbal/src/main.cpp' },
    { label: '云台接线与权限策略', path: 'applications/sentry_gimbal/src/board_config.hpp' },
    { label: '底盘正式入口', path: 'applications/sentry_chassis/src/main.cpp' },
    { label: '底盘接线、几何与功率参数', path: 'applications/sentry_chassis/src/board_config.hpp' },
    { label: '板间三种传输后端', path: 'docs/modules/communication/interboard-transports.md' },
    { label: '视觉 AB 数据合同', path: 'docs/modules/communication/vision.md' },
    { label: 'IMU 独立源、姿态、温控', path: 'docs/modules/drivers/imu.md' },
    { label: '搜索行为设计（非当前实现）', path: 'docs/dev/哨兵空闲巡航与搜索模式规划.md' }
  ]
};
