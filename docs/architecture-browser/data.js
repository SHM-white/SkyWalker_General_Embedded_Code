'use strict';

// Logical module boundaries; detail comes from current public headers and examples.
window.SKYWALKER_CATEGORIES = ['工程与板级', '驱动与感知', '控制算法', '通信与输入', '机器人与仲裁', '调试与观测'];
window.SKYWALKER_GRAPH_COLUMNS = [
  {title: '设备与测量', ids: ['boards', 'imu', 'kalman', 'uart']},
  {title: '协议与输入', ids: ['remote', 'referee', 'vision', 'interboard']},
  {title: '决策与编排', ids: ['command-sources', 'command', 'application', 'telemetry']},
  {title: '机构与控制', ids: ['gimbal', 'chassis', 'motor-control', 'pid']},
  {title: '总线与电机', ids: ['motor-dji', 'motor-dm']}
];

window.SKYWALKER_MODULES = [{
  id: 'application', title: '双主控应用与执行器', category: '机器人与仲裁', status: 'partial',
  summary: '把公开模块装成真实机器人：云台板生成统一命令，底盘板独立执行四舵轮，两边各自管理硬件与恢复。',
  responsibility: 'applications/sentry_gimbal 与 sentry_chassis 持有设备、静态对象、线程、板级配置和执行器。GimbalExecutor、ChassisExecutor 是各应用私有封装；公开模块不会自动把整个机器人装配完。',
  statusNote: '已有双板软件骨架；正式云台仅单 Yaw，未接 IMU/视觉/Pitch/发射。两端 connections_configured=false，裁判 profile 未选，底盘功率模型未标定。',
  depends: ['command', 'interboard', 'gimbal', 'chassis', 'motor-dji', 'boards'],
  source: [
    {label: '云台应用线程与来源注册', path: 'applications/sentry_gimbal/src/main.cpp'},
    {label: '云台执行器声明', path: 'applications/sentry_gimbal/src/gimbal_executor.hpp'},
    {label: '云台执行与恢复', path: 'applications/sentry_gimbal/src/gimbal_executor.cpp'},
    {label: '云台板级绑定', path: 'applications/sentry_gimbal/src/board_config.hpp'},
    {label: '底盘应用线程', path: 'applications/sentry_chassis/src/main.cpp'},
    {label: '底盘执行器声明', path: 'applications/sentry_chassis/src/chassis_executor.hpp'},
    {label: '底盘执行与恢复', path: 'applications/sentry_chassis/src/chassis_executor.cpp'},
    {label: '八电机硬件适配', path: 'applications/sentry_chassis/src/chassis_hardware.hpp'},
    {label: '底盘板级绑定', path: 'applications/sentry_chassis/src/board_config.hpp'}
  ],
  docs: [{label: '双主控应用', path: 'docs/applications/dual-controller.md'}, {label: '模块联动', path: 'docs/applications/module-integration.md'}],
  interfaces: [
    {signature: 'RunStatus GimbalExecutor::begin()', description: '检查应用连接配置，构造本地电机/轴的执行上下文。返回执行状态；初始化请求和反馈 ready 是不同阶段。', parameters: [], returns: 'RunStatus 值副本，含 state、reason、ready、error 等。调用方读取状态，而不是把 begin 当作已开始运动。', context: '云台执行线程唯一所有者；对象在控制循环前静态创建。', errors: '接线门禁、设备、电机模式、参考或启动条件不满足时保持不可执行，原因见 RunStatus。'},
    {signature: 'RunStatus GimbalExecutor::update(const GimbalCommand &command, core::TimeUs now_us)', description: '推进单 Yaw 执行、反馈准备、目标有效期、显式使能、控制更新和 CanBus 提交；异常时撤销并推进恢复。当前不消费 command.pitch。', parameters: [{name: 'command', meaning: '命令服务生成的带原始时间戳云台命令；不自行刷新 stamp。'}, {name: 'now_us', meaning: '单调时间，单位 µs，使用 core::monotonicTimeUs()；真实周期由执行器计算。'}], returns: '当前 RunStatus；ready、状态原因和错误用于跨板摘要及日志。', context: '单一 gimbalTask 每约 5 ms 调用；不能从串口/CAN ISR 调用。', errors: '过期命令、Safe、反馈/参考失效、异常 dt、CAN 故障与恢复等待均会限制输出；正式配置默认阻断。'},
    {signature: 'explicit ChassisExecutor(communication::InterBoardEndpoint &link)', description: '绑定长期存活的板间端点，执行器从该端点快照消费底盘目标与约束。构造不接管 poll 的所有权。', parameters: [{name: 'link', meaning: '底盘角色的端点；linkTask 是 poll() 的唯一调用方。'}], returns: '构造应用私有对象，不启动线程。', context: '在 chassisTask 中静态构造；引用的端点先存在且一直存活。', errors: '运行配置错误由 begin()/update() 的 RunStatus 报告。'},
    {signature: 'RunStatus ChassisExecutor::begin(); RunStatus ChassisExecutor::update(core::TimeUs now_us)', description: '初始化八电机硬件域并推进四舵轮执行。校验心跳、命令年龄、目标 boot ID、resume generation、反馈和功率预算；允许后执行解算与逐总线提交。', parameters: [{name: 'now_us', meaning: '本地单调微秒时间；控制周期不是固定写死的 0.005 s。'}], returns: 'RunStatus 含本地状态、ready、generation、last_command_sequence 与 error；现有类型不含状态生产时间。', context: '唯一 chassisTask 约每 5 ms 调用；端点通信推进独立在 linkTask。', errors: '过期/身份错/恢复代次错/预算不可信/硬件未就绪时本地撤销；联网不等于允许出力。'}
  ],
  examples: [
    {title: '云台应用：启动来源服务，再分别消费命令', language: 'cpp', code: '#include <robotics/command/command_manager.hpp>\n#include <robotics/command/receiver_sources.hpp>\n#include <core/clock.hpp>\n#include "gimbal_executor.hpp"\n\n// commands、remote_source、permission_source、link 均按 main.cpp 静态构造。\n// 启动线程，按顺序执行：\nint startCommands() {\n    int ret = commands.registerSource(remote_source);\n    if (ret == 0) ret = commands.bindPermissions(permission_source);\n    if (ret == 0) ret = commands.start();\n    return ret;\n}\n\n// linkTask 周期片段：端点 poll 只有一个调用线程。\nvoid linkTick() {\n    skywalker::robotics::CommandSnapshot frame{};\n    if (commands.snapshot(frame) == 0) {\n        link.submit(frame.decision.command.chassis);\n        link.setReferee(frame.observed.referee);\n    }\n    link.poll(k_uptime_get());\n}\n\n// gimbalTask 周期片段：executor.begin() 在循环前调用一次。\nvoid gimbalTick(GimbalExecutor &executor) {\n    skywalker::robotics::RobotCommand command{};\n    if (commands.current(command) != 0) return;\n    auto status = executor.update(command.gimbal,\n        skywalker::core::monotonicTimeUs());\n    link.setStatus(status);\n}', notes: '这是放入现有应用上下文的调用片段，变量的静态构造见云台 main.cpp，不能当独立 cpp 文件构建。start 成功只说明服务启动；原始命令年龄仍由消费者检查。GimbalExecutor 头文件属于 sentry_gimbal 私有范围。'},
    {title: '底盘应用：通信推进与机构执行分别拥有线程', language: 'cpp', code: '// applications/sentry_chassis/src/main.cpp 的组织方式。\n// link 为 ChassisController 角色的静态 InterBoardEndpoint。\nvoid linkTask(void *, void *, void *) {\n    for (;;) {\n        link.poll(k_uptime_get());\n        k_sleep(K_MSEC(1));\n    }\n}\n\nvoid chassisTask(void *, void *, void *) {\n    static ChassisExecutor executor(link);\n    auto initial = executor.begin();\n    // initial.state/reason/error 可记录到日志。\n    for (;;) {\n        auto status = executor.update(skywalker::core::monotonicTimeUs());\n        link.setStatus(status);\n        k_sleep(K_MSEC(5));\n    }\n}', notes: '板级对象、包括静态 DMA 与 transport 的构造使用真实应用配置。这里的 setStatus 发布的是执行状态摘要；它没有自动测量车体 vx/vy/wz，也不为旧 RunStatus 增加生产时间。'}
  ],
  lifecycle: ['按物理接线配置两个应用自己的 board_config.hpp 与 app.overlay，选定裁判 profile、电机模式和端口。', '静态构造接收器、DMA、来源、命令服务和板间端点；接收对象必须覆盖 worker 生命周期。', 'main() 注册 RemoteSource，绑定权限，启动 CommandManager；此后不再注册来源。', 'linkTask 唯一推进端点；云台/底盘控制线程分别 begin 并周期 update。', '每个执行器在自己的线程完成反馈准备、参考初始化、使能、目标更新、总线提交和异常撤销。', '整车目标还需应用装配视觉/IMU、双轴与任务所需发射/搜索；查看“最终上车蓝图”逐项补齐。'],
  pitfalls: ['不要从旧图寻找 GlobalSafetyManager、GimbalLocalSafety、ChassisLocalSafety 或 CommandRouter；当前职责分别在 CommandArbiter 与应用执行器/消费者。', 'CommandSnapshot 是非消费式值副本；linkTask 与 gimbalTask 的两次读取可能对应不同 sequence。', '每条物理 CAN 只由一个 CanBus 管理；Endpoint 的 CAN 传输后端需独占另一控制器。', '应用线程存活、通信在线、命令新鲜、执行 ready、电机 TX 完成和机构真正停止是不同状态。', '上车配置不能只翻 connections_configured：参考、单位、方向、急停、功率与恢复上下文都要真实绑定。'],
  config: [{name: 'connections_configured', description: '两端默认 false。确认实际设备、ID、机械参数与门控后由应用配置。'}, {name: 'command_policy.allow_auto', description: '正式云台默认 false；需要注册视觉来源并完成惯性/机械目标适配后启用。'}, {name: 'referee_version / require_referee_for_motion', description: '默认 Unspecified / true；需真实协议 profile、UART 和许可来源。'}, {name: 'power_model_calibrated / require_power_budget', description: '底盘默认 false / true；真实功率模型与预算必须可信。'}, {name: 'interboard_transport.kind', description: '默认 Uart；两板匹配选择 UART/RS485/CAN，端点 API 不随物理后端改变。'}]
}];
