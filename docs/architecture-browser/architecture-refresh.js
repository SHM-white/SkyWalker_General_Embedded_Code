'use strict';

// v5 overlay: align command-source service and current application wiring with main@e1ac0a1.
// The older base nodes are retained where useful, but their current labels are replaced below.
const UPDATED_COMMIT = 'e1ac0a1';
const updatedSourceUrl = path =>
  `https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/${UPDATED_COMMIT}/${path.split('/').map(encodeURIComponent).join('/')}`;

const imuNode = {
  id: 'imu_stack', lane: 'gimbal', order: 8, status: 'verify', kicker: 'sensor pipeline', title: 'IMU / ImuReceiver',
  summary: 'BMI088 + EKF / DM-IMU-L1 RS485 独立采集',
  description: 'IMU 已拆成独立 Source/State/Receiver：板载 BMI088 可接 QuaternionEkf 与 ImuHeater，外置 DM-IMU-L1 可经 RS485 主动帧进入统一 Snapshot。模块代码已实现，但 sentry_gimbal 尚未实例化 ImuReceiver，也未把姿态送入云台控制或视觉反馈。',
  interfaces: ['ImuReceiver.start() / snapshot() / status()', 'ImuSource: init / service / snapshot', 'Snapshot { sample, fresh_mask, reference, diagnostics }'],
  constraints: ['每个 Source 只有一个采集所有者；使用 Receiver 后应用不得并发调用 init/service', 'Receiver、Source、估计器、Heater 与 DMA 必须覆盖工作线程寿命', '外置 RS485 源不由 MCU PWM 温控', 'DM-IMU 单位、安装方向、四元数方向与持续频率仍需实机确认', '当前正式 sentry_gimbal 没有消费该快照'],
  dependsOn: [], provides: ['vision_auto'],
  sources: ['include/drivers/imu/imu.hpp', 'include/drivers/imu/imu_receiver.hpp', 'include/drivers/imu/bmi088_imu.hpp', 'include/drivers/imu/dm_imu_rs485.hpp', 'drivers/imu/imu_receiver.cpp', 'drivers/imu/bmi088_imu.cpp', 'drivers/imu/dm_imu_rs485.cpp', 'docs/modules/drivers/imu.md'],
};
const visionNode = {
  id: 'vision_link', lane: 'gimbal', order: 9, status: 'verify', kicker: 'vision UART', title: 'VisionReceiver / AB 协议',
  summary: '115200 8N1 · 29B 下行 / 43B 上行',
  description: 'VisionProtocol、VisionLink、VisionReceiver 已作为独立通信模块实现：解析视觉 yaw/pitch 目标并可编码姿态/弹速反馈。它不持有 IMU、不直接生成 RobotCommand；当前 sentry_gimbal 也尚未实例化该接收器。',
  interfaces: ['VisionReceiver.start() / snapshot() / setFeedback()', 'VisionLink.processRxBytes() / encodeFeedback()', 'AbProtocol: VisionToGimbal 29 B / GimbalToVision 43 B'],
  constraints: ['AB 固定 115200 8N1；CRC 与 DM-IMU 算法不同', 'mode=0 是合法停止消息，会清空 active 目标', 'fire_requested 是电平请求，不是可靠离散发射事件队列', '反馈字段必须按各自时间戳和 reference 校验，过期时不得填造发送', 'Receiver、Protocol 与独占 nocache DMA 必须长期存活', '真实 UART、上位机配置和坐标约定仍待实机验证'],
  dependsOn: [], provides: ['vision_auto'],
  sources: ['include/communication/vision/vision_protocol.hpp', 'include/communication/vision/vision_link.hpp', 'include/communication/vision/vision_receiver.hpp', 'include/communication/vision/ab_protocol.hpp', 'lib/communication/vision_link.cpp', 'lib/communication/vision_receiver.cpp', 'lib/communication/vision_ab_protocol.cpp', 'docs/modules/communication/vision.md'],
};
for (const node of [imuNode, visionNode]) {
  if (!byId[node.id]) {
    nodes.push(node);
    byId[node.id] = node;
  }
}

Object.assign(byId.command_manager, {
  description: '统一生成 chassis、gimbal、shooter 命令快照和 producer sequence。当前 step() 入口仍是 OperatorIntent；正式应用只接入 ManualCommandMapper，视觉 Auto 尚未装配。',
  sources: ['include/robotics/command/command_manager.hpp', 'include/robotics/command/command_arbiter.hpp', 'include/robotics/command/command_source.hpp', 'include/robotics/command/receiver_sources.hpp', 'lib/robotics/command_manager.cpp', 'applications/sentry_gimbal/src/main.cpp'],
});
if (!byId.command_manager.constraints.includes('不存在可直接提交 VisionTarget 的 AutonomousIntent API'))
  byId.command_manager.constraints.push('不存在可直接提交 VisionTarget 的 AutonomousIntent API');

Object.assign(byId.vision_auto, {
  order: 10,
  status: 'todo',
  kicker: 'application assembly',
  title: 'Vision / Auto 应用装配',
  summary: '独立模块已就绪，主应用仲裁与控制接入未完成',
  description: '下一步不是再写串口解析，而是在 sentry_gimbal 组装 VisionReceiver 与 ImuReceiver：向视觉反馈姿态，定义 Manual/Auto 仲裁，并把视觉目标转换为现有 OperatorIntent/RobotCommand 路径或扩展 CommandManager。当前主应用只读取 DR16，因此自瞄不能被视为已接入。',
  interfaces: ['planned: VisionReceiver::Snapshot + imu::Snapshot → application policy', 'current: CommandManager::step(const OperatorIntent&, ...)', 'planned: fresh/reference/safety checked aim → gimbal command'],
  constraints: ['视觉目标不能绕过 GlobalSafetyManager / GimbalLocalSafety', '必须按 aim_fresh、IMU fresh_mask 与 reference 判断数据时效', '需要明确人工/自瞄切换和失联回退策略', 'fire_requested 不能直接当成不会丢失的单发事件', '不要把通信模块已实现等同于 sentry_gimbal 已接入'],
  dependsOn: ['vision_link', 'imu_stack'],
  provides: [],
  sources: ['docs/modules/communication/vision.md', 'docs/modules/drivers/imu.md', 'applications/sentry_gimbal/src/main.cpp', 'include/robotics/command/command_manager.hpp'],
});
byId.shooter.order = 11;
byId.shooter.sources = ['include/robotics/messages/command.hpp', 'docs/dev/双主控框架使用说明.md'];
byId.reset_event.sources = ['docs/dev/双主控框架使用说明.md', 'include/communication/interboard/interboard_protocol.hpp'];
if (!byId.async_uart.sources.includes('docs/guides/uart-dma.md')) byId.async_uart.sources.push('docs/guides/uart-dma.md');
if (!byId.motor_layer.sources.includes('drivers/motor/group.cpp')) byId.motor_layer.sources.splice(-1, 0, 'drivers/motor/group.cpp');

plain.command_manager = ['决定机器人要做什么', '把人工 OperatorIntent 变成带单位的目标，再交给 Router 分发。当前正式入口还没有视觉专用命令类型。', 'OperatorIntent + GlobalSafetyDecision', 'RobotCommand：底盘、云台、发射机构目标与序号'];
plain.imu_stack = ['采集并统一姿态数据', '板载 BMI088 与外置 DM-IMU-L1 已统一成独立 IMU Source/Receiver 快照；代码已实现，但 sentry_gimbal 还没有实例化并消费。', 'BMI088 SPI / DM-IMU RS485 主动帧', 'imu::Snapshot：加速度、角速度、姿态、fresh_mask 与 reference'];
plain.vision_link = ['收发视觉 AB 串口', 'VisionReceiver 已能解析目标并编码姿态/弹速反馈，但它只是通信模块，不直接生成 RobotCommand；正式云台应用还没有装配。', 'AB 视觉帧：mode、yaw/pitch 及导数', 'Vision Snapshot / 可选 43B GimbalToVision 反馈'];
plain.vision_auto = ['把已实现模块装进自瞄主链', '还需要在 sentry_gimbal 里实例化 VisionReceiver/ImuReceiver、定义人工与自瞄仲裁，并把目标接到安全与云台控制路径。', 'Vision Snapshot + IMU Snapshot + 模式/安全状态', '待实现：经过时效、参考与安全检查的自瞄控制输入'];
stageMap.imu_stack = 'inputs';
stageMap.vision_link = 'inputs';
stageMap.vision_auto = 'command';

scenes.sensing = {title:'看感知接入',summary:'IMU 与视觉串口模块已经独立实现；当前缺口在 sentry_gimbal 的应用装配、模式仲裁和控制接入。',steps:[
  ['imu_stack','板载 BMI088 可经 QuaternionEkf 发布姿态，外置 DM-IMU-L1 可经 RS485 发布统一 Snapshot；两种源都可由 ImuReceiver 独占采集。'],
  ['vision_link','VisionReceiver 解析 AB 下行目标，并可发送由调用者提供的姿态/弹速反馈；通信层不持有 IMU，也不直接控制电机。'],
  ['vision_auto','正式 sentry_gimbal 目前没有实例化这两个 Receiver，也没有 Manual/Auto 仲裁，因此“视觉协议已实现”不等于“自瞄已接入”。'],
  ['command_manager','CommandManager 当前只接受 OperatorIntent。接入自瞄时要么把视觉目标适配到现有意图/命令边界，要么显式扩展接口，并继续经过全局安全。'],
  ['yaw_gimbal','最终目标仍需经过 GimbalLocalSafety 与 GimbalAxis；视觉失联、IMU 过期或 reference 改变时不能沿用旧目标。']
]};

nodes.sort((a,b)=>lanes.findIndex(l=>l.id===a.lane)-lanes.findIndex(l=>l.id===b.lane)||a.order-b.order);

renderDetail = function(){
 const n=byId[state.selected],p=plain[n.id];
 const downstream=n.provides.filter(id=>byId[id]?.status!=='todo');
 const future=n.provides.filter(id=>byId[id]?.status==='todo');
 document.getElementById('detail').innerHTML=`<h2 class="detail-title">${escapeHTML(p[0])}</h2><div class="class-name">${escapeHTML(n.title)}</div><div class="badges">${badges(n)}</div><p class="plain-description">${escapeHTML(p[1])}</p><div class="io-block"><span>INPUT / 输入</span><p>${escapeHTML(p[2])}</p></div><div class="io-block"><span>OUTPUT / 输出</span><p>${escapeHTML(p[3])}</p></div><div class="detail-tabs" role="group" aria-label="详情内容"><button data-tab="overview" class="${state.tab==='overview'?'on':''}" aria-pressed="${state.tab==='overview'}">关系与约束</button><button data-tab="code" class="${state.tab==='code'?'on':''}" aria-pressed="${state.tab==='code'}">接口与源码</button></div><div class="detail-body">${state.tab==='code'?`<h3>类接口 / 数据契约</h3>${n.interfaces.map(s=>`<code class="code">${escapeHTML(s)}</code>`).join('')}<h3>源码固定到更新基线 ${UPDATED_COMMIT.slice(0,7)}</h3><div class="sources">${n.sources.map(s=>`<a target="_blank" rel="noreferrer" href="${updatedSourceUrl(s)}">${escapeHTML(s)} ↗</a>`).join('')}</div>`:`<h3>上游数据 / 协作模块</h3><div class="relations">${n.dependsOn.length?n.dependsOn.map(moduleButton).join(''):'<span class="class-name">底层数据或设备输入</span>'}</div>${downstream.length?`<h3>下游使用</h3><div class="relations">${downstream.map(moduleButton).join('')}</div>`:''}${future.length?`<h3>后续扩展</h3><div class="relations">${future.map(moduleButton).join('')}</div>`:''}<h3>必须遵守的约束</h3><ul class="constraints">${n.constraints.map(s=>`<li>${escapeHTML(s)}</li>`).join('')}</ul>`}</div>`;
 document.querySelectorAll('[data-module]').forEach(el=>el.setAttribute('aria-pressed',String(el.dataset.module===state.selected)));
 document.querySelectorAll('.flow-node').forEach(el=>el.classList.toggle('selected',el.id==='stage-'+stageMap[state.selected]));
};

setScene = function(scene){
 stop();state.scene=scene;state.step=-1;
 document.querySelectorAll('[data-scene]').forEach(el=>{el.classList.toggle('on',el.dataset.scene===scene);el.setAttribute('aria-pressed',String(el.dataset.scene===scene));});
 const s=scenes[scene],summary=document.getElementById('sceneSummary');summary.innerHTML=`<b>${s.title}</b><span>${s.summary}</span>`;summary.classList.toggle('warn',['timeout','recovery'].includes(scene));
 document.querySelectorAll('.flow-node').forEach(el=>{el.classList.remove('current');el.classList.toggle('blocked',scene==='timeout'&&['stage-safety','stage-swerve','stage-motors'].includes(el.id));el.classList.toggle('healthy',scene==='timeout'&&el.id==='stage-yaw');});
 document.getElementById('stepText').textContent=scene==='manual'?'点击任意模块查看详情，或用「逐步讲解」沿两条分支走一遍。':scene==='sensing'?'点击「逐步讲解」查看 IMU / Vision 已实现边界与应用装配缺口。':'点击「逐步讲解」查看条件、判断和恢复顺序；这里展示的是代码逻辑。';
 document.getElementById('stepCount').textContent=`0 / ${s.steps.length}`;document.querySelector('.wire-label').textContent=scene==='timeout'?'链路中断 ×':'底盘命令 →';
 selectModule(scene==='manual'?'command_manager':scene==='sensing'?'vision_link':scene==='timeout'?'chassis_safety':'chassis_hardware');drawWire();
};

const metaLink=document.querySelector('.meta a');
if(metaLink){metaLink.href=`https://github.com/SHM-white/SkyWalker_General_Embedded_Code/tree/${UPDATED_COMMIT}`;metaLink.textContent=`更新基线 ${UPDATED_COMMIT.slice(0,7)} ↗`;}
const note=document.querySelector('.scenario-note');
if(note&&!document.querySelector('[data-scene="sensing"]')){const b=document.createElement('button');b.dataset.scene='sensing';b.setAttribute('aria-pressed','false');b.textContent='视觉 / IMU';note.before(b);}
const head=document.querySelector('.board.gimbal .board-head');
if(head&&!document.getElementById('sensingModules')){const bar=document.createElement('div');bar.id='sensingModules';bar.className='permission-label';bar.innerHTML='<span>感知 / 姿态 · 独立模块已实现，主应用待装配</span><div class="mini-modules"><button data-module="imu_stack">IMU / ImuReceiver ↗</button> · <button data-module="vision_link">Vision AB ↗</button> · <button data-module="vision_auto">Auto 装配 ↗</button></div>';head.after(bar);}
const baseline=document.querySelector('.baseline');if(baseline)baseline.textContent=`双板总览、电机、遥控、视觉与 IMU 说明已同步到代码快照 ${UPDATED_COMMIT.slice(0,7)}。IMU / Vision 独立模块已有代码，但 sentry_gimbal 仍未装配自瞄主链；图示不是整机实测结果。`;
const count=document.querySelector('.count-label');if(count)count.textContent=`${nodes.length} 个模块 ＋`;
const pending=document.querySelector('.pending-links [data-module="vision_auto"]');if(pending)pending.textContent='Vision / Auto 装配 ↗';
const readiness=document.querySelector('.readiness-grid article:nth-child(1) p');if(readiness)readiness.textContent='双主控应用、板间消息、统一多品牌电机层、视觉 AB 通信与双 IMU 独立模块已有实现。';
const todoTitle=document.querySelector('.readiness-grid article:nth-child(3) h3');if(todoTitle)todoTitle.textContent='应用装配与离散事件扩展';
const footer=document.querySelector('footer span');if(footer)footer.textContent='架构阅读视图 · v4 · 同步视觉 / IMU 与统一电机层';

renderCatalog();
setScene('manual');

// Current command path overlay.
const commandSourcesNode = {
  id: 'command_sources', lane: 'gimbal', order: 3.5, status: 'done',
  kicker: 'robotics/command', title: 'Registered input sources',
  summary: 'Operator / Aim / Permission adapters',
  description: 'RemoteSource、VisionSource 与 RefereePermissionSource 将 receiver 快照适配到统一来源接口。每个角色最多一个来源；启动后来源与 receiver 保持静态生命周期。',
  interfaces: ['ICommandSource::role / start / sample', 'IPermissionSource::start / sample', 'SourceDiagnostics'],
  constraints: ['CommandManager::start 前注册来源', '读取保留原始 stamp，不刷新来源有效期', 'RefereePermissionSource 在仲裁 worker 中轮询 RefereeReceiver'],
  dependsOn: ['inputs'], provides: ['command_manager'],
  sources: ['include/robotics/command/command_source.hpp', 'include/robotics/command/receiver_sources.hpp', 'lib/robotics/receiver_sources.cpp'],
};
if (!byId.command_sources) {
  nodes.push(commandSourcesNode);
  byId.command_sources = commandSourcesNode;
}
Object.assign(byId.global_safety, {
  title: 'CommandArbiter',
  kicker: 'robotics/command',
  summary: '同步决策核心',
  description: 'GlobalSafetyManager 已移除。CommandArbiter 接收 CommandInputs，按操作者模式、来源 freshness、视觉 reference 和裁判许可生成 CommandDecision；应用执行器仍负责本地硬件条件与恢复。',
  interfaces: ['CommandArbiter::update(const CommandInputs&)', 'CommandArbiter::configError / reset'],
  constraints: ['单线程所有者调用 update/reset', '来源时间戳不可在拷贝时刷新', '不是完整的电机执行安全管理器'],
  dependsOn: ['manual_mapper', 'inputs'], provides: ['command_manager'],
  sources: ['include/robotics/command/command_arbiter.hpp', 'include/robotics/command/command_inputs.hpp', 'lib/robotics/command.cpp'],
});
Object.assign(byId.command_manager, {
  title: 'CommandManager service',
  kicker: 'robotics/command',
  summary: '注册来源 → 仲裁 worker → snapshots',
  description: 'CommandManager 管来源注册、启动与后台采样；worker 调用 CommandArbiter 并发布完整 CommandSnapshot。它不是同步 update(inputs) 接口，不操作电机。',
  interfaces: ['registerSource / bindPermissions / start', 'current(RobotCommand&) / snapshot(CommandSnapshot&)'],
  constraints: ['start 前注册，至少一个 Operator 来源', '最多一个 Operator 与一个 Aim 来源', 'snapshot/current 非消费式且不刷新时间', '无 stop/restart；sources 和 receivers 静态存活'],
  dependsOn: ['command_sources', 'global_safety'], provides: ['command_router'],
  sources: ['include/robotics/command/command_manager.hpp', 'include/robotics/command/command_source.hpp', 'lib/robotics/command_manager.cpp'],
});
Object.assign(byId.command_router, {
  title: 'CommandSnapshot consumers',
  kicker: 'application threads',
  summary: 'linkTask / gimbalTask consume independent copies',
  description: '当前 sentry_gimbal 没有 CommandRouter 类。linkTask 读取 CommandSnapshot 并发布板间消息；gimbalTask 读取 current() 并调用 GimbalExecutor。',
  interfaces: ['CommandManager::snapshot', 'CommandManager::current', 'InterBoardEndpoint::submit / poll'],
  constraints: ['读取不消费结果、不延长命令 stamp', '不同线程的两次读取可能不是同一 sequence', '消费者仍需执行本地超时与状态检查'],
  dependsOn: ['command_manager'], provides: ['gimbal_safety', 'interboard_codec'],
  sources: ['applications/sentry_gimbal/src/main.cpp', 'include/communication/interboard/interboard_endpoint.hpp'],
});
Object.assign(byId.gimbal_safety, {
  title: 'GimbalExecutor (application)',
  kicker: 'application execution',
  summary: '单 yaw 初始化、执行与恢复',
  description: 'GimbalLocalSafety 已移除。应用私有 GimbalExecutor 组合 CanBus、Motor 与 GimbalAxis，执行命令 freshness、反馈准备、显式使能和本地恢复。',
  interfaces: ['GimbalExecutor::begin / update', 'GimbalAxis::poll / reset / update'],
  constraints: ['board_config::connections_configured 默认为 false', '当前没有注册 VisionSource', '急停硬件输入仍需按机器人接入'],
  dependsOn: ['command_router', 'motor_layer'], provides: ['yaw_gimbal'],
  sources: ['applications/sentry_gimbal/src/gimbal_executor.hpp', 'applications/sentry_gimbal/src/gimbal_executor.cpp', 'include/robotics/gimbal/gimbal_axis.hpp'],
});
Object.assign(byId.chassis_safety, {
  title: 'ChassisExecutor (application)',
  kicker: 'application execution',
  summary: '板间上下文、本地底盘恢复与功率门控',
  description: 'ChassisLocalSafety 已移除。ChassisExecutor 读取 InterBoardEndpoint 快照，检查命令年龄、boot/generation、反馈和功率条件，再协调 SwerveChassis 与 DjiChassisHardware。',
  interfaces: ['ChassisExecutor::begin / update', 'InterBoardEndpoint::snapshot'],
  constraints: ['connections_configured 默认为 false', 'power_model_calibrated 默认为 false', '本地执行器自行撤销输出'],
  dependsOn: ['async_uart', 'swerve', 'chassis_hardware'], provides: ['power_limiter'],
  sources: ['applications/sentry_chassis/src/chassis_executor.hpp', 'applications/sentry_chassis/src/chassis_executor.cpp', 'applications/sentry_chassis/src/chassis_hardware.cpp'],
});
plain.global_safety = ['仲裁命令输入与许可', '当前由同步 CommandArbiter 处理命令策略；旧 GlobalSafetyManager 不在源码中。', 'CommandInputs', 'CommandDecision'];
plain.command_manager = ['采样来源并发布完整决策', 'CommandManager 启动来源和后台 worker，调用 CommandArbiter，再发布可重复读取的 CommandSnapshot。', 'ICommandSource / IPermissionSource', 'CommandSnapshot / RobotCommand'];
plain.command_sources = ['适配通信来源', 'RemoteSource、VisionSource 与 RefereePermissionSource 保留底层接收器的原始测量时间。', 'Receiver snapshots', 'SourceSample + SourceDiagnostics'];
plain.command_router = ['应用线程读取命令', '当前没有独立 CommandRouter 类；linkTask 和 gimbalTask 分别读 snapshot/current 并调用本地接口。', 'CommandSnapshot', 'InterBoardEndpoint / GimbalExecutor'];
plain.gimbal_safety = ['本地云台执行', 'GimbalExecutor 是应用私有封装，负责反馈、命令年龄和执行器恢复；旧 GimbalLocalSafety 已删除。', 'RobotCommand::gimbal + Motor snapshot', 'GimbalAxis → Motor / CanBus'];
plain.chassis_safety = ['本地底盘执行', 'ChassisExecutor 在本机检查 peer、命令时间和恢复代次；旧 ChassisLocalSafety 已删除。', 'InterBoardEndpoint::Snapshot', 'SwerveChassis / DjiChassisHardware'];
plain.vision_auto = ['视觉输入进入命令仲裁', '三源台架已注册 VisionSource；正式 sentry_gimbal 仍未注册 VisionSource，也没有装配 IMU 反馈与云台视觉闭环。', 'VisionReceiver → VisionSource(Aim)', 'CommandArbiter → CommandSnapshot'];
stageMap.command_sources = 'inputs';
stageMap.global_safety = 'command';
stageMap.command_router = 'command';
stageMap.gimbal_safety = 'yaw';
stageMap.chassis_safety = 'safety';
byId.vision_auto.description = 'CommandArbiter 与 VisionSource 已支持台架级 Aim 输入；samples/robotics/command_manager 已接线。正式 sentry_gimbal 只注册 RemoteSource 和 RefereePermissionSource，未注册 VisionSource，因此自动瞄准仍未接入应用。';
byId.vision_auto.interfaces = ['current sample: VisionSource → CommandManager worker', 'current application: RemoteSource + RefereePermissionSource', 'future: IMU feedback producer → vision protocol feedback'];
byId.vision_auto.constraints = ['正式应用 allow_auto=false', '必须配置新的 Aim 来源并核对 reference 与超时', '视觉模块已实现不代表自瞄执行已接入', 'fire_requested 不是可靠的单发事件队列'];
byId.vision_auto.sources = ['include/robotics/command/receiver_sources.hpp', 'samples/robotics/command_manager/src/main.cpp', 'applications/sentry_gimbal/src/main.cpp', 'docs/modules/robotics/command-service.md'];
plain.vision_auto = ['接入正式自瞄流程', '仲裁核心和视觉 source adapter 已存在，缺口在正式应用装配、姿态反馈生产者和最终轴控制闭环。', 'VisionSource + policy + optional IMU feedback', '新鲜且 reference 合法的云台目标'];
scenes.manual = {title:'看命令服务',summary:'来源在启动前注册；后台 CommandManager 调用 CommandArbiter 并发布非消费式快照。',steps:[
  ['command_sources','RemoteSource、VisionSource 和 RefereePermissionSource 适配 receiver，不重置原始时间戳。'],
  ['command_manager','启动线程先注册至少一个 Operator 来源，按策略绑定权限源，再调用 start。'],
  ['global_safety','CommandManager worker 周期采样并调用同步 CommandArbiter，执行来源时效、Auto/Manual 与许可策略。'],
  ['command_router','消费者调用 snapshot/current 取得副本；读快照不会推进输入时间或消费本次输出。'],
  ['gimbal_safety','应用 GimbalExecutor 将最终云台命令与本地 Motor 反馈和恢复条件结合。'],
  ['chassis_safety','底盘 ChassisExecutor 在本地检查板间上下文、命令新鲜度和输出准备。']
]};
scenes.timeout = {title:'看输入过期',summary:'来源使用原始 stamp；过期输入不能通过重复读取或重新发布而续命。',steps:[
  ['command_sources','来源错误和 receiver 状态写入 diagnostics；无新帧时仍保留原始时间。'],
  ['command_manager','后台 worker 持续调用 CommandArbiter，按各自 timeout 生成 Disabled / Hold 决策。'],
  ['command_router','snapshot/current 只是非消费式读取，不会刷新 command stamp。'],
  ['gimbal_safety','GimbalExecutor 检查本地命令期限和反馈；无效时撤销云台输出。'],
  ['chassis_safety','ChassisExecutor 独立检查 peer heartbeat、命令时间、boot 与恢复代次。']
]};
scenes.recovery = {title:'看恢复边界',summary:'输入恢复与电机重新运动是两个阶段；应用须验证本地反馈并等待新的合法命令。',steps:[
  ['command_sources','新的有效来源样本沿用新的生产时间与序号。'],
  ['command_manager','worker 发布新决策；读取者可观察 sequence 与来源诊断。'],
  ['gimbal_safety','执行器重新检查参考与反馈，按新授权 reset 后显式 enable。'],
  ['chassis_safety','底盘确认当前 peer boot/generation 与本地恢复上下文匹配后再准备电机。'],
  ['motor_layer','CanBus 异步提交和安全帧结果不等于机械停止或立即恢复。']
]};
nodes.sort((a,b)=>lanes.findIndex(l=>l.id===a.lane)-lanes.findIndex(l=>l.id===b.lane)||a.order-b.order);
renderCatalog();
setScene('manual');
