'use strict';

// v4 overlay: synchronize the browser with main@fcf2c7f without duplicating the
// original v3 data/app implementation. Loaded after app.js, then re-renders it.
const UPDATED_COMMIT = 'fcf2c7fbcda02ee1083b8da19c9e2b19f63b837c';
const updatedSourceUrl = path =>
  `https://github.com/SHM-white/SkyWalker_General_Embedded_Code/blob/${UPDATED_COMMIT}/${path.split('/').map(encodeURIComponent).join('/')}`;

const imuNode = {
  id: 'imu_stack', lane: 'gimbal', order: 8, status: 'verify', kicker: 'sensor pipeline', title: 'IMU / ImuReceiver',
  summary: 'BMI088 + EKF / DM-IMU-L1 RS485 独立采集',
  description: 'IMU 已拆成独立 Source/State/Receiver：板载 BMI088 可接 QuaternionEkf 与 ImuHeater，外置 DM-IMU-L1 可经 RS485 主动帧进入统一 Snapshot。模块代码已实现，但 sentry_gimbal 尚未实例化 ImuReceiver，也未把姿态送入云台控制或视觉反馈。',
  interfaces: ['ImuReceiver.start() / snapshot() / status()', 'ImuSource: init / service / snapshot', 'Snapshot { sample, fresh_mask, reference, diagnostics }'],
  constraints: ['每个 Source 只有一个采集所有者；使用 Receiver 后应用不得并发调用 init/service', 'Receiver、Source、估计器、Heater 与 DMA 必须覆盖工作线程寿命', '外置 RS485 源不由 MCU PWM 温控', 'DM-IMU 单位、安装方向、四元数方向与持续频率仍需实机确认', '当前正式 sentry_gimbal 没有消费该快照'],
  dependsOn: [], provides: ['vision_auto'],
  sources: ['include/drivers/imu/imu.hpp', 'include/drivers/imu/imu_receiver.hpp', 'include/drivers/imu/bmi088_imu.hpp', 'include/drivers/imu/dm_imu_rs485.hpp', 'drivers/imu/imu_receiver.cpp', 'drivers/imu/bmi088_imu.cpp', 'drivers/imu/dm_imu_rs485.cpp', 'docs/06-drivers-imu.md'],
};
const visionNode = {
  id: 'vision_link', lane: 'gimbal', order: 9, status: 'verify', kicker: 'vision UART', title: 'VisionReceiver / AB 协议',
  summary: '115200 8N1 · 29B 下行 / 43B 上行',
  description: 'VisionProtocol、VisionLink、VisionReceiver 已作为独立通信模块实现：解析视觉 yaw/pitch 目标并可编码姿态/弹速反馈。它不持有 IMU、不直接生成 RobotCommand；当前 sentry_gimbal 也尚未实例化该接收器。',
  interfaces: ['VisionReceiver.start() / snapshot() / setFeedback()', 'VisionLink.processRxBytes() / encodeFeedback()', 'AbProtocol: VisionToGimbal 29 B / GimbalToVision 43 B'],
  constraints: ['AB 固定 115200 8N1；CRC 与 DM-IMU 算法不同', 'mode=0 是合法停止消息，会清空 active 目标', 'fire_requested 是电平请求，不是可靠离散发射事件队列', '反馈字段必须按各自时间戳和 reference 校验，过期时不得填造发送', 'Receiver、Protocol 与独占 nocache DMA 必须长期存活', '真实 UART、上位机配置和坐标约定仍待实机验证'],
  dependsOn: [], provides: ['vision_auto'],
  sources: ['include/communication/vision/vision_protocol.hpp', 'include/communication/vision/vision_link.hpp', 'include/communication/vision/vision_receiver.hpp', 'include/communication/vision/ab_protocol.hpp', 'lib/communication/vision_link.cpp', 'lib/communication/vision_receiver.cpp', 'lib/communication/vision_ab_protocol.cpp', 'docs/18-vision.md'],
};
for (const node of [imuNode, visionNode]) {
  if (!byId[node.id]) {
    nodes.push(node);
    byId[node.id] = node;
  }
}

Object.assign(byId.command_manager, {
  description: '统一生成 chassis、gimbal、shooter 命令快照和 producer sequence。当前 step() 入口仍是 OperatorIntent；正式应用只接入 ManualCommandMapper，视觉 Auto 尚未装配。',
  sources: ['include/robotics/command/command_manager.hpp', 'include/robotics/messages/command.hpp', 'lib/robotics/command.cpp', 'applications/sentry_gimbal/src/main.cpp'],
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
  sources: ['docs/18-vision.md', 'docs/06-drivers-imu.md', 'docs/dev/视觉与IMU独立模块最终规划.md', 'applications/sentry_gimbal/src/main.cpp', 'include/robotics/command/command_manager.hpp'],
});
byId.shooter.order = 11;
byId.shooter.sources = ['include/robotics/messages/command.hpp', 'docs/dev/双主控框架使用说明.md'];
byId.reset_event.sources = ['docs/dev/双主控框架使用说明.md', 'include/communication/interboard/interboard_protocol.hpp'];
if (!byId.async_uart.sources.includes('docs/16-uart-dma-nocache.md')) byId.async_uart.sources.push('docs/16-uart-dma-nocache.md');
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
