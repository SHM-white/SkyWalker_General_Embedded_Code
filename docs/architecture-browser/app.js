'use strict';
const byId = Object.fromEntries(nodes.map(n => [n.id, n]));
const plain = {
 inputs: ['接收操作意图', '把遥控数据解码为操作状态；裁判数据另行提供输出许可和功率额度。', 'DR16 遥控字节 / 裁判串口字节', 'RemoteState / RefereeState'],
 manual_mapper: ['把摇杆变成意图', '把摇杆和键鼠转换成归一化意图。这里还没有电机电流，也不操作硬件。', 'RemoteState', 'OperatorIntent：归一化速度、模式与来源'],
 global_safety: ['决定哪些输出允许执行', '分别为云台、底盘、发射机构给出许可。底盘掉线只影响底盘；全局急停才锁存全部输出。', '操作使能、裁判许可、底盘心跳与执行状态', 'GlobalSafetyDecision：各输出域的 Active / Hold / Disable'],
 command_manager: ['决定机器人要做什么', '把人的操作意图变成带单位的目标，再交给 Router 分发。例如：向前 1 m/s、云台以指定角速度转动。', 'OperatorIntent + GlobalSafetyDecision', 'RobotCommand：底盘、云台、发射机构目标与序号'],
 command_router: ['把命令分成两路', '云台命令留在本板；底盘命令加上安全许可与恢复上下文，经串口发送。两路分别提交。', 'RobotCommand + 安全决策 + 对端心跳', 'LocalGimbalCommand / RemoteChassisControl'],
 gimbal_safety: ['云台执行前的安全检查', '检查本地反馈、硬件、许可与急停。反馈可信但目标过期时可进入 Hold，避免沿用旧转速继续转。', '本地反馈、命令时效、硬件状态、全局许可', 'LocalSafetyDecision：本周期允许的云台动作'],
 yaw_gimbal: ['在云台板控制小 Yaw', 'YawGimbal 把云台目标交给 PositionMotor 做位置闭环；底盘板不参与这条本地电机回路。', 'GimbalCommand + 安全动作 + dt', '本地云台电机的位置目标与输出'],
 interboard_codec: ['把目标打包成串口帧', '只负责确定格式的编码、校验和解析；不会在这里计算轮速或位置环。V1 实现四类消息。', 'Heartbeat / ChassisControl / ChassisConstraint / ChassisFeedback', '最多 142 字节的 V1 帧，CRC16 校验'],
 async_uart: ['在两块板之间传值', '向底盘传命令和约束，向云台回传心跳与执行状态。两板各自控制电机，串口不传实时 PID 运算。', '异步 UART 字节流 / 编码后的发送帧', '最新有效消息快照，含本地接收时间与会话信息'],
 chassis_safety: ['确认这条命令可以执行', '收到命令不等于立即驱动。底盘还要核对时效、心跳、许可和本地硬件；恢复后必须等到新的合法命令。', '远端命令、心跳、boot_id / generation、本地反馈', 'LocalSafetyDecision：允许执行，或继续等待'],
 swerve: ['把车体运动分解到四轮', '将前后、左右、旋转速度解算为四组舵向和驱动目标。轮子位置、半径、方向都来自底盘配置。', 'ChassisCommand + ChassisFeedback + dt', 'ChassisOutput：FL / FR / RL / RR 四组控制输出'],
 power_limiter: ['给底盘输出做功率缩放', '按估计功率、裁判额度与缓冲能量计算缩放比例。已有台架算法，比赛功率控制仍需实测标定。', 'ChassisPowerInput + dt', 'ChassisPowerDecision.effort_scale'],
 chassis_hardware: ['统一管理八电机输出', '把四个舵电机和四个驱动电机按物理 CAN 分组，统一发送。任一 Bus 出错，就暂停整个底盘输出域。', 'ChassisOutput + effort_scale', '1 或 2 个共享 DJI Bus，统一 arm / flush / stop'],
 motor_layer: ['电机闭环与底层驱动', '这是两板共用的控制基础。云台使用 PositionMotor / Backend；八电机底盘使用聚合硬件适配器与共享 Bus。', '位置 / 速度目标，或底盘已计算的 effort', 'CAN 电机命令与反馈；DJI 单位 A，DM MIT 单位 N·m'],
 vision_auto: ['自主目标源还未接入', '当前没有视觉或自主导航生产者。Auto 模式存在，但不能因此把自主控制当成已完成。', '规划中的视觉 / 导航目标', '待实现：自主命令生产者'],
 shooter: ['发射机构还缺执行链', 'ShooterCommand 结构已经存在，摩擦轮、拨弹和离散射击事件尚未接入正式应用。', '已有 ShooterCommand 消息结构', '待实现：执行器与离散事件处理'],
 reset_event: ['跨板复位还缺显式协议', '全局急停可让底盘锁存；跨板明确复位尚未实现。当前底盘复位需要本地 reset hook。', '已解除的急停输入 + 明确复位请求', '待实现：跨板复位事件；当前使用本地 hook']
};
const stageMap = {inputs:'inputs',manual_mapper:'inputs',global_safety:'command',command_manager:'command',command_router:'command',gimbal_safety:'yaw',yaw_gimbal:'yaw',chassis_safety:'safety',swerve:'swerve',power_limiter:'swerve',chassis_hardware:'motors',motor_layer:'motors'};
const scenes = {
 manual: {title:'看主线',summary:'操作输入 → 统一命令 → 分两路：本地小 Yaw / 串口到底盘。底盘再完成安全检查、解算和输出。',steps:[
  ['inputs','遥控服务接收输入；ManualCommandMapper 将摇杆 / 键鼠映射为操作意图。'],
  ['global_safety','全局安全先决定各输出域是否允许执行；许可作为 CommandManager 的输入。'],
  ['command_manager','CommandManager 把意图转换成带单位的 RobotCommand，并推进命令序号。'],
  ['command_router','Router 分出两路：云台目标留在本板，底盘目标发往串口。'],
  ['yaw_gimbal','本地分支：GimbalLocalSafety 检查许可和反馈，YawGimbal + PositionMotor 控制小 Yaw。'],
  ['async_uart','远端分支：底盘目标经 V1 编码和串口发送；底盘收到的是目标与许可，不是云台板计算的轮速。'],
  ['chassis_safety','底盘检查心跳、命令时效、恢复上下文和本地反馈，通过后才允许执行。'],
  ['swerve','SwerveChassis 解算四轮输出，经功率缩放后送入硬件适配器。'],
  ['chassis_hardware','DjiChassisHardware 用共享 CAN Bus 统一驱动八电机，并发布执行状态供云台判断。']
 ]},
 timeout:{title:'看隔离',summary:'串口掉线时，底盘本地禁止输出；云台独立判定底盘失联，健康的小 Yaw 可继续工作。',steps:[
  ['async_uart','场景条件：板间串口中断，无法依靠对端立即回传故障。以下是代码预期逻辑。'],
  ['chassis_safety','底盘根据自己的本地接收时间判断：心跳超过默认 100 ms，进入 Waiting / Disable 并清稳定计数。'],
  ['chassis_hardware','底盘撤回所有 Bus 的输出；这是软件输出暂停，实际停转时间需要实测。'],
  ['global_safety','云台也独立检查底盘心跳 / 反馈是否过期，只禁止 chassis，并显示 Degraded。'],
  ['yaw_gimbal','如果云台本地许可、反馈和硬件仍有效，小 Yaw 可以继续运行。全局急停仍会禁用全部输出。']
 ]},
 recovery:{title:'看恢复',summary:'供电与反馈恢复后，底盘先重新准备，再回传恢复上下文；云台产生新命令，底盘等待 3 条后重新使能。',steps:[
  ['chassis_hardware','反馈中断：暂停输出。供电许可恢复后，pollRecovery 周期性推进准备，先等新反馈。'],
  ['swerve','八电机反馈恢复并满足稳定条件后，重置测量参考和舵轮算法，不继续追赶旧目标。'],
  ['async_uart','底盘进入 Ready，推进 resume_generation，并用心跳回传 boot_id / generation。'],
  ['command_manager','云台从当前输入重新产生命令。Router 回显底盘的 boot_id 和 generation。'],
  ['chassis_safety','底盘确认上下文一致，并累计连续 3 条新的 producer sequence；重复旧命令不计数。'],
  ['chassis_hardware','本地条件全部满足后再 arm 并执行。普通掉线可以自动恢复；显式急停仍需明确复位。']
 ]}
};
const state={selected:'command_manager',scene:'manual',tab:'overview',step:-1,timer:null,filter:'all',query:''};
const escapeHTML=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const sourceUrl=p=>REPO+p.split('/').map(encodeURIComponent).join('/');
function badges(n){return n.status==='todo'?'<span class="badge todo">待实现</span>':'<span class="badge done">已有代码</span>'+(n.status==='verify'?'<span class="badge verify">待实机</span>':'');}
function moduleButton(id){const n=byId[id];return n?`<button data-module="${id}" class="${n.status==='todo'?'future':''}">${escapeHTML(n.title)}${n.status==='todo'?' · 待实现':''}</button>`:'';}
function renderDetail(){
 const n=byId[state.selected],p=plain[n.id];
 const downstream=n.provides.filter(id=>byId[id]?.status!=='todo');
 const future=n.provides.filter(id=>byId[id]?.status==='todo');
 document.getElementById('detail').innerHTML=`<h2 class="detail-title">${escapeHTML(p[0])}</h2><div class="class-name">${escapeHTML(n.title)}</div><div class="badges">${badges(n)}</div><p class="plain-description">${escapeHTML(p[1])}</p><div class="io-block"><span>INPUT / 输入</span><p>${escapeHTML(p[2])}</p></div><div class="io-block"><span>OUTPUT / 输出</span><p>${escapeHTML(p[3])}</p></div><div class="detail-tabs" role="group" aria-label="详情内容"><button data-tab="overview" class="${state.tab==='overview'?'on':''}" aria-pressed="${state.tab==='overview'}">关系与约束</button><button data-tab="code" class="${state.tab==='code'?'on':''}" aria-pressed="${state.tab==='code'}">接口与源码</button></div><div class="detail-body">${state.tab==='code'?`<h3>类接口 / 数据契约</h3>${n.interfaces.map(s=>`<code class="code">${escapeHTML(s)}</code>`).join('')}<h3>源码固定到 77b0538</h3><div class="sources">${n.sources.map(s=>`<a target="_blank" rel="noreferrer" href="${sourceUrl(s)}">${escapeHTML(s)} ↗</a>`).join('')}</div>`:`<h3>上游数据 / 协作模块</h3><div class="relations">${n.dependsOn.length?n.dependsOn.map(moduleButton).join(''):'<span class="class-name">底层数据或设备输入</span>'}</div>${downstream.length?`<h3>下游使用</h3><div class="relations">${downstream.map(moduleButton).join('')}</div>`:''}${future.length?`<h3>后续扩展</h3><div class="relations">${future.map(moduleButton).join('')}</div>`:''}<h3>必须遵守的约束</h3><ul class="constraints">${n.constraints.map(s=>`<li>${escapeHTML(s)}</li>`).join('')}</ul>`}</div>`;
 document.querySelectorAll('[data-module]').forEach(el=>el.setAttribute('aria-pressed',String(el.dataset.module===state.selected)));
 document.querySelectorAll('.flow-node').forEach(el=>el.classList.toggle('selected',el.id==='stage-'+stageMap[state.selected]));
}
function selectModule(id,scroll=false){if(!byId[id])return;state.selected=id;renderDetail();if(scroll)document.querySelector('.inspector').scrollIntoView({behavior:motion(),block:'start'});}
function motion(){return matchMedia('(prefers-reduced-motion: reduce)').matches?'instant':'smooth';}
function stop(){if(state.timer!==null)clearInterval(state.timer);state.timer=null;document.getElementById('play').textContent='▶ 逐步讲解';}
function setScene(scene){stop();state.scene=scene;state.step=-1;document.querySelectorAll('[data-scene]').forEach(el=>{el.classList.toggle('on',el.dataset.scene===scene);el.setAttribute('aria-pressed',String(el.dataset.scene===scene));});const s=scenes[scene];const summary=document.getElementById('sceneSummary');summary.innerHTML=`<b>${s.title}</b><span>${s.summary}</span>`;summary.classList.toggle('warn',scene!=='manual');document.querySelectorAll('.flow-node').forEach(el=>{el.classList.remove('current');el.classList.toggle('blocked',scene==='timeout'&&['stage-safety','stage-swerve','stage-motors'].includes(el.id));el.classList.toggle('healthy',scene==='timeout'&&el.id==='stage-yaw');});document.getElementById('stepText').textContent=scene==='manual'?'点击任意模块查看详情，或用「逐步讲解」沿两条分支走一遍。':'点击「逐步讲解」查看条件、判断和恢复顺序；这里展示的是代码逻辑。';document.getElementById('stepCount').textContent=`0 / ${s.steps.length}`;document.querySelector('.wire-label').textContent=scene==='timeout'?'链路中断 ×':'底盘命令 →';selectModule(scene==='manual'?'command_manager':scene==='timeout'?'chassis_safety':'chassis_hardware');drawWire();}
function advance(){const steps=scenes[state.scene].steps;if(state.step>=steps.length-1)state.step=-1;state.step++;const [id,explanation]=steps[state.step];selectModule(id);document.querySelectorAll('.flow-node').forEach(el=>el.classList.toggle('current',el.id==='stage-'+stageMap[id]));document.getElementById('stepText').textContent=explanation;document.getElementById('stepCount').textContent=`${state.step+1} / ${steps.length}`;if(state.step===steps.length-1)stop();}
function drawWire(){const area=document.getElementById('diagram'),svg=document.getElementById('linkSvg');if(innerWidth<=600){svg.innerHTML='';return;}const a=document.getElementById('stage-command').getBoundingClientRect(),b=document.getElementById('stage-safety').getBoundingClientRect(),r=area.getBoundingClientRect();const x1=a.right-r.left+1,y1=a.top-r.top+a.height/2,x2=b.left-r.left-4,y2=b.top-r.top+b.height/2,m=(x1+x2)/2;const color=state.scene==='timeout'?'#e3b16b':'#77abff';svg.setAttribute('viewBox',`0 0 ${r.width} ${r.height}`);svg.innerHTML=`<defs><marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto"><path d="M 0 0 L 10 5 L 0 10 z" fill="${color}"/></marker></defs><path class="signal ${state.scene==='timeout'?'closed':''}" d="M ${x1} ${y1} H ${m} V ${y2} H ${x2}" marker-end="url(#arrow)"/>`;}
function renderCatalog(){const q=state.query.trim().toLowerCase();const matches=nodes.filter(n=>(state.filter==='all'||state.filter==='implemented'&&n.status!=='todo'||n.status===state.filter)&&[n.title,n.summary,n.description,...n.interfaces,...n.sources,...plain[n.id]].join(' ').toLowerCase().includes(q));document.getElementById('empty').hidden=!!matches.length;document.getElementById('moduleList').innerHTML=lanes.map(l=>{const list=matches.filter(n=>n.lane===l.id);return list.length?`<div class="catalog-group"><h3>${l.title}${l.id==='chassis'?' / 共用电机控制层':''}</h3><div class="catalog-grid">${list.map(n=>`<button class="catalog-item" data-module="${n.id}" data-scroll="true"><strong>${escapeHTML(n.title)}</strong><small>${escapeHTML(plain[n.id][0])}</small><div class="badges">${badges(n)}</div></button>`).join('')}</div></div>`:'';}).join('');}
document.addEventListener('click',e=>{const el=e.target.closest('button');if(!el)return;if(el.dataset.module){stop();selectModule(el.dataset.module,el.dataset.scroll==='true'||innerWidth<=950||el.closest('.pending-links'));}if(el.dataset.tab){state.tab=el.dataset.tab;renderDetail();}if(el.dataset.scene)setScene(el.dataset.scene);if(el.dataset.filter){state.filter=el.dataset.filter;document.querySelectorAll('[data-filter]').forEach(b=>{b.classList.toggle('on',b===el);b.setAttribute('aria-pressed',String(b===el));});renderCatalog();}});
document.getElementById('search').addEventListener('input',e=>{state.query=e.target.value;renderCatalog();});
document.getElementById('play').addEventListener('click',()=>{if(state.timer!==null){stop();return;}if(state.step>=scenes[state.scene].steps.length-1)state.step=-1;advance();if(state.step<scenes[state.scene].steps.length-1){state.timer=setInterval(advance,4500);document.getElementById('play').textContent='Ⅱ 暂停讲解';}});
document.getElementById('next').addEventListener('click',()=>{stop();advance();});document.getElementById('reset').addEventListener('click',()=>setScene(state.scene));document.querySelector('.jump').addEventListener('click',()=>document.getElementById('catalogDetails').open=true);window.addEventListener('resize',()=>requestAnimationFrame(drawWire));document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});renderCatalog();setScene('manual');new ResizeObserver(drawWire).observe(document.getElementById('diagram'));
