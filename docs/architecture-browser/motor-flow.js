'use strict';
// This is an explanatory timeline, not a simulator or live CAN telemetry.
(() => {
  const root = document.getElementById('motor-workflow');
  const modules = {
    app: ['业务控制', '输入年龄 · 新使能请求', '应用线程', 'samples/robotics/gimbal_control/src/main.cpp'],
    control: ['Position / Velocity', '读反馈 → PID → effort', '调用 update 的线程', 'lib/control/motor_control.cpp'],
    motor: ['Motor', '暂存命令 · 反馈 · 代次', '业务 + I/O，短锁', 'drivers/motor/motor.cpp'],
    group: ['Group', '共同许可 · 故障联动', '请求线程 / 成员 I/O', 'drivers/motor/group.cpp'],
    publication: ['commit / 发布区', '完整命令快照 + sequence', 'commit 调用线程', 'drivers/motor/can_bus.cpp'],
    candidate: ['TxCandidate', '固定快照 → DJI/DM 编码', '本 CAN I/O', 'drivers/motor/can_bus.cpp'],
    authorize: ['最终发送授权', 'Group → Motor → TX 锁', '本 CAN I/O，短临界区', 'drivers/motor/can_bus.cpp'],
    can: ['Zephyr CAN', 'can_send / start / stop', '锁外调用底层驱动', 'drivers/motor/can_bus.cpp'],
    callback: ['RX / TX 回调', 'RX 队列 / TX 完成槽', 'ISR 或取消上下文', 'drivers/motor/can_bus.cpp'],
    feedback: ['反馈处理', '路由 → decode → 检查 gap', '本 CAN I/O', 'drivers/motor/motor.cpp'],
    completion: ['processTx', '按原代次结算真实动作', '本 CAN I/O', 'drivers/motor/can_bus.cpp'],
    recovery: ['期限与恢复', '超时撤销 · stop/start', '本 CAN I/O', 'drivers/motor/can_bus.cpp']
  };
  // [node, title, explanation, data, state, edges]
  const flows = {
    command: {title:'目标如何变成 CAN 帧', summary:'setter → commit → 完整候选 → 共同授权 → TX 完成。点击节点可查职责，下面的步骤显示数据在哪个线程中变化。', steps:[
      ['app','读取本轮输入','应用检查目标来源时间、输出许可和真实 dt。不同业务可以分线程，每个控制器仍由一个写入方负责。','输入：新鲜目标 + dt；条件：group.active()','Active',[]],
      ['control','控制算法在业务线程运行','PositionMotor / VelocityMotor 读取 Motor.snapshot()，执行限幅、PID 与前馈。新使能/参考代次的首轮输出零 effort。','输出：DJI A / DM MIT N·m','Active',[['app','control']]],
      ['motor','暂存目标，不立即发送','setter 检查命令和生产者身份，保存原始 written_ms、revision、enable_generation。update 不替应用 commit。','例：1 A，enable_generation = 7','Active',[['control','motor']]],
      ['publication','发布整条 CAN 的命令','commit 收集挂接端点，在 publication_lock 下发布完整数组并递增 sequence，然后唤醒 I/O。成功仅表示发布。','例：sequence = 42；written_ms 保留','Published',[['motor','publication']]],
      ['candidate','一次读取，锁外编码','候选保存命令和同一个 sequence，捕获端点状态及操作代次。新 commit 不会在编码中途替换某个 DJI 槽。','DJI 四槽 / DM 单帧；sequence = 42','Candidate',[['publication','candidate']]],
      ['authorize','共同检查许可并预留在途槽','按固定 Group 顺序 → Motor → TX 获取短锁，检查代次、许可与时效。失配重建，不给旧字节补上新代次。','每 CAN 最多一个已授权/在途帧','Authorized',[['candidate','authorize'],['group','authorize']]],
      ['can','释放锁，再调用 can_send','K_NO_WAIT 加完成回调。此后发生 disable，已获授权的这一帧可能残留，但后续仍需真实安全输出。','can_send(..., onTxDone, bus)','In flight',[['authorize','can']]],
      ['completion','回调记录，I/O 结算','onTxDone 不递归发送。processTx 核对总线代次，更新 last_tx 并继续下一帧；完成发送不等于机械动作完成。','TxResult { sequence, purpose, error }','TX complete',[['can','callback'],['callback','completion']]]
    ]},
    startup: {title:'启动与异步使能',summary:'先绑定全部端点，再启动物理 CAN；ready 和 active 含义不同，DM 还需要反馈及中性命令。',steps:[
      ['app','冻结拓扑','构造长期存活的 Motor / Group / CanBus，先 attach 所有跨 CAN 成员。每个物理 CAN 仅一个 owner。','attach → start → configure','Unstarted',[]],
      ['can','start 不开启运动','验证配置和 ID 冲突，安装过滤器、启动 CAN，再创建 I/O 线程并主动唤醒。','Bus = Running；Motor = Offline','Offline',[['app','can']]],
      ['candidate','建立安全状态','DJI 请求零槽帧；DM 请求 Disable。不能等一个尚不存在的反馈来触发首次探测。','stop generation 对应实际安全动作','Pending',[['can','candidate']]],
      ['feedback','接收真实反馈并累计稳定窗口','DJI 需安全发送完成，DM 需安全 TX 后的 Disabled 反馈。反馈持续稳定后 ready 成立，位置参考另行检查。','ready() ≠ active()','Disabled',[['callback','feedback'],['feedback','motor']]],
      ['group','应用显式 enable','Group 检查所有成员 ready，建立统一使能代次。返回 0 是请求受理；暂存旧命令失效。','enable_generation = 新代次','Enabling',[['app','group'],['group','motor']]],
      ['can','协议准备','DJI 再完成安全准备；DM 发 Enable，等待 TX 后 Enabled 反馈，再发模式对应中性命令。','MIT 零 effort / 速度零 / 原生位置 + 零速度','Enabling',[['candidate','authorize'],['authorize','can']]],
      ['group','全组成员准备完毕','memberPrepared 核对同一使能代次，全组准备完成才开放共同许可。下一控制周期产生新目标。','group.active() = true','Active',[['completion','group'],['group','app']]]
    ]},
    feedback: {title:'反馈、超时与位置参考',summary:'接收时间和处理时间不同；恢复帧不能抹掉已发生的间断。',steps:[
      ['callback','接收回调只保存事件','保存 frame、真实接收毫秒、bus generation 和 callback order。队列满则记录溢出并唤醒，不执行 PID。','RxEvent；固定容量队列','RX queued',[['can','callback']]],
      ['feedback','路由并解码','检查总线代次、标准帧与 DLC。DJI 按 CAN ID；DM 共享 Master ID 时再按帧内 motor ID 分发。','原始字节 → rad / rad/s / A / N·m','Decoded',[['callback','feedback']]],
      ['motor','覆盖时间戳前检查间断','例：上次 100 ms，时限 20 ms，新帧 121 ms。旧 Active/Enabling 已过期，必须先撤销许可。','121 − 100 > 20；相等尚未过期','Expired',[['feedback','motor']]],
      ['group','故障联动不依赖下一轮业务','端点锁外 trip，关闭同组许可并唤醒成员所在 CAN。独立组不因单端点掉线而被连带。','Group.last_fault.source_motor','Revoked',[['motor','group'],['group','recovery']]],
      ['motor','恢复观察与坐标分开','新帧重建稳定窗口，但不会自动恢复运动或连续位置参考。位置控制按可信机械参考 reseed，再 reset。','snapshot；reference_generation','Recovering',[['feedback','motor'],['motor','control']]]
    ]},
    stop: {title:'同帧独立电机的停机竞态',summary:'重现 F1：为 B 构造安全帧时 A 仍非零，随后 A 请求停机。旧帧不能确认 A 的新停机。',steps:[
      ['candidate','构造 B 的安全帧','A、B 共享 DJI 命令 ID，但属于独立组。候选 A = 1 A，B = 0；只记录 B 的安全动作。','A stop = 10；B stop = 20','Candidate',[]],
      ['group','A 在授权前请求停机','A 许可关闭，stop generation 由 10 变成 11，新的安全请求持久保存，不依赖 commit。','A stop = 11；Pending','Revoked',[['app','group'],['group','motor']]],
      ['authorize','旧候选被拒绝','共同授权发现代次不匹配，内部 -EAGAIN。不能用旧 A = 1 A 字节配上新的 stop = 11。','不调用 can_send；等待重建','Retry',[['candidate','authorize'],['motor','authorize']]],
      ['candidate','重建真实安全槽','重新读取状态后 A = 0，B = 0；候选明确携带各自的停机代次，且命令/批次仍一致。','A = 0 / stop 11；B = 0 / stop 20','Safe candidate',[['authorize','candidate']]],
      ['completion','只结算实际安全动作','对应 TX 完成后才能置 TxComplete。若 disable 在授权之后发生，允许一帧残留，但那帧不完成后来的停机。','新停机必须等新安全动作','TxComplete',[['candidate','can'],['can','callback'],['callback','completion']]]
    ]},
    recovery: {title:'CAN 故障与快速恢复',summary:'CAN 恢复、反馈稳定、位置参考和业务解锁是不同阶段。总线恢复不自动运动。',steps:[
      ['recovery','检测 CAN 故障','发送错误、RX 溢出或控制器异常使总线进入 Recovering。清发布区、推进 bus generation，撤销本 CAN 端点。','TransportError / RxOverflow','Recovering',[['callback','recovery']]],
      ['group','传播相关组的撤销','跨 CAN Group 关闭共同许可，唤醒另一 CAN；另一 CAN 的独立组仍可继续服务。','故障域：CAN + 相关 Group','Revoked',[['recovery','group'],['group','motor']]],
      ['can','终结旧发送，再重启','stop/取消尚未完成时不复用在途上下文，按重试期限等待。成功 start 后仍请求安全输出。','旧回调不能推进新 bus generation','Disabled',[['recovery','can']]],
      ['feedback','快速反馈也重新计时','旧稳定起点已清零，即使新帧与上帧间隔小于 timeout，首个合格恢复帧仍建立新稳定起点。','stable_since = 本次恢复帧接收时间','Stabilizing',[['callback','feedback'],['feedback','motor']]],
      ['app','应用重新解锁','确认安全准备与稳定反馈，必要时重建位置参考/reset，然后按新的业务许可显式 enable，产生新目标。','不重放旧 published / staged 命令','Ready, not active',[['motor','app'],['app','group']]]
    ]},
    fault: {title:'锁存 Fault 与显式清除',summary:'无效命令等需要确认的故障，不会被后来的通信恢复覆盖。',steps:[
      ['motor','锁存需要清除的根因','InvalidCommand、DriveFault、ControlRejected 等保存锁存故障并关闭输出，再在锁外通知 Group。','例：InvalidCommand / -EINVAL','Fault',[]],
      ['recovery','随后又发生通信故障','继续失效通信与位置参考，但 Motor 保持 Fault 和原根因。BusStatus 记录总线错误。','Motor = Fault；Bus = Recovering','Fault',[['motor','recovery']]],
      ['feedback','通信恢复不清 Fault','即使反馈稳定、安全准备完成，ready 仍不成立，enable 返回故障条件；不能绕过用户确认。','需要显式 clearFault','Fault',[['callback','feedback'],['feedback','motor']]],
      ['group','显式清故障请求','组内调用 group.clearFault，独立端点调用 motor.clearFault。DM 发 ClearError 并等待后续 Disabled 反馈。','clear 请求对应自己的 stop generation','Clearing',[['app','group'],['group','can']]],
      ['completion','完成必须匹配原请求','旧 ClearError 完成不能确认被 disable 取消后的新清故障请求。清除后重新安全准备，等待 ready，再显式 enable。','代次匹配 → 清锁存；不会直接 Active','Disabled / Offline',[['callback','completion'],['completion','motor']]]
    ]}
  };
  const examples = {
    dji: {label:'DJI 同 CAN',note:'初始化和周期是不同阶段。电机须已配置、反馈稳定且业务许可有效；所有返回值都应处理。完整可复用函数见 docs/17。',code:`// 静态生命周期；两台 Motor 已按实际硬件配置。
static motor::Group axes(yaw_drive, pitch_drive);
static motor::CanBus bus(board_config::yaw_can);
int ret = bus.attach(yaw_drive, pitch_drive);
if (ret == 0) ret = bus.start();
// 初始化失败时停止进入运行流程。

// 明确的新使能动作：
if (new_enable_request && axes.ready())
    ret = axes.enable(); // 受理不等于 Active

// 后续 active 周期：
if (axes.active()) {
    ret = yaw_drive.setCurrent(yaw_a);
    if (ret == 0) ret = pitch_drive.setCurrent(pitch_a);
    if (ret == 0) ret = bus.commit().error;
    if (ret < 0) axes.disable();
}`},
    split: {label:'跨 CAN 云台',note:'和 gimbal_control 当前写法一致。同 CAN 时不启动 pitch_bus。两次 commit 不保证同步到达电机。',code:`const bool split = board_config::yaw_can != board_config::pitch_can;
int ret = split ? yaw_bus.attach(yaw_drive)
                : yaw_bus.attach(yaw_drive, pitch_drive);
if (ret == 0 && split) ret = pitch_bus.attach(pitch_drive);
if (ret == 0) ret = yaw_bus.start();
if (ret == 0 && split) ret = pitch_bus.start();
// 检查 ret；配置两轴，等待 ready、参考有效，再显式 enable。

// 只有 gimbal.active() 时执行：
ret = yaw.update(yaw_rate, remote.stamp, dt);
if (ret == 0) ret = pitch.update(pitch_rate, remote.stamp, dt);
if (ret == 0) ret = yaw_bus.commit().error;
if (ret == 0 && split) ret = pitch_bus.commit().error;
if (ret < 0) gimbal.disable();`},
    dm: {label:'DM / 控制器',note:'根据持久模式选择 setter。软件 PID 绑定后，不再直接 setter 同一个 Motor；组内使用 Group 管理生命周期。',code:`// 已 start、完成模式准备，并且 drive.active()。
int ret = drive.setTorque(torque_nm); // MIT 前馈力矩
// 原生速度模式改用 setVelocity(rad_s)。
// 原生位置速度模式改用 setPositionVelocity(rad, vmax)。
if (ret == 0) ret = bus.commit().error;
if (ret < 0) (void)drive.disable(); // 组内用 group.disable()

// 若绑定了软件 PositionMotor / VelocityMotor：
// start → axis.configure → ready → axis.reset → enable → active
ret = axis.update(target, actual_dt_s); // 代替上述 setter
if (ret == 0) ret = bus.commit().error;
if (ret < 0) (void)drive.disable();
// 恢复后：必要时 reseedPosition，再 reset，显式 enable。`},
    threads: {label:'多线程协调',note:'这是组织方式伪代码。线程总数不固定，但同一控制器只由一个任务 update/reset。不能只锁 commit 就认为多个 setter 构成同一业务批次。',code:`计算线程：
    计算目标，保留源消息 timestamp / sequence
    发布目标快照（不碰别的任务的 PID）

指定发布者：
    读取各目标快照并检查输入年龄
    过期 → 禁止相关组；不把旧消息反复写成新目标
    更新自己拥有的控制器 / setter
    对共享 CAN 统一 commit

另一种方式：共同应用 mutex 保护 setter/update + commit
    所有生产者遵守同一个锁约定
    锁外等待周期、反馈及 TX
    需要 yaw/pitch 同周期配对时再加批次协调

不同 CAN：各任务可以独立更新与 commit
同一 CAN：始终保留唯一 owner 和统一发送授权`}
  };
  let scene = 'command', step = 0, timer = null, example = 'dji';
  const el = id => document.getElementById(id);
  const esc = text => String(text).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const lanes = [
    ['业务与端点',['app','control','motor','group']],
    ['发布与发送',['publication','candidate','authorize','can']],
    ['事件与恢复',['callback','feedback','completion','recovery']]
  ];
  el('motor-map').innerHTML = '<svg id="motor-wires" aria-hidden="true"></svg>' + lanes.map(([title, ids]) => `<div class="motor-lane"><h3>${title}</h3>${ids.map(id => `<button class="motor-node" id="motor-node-${id}" data-motor-node="${id}"><strong>${modules[id][0]}</strong><small>${modules[id][1]}</small><span>${modules[id][2]}</span></button>`).join('')}</div>`).join('');
  function draw() {
    const svg = el('motor-wires'), bounds = el('motor-map').getBoundingClientRect();
    svg.setAttribute('viewBox', `0 0 ${bounds.width} ${bounds.height}`);
    svg.innerHTML = '<defs><marker id="motor-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto"><path d="M0 0 L10 5 L0 10Z" fill="#7bcab0"/></marker></defs>' + flows[scene].steps[step][5].map(([a,b]) => {
      const ar = el('motor-node-'+a).getBoundingClientRect(), br = el('motor-node-'+b).getBoundingClientRect();
      const same = Math.abs(ar.left - br.left) < 5;
      const forward = same ? ar.top < br.top : ar.left < br.left;
      // Route same-column calls through the gutter, never through the
      // unrelated nodes between source and destination.
      if (same) {
        const x1=ar.right-bounds.left, x2=br.right-bounds.left;
        const y1=ar.top+ar.height/2-bounds.top, y2=br.top+br.height/2-bounds.top;
        return `<path d="M${x1},${y1} H${Math.max(x1,x2)+12} V${y2} H${x2}"/>`;
      }
      const x1=(forward?ar.right:ar.left)-bounds.left;
      const x2=(forward?br.left:br.right)-bounds.left;
      const y1=ar.top+ar.height/2-bounds.top, y2=br.top+br.height/2-bounds.top;
      if (Math.abs(ar.left-br.left)>ar.width*1.8) {
        const out1=x1+(forward?12:-12), out2=x2+(forward?-12:12);
        return `<path d="M${x1},${y1} H${out1} V28 H${out2} V${y2} H${x2}"/>`;
      }
      return `<path d="M${x1},${y1} C${(x1+x2)/2},${y1} ${(x1+x2)/2},${y2} ${x2},${y2}"/>`;
    }).join('');
  }
  function inspect(id) {
    const m = modules[id];
    el('motor-module').innerHTML = `<strong>${esc(m[0])}</strong><span>${esc(m[2])}</span><p>${esc(m[1])}</p><code>${esc(m[3])}</code>`;
    root.querySelectorAll('[data-motor-node]').forEach(b => b.setAttribute('aria-pressed', String(b.dataset.motorNode===id)));
  }
  function render() {
    const f = flows[scene], [id,title,text,data,status] = f.steps[step];
    el('motor-summary').textContent = f.summary;
    el('motor-step-title').textContent = title;
    el('motor-step-text').textContent = text;
    el('motor-data').textContent = data;
    el('motor-state').textContent = status;
    el('motor-count').textContent = `${step+1} / ${f.steps.length}`;
    el('motor-prev').disabled = step===0;
    el('motor-next').disabled = step===f.steps.length-1;
    root.querySelectorAll('[data-motor-scene]').forEach(b => { const on=b.dataset.motorScene===scene;b.classList.toggle('on',on);b.setAttribute('aria-pressed',String(on)); });
    root.querySelectorAll('[data-motor-node]').forEach(b => b.classList.toggle('current', b.dataset.motorNode===id));
    el('motor-timeline').innerHTML=f.steps.map((s,i)=>`<button data-motor-step="${i}" class="${i===step?'on':''}" aria-current="${i===step?'step':'false'}"><span>${String(i+1).padStart(2,'0')}</span>${esc(s[1])}</button>`).join('');
    inspect(id); requestAnimationFrame(draw);
  }
  function stop() { if(timer!==null)clearInterval(timer);timer=null;el('motor-play').textContent='▶ 播放链路'; }
  function showExample() { el('motor-code').textContent=examples[example].code;el('motor-example-note').textContent=examples[example].note;root.querySelectorAll('[data-motor-example]').forEach(b=>{b.classList.toggle('on',b.dataset.motorExample===example);b.setAttribute('aria-pressed',String(b.dataset.motorExample===example));});el('motor-copy-status').textContent=''; }
  root.addEventListener('click', e => {
    const b=e.target.closest('button');if(!b)return;
    if(b.dataset.motorScene){stop();scene=b.dataset.motorScene;step=0;render();}
    if(b.dataset.motorStep!==undefined){stop();step=Number(b.dataset.motorStep);render();}
    if(b.dataset.motorNode){stop();inspect(b.dataset.motorNode);}
    if(b.dataset.motorExample){example=b.dataset.motorExample;showExample();}
  });
  el('motor-prev').addEventListener('click',()=>{stop();step=Math.max(0,step-1);render();});
  el('motor-next').addEventListener('click',()=>{stop();step=Math.min(flows[scene].steps.length-1,step+1);render();});
  el('motor-reset').addEventListener('click',()=>{stop();step=0;render();});
  el('motor-play').addEventListener('click',()=>{
    if(timer!==null){stop();return;}
    if(step===flows[scene].steps.length-1)step=0;
    render();el('motor-play').textContent='Ⅱ 暂停链路';
    timer=setInterval(()=>{step++;render();if(step===flows[scene].steps.length-1)stop();},4000);
  });
  el('motor-copy').addEventListener('click',async()=>{
    try { await navigator.clipboard.writeText(examples[example].code);el('motor-copy-status').textContent='已复制'; }
    catch { const range=document.createRange();range.selectNodeContents(el('motor-code'));const selection=window.getSelection();selection.removeAllRanges();selection.addRange(range);el('motor-copy-status').textContent='已选中代码，请使用系统复制快捷键'; }
  });
  document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});
  new ResizeObserver(draw).observe(el('motor-map'));
  render();showExample();
})();
