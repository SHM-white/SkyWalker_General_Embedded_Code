/* Hardware and control API reference. Paths are repository relative. */
(() => {
  const api = (signature, description, parameters, returns, context, errors) => ({
    signature, description, parameters: parameters.map(([name, meaning]) => ({ name, meaning })),
    returns, context, errors
  });
  const thread = '应用线程调用；把同一总线的 setter/update → commit 交给一个控制周期所有者。CAN 中断回调只入队/记录完成，解码、超时和真正发送由 CanBus 的 I/O 线程处理。';
  const commonMotor = () => [
    api('MotorInfo Motor::info() const', '查询硬件能力位、允许的电流/力矩上限与超时参数；能力表示支持什么，反馈 valid 表示本次哪些字段可用。', [], '返回配置快照，不访问 CAN。', '任意读取线程；构造后的配置不变化。', '无错误码；不等于已连接或已使能。'),
    api('MotorSnapshot Motor::snapshot() const', '复制反馈与生命周期：state、feedback_fresh、output_permitted、位置参考代次、last_fault 和停机 stop 报告。连续位置以首帧为 0 rad，断联后位置参考会失效。', [], '值拷贝；读取时重新计算反馈新鲜度，过期时强制令副本 output_permitted=false。', '允许多个读取线程；使用自旋锁保护副本，无 CAN I/O。', '检查 feedback_fresh 和 feedback.valid；不要仅看数值非零或 state=Active。'),
    api('bool Motor::ready() const; bool Motor::active() const', 'ready 表示已完成安全输出、反馈持续稳定并可申请使能；active 表示当前允许运动输出，还会核对所属 Group 的共同权限。', [], '布尔值；enable() 返回 0 后仍须等 active()。', '控制线程和状态读取线程。', 'false 不是错误码；到 snapshot()/bus.status() 找原因。'),
    api('int Motor::enable()', '申请异步使能，从 Disabled/Offline 的就绪状态进入 Enabling，再由协议流程进入 Active。加入 Group 的电机必须由 Group 统一使能。', [], '0=请求已接收，不表示驱动器已完成使能。', thread, '-EACCES：总线未启动或电机属于 Group；-EAGAIN：尚未就绪；-EALREADY：已使能/正在使能；-EIO：故障未清；-ERANGE/-ENODATA：已绑定控制器的反馈保护拒绝；-EOVERFLOW：代次耗尽。'),
    api('int Motor::disable()', '立即撤销软件输出权限，清掉暂存目标，并异步请求安全输出。DJI 发零电流，DM 发禁用命令；Group 成员须改用 Group::disable()。', [], '0=请求已接受。完成状态从 snapshot().stop 读取。', thread, '-EACCES：总线未启动或属于 Group。CAN 停机错误在 stop.tx_error；通信中断时不能把软件禁用当作电机已确认停止。'),
    api('int Motor::clearFault()', '电机停用时申请清除锁存故障；清故障后仍须等 ready 并重新 enable。反馈/传输恢复不会代替对驱动故障或非法命令的确认。', [], '0=清除请求已提交；过程异步。', thread, '-EACCES：总线未启动或属于 Group；-EBUSY：Active/Enabling；-EALREADY：无锁存故障或清除已经在进行。'),
    api('int Motor::reseedPosition(double known_position_rad)', '在已停用、反馈新鲜时，把连续位置重设为已知机械位置，并增加 reference_generation；用于断联后重新找零或位置标定，不向电机写入固件零点。', [['known_position_rad', '已确认的输出轴位置，rad；必须有限并可表示。']], '0=位置参考已建立。', '控制周期所有者调用；不能在 Active/Enabling 时调用。', '-EINVAL：非有限数；-ERANGE：范围/编码累计溢出；-ENOTSUP：无位置反馈；-EBUSY：运行中；-EAGAIN：反馈未新鲜；-EOVERFLOW：参考代次耗尽。'),
    api('CanBus::CanBus(const device *can, BusOptions options = {})', '一个物理 CAN 控制器配一个共享总线对象；DJI 和 DM 可以挂同一个对象。构造只保存设备并初始化内部信号量。', [['can', 'DEVICE_DT_GET 得到的物理 CAN device。'], ['options', 'tx_timeout_ms 默认 2 ms；恢复失败后 recovery_retry_ms 默认 100 ms。']], '构造对象，不启动 CAN。', '启动阶段；总线、电机、Group 都必须活到固件结束，不提供运行中析构/停止接口。', '设备就绪、配置与拓扑在 start() 时检查。'),
    api('int CanBus::attach(Motor &motor); template<class... Motors> int CanBus::attach(Motor &first, Motor &second, Motors &...rest)', '启动前把一个或一批电机接入同一物理总线；批量登记先检查整批再绑定。', [['motor / first / second / rest', '长寿命电机对象引用，不能重复登记，也不能已绑定别的总线。']], '0=全部绑定。', '单一初始化线程，在 start() 前完成；拓扑运行中不可变。', '-EACCES：总线已离开 Unstarted；-EALREADY：重复或已经绑定；-ENOSPC：超过每总线容量。'),
    api('int CanBus::start()', '验证型号、ID、限幅、Group 与 TX/RX 冲突，注册接收过滤器，独占并启动 CAN，再创建 I/O 线程；启动后先准备安全输出而非直接允许运动。', [], '0=总线线程已启动；电机还需反馈稳定。', '初始化线程，会调用 Zephyr CAN API；不从中断调用。', '-ENODEV：设备不可用；-EINVAL/-ERANGE/-ENOTSUP：配置错误；-EADDRINUSE：CAN ID 冲突；-EBUSY：控制器已有所有者或已被别人启动；-ENOSPC：全局总线/过滤器资源不足；-EALREADY：已运行。也透传 CAN 驱动错误。'),
    api('CommitResult CanBus::commit()', '把该总线所有电机的暂存目标一起发布给 I/O 线程。后发布的批次可以覆盖尚未发送完的旧批次；这是最新目标发布机制，不是逐条可靠命令队列。', [], 'CommitResult{error, sequence}；error=0 仅表示发布成功，sequence 用于对照发送诊断。', thread, '-EAGAIN：Recovering；-EACCES：总线未运行；-EOVERFLOW：序号耗尽。发送完成另看 status().last_tx，不能把 commit 成功当成所有电机已收到。'),
    api('BusStatus CanBus::status() const', '查询 Running/Recovering/ConfigBlocked、最近发送的 purpose/error/sequence、覆盖批次数 superseded_batches、接收溢出和非法帧计数。', [], '诊断副本；last_tx 是最近一帧，不是整批完成凭据。', '任意诊断线程，无阻塞 CAN 操作。', 'last_error 与 last_tx.error 是底层发送/恢复错误；rx_overflows 增长表示 I/O 消费能力不足。'),
    api('template<class... Others> Group::Group(Motor &first, Others &...others)', '建立共同使能/共同停机的机械协作组；可以跨 CAN，总线发布仍需分别 commit。某成员故障会撤销全组运动权限。', [['first / others', '成员电机引用；每个电机只能属于一个 Group，必须在任一总线启动前组装。']], '构造 Group，暂不使能。', '初始化线程；Group 与成员均须全固件生命周期存活。', '构造时不能返回错误；重复归属、超容量等由总线 start() / Group::enable() 拒绝。'),
    api('bool Group::ready() const; bool Group::active() const; GroupStatus Group::status() const', 'ready 要求所有成员就绪；active 要求全组处于同一使能代次且每个成员有效。status 可指出 blocking_member、enable_pending 和根故障。', [], '布尔值或诊断副本；跨成员副本不表示硬件同一时刻状态。', '控制/诊断线程，读取中不执行 CAN 操作。', 'blocking_member 是阻塞成员指针；last_fault.source_motor 是最初触发成员。'),
    api('int Group::enable(); void Group::disable(); int Group::clearFault()', 'enable 预检全组并申请同一代次，仅在每个成员准备完成后开放共同权限；disable 立即撤销全组权限并请求每个电机停机；clearFault 在停用时为故障成员申请清除。', [], 'enable/clearFault 返回 0=申请成功；disable 无返回值。逐成员停机进度仍从 MotorSnapshot 查询。', '控制线程；跨总线不是物理同时发送。', 'enable：-EAGAIN 未就绪/-EALREADY 已申请/-EIO 锁存故障/-EACCES 未绑定或未启动/-EINVAL/-ENOSPC 拓扑错误；clearFault：-EBUSY 运行/使能/停机中，并返回首个成员错误。')
  ];
  window.SKYWALKER_MODULES = (window.SKYWALKER_MODULES || []).concat([
    {
      id: 'motor-dji', title: 'DJI 电机与共享 CAN 总线', category: '驱动与感知',
      summary: '以安培发布目标电流，统一接收 GM6020 / M3508 / M2006 的输出轴反馈。',
      responsibility: '协议层负责编解码；Motor 保存反馈、命令与故障状态；CanBus 独占物理 CAN 并负责后台 I/O；Group 处理协作电机的共同权限。位置/速度闭环交给 motor-control。',
      status: 'ready', statusNote: '驱动和 CAN/Group 生命周期已实现并有样例。可由云台/底盘应用接入；当前整车仍需确认 board_config 接线、型号和限幅后启用，不能据模块可用宣称整车已上车。',
      source: [{label:'统一电机 API',path:'include/drivers/motor/motor.hpp'},{label:'共享 CAN API',path:'include/drivers/motor/can_bus.hpp'},{label:'Group API',path:'include/drivers/motor/group.hpp'},{label:'DJI 型号工厂',path:'include/drivers/motor/dji_motor.hpp'},{label:'生命周期实现',path:'drivers/motor/motor.cpp'},{label:'协议实现',path:'drivers/motor/dji/dji_protocol.cpp'},{label:'完整驱动样例',path:'samples/motor/dji_unified/src/main.cpp'}],
      docs: [{label:'DJI Markdown',path:'docs/modules/drivers/motor-dji.md'},{label:'电机操作顺序',path:'docs/guides/motor-workflow.md'}],
      depends: ['boards'],
      interfaces: [
        api('dji::Config dji::gm6020(const Gm6020Options &options); dji::Config dji::m3508(const M3508Options &options); dji::Config dji::m2006(const M2006Options &options)', '构建不同型号的值配置，然后传给 Motor；工厂不创建设备、不发 CAN、不验证全部选项。', [['options.id','GM6020 为 1–7，其余为 1–8。'],['options.current_limit_a','软件电流上限：GM6020 ≤3 A，M3508 ≤20 A，M2006 ≤10 A，均须 >0。'],['options.gear_ratio','M3508 默认 19，M2006 默认 36；电机轴/输出轴转速比，必须 >0。'],['GM6020 专有选项','encoder_zero_ticks 0–8191 定义绝对零点；current_mode_confirmed 须确认固件电流模式后置 true。'],['options.timing','反馈、命令、稳定恢复、使能超时，单位 ms，全部须 >0。']], '返回 dji::Config；Motor(dji::Config) 保存一份配置。', '初始化阶段纯函数；构造后的 Motor 不可拷贝/移动。', '配置错误在 CanBus::start() 暴露；GM6020 未确认电流模式为 -EINVAL。'),
        api('int Motor::setCurrent(float ampere)', '暂存目标电流；在下一次 bus.commit() 后才允许 I/O 线程发送。GM6020 不支持把电压模式的原始数值冒充安培命令。', [['ampere','有符号电流，A；绝对值不超过配置 current_limit_a。']], '0=暂存成功。', thread, '-EACCES：电机未 Active/权限被撤销/控制器已独占输出；-EINVAL：非有限数；-ERANGE：超限；-ENOTSUP：不是电流型电机。Active 下非法命令会触发故障并停机。'),
        api('int dji::describe(const Config &config, Descriptor &out)', '计算反馈 CAN ID、命令帧 ID 和帧内槽位。GM6020 反馈 0x204+id，命令 0x1FE/0x2FE；M3508/M2006 反馈 0x200+id，命令 0x200/0x1FF。', [['config','DJI 配置。'],['out','成功时写入描述；command_slot 为 0–3，同一帧最多四电机。']], '0=成功；描述不代替完整配置验证。', '无 I/O 的纯调用；通常应用不必自己映射 ID。', '-ERANGE：型号/电机 ID 不合法。不同协议 TX/RX 冲突最终由总线 start() 检查。'),
        api('bool dji::decodeFeedback(const can_frame &frame, RawFeedback &out); int dji::buildCommandFrame(can_frame &frame, uint16_t command_id, const int16_t command_raw[4])', '底层协议工具：解码编码器/RPM/原始电流/温度；把四个原始电流槽位编码成命令帧。正常应用使用 Motor，避免绕过权限和限幅。', [['frame','标准 CAN 数据帧。'],['out','成功解码后的原始协议数据，不是 SI 单位输出轴反馈。'],['command_id','DJI 的合法组帧 ID。'],['command_raw','四个槽位的 int16 原始值，按高字节在前编码。']], 'decodeFeedback 返回是否帧合法；buildCommandFrame 返回 0 或负 errno。', '纯编解码，不执行 CAN 发送；驱动内部在 I/O 线程调用。', '非法帧 decode 为 false；不由这些工具维护 freshness/Group/命令超时。')
      ].concat(commonMotor()),
      examples: [{ title:'单电机：启动 → 等反馈 → 使能 → 电流发布 → 停机', language:'cpp', code:`#include <drivers/motor/can_bus.hpp>
#include <zephyr/kernel.h>
#include <cerrno>
using namespace skywalker::motor;

int main() {
  // 示例参数：必须先确认实物为电流模式 GM6020、ID=4。
  static Motor motor{dji::gm6020({.id=4, .current_limit_a=0.5f,
      .encoder_zero_ticks=0, .current_mode_confirmed=true})};
  static CanBus bus{DEVICE_DT_GET(DT_NODELABEL(can1))};
  int r = bus.attach(motor);
  if (!r) r = bus.start();
  if (r < 0) return r;
  auto deadline = k_uptime_get() + 3000;
  while (!motor.ready() && k_uptime_get() < deadline) k_msleep(1);
  if (!motor.ready()) return -ETIMEDOUT;
  r = motor.enable();
  if (r < 0) return r;
  deadline = k_uptime_get() + 1000;
  while (!motor.active() && k_uptime_get() < deadline) k_msleep(1);
  if (!motor.active()) { (void)motor.disable(); return -ETIMEDOUT; }
  for (int cycle=0; cycle<100; ++cycle) {
    r = motor.setCurrent(0.1f); // A，单个控制线程写目标
    if (!r) r = bus.commit().error;
    if (r < 0) break;
    k_msleep(5); // 小于 command_timeout_ms，持续刷新目标
  }
  const int stop = motor.disable();
  return r < 0 ? r : stop;
}`, notes:'对象必须保持 static：后台线程/回调在 main 返回后仍引用它们。示例电流、ID、零点为台架占位，不是整车参数；停机返回只代表申请，实际进度读 motor.snapshot().stop。开启 CAN、CPP、SKYWALKER_DRIVER_MOTOR、SKYWALKER_MOTOR_DJI，并确保 can1 节点 okay。'}],
      lifecycle: ['静态构造 Motor/CanBus；先组装可选 Group，再 attach 全部电机。','start() 验证物理资源、CAN ID 与配置，后台先发安全输出。','反馈新鲜且持续稳定后 ready()；单电机或 Group 申请 enable()，等 active()。','每周期 setCurrent()/闭环 update()，随后每条总线各 commit()。','命令或反馈超时撤销输出；故障诊断后按条件恢复反馈、重新找零、clearFault 和重新使能。','停机用 disable()；DJI 的 TxComplete 表示零电流帧完成发送，不能等同驱动器有单独停机 ACK。'],
      pitfalls: ['位置、速度已换算到输出轴：rad、rad/s；原始 RPM 不可直接拿去喂 PID。','M2006 无有效温度能力，启用温度保护的封装会返回 -ENOTSUP。','DJI 编码器展开依赖相邻反馈变化小于半圈；断联后连续位置参考失效，位置应用应重新找零。','一个物理 CAN 只能有一个启动所有者；同总线多个控制线程必须协调 update+commit 的整个批次。','Group 只统一权限和故障传播，跨 CAN 仍要分别 commit；它不保证物理同时发送。'],
      config: [{name:'CONFIG_SKYWALKER_DRIVER_MOTOR / CONFIG_SKYWALKER_MOTOR_DJI',description:'纳入统一电机子系统与 DJI 协议。还需要 CAN、CPP，应用样例使用 C++20。'},{name:'Timing',description:'默认反馈20 ms / 命令10 ms / 稳定恢复20 ms / 使能100 ms；实际控制周期必须落在命令有效期内。'},{name:'MAX_MOTORS_PER_BUS / MAX_BUSES / MAX_GROUP_MEMBERS / RX_QUEUE_DEPTH',description:'对应 CONFIG_SKYWALKER_MOTOR_*；默认16 / 3 / 16 / 32，限制静态资源容量。'},{name:'IO_PRIORITY / IO_STACK_SIZE',description:'默认抢占优先级5、栈3072字节；每条总线一个 I/O 线程。'}]
    },
    {
      id:'motor-dm', title:'达妙电机：MIT / 速度 / 位置速度', category:'驱动与感知',
      summary:'J4310-2EC-V1.1 三种持久化模式，统一力矩、反馈、使能与故障流程。',
      responsibility:'使用同一 Motor/CanBus/Group。MIT 模式可发纯力矩或位置/速度/增益/前馈组合；速度和位置速度模式由驱动器闭环。主控侧闭环封装只使用 MIT 的力矩输出。',
      status:'ready', statusNote:'J4310 三模式协议及恢复流程已实现，有独立样例和混合拓扑样例；云台应用可接入。上车前仍须确认驱动器已保存模式、Master ID、PMAX/VMAX/TMAX 与机械零点，当前配置不是已验收整车参数。',
      source:[{label:'DM 型号配置',path:'include/drivers/motor/dm_motor.hpp'},{label:'DM 协议 API',path:'include/drivers/motor/dm_protocol.hpp'},{label:'协议实现',path:'drivers/motor/dm/dm_protocol.cpp'},{label:'统一电机状态实现',path:'drivers/motor/motor.cpp'},{label:'MIT 完整样例',path:'samples/motor/dm_mit_control/src/main.cpp'},{label:'DM 台架生命周期辅助',path:'samples/motor/dm_mit_control/src/dm_sample_support.cpp'}],
      docs:[{label:'DM Markdown',path:'docs/modules/drivers/motor-dm.md'},{label:'三模式台架说明',path:'samples/motor/DM_J4310_EXAMPLES.md'}], depends:['boards'],
      interfaces:[
        api('dm::Config dm::j4310Mit(const J4310Options &options); dm::Config dm::j4310Velocity(const J4310Options &options); dm::Config dm::j4310PositionVelocity(const J4310Options &options)', '构建 MIT、速度或位置速度模式配置；主控工厂不会把驱动器固件自动切换模式。', [['options.id','电机 ID 1–15。'],['options.master_id','驱动器反馈的标准 CAN ID 0–0x7FF；不能与总线 TX ID 冲突。'],['position_max_rad / velocity_max_rad_s / torque_max_nm','真实固件 PMAX/VMAX/TMAX，须为正且匹配工具中已保存的值。'],['torque_limit_nm','应用软件力矩上限 >0 且 ≤TMAX。'],['timing','默认 50/20/50/3000 ms：反馈/命令/稳定恢复/使能。']], '返回 dm::Config；交给 Motor(dm::Config)。', '初始化阶段纯配置；对象全固件生命周期存活。', '全部配置验证在 CanBus::start()；不支持型号/模式 -EINVAL；范围错误 -ERANGE。'),
        api('int Motor::setTorque(float newton_meter)', 'MIT 模式纯力矩目标；内部令位置、速度、kp、kd 为 0，只使用力矩前馈。与主控 VelocityMotor/PositionMotor 的输出匹配。', [['newton_meter','有符号 N·m，绝对值 ≤torque_limit_nm。']], '0=暂存成功；之后仍需 commit。', thread, '-ENOTSUP：不是 MIT 模式；-EINVAL：非有限数；-ERANGE：超力矩限幅；-EACCES：未 Active 或输出已被封装控制器独占。非法 Active 命令触发锁存故障。'),
        api('int Motor::setMit(const dm::MitCommand &command)', 'MIT 组合命令交给驱动器内部控制：目标位置/速度、位置刚度 kp、速度阻尼 kd 和力矩前馈。', [['command.position_rad','原生电机位置目标，rad，绝对值 ≤PMAX；不等同 Motor 累计位置的重新标定坐标。'],['command.velocity_rad_s','速度目标，rad/s，绝对值 ≤VMAX。'],['command.kp / kd','协议增益，kp 0–500，kd 0–5。'],['command.torque_ff_nm','力矩前馈，N·m，绝对值 ≤应用 torque_limit_nm。']], '0=暂存成功；随后 commit。', thread, '-ENOTSUP：模式非 MIT；-EINVAL：非有限字段；-ERANGE：位置/速度/增益/力矩超范围；-EACCES：无输出权限。'),
        api('int Motor::setVelocity(float rad_s)', '速度模式命令，由电机驱动器内部闭环；无需再叠一层同名主控速度封装。', [['rad_s','有符号 rad/s，绝对值 ≤VMAX。']], '0=暂存；commit 后发布。', thread, '-ENOTSUP：固件配置模式不是 Velocity；-EINVAL：非有限数；-ERANGE：超范围；-EACCES：未允许运动。'),
        api('int Motor::setPositionVelocity(float rad, float max_rad_s)', '位置速度模式命令：指定原生位置和速度上限。该接口不给负的最大速度。', [['rad','原生电机位置，rad，绝对值 ≤PMAX。'],['max_rad_s','非负速度上限，rad/s，0–VMAX。']], '0=暂存；commit 后发布。', thread, '-ENOTSUP：模式不是 PositionVelocity；-EINVAL：非有限数；-ERANGE：位置超范围、速度为负或超过VMAX；-EACCES：无权限。'),
        api('int dm::describe(const Config &config, Descriptor &out); int dm::controlFrameId(ControlMode mode, uint16_t motor_id, uint16_t &out)', '计算三模式控制帧 ID：MIT=id，位置速度=0x100+id，速度=0x200+id；反馈 ID 来自 master_id，并用帧内 motor_id 分流。', [['config / mode','必须与固件持久化模式相同。'],['motor_id','电机地址1–15。'],['out','成功时写描述或控制帧 ID。']], '0=成功，负 errno=参数错误。', '纯调用，无 CAN I/O。', '-EINVAL/-ERANGE：型号、模式或地址不合法。CanBus 允许不同 DM 电机共用反馈 master_id，但帧内 motor_id 必须不同。'),
        api('int dm::buildMitFrame(uint16_t motor_id, const Limits &limits, const MitCommand &command, can_frame &out)', 'MIT 量化编码工具；位置16位，速度/kp/kd/力矩各12位。正常应用直接使用 Motor::setMit。', [['motor_id','电机地址。'],['limits','匹配固件的 PMAX/VMAX/TMAX。'],['command','MIT五字段。'],['out','编码后的标准 CAN 帧。']], '0=完成编帧，负 errno=无效/超范围；不发送。', '驱动 I/O 线程内部使用的纯编码。', '协议范围不等于应用 torque_limit_nm；绕过 Motor 会绕开生命周期和软件力矩限幅。'),
        api('int dm::buildVelocityFrame(uint16_t motor_id, float velocity_rad_s, can_frame &out); int dm::buildPositionVelocityFrame(uint16_t motor_id, float position_rad, float velocity_rad_s, can_frame &out)', '编码速度/位置速度模式的浮点命令帧；仅构造字节，不建立整车权限。', [['motor_id','电机地址。'],['position_rad / velocity_rad_s','对应物理量；应用层额外限幅由 Motor 执行。'],['out','目标CAN帧。']], '0=完成编帧。', '纯调用，无 I/O。', '地址/非有限输入错误返回负 errno；主控应调用 Motor setter，以免漏 PMAX/VMAX 校验。'),
        api('int dm::buildSpecialFrame(ControlMode mode, uint16_t motor_id, SpecialCommand command, can_frame &out); int dm::decodeFeedback(const can_frame &frame, uint8_t expected_motor_id, const Limits &limits, DecodedFeedback &out); bool dm::isFaultStatus(DriveStatus status)', '特殊命令 Enable/Disable/ClearError/SaveZero、反馈状态和温度解码。SaveZero 是底层协议能力，统一 Motor 不公开保存硬件零点操作。', [['mode / motor_id','真实固件模式和地址。'],['command','特殊协议命令枚举。'],['frame / expected_motor_id / limits','标准反馈帧、预期帧内电机地址和真实量程。'],['out / status','编码/解码输出或待分类驱动状态。']], '构帧/解码返回0或负errno；isFaultStatus 返回故障判断。', '纯编解码；驱动将回调入队后在I/O线程解析。', '不能仅以 CAN TX完成认为已禁用：DM stop 的 DriveConfirmed 才表示后续反馈确认禁用；Unreachable 表示无法确认。')
      ].concat(commonMotor()),
      examples:[{title:'MIT 力矩：用统一总线，持续刷新目标',language:'cpp',code:`#include <drivers/motor/can_bus.hpp>
#include <zephyr/kernel.h>
#include <cerrno>
using namespace skywalker::motor;

int main() {
  // 取自台架样例：请替换为工具中已保存的实物参数。
  static Motor motor{dm::j4310Mit({.id=1, .master_id=0x11,
      .position_max_rad=12.5f, .velocity_max_rad_s=30.0f,
      .torque_max_nm=10.0f, .torque_limit_nm=0.5f})};
  static CanBus bus{DEVICE_DT_GET(DT_NODELABEL(can1))};
  int r = bus.attach(motor);
  if (!r) r = bus.start();
  if (r < 0) return r;
  auto deadline = k_uptime_get() + 4000;
  while (!motor.ready() && k_uptime_get()<deadline) k_msleep(5);
  if (!motor.ready()) return -ETIMEDOUT;
  r = motor.enable();
  if (r < 0) return r;
  deadline = k_uptime_get() + 4000;
  while (!motor.active() && k_uptime_get()<deadline) k_msleep(1);
  if (!motor.active()) { (void)motor.disable(); return -ETIMEDOUT; }
  for (int cycle=0; cycle<100; ++cycle) {
    const auto s = motor.snapshot();
    if (!s.feedback_fresh) { r=-ESTALE; break; }
    r = motor.setTorque(0.2f); // N·m；MIT 的纯力矩输出
    if (!r) r = bus.commit().error;
    if (r < 0) break;
    k_msleep(5);
  }
  (void)motor.disable();
  return r;
}`,notes:'开启 SKYWALKER_DRIVER_MOTOR、SKYWALKER_MOTOR_DM、CAN、CPP。MC02 的 XT30 默认为关闭，若电机由该口供电，需按 boards 示例先显式 regulator_enable。例中 PMAX/VMAX/TMAX 为现有台架数据占位；该简例不代替整车温度/速度保护。'}],
      lifecycle:['工具中确认并保存模式、Motor ID、Master ID、PMAX/VMAX/TMAX 和零点；主控工厂仅描述它们。','显式使能所需电源后构造/attach/start，后台发送禁用并等待反馈确认。','等待稳定反馈 ready；申请 enable 并等待驱动 Enabled 反馈对应的新代次 Active。','每周期更新与当前模式匹配的目标，然后 commit；反馈异常或命令超时会撤销输出。','停机看 DriveConfirmed；若离线/总线错误则只能报告 Unreachable，恢复后后台继续确认安全状态。','需要锁存清故障时由 Motor/Group::clearFault 申请，完成后重新等待 ready 与 enable。'],
      pitfalls:['量程配置不一致会导致同一位模式解出错误的角度/力矩，不能只靠 ID 对上判断配置正确。','MIT 的电机内 kp/kd 和主控 PID 是两种不同控制位置；主控闭环封装通过纯 setTorque 输出。','通用反馈 torque_nm 是反馈力矩；不能把它当 current_a；DM 温度分别有 MOS 和转子字段。','原生电机 position 与 Motor 的首帧归零累计位置不同；reseedPosition 只改主控坐标，不改固件零点。','同一总线不同协议的 TX/RX ID 相撞时 start 会拒绝；更换模式会改变 TX ID。'],
      config:[{name:'CONFIG_SKYWALKER_MOTOR_DM',description:'纳入DM协议；共享子系统仍需 CONFIG_SKYWALKER_DRIVER_MOTOR=y。'},{name:'J4310Options',description:'所有量程与模式以实物保存值为准；应用力矩上限单独设置。'},{name:'Timing{50,20,50,3000}',description:'默认毫秒参数适应DM握手；5 ms台架周期小于20 ms命令有效期。'},{name:'BusOptions / Group',description:'与DJI共用。可以同物理CAN混合品牌，也可以一个Group跨CAN。'}]
    },
    {
      id:'imu',title:'IMU：独立采集、姿态与加热',category:'驱动与感知',
      summary:'BMI088 和达妙 RS485 共用带时间戳的快照，不同来源独立采集与独立诊断。',
      responsibility:'ImuSource 负责某种真实输入；ImuState 按字段发布测量与新鲜度；ImuReceiver 独占采集循环并可管理 ImuHeater。BMI088 可挂 QuaternionEkf 估计姿态；RS485 直接使用设备主动上报的四元数。',
      status:'partial',statusNote:'BMI088、独立 EKF、加热、DM 主动 RS485 和工作线程已实现，有 dual_imu 样例。DM CAN 仅抽象接口，未实现。上车应用仍需选定控制所用姿态来源、安装旋转、参考系和超时策略，不能将双IMU样例当成已融合的整车姿态。',
      source:[{label:'统一测量类型',path:'include/drivers/imu/imu_types.hpp'},{label:'输入源接口',path:'include/drivers/imu/imu.hpp'},{label:'接收线程API',path:'include/drivers/imu/imu_receiver.hpp'},{label:'BMI088输入',path:'drivers/imu/bmi088_imu.cpp'},{label:'RS485输入',path:'drivers/imu/dm_imu_rs485.cpp'},{label:'加热保护',path:'drivers/imu/imu_heater.cpp'},{label:'双IMU样例',path:'samples/imu/dual_imu/src/main.cpp'}],
      docs:[{label:'IMU Markdown',path:'docs/modules/drivers/imu.md'},{label:'双IMU台架',path:'samples/imu/dual_imu/README.md'}],depends:['boards','kalman','pid','uart'],
      interfaces:[
        api('virtual int ImuSource::init(); virtual int ImuSource::service(); virtual Snapshot ImuSource::snapshot() const', '所有输入源的最小契约：init 初始化；service 驱动一次采集/解析；snapshot 读取副本。构造不执行I/O。', [], 'init：0成功；service：0有进展/-EAGAIN无新数据/其他负errno；snapshot：测量、状态、capabilities、fresh_mask和诊断。', '只有一个所有者调用init/service；任何线程可copy snapshot；已注册回调的对象与依赖须一直存活。', 'Running 不表示每个字段都新鲜；检查 fresh_mask 和 Measurement.stamp.valid。'),
        api('Bmi088Imu::Bmi088Imu(const device *accel, const device *gyro, const Config &c, control::QuaternionEkf *estimator = nullptr); int Bmi088Imu::init(); int Bmi088Imu::service()', '通过Zephyr传感器API读取加速度/角速度/温度；可选独立EKF生成姿态，sensor_to_body 将传感器轴旋转到机器人机体轴。', [['accel / gyro','加速度和陀螺仪device，通常来自accel0/gyro0 alias。'],['c','采样周期默认1250us，温度周期10000us，最大输入时间偏差2000us，参考{id,epoch}、安装旋转与freshness。'],['estimator','可选长寿命 QuaternionEkf；交给该source独占，source.init会初始化它。']], 'init 0成功；service 0已处理/-EAGAIN尚未到采样时间。', '所有者线程，会做SPI/sensor I/O；若ImuReceiver接管，其他线程不得直接再init/service或更新estimator。', '-EALREADY：重复init；-EINVAL：周期/安装旋转不合法；-ENODEV：传感器不可用；-ENOTSUP：传入EKF但未编入；-EACCES：未初始化；-ESTALE：加速度/陀螺仪时间差超限；透传sensor错误。'),
        api('DmImuRs485Source::DmImuRs485Source(const device *uart, communication::AsyncUart::DmaBuffers &dma, const Config &c); int DmImuRs485Source::init(); int DmImuRs485Source::service(); int DmImuRs485Source::resetReference()', '消费DM-IMU-L1主动串行帧，应用缩放和安装旋转；遇UART故障可内部重试。resetReference在已知设备复位/清零/标定后递增epoch并撤销旧姿态，不向设备发送配置命令。', [['uart / dma','独占UART与长寿命DMA存储；H7使用__nocache。'],['c.protocol','设备id、半帧组装超时。'],['c.reference / sensor_to_body','独立参考身份与传感器→机体安装旋转。'],['c.device_quaternion_is_world_to_sensor','若固件上报世界→传感器，则先共轭成传感器→世界。'],['c.acceleration_scale / angular_velocity_scale','正数缩放系数，转换为m/s²和rad/s；取决于设备已保存输出单位。']], 'init/service：0成功或负errno；resetReference：0完成本地重置。', '一个所有者调用；UART ISR只推进AsyncUart事件，不在ISR解析或算姿态。接收线程运行时不得外部直接resetReference。', '-EINVAL：配置错误；-EALREADY：重复init；-EACCES：尚未初始化；-EAGAIN：无新帧/等待恢复；传输溢出丢掉半帧并记录transportGap；CRC/帧错误在diagnostics报告。'),
        api('ImuReceiver::ImuReceiver(ImuSource &source, const Config &config, ImuHeater *heater = nullptr); int ImuReceiver::start()', '启动一个源的专有工作线程，在线程内部异步init并持续service；可选加热器与此线程同所有者。', [['source','生命周期覆盖工作线程的输入源。'],['config.poll_interval_us','轮询间隔，默认1000us；具体source决定真正采样频率。'],['config.priority','合法抢占优先级；默认CONFIG_SKYWALKER_IMU_RX_PRIORITY。'],['heater','可选长寿命加热器；由worker init/update管理。']], '0=worker已启动，不等于source初始化成功或姿态可用。', '一次启动；无stop/restart或运行中销毁支持。不能再从别的线程调用source.init/service或heater.init/update/disable。', '-EINVAL：轮询/优先级无效；-ENOTSUP：加热器未编入；-EALREADY：已经start。'),
        api('Snapshot ImuReceiver::snapshot() const; ImuReceiver::Status ImuReceiver::status() const', 'snapshot直接让source按读取时刻算freshness；status分别报告异步初始化、service以及heater错误/占空比。没有第二份样本缓存。', [], '独立副本；init_complete/init_error 判断初始化结果；service_error保留最近非-EAGAIN返回。', '允许多个读线程；snapshot与status彼此不是原子组合。', 'start成功后立即读到Uninitialized属于正常；业务要等姿态字段新鲜且quality达到要求。'),
        api('int ImuState::init(uint32_t capabilities, core::OrientationReference reference); int ImuState::publish(const Update &update); Snapshot ImuState::snapshot() const', '具体source共享的字段发布器，Update.updated_mask指示本次真正更新的字段；其余字段保留独立时间戳。', [['capabilities','Accel/Gyro/Orientation/Temperature能力位。'],['reference','参考系身份和重置epoch；身份不同不能直接混用姿态。'],['update','已转换为机体轴、SI单位的测量与更新位。']], 'init/publish返回0或负errno；snapshot按Freshness逐字段计算fresh_mask。', '写者由source所有者串行；自旋锁保护多读者副本。', '无效/非有限更新不应视作新测量；diagnostics.invalid_updates报告被拒绝内容。'),
        api('int DmImuParser::init(); int DmImuParser::consume(const uint8_t *bytes, size_t size, core::TimeUs now, DmImuSink &sink); void DmImuParser::discardPartial(); Statistics DmImuParser::statistics() const', '主动帧字节组装、CRC与更新投递；consume(nullptr,0,now,sink)也可推进组装超时；传输缺口要discardPartial，避免拼接缺失帧。', [['bytes / size','任意分片，size=0允许空指针。'],['now','当前单调时间us。'],['sink','同线程接收解码Update的对象。']], 'consume返回成功接受帧数或负errno；statistics返回计数。', '所有方法仅解析器所有者线程，无内部并发保证。', 'CRC错误/无效帧/组装超时计数分别见Statistics；不执行配置写命令，不实现CAN。'),
        api('int ImuHeater::init(); int ImuHeater::update(const core::Measurement<float> &temperature, core::TimeUs now); int ImuHeater::disable(); ImuHeater::Snapshot ImuHeater::snapshot() const', '独立温控PID与PWM安全管理。init明确写0占空比；温度过期、超上限或无效时尝试关闭；snapshot分别保留原因和关闭操作错误。', [['temperature','带有效性与时间戳的摄氏温度。'],['now','单调时间us；不允许倒退。'],['Config','PWM设备/周期，目标默认50°C、最大65°C，温度超时200ms、控制周期100ms，输出PID限幅0–1。']], '0成功；update -EAGAIN未到周期；其他负errno为错误。', '一个线程拥有init/update/disable；snapshot多读者。已附加ImuReceiver后全部控制由worker处理。', '-EACCES：未init；-EALREADY：重复init；-EINVAL/-ENODEV：配置/设备错；-ESTALE：过期/时间倒退；-ERANGE：≥最大温度、低于-40或非有限；PWM关断失败会返回底层错误并保留disable_error。')
      ],
      examples:[{title:'板载BMI088 + EKF：异步采集，业务只读快照',language:'cpp',code:`#include <drivers/imu/bmi088_imu.hpp>
#include <drivers/imu/imu_receiver.hpp>
#include <core/attitude.hpp>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <cerrno>
namespace imu = skywalker::imu;

int main() {
  static skywalker::control::QuaternionEkf ekf{
      skywalker::control::QuaternionEkf::Config{}};
  static imu::Bmi088Imu source{
      DEVICE_DT_GET(DT_ALIAS(accel0)), DEVICE_DT_GET(DT_ALIAS(gyro0)),
      imu::Bmi088Imu::Config{.reference={1,1}, .sensor_to_body={}}, &ekf};
  static imu::ImuReceiver receiver{
      source, imu::ImuReceiver::Config{.poll_interval_us=500, .priority=5}};
  const int r = receiver.start();
  if (r < 0) return r;
  // start只启动线程；init与静止初始化由worker完成。
  for (;;) {
    const auto s = receiver.snapshot();
    const auto diagnostic = receiver.status();
    if (diagnostic.init_complete && diagnostic.init_error < 0)
      return diagnostic.init_error;
    if ((s.fresh_mask & imu::Orientation) &&
        s.sample.attitude_quality == imu::AttitudeQuality::Tracking) {
      const auto e = skywalker::core::euler(s.sample.orientation.value);
      printk("roll=%d pitch=%d mrad\\n", int(e.roll*1000), int(e.pitch*1000));
    }
    k_msleep(20);
  }
}`,notes:'安装旋转的单位四元数仅适用于传感器轴与机体轴一致；reference={1,1}是示例身份。启用IMU/BMI088/RECEIVER、ATTITUDE_EKF、SENSOR/BMI08X、FPU/FPU_SHARING以及CPP。初始化需稳定静止；加速度仅能校正倾斜，yaw会漂移。完整RS485+加热例见dual_imu样例。'}],
      lifecycle:['构造输入源、可选EKF和heater；UART DMA、所有对象长期存活。','调用receiver.start；worker负责heater.init和source.init，应用通过status等异步结果。','单一worker周期service，source将各字段独立发布；姿态初始化阶段不可直接控制云台。','控制/遥测线程只snapshot并检查fresh_mask、参考身份/epoch和姿态质量。','温度过期或错误由同一worker关闭heater；源失败与加热关闭失败分别诊断。','设备复位或参考变化必须撤销旧姿态并更新epoch；没有通用自动双IMU融合/切换逻辑。'],
      pitfalls:['DM CAN 类保留纯虚init/service/snapshot，不能直接实例化；没有CAN接收器或对应Kconfig。','两颗IMU不代表已融合。reference身份与epoch必须匹配消费者约定，不能随意相减不同零点的yaw。','fresh_mask按字段变化；Running但Orientation不新鲜时不能使用上次四元数继续控制。','ImuReceiver没有stop或析构协议，main栈上的短寿命对象不可交给它。','H7的RS485 DMA缓冲需要__nocache和CONFIG_NOCACHE_MEMORY；避免CPU缓存看到旧字节。','加热停机要看disable_error；软件目标占空比0并不自动证明PWM外设写入成功。'],
      config:[{name:'CONFIG_SKYWALKER_IMU / BMI088 / RECEIVER',description:'基础、板载传感器源、独立采集线程分别按需开启。'},{name:'CONFIG_SKYWALKER_ATTITUDE_EKF',description:'为BMI088生成姿态；不依赖设备树Kalman或Matrix库。'},{name:'CONFIG_SKYWALKER_IMU_DM_RS485',description:'依赖UART_TRANSPORT并选择DM主动协议；设备单位、主动输出和速率需工具配置。'},{name:'CONFIG_SKYWALKER_IMU_HEATER',description:'需PWM，选择控制算法库；控制与source独立，但可由同一receiver持有。'},{name:'Freshness',description:'默认加速度/角速度/姿态20ms、温度1s；heater自己另外要求温度200ms内新鲜。'},{name:'RX_STACK_SIZE / RX_PRIORITY',description:'默认8192字节/优先级6；每个receiver独立线程，涉及浮点时按FPU配置分配寄存器上下文。'}]
    },
    {
      id:'kalman',title:'姿态 EKF 与通用 Kalman / Matrix',category:'控制算法',
      summary:'明确两条独立路线：BMI088 四元数 EKF；CMSIS-DSP 线性 Kalman 和矩阵工具。',
      responsibility:'QuaternionEkf维护每实例四元数、协方差、零偏和静止初始化。legacy KalmanFilter提供通用线性预测/校正，矩阵由调用者或设备树静态分配，两者没有调用依赖。',
      status:'ready',statusNote:'独立QuaternionEkf已接BMI088输入与dual_imu样例；通用Kalman与Matrix为可选数学工具，不在当前BMI088姿态链中。整车仍需明确yaw外部参考、源选择和降级策略，EKF本身不提供绝对航向或双IMU融合。',
      source:[{label:'独立EKF API',path:'include/control/attitude_ekf.hpp'},{label:'EKF实现',path:'lib/control/attitude_ekf.cpp'},{label:'通用Kalman API',path:'include/drivers/kalman_filter/kalman_filter.h'},{label:'通用Kalman实现',path:'drivers/kalman_filter/kalman_filter.c'},{label:'Matrix封装',path:'include/lib/matrix/matrix.h'},{label:'数学选项',path:'lib/Kconfig'}],docs:[{label:'Kalman与Matrix Markdown',path:'docs/modules/drivers/kalman-matrix.md'}],depends:[],
      interfaces:[
        api('QuaternionEkf::QuaternionEkf(const Config &c); int QuaternionEkf::init()', '构造保存配置，init验证全部正数/范围并重置每实例状态。不依赖Zephyr Kalman device，也不动态分配内存。', [['c','dt范围、重力和加速度门限、过程/测量噪声、创新门限、滤波/零偏时间常数及静止初始化样本数。']], 'init 0成功。', '单所有者；如果传给Bmi088Imu，则由该source.init初始化，不要先自行init。', '-EINVAL：非有限/非正配置、dt范围逆序、初始化样本数<2；-EALREADY：已初始化配置。'),
        api('int QuaternionEkf::update(const Input &input)', '输入加速度与角速度，静止初始化后用陀螺仪预测、重力方向校正倾斜；强加速度时保留预测并标记Degraded。yaw无可观测绝对约束。', [['input.accel_m_s2','同一传感器坐标的加速度，m/s²。'],['input.gyro_rad_s','角速度，rad/s。'],['input.time_us','两种输入对应的单调时间，us，必须严格递增。']], '0=有有效输出；-EAGAIN=初始化中/采样间隔过短/间隔过长触发重置；读取quality区分Tracking与Degraded。', '算法所有者线程；没有内部锁，不从别的线程并发update/reset/read。Bmi088消费者应通过source快照读取。', '-EACCES：未init；-EINVAL：非有限输入；-ESTALE：时间重复/倒退；-ERANGE：数值异常并重置。超过dt_max_s也会重置，generation递增。'),
        api('void QuaternionEkf::reset(); core::Quaternion QuaternionEkf::attitude() const; Quality QuaternionEkf::quality() const; uint32_t QuaternionEkf::generation() const', 'reset清除历史和零偏、进入Initializing、增加非零generation。attitude为q_WS，表示传感器坐标→局部重力对齐世界坐标的旋转。', [], '无返回码或值副本；Initializing时不得把默认四元数当有效姿态。', '仅所有者串行；source发布姿态时把estimator代次变化映射成OrientationReference.epoch。', 'reset不重新验证配置；Degraded仍有输出但加速度校正不足，需要消费者制定策略。'),
        api('void KalmanFilter_Predict(KalmanFilter *kf)', '通用线性预测：X=F·X，P=F·P·Fᵀ+Q。所有矩阵pData预先绑定，不由此函数分配持久内存。', [['kf','F/P/Q为n×n，X为n×1；有效非空缓冲。']], 'void；内部CMSIS运算返回状态未向调用方传播。', '同一滤波实例单线程所有者；按n²增长的临时栈数组，不宜在ISR使用。', '不会检查空指针、全部维度/数值或返回矩阵运算错误；调用者必须保证合法性，不能据void调用完成判定数值有效。'),
        api('void KalmanFilter_Correct(KalmanFilter *kf, const Matrix *z)', '线性校正：由P/H/R算K，再更新X/P；这是线性矩阵步骤，不是自动四元数EKF。', [['kf','H=m×n，R=m×m，K=n×m，F/P/Q/X与predict一致。'],['z','m×1观测矩阵，缓冲与维度正确；创新协方差须可逆。']], 'void，结果写入kf->X/P/K。', '单所有者线程；多块n²/m²/nm临时栈缓冲。', '当前实现不传播Matrix_Inverse失败；不可逆/错误维度/非有限数可污染状态，调用方须建立模型并保证条件。'),
        api('Matrix_Init(Matrix *matrix, uint16_t rows, uint16_t cols, float *buffer); void Matrix_Zero(Matrix *matrix); void Matrix_SetDiag(Matrix *matrix, float value)', 'Matrix是arm_matrix_instance_f32别名。Init绑定已有缓冲，Zero清零，SetDiag先清零再置对角元素；矩阵不拥有buffer。', [['matrix','待初始化/写入矩阵。'],['rows / cols','行列，buffer至少rows×cols个float。'],['buffer','调用者持有、存活时间覆盖全部使用。'],['value','对角值。']], 'void，修改结构体或缓冲。', '调用者管理内存与并发，不自动拷贝buffer。', '不提供空指针或容量检查；切勿把局部数组地址交给更长寿命对象。'),
        api('Matrix_Add / Matrix_Subtract / Matrix_Multiply / Matrix_Transpose / Matrix_Inverse', '宏别名分别对应CMSIS-DSP arm_mat_add_f32/sub_f32/mult_f32/trans_f32/inverse_f32；通过输入矩阵指针与已初始化目标矩阵运算。', [['输入 Matrix*','运算维度必须兼容。'],['目标 Matrix*','事先初始化正确尺寸与足量buffer；不要假设所有运算支持输入输出重叠。']], 'arm_status；调用者应处理维度不匹配、奇异矩阵等返回。', '共享buffer访问需要调用者串行/加锁。', '独立矩阵API有返回状态，但legacy Kalman封装当前未传播。')
      ],
      examples:[{title:'独立EKF：静止初始化与时间戳驱动',language:'cpp',code:`#include <control/attitude_ekf.hpp>
#include <cerrno>
using skywalker::control::QuaternionEkf;

int estimateExample() {
  QuaternionEkf estimator{QuaternionEkf::Config{}};
  int r = estimator.init();
  if (r < 0) return r;
  for (unsigned i=0; i<120; ++i) {
    // 演示输入；实机改成新鲜的同坐标传感器测量。
    const QuaternionEkf::Input in{
      .accel_m_s2={0,0,9.80665f}, .gyro_rad_s={0,0,0},
      .time_us=1000u + i*1250u};
    r = estimator.update(in);
    if (r < 0 && r != -EAGAIN) return r;
    if (!r && estimator.quality()==QuaternionEkf::Quality::Tracking) {
      const auto q = estimator.attitude();
      (void)q; // q_WS；主控若要机体姿态，还要应用安装旋转
    }
  }
  return r;
}`,notes:'此例只说明数学接口，不驱动电机。Bmi088Imu内由source调用init/update并发布已经变换到机体的姿态；不要在应用线程再操作同一个estimator。'},{title:'一维线性Kalman：调用者绑定全部矩阵',language:'cpp',code:`#include <drivers/kalman_filter/kalman_filter.h>

float smoothExample(float measurement) {
  // 这些持久缓冲的维度全部n=m=1，示范平滑常量观测。
  static float f=1, h=1, r=1, x=0, p=1000, q=0.001f, gain=0;
  static KalmanFilter kf{};
  static bool bound=false;
  if (!bound) {
    Matrix_Init(&kf.F,1,1,&f); Matrix_Init(&kf.H,1,1,&h);
    Matrix_Init(&kf.R,1,1,&r); Matrix_Init(&kf.X,1,1,&x);
    Matrix_Init(&kf.P,1,1,&p); Matrix_Init(&kf.Q,1,1,&q);
    Matrix_Init(&kf.K,1,1,&gain); bound=true;
  }
  Matrix z{};
  Matrix_Init(&z,1,1,&measurement);
  KalmanFilter_Predict(&kf);
  KalmanFilter_Correct(&kf,&z);
  return kf.X.pData[0];
}`,notes:'输入measurement必须先保证有限；此函数只能被一个线程串行调用，例中R=1使1维创新协方差可逆。通用驱动/Matrix选项按需开启；本例不需要设备树filter节点。'}],
      lifecycle:['选定独立姿态EKF或通用线性模型，避免把二者串接成重复滤波。','QuaternionEkf：构造→init→稳定静止采样初始化→update→按quality使用；大时间缺口重置代次。','通用Kalman：分配全部F/H/R/X/P/Q/K与观测缓冲→绑定尺寸→设置模型→Predict→Correct。','重置姿态/参考系后，下游必须丢弃旧epoch的控制目标和历史。'],
      pitfalls:['没有磁力计/外部绝对观测的重力EKF不能提供绝对yaw；把yaw作为整车航向需要额外策略。','legacy Kalman的void接口忽略内部矩阵错误，不能当作有错误恢复机制的通用安全封装。','CMSIS Matrix_Init只绑定内存，不分配、不复制；实例缓冲不可共享给独立滤波器。','通用Kalman设备树state-dim/measure-dim只是尺寸；当前BMI088不要求4/3节点。'],
      config:[{name:'CONFIG_SKYWALKER_ATTITUDE_EKF',description:'独立姿态算法，配合控制库构建；默认dt 0.2–20ms、初始化100个静止样本。'},{name:'CONFIG_SKYWALKER_LIB_MATRIX / CONFIG_SKYWALKER_DRIVER_KALMAN_FILTER',description:'选择CMSIS矩阵与通用线性Kalman驱动；与BMI088独立EKF分别开启。'},{name:'QuaternionEkf::Config',description:'accel_gate_m_s2过滤明显非重力输入，innovation_gate控制测量校正，stationary_gyro_rad_s与initialization_samples控制静止初始化。'},{name:'CONFIG_SKYWALKER_LIB_MATRIX_STORAGE',description:'可选矩阵Flash存储，另需可用Flash/storage partition；不是姿态估计必需项。'}]
    },
    {
      id:'pid',title:'PID、前馈、斜坡与角度工具',category:'控制算法',
      summary:'无I/O的纯控制计算，明确单位、采样间隔、积分限制与失败语义。',
      responsibility:'PID算反馈校正，前馈按参考运动估算需要的输出；斜坡限制目标变化率，角度工具解决±π跳变；C速度/位置环把这些算子串起来。硬件权限、反馈时效和CAN由上层封装处理。',
      status:'ready',statusNote:'纯算法已由MotorControl、IMU加热与台架样例复用；可接入上车控制。增益和限幅仍是设备/机构相关参数，现有样例数值不代表整车调参完成。',
      source:[{label:'PID API',path:'include/control/pid.h'},{label:'前馈API',path:'include/control/feedforward_pid.h'},{label:'角度API',path:'include/control/angle.h'},{label:'斜坡API',path:'include/control/slew_rate_limiter.h'},{label:'PID实现',path:'lib/control/pid.c'},{label:'串级位置环',path:'lib/control/motor_position.c'},{label:'纯算法样例',path:'samples/control/src/main.c'}],docs:[{label:'控制算法Markdown',path:'docs/modules/control/algorithms.md'}],depends:[],
      interfaces:[
        api('int control_pid_validate(const control_pid_config *config)', '验证有限参数、非负增益/死区/微分时间常数、积分/输出上下限与dt范围。积分上下限限制的是Ki乘过之后的I项输出。', [['config','kp/ki/kd；derivative_tau_s；integral_min/max；output_min/max；deadband；dt_min_s/max_s。']], '0合法，-EINVAL不合法。', '无I/O、无共享状态，任意线程。', '-EINVAL：空指针、非有限数、负增益或范围不合法；dt_min必须>0。'),
        api('int control_pid_reset(control_pid_state *state, float current_measurement)', '把I项清0、测量历史设为当前值、微分速率清0；避免重新开始时旧积分和导数冲击。', [['state','每个控制环独有的状态。'],['current_measurement','当前测量，与后续measurement同单位。']], '0成功，-EINVAL失败。', '由同一算法所有者串行调用；不带锁。', '失败不写state；正积分下限的配置须自行保证初始I=0也落在范围内。'),
        api('int control_pid_step(control_pid_state *state, const control_pid_config *config, const control_pid_input *input, control_pid_result *result)', '一次PID。P使用误差，D使用负的测量变化率并可低通；输出饱和且误差继续推向饱和时冻结I，支持显式freeze_integrator。', [['state','上次状态；首次未initialized会就地初始化测量历史。'],['input.setpoint / measurement','参考/测量，单位相同。'],['input.dt_s','真实采样间隔秒，必须落在配置范围。'],['input.freeze_integrator','true保留I项，仍计算P/D。'],['result','error/effective_error、p/i/d、未限幅输出、最终output与saturated。']], '0成功；仅成功时更新state/result，失败保持调用者原值。', '单实例单所有者纯计算，无CAN/驱动操作。', '-EINVAL：空指针/非有限输入或配置；-ERANGE：dt越界、积分状态越界或计算溢出。调用方必须处理失败并停止使用本周期旧result。'),
        api('int control_feedforward_validate(const control_feedforward_config *config); int control_feedforward_calculate(const control_feedforward_config *config, const control_feedforward_reference *reference, float *output)', '计算偏置、静摩擦方向项、速度项、加速度项与可选sin/cos重力项的总和。优先由速度参考确定静摩擦方向，速度近零时再看加速度。', [['config','k_bias/static/velocity/acceleration/gravity，方向判定阈值velocity_epsilon/acceleration_epsilon，gravity_model。'],['reference','position_ref_rad、velocity_ref、acceleration_ref。'],['output','输出物理单位由全部系数共同决定，需与PID输出一致。']], '0成功，输出不自动限幅；失败不写output。', '无状态纯函数。', '-EINVAL：指针/非有限值/负阈值/无效gravity_model；-ERANGE：计算溢出。'),
        api('int control_feedforward_pid_validate(const control_feedforward_pid_config *config); int control_feedforward_pid_reset(control_feedforward_pid_state *state, float current_measurement); int control_feedforward_pid_step(control_feedforward_pid_state *state, const control_feedforward_pid_config *config, const control_feedforward_pid_input *input, control_feedforward_pid_result *result)', '先算前馈，再在PID内部合成和限幅；抗积分饱和考虑PID+前馈总量，而不是先限PID再另加前馈。', [['config','feedback PID配置与feedforward配置。'],['state','PID历史。'],['input','feedback测量/目标/dt + reference位置/速度/加速度。'],['result','反馈项细节、feedforward和总output。']], '0成功；step失败不修改state/result。', '单所有者纯计算。', '透传PID/前馈的-EINVAL/-ERANGE；每种量纲的系数须按输出单位配套。'),
        api('int control_slew_rate_validate(const control_slew_rate_config *config); int control_slew_rate_reset(control_slew_rate_state *state, float current_value); int control_slew_rate_step(control_slew_rate_state *state, const control_slew_rate_config *config, float requested_value, float dt_s, float *limited_value, float *limited_rate)', '斜坡限制每周期目标变化，输出限制后的值和真实变化率。上升/下降按数值方向选择，不是按速度绝对值变大/变小选择。', [['config','rising_rate_per_s / falling_rate_per_s为非负变化率。'],['state / current_value','上次限制目标；step之前须reset。'],['requested_value / dt_s','本周期目标和正的秒间隔。'],['limited_value / limited_rate','限制值和每秒实际变化。']], '0成功；失败不修改状态/输出。', '每个目标一份独立state。', '-EACCES：未reset；-EINVAL：空指针/非有限值/负配置；-ERANGE：dt≤0或计算溢出。率=0表示该方向不变化。'),
        api('int control_angle_unwrap_reset(control_angle_unwrapper *state, float wrapped_rad); int control_angle_unwrap_step(control_angle_unwrapper *state, float wrapped_rad, float *continuous_rad)', '把每次单圈角度展开成连续角度；增量选[-π,π)的最短方向，要求相邻真实运动少于半圈。', [['state','独立展开历史，先reset。'],['wrapped_rad','单圈角度rad，函数规范化到[-π,π)。'],['continuous_rad','累计连续角度输出。']], '0成功，失败不更新。', '单所有者，适用于按时间有序的角度样本。', '-EACCES：未初始化；-EINVAL：指针/非有限数；-ERANGE：运算溢出。不能恢复丢帧期间未知圈数。'),
        api('int control_shortest_angle_error(float target_rad, float measurement_rad, float *error_rad); int control_angle_nearest_continuous_target(float requested_absolute_rad, float measured_absolute_rad, float measured_continuous_rad, float *continuous_target_rad)', '前者算最短角度误差，后者把单圈绝对目标映射到当前连续坐标中最近的一圈。恰好+π会规范化成-π。', [['target_rad / requested_absolute_rad','绝对单圈目标，rad。'],['measurement_rad / measured_absolute_rad','同一固定零点的单圈测量。'],['measured_continuous_rad','当前连续位置。'],['error_rad / continuous_target_rad','误差或最近的连续目标。']], '0成功。', '无历史纯调用。', '-EINVAL：非有限/空输出；-ERANGE：差值或加法溢出。绝对测量与连续测量必须来自同一电机参考。'),
        api('int control_motor_velocity_validate(const control_motor_velocity_config *config); int control_motor_velocity_reset(control_motor_velocity_state *state, float measured_velocity_rad_s, float initial_reference_rad_s); int control_motor_velocity_step(control_motor_velocity_state *state, const control_motor_velocity_config *config, const control_motor_velocity_input *input, control_motor_velocity_output *output)', '纯速度环串联测量低通、目标斜坡、软死区、前馈PID和执行量限幅。它不检查电机在线或发送CAN。', [['config','regulator、reference_slew、measurement_filter_tau_s、soft_deadband_rad_s、目标速度与执行量上限。'],['state','每个电机独立状态，先reset。'],['input','目标/测量rad/s、前馈位置rad、dt秒、积分冻结标志。'],['output','限制目标、加速度、滤波速度、误差、调节器分量与effort_command。']], '0成功；失败保持state/output。', '控制所有者纯计算。', '-EACCES：没reset；-EINVAL：配置/非有限值；-ERANGE：目标/dt超限或数值异常。effort单位由调用者决定。'),
        api('int control_motor_position_validate(const control_motor_position_config *config); int control_motor_position_reset(control_motor_position_state *state, const control_motor_position_config *config, float measured_position_rad, float measured_velocity_rad_s); int control_motor_position_step(control_motor_position_state *state, const control_motor_position_config *config, const control_motor_position_input *input, control_motor_position_output *output)', '串级位置环：外环位置PID给速度参考，内环速度控制给执行量。当前位置环I项冻结；不会因配置ki而持续积累位置积分。', [['config','position外环PID + velocity内环；外环输出范围不能超过内环最大目标速度。'],['state','先reset的外环/内环历史。'],['input','连续目标/位置rad、测量rad/s、dt；has_position_reference可指定重力前馈使用的物理角度。'],['output','外环结果、内环结果与effort_command。']], '0成功；失败不改外环、内环或output。', '单所有者纯计算；参考系与电机权限由PositionMotor负责。', '透传速度/PID错误；-ERANGE：外环输出范围与内环目标范围不一致。')
      ],
      examples:[{title:'前馈 + PID 一次更新：检查错误，再使用总输出',language:'cpp',code:`#include <control/feedforward_pid.h>

int calculateEffort(float measured_rad_s, float dt_s, float &ampere) {
  static control_feedforward_pid_state state{};
  static const control_feedforward_pid_config cfg{
    .feedback={.kp=0.02f,.ki=0.05f,.kd=0,.derivative_tau_s=0,
      .integral_min=-0.1f,.integral_max=0.1f,
      .output_min=-0.8f,.output_max=0.8f,.deadband=0,
      .dt_min_s=0.001f,.dt_max_s=0.020f},
    .feedforward={.k_bias=0,.k_static=0.005f,.k_velocity=0,
      .k_acceleration=0,.k_gravity=0,.velocity_epsilon=0,
      .acceleration_epsilon=0,.gravity_model=CONTROL_GRAVITY_NONE}};
  if (!state.feedback.initialized) {
    const int r=control_feedforward_pid_reset(&state,measured_rad_s);
    if (r<0) return r;
  }
  const control_feedforward_pid_input in{
    .feedback={.setpoint=2.0f,.measurement=measured_rad_s,
      .dt_s=dt_s,.freeze_integrator=false},
    .reference={.position_ref_rad=0,.velocity_ref=2.0f,.acceleration_ref=0}};
  control_feedforward_pid_result out{};
  const int r=control_feedforward_pid_step(&state,&cfg,&in,&out);
  if (!r) ampere=out.output;
  return r; // 调用者遇负值应撤销输出，不使用旧ampere
}`,notes:'增益取自台架结构作为占位，输出按A配置；它只算数值，尚无电机反馈时效、权限和安全检查。多电机必须每电机一份state；需要硬件集成时优先用VelocityMotor/PositionMotor。开启SKYWALKER_LIB_CONTROL。'}],
      lifecycle:['为每个控制环设置物理单位、限幅、允许dt和独立state。','启动/恢复时reset，避免复用旧积分与测量导数。','按真实周期dt进行step；仅返回0时使用本周期结果。','暂停、重使能或位置参考变更时，由硬件封装重置算法历史。'],
      pitfalls:['积分限幅是I项输出单位，不是误差积分原始量；不能把其他PID库参数直接复制。','D作用于测量变化率，目标突变不会直接产生微分踢；这与误差微分算法不同。','纯算法不会检查传感器时间戳，返回0不等于电机允许输出。','位置外环I冻结，改大ki不会得到想象中的位置积分效果。','速度斜坡上升/下降按数值方向；负速度增大绝对值属于数值下降。'],
      config:[{name:'CONFIG_SKYWALKER_LIB_CONTROL',description:'编入纯C算法，无Zephyrdevice绑定；状态均由调用者持有。'},{name:'dt_min_s / dt_max_s',description:'每周期用真实经过时间；dt超限返回错误，不会静默裁剪。'},{name:'integral_min/max / output_min/max',description:'按执行量单位设置，可为不对称范围；前馈PID共享总输出限幅。'},{name:'requested_velocity_abs_max_rad_s / effort_abs_max',description:'纯环速度/执行量限制；还需与Motor硬件限幅与安全最大测量速度配套。'}]
    },
    {
      id:'motor-control',title:'VelocityMotor / PositionMotor 硬件闭环',category:'控制算法',
      summary:'把纯算法绑定到真实电机反馈与输出权限，给上层轴对象提供单一控制入口。',
      responsibility:'配置时校验能力、单位、限幅和保护条件，独占Motor执行量生产者；update读取新鲜反馈、运行速度或串级位置环，再暂存A/N·m命令；上层仍负责使能与总线commit。',
      status:'ready',statusNote:'速度/位置闭环、反馈/温度/速度保护和参考代次处理已实现；台架与云台样例调用。可接上车Gimbal/Chassis；当前整车接线/标定/控制增益仍需应用配置后启用。',
      source:[{label:'速度封装API',path:'include/control/velocity_motor.hpp'},{label:'位置封装API',path:'include/control/position_motor.hpp'},{label:'单位与安全配置',path:'include/control/motor_common.hpp'},{label:'闭环绑定实现',path:'lib/control/motor_control.cpp'},{label:'速度闭环完整样例',path:'samples/motor/dji_speed_control/src/main.cpp'},{label:'位置闭环样例',path:'samples/motor/dji_position_control/src/main.cpp'}],docs:[{label:'MotorControl Markdown',path:'docs/modules/control/motor-control.md'},{label:'调用顺序',path:'docs/modules/call-examples.md'}],depends:['motor-dji','motor-dm','pid'],
      interfaces:[
        api('VelocityMotor::VelocityMotor(motor::Motor &motor, const Config &config); PositionMotor::PositionMotor(motor::Motor &motor, const Config &config)', '绑定一个长寿命Motor引用与值配置；构造不产生CAN输出。VelocityMotor处理rad/s目标，PositionMotor处理位置rad目标。', [['motor','已构造的真实Motor，将在configure后独占其执行量输出。'],['config.loop','纯C速度或串级位置环配置。'],['config.effort_unit','DJI用Ampere；DM MIT用NewtonMeter；Unspecified非法。'],['config.safety','测量速度最大绝对值须>0；温度最大值0关闭温度检查，>0要求有温度反馈。'],['PositionMotor config.reference','StartupRelative / DriverContinuous / AbsoluteNearest。']], '构造对象；无配置验证返回。', '长寿命对象，不可复制；configure/update/reset由同一控制所有者串行调用。', '不能通过DM速度/位置速度模式绑定NewtonMeter，因为该模式不具备CommandTorque。'),
        api('int VelocityMotor::configure(); int PositionMotor::configure()', '启动总线后、使能电机前验证环参数、单位、能力、硬件输出上限、速度保护范围，然后绑定唯一生产者。', [], '0=绑定完成，不代表motor.ready或Active。', '控制初始化线程；必须先bus.start，再configure；与已运行update不能并发。', '-EALREADY：已配置；-EACCES：bus未启动；-EBUSY：电机Active/Enabling或已有其他生产者；-EINVAL：单位/安全配置非法；-ENOTSUP：能力/温度/绝对位置不支持；-ERANGE：执行量超硬件限幅、目标速度超保护、串级dt无交集。'),
        api('int VelocityMotor::update(float target_rad_s, float dt_s)', '读取MotorSnapshot，检查Active、新鲜有效速度、温度/速度保护，算速度闭环并暂存执行量；每个使能/参考新代次的首周期重置历史并输出零执行量。', [['target_rad_s','有限速度目标，rad/s，绝对值≤loop.requested_velocity_abs_max_rad_s。'],['dt_s','真实周期秒，落在regulator.feedback配置范围。']], '0=本周期已暂存；应用末尾仍须bus.commit。', '单控制周期所有者；telemetry由锁保护，可多读。', '-EACCES：未configure/未Active/权限撤销；-ESTALE：反馈过期；-ENODATA：必要字段缺失；-EINVAL：无效目标/反馈；-ERANGE：dt/目标/测量速度/温度越界。Active状态的控制错误会撤销Motor输出并传播Group停机。'),
        api('int PositionMotor::update(double target_position_rad, float dt_s)', '解析参考坐标，检查有效位置参考并算外位置环→内速度环→执行量。连续坐标较大时内部换原点维持float局部精度。', [['target_position_rad','rad；StartupRelative相对锚点，DriverContinuous使用驱动累计坐标，AbsoluteNearest是固定零点下的单圈目标。'],['dt_s','落在外位置PID与内速度PID范围交集。']], '0=暂存成功；新代次首周期先输出零执行量，之后commit。', '同一控制所有者串行；不从ISR调用或与reset并发。', '除速度封装错误外：-ENODATA位置参考失效；-ENOTSUP配置时无绝对位置能力；-ERANGE目标超可表示范围。Active状态非法周期会触发安全停机。'),
        api('int VelocityMotor::reset(); int PositionMotor::reset()', '停用且反馈新鲜时显式重置历史并检查保护条件。PositionMotor在StartupRelative下将当前机械位置确定为新的相对锚点；不改驱动器累计坐标。', [], '0=状态已重置；telemetry先发布无有效周期的副本。', '使能前/停用后控制所有者调用。重使能代次首周期也会自动重置；不是随时运行中的目标清零接口。', '-EACCES：未configure；-EBUSY：Active/Enabling；-EAGAIN：反馈不新鲜；-ENODATA：位置参考/字段无效；透传测量保护错误。'),
        api('VelocityMotor::Telemetry VelocityMotor::telemetry() const; PositionMotor::Telemetry PositionMotor::telemetry() const; PositionReference PositionMotor::reference() const', 'telemetry报告最近一次更新的目标、dt、反馈、算法分项、effort单位、valid/error；reference只查询位置参考模式。', [], '受锁保护值副本；并非在读取时重新算freshness的实时Motor快照。', '允许独立遥测线程读取；控制state本身仍只有一个写者。', 'valid=false或error<0时不要把上次输出当本周期控制成功；需要当前硬件健康时另读motor.snapshot。'),
        api('enum class PositionReference { StartupRelative, DriverContinuous, AbsoluteNearest }; enum class EffortUnit { Unspecified, Ampere, NewtonMeter }', '坐标和执行量是显式契约。StartupRelative为初始化/显式reset锚点；DriverContinuous使用Motor累计位置；AbsoluteNearest把绝对角映射到最近连续圈。', [['AbsoluteNearest','要求FeedbackAbsolutePosition；当前GM6020固定编码器零点支持，DM累计位置不自动等同绝对位置。'],['Ampere / NewtonMeter','必须和Motor的CommandCurrent/CommandTorque能力及算法系数相符。']], '类型配置，无运行期转换。', '初始化阶段选择；不能运行中切模式。', 'reseedPosition改变reference_generation；断联位置参考不可靠时需要机械重新找零而非自动相信旧锚点。')
      ],
      examples:[{title:'速度闭环：完整配置、绑定、使能与每周期commit',language:'cpp',code:`#include <control/velocity_motor.hpp>
#include <drivers/motor/can_bus.hpp>
#include <zephyr/kernel.h>
#include <cerrno>

skywalker::control::VelocityMotor::Config makeConfig() {
  skywalker::control::VelocityMotor::Config c{};
  c.loop.regulator.feedback={.kp=0.02f,.ki=0.05f,.kd=0,
    .derivative_tau_s=0,.integral_min=-0.1f,.integral_max=0.1f,
    .output_min=-0.5f,.output_max=0.5f,.deadband=0,
    .dt_min_s=0.001f,.dt_max_s=0.020f};
  c.loop.reference_slew={100,100};
  c.loop.measurement_filter_tau_s=0.025f;
  c.loop.soft_deadband_rad_s=0.2f;
  c.loop.requested_velocity_abs_max_rad_s=10;
  c.loop.effort_abs_max=0.5f;
  c.effort_unit=skywalker::control::EffortUnit::Ampere;
  c.safety={20,0}; // rad/s测量保护；0关闭温度保护
  return c;
}

int main() {
  using namespace skywalker::motor;
  static Motor drive{dji::gm6020({.id=4,.current_limit_a=0.5f,
    .encoder_zero_ticks=0,.current_mode_confirmed=true})};
  static CanBus bus{DEVICE_DT_GET(DT_NODELABEL(can1))};
  static skywalker::control::VelocityMotor axis{drive,makeConfig()};
  int r=bus.attach(drive);
  if (!r) r=bus.start();
  if (!r) r=axis.configure(); // bus已经启动，motor仍未使能
  if (r<0) return r;
  auto deadline=k_uptime_get()+3000;
  while (!drive.ready() && k_uptime_get()<deadline) k_msleep(1);
  if (!drive.ready()) return -ETIMEDOUT;
  r=axis.reset();
  if (!r) r=drive.enable();
  if (r<0) return r;
  deadline=k_uptime_get()+1000;
  while (!drive.active() && k_uptime_get()<deadline) k_msleep(1);
  if (!drive.active()) { (void)drive.disable(); return -ETIMEDOUT; }
  auto previous=k_uptime_get();
  for (;;) {
    k_msleep(5);
    const auto now=k_uptime_get();
    r=axis.update(2.0f,float(now-previous)*0.001f);
    previous=now;
    if (!r) r=bus.commit().error;
    if (r<0) { (void)drive.disable(); return r; }
  }
}`,notes:'配置值和GM6020设备信息仅示范，须按机构替换。开启SKYWALKER_LIB_MOTOR_CONTROL和其依赖。configure成功后禁止应用再drive.setCurrent，否则-EACCES；上层只能用axis.update。位置闭环的完整配置参考dji_position_control样例。'}],
      lifecycle:['构造Motor、总线、闭环对象；建立Group并attach。','bus.start成功后configure，硬件还未使能；绑定唯一执行量生产者。','等反馈稳定和有效参考→可选显式reset→Motor/Group enable→等Active。','控制周期调用update，随后每条总线commit；首代次周期先零输出再建立闭环。','反馈时效/速度/温度/dt错误触发停机；修复原因后重新建立参考并按故障类型清除。'],
      pitfalls:['configure的顺序必须在bus.start之后、enable之前；先enable再configure会-EBUSY。','有封装生产者时直接Motor::setCurrent/setTorque会-EACCES，不能混合两个输出所有者。','DM纯速度模式不支持主控NewtonMeter封装；要主控闭环需DM MIT模式。','温度最大值0明确关闭保护；M2006不能依赖不存在的温度反馈。','位置参考丢失后reseedPosition需要真实机械依据；把旧角度原样塞回不能恢复未知圈数。','update并不发CAN；漏commit会导致命令有效期耗尽。'],
      config:[{name:'CONFIG_SKYWALKER_LIB_MOTOR_CONTROL',description:'需CPP、统一电机子系统和至少一个品牌；包含纯控制库。'},{name:'EffortUnit',description:'DJI=安培、DM MIT=牛米，系数与输出限幅必须同单位。'},{name:'MotorSafety',description:'测量最大速度必须为正且覆盖目标速度范围；温度>0要求有效温度能力。'},{name:'PositionReference',description:'按机构选择相对锚点、驱动累计或最近绝对圈；GM6020 encoder_zero_ticks决定绝对零点。'}]
    },
    {
      id:'boards',title:'板级：MC02 / RoboMaster Type-C',category:'工程与板级',
      summary:'把物理引脚、外设、alias、时钟、DMA与电源连接交给Zephyr，业务参数保留在应用配置。',
      responsibility:'DTS描述物理设备与连接，overlay选择应用实际使用的设备，Kconfig决定哪些实现进入构建，board_config.hpp提供电机ID/限幅/机械方向/链路身份。它们是四个不同层次。',
      status:'ready',statusNote:'两套板级DTS、默认配置和烧录runner已存在。上车程序仍须确认具体电机/串口/CAN拓扑和电源连接；connections_configured=false是应用中的显式未配置状态，不能仅修改DTS就认为整车已接入。',
      source:[{label:'MC02设备树',path:'boards/damiao/dm_mc02/dm_mc02.dts'},{label:'MC02默认配置',path:'boards/damiao/dm_mc02/dm_mc02_defconfig'},{label:'MC02烧录配置',path:'boards/damiao/dm_mc02/board.cmake'},{label:'Type-C设备树',path:'boards/rm_typec/rm_typec.dts'},{label:'Type-C默认配置',path:'boards/rm_typec/rm_typec_defconfig'},{label:'双IMU板级参数示例',path:'samples/imu/dual_imu/src/board_config.hpp'},{label:'云台接线参数示例',path:'samples/robotics/gimbal_control/src/board_config.hpp'}],docs:[{label:'板级Markdown',path:'docs/getting-started/boards.md'},{label:'DMA与缓存',path:'docs/guides/uart-dma.md'}],depends:[],
      interfaces:[
        api('DEVICE_DT_GET(DT_NODELABEL(can1)); DEVICE_DT_GET(DT_ALIAS(remote_uart)); DEVICE_DT_GET(DT_ALIAS(accel0)); DEVICE_DT_GET(DT_ALIAS(gyro0))', '取得Zephyr已描述设备的指针；node label指DTS节点标号，alias指应用连接名称。获取指针不启动业务驱动或允许电机输出。', [['DT_NODELABEL(name)','直接选择节点label，例如物理can1/usart1。'],['DT_ALIAS(name)','选板或overlay约定的alias，DTS中横线在C宏里变下划线：remote-uart→remote_uart。']], 'const device*；缺节点/未生成device通常是编译或链接错误，不是运行期空指针。', '构造/初始化阶段使用；先device_is_ready，再交给Motor/UART/IMU等模块。', 'Type-C的alias usart1指USART6，node label usart1指物理USART1，两者不可混淆；各应用overlay可能改alias。'),
        api('bool device_is_ready(const device *dev)', '确认Zephyr驱动初始化成功，只证明设备层可用；不证明远端电机在线、UART协议正确或传感器姿态已初始化。', [['dev','从DEVICE_DT_GET得到的设备。']], 'true已就绪；false应用通常返回-ENODEV。', '初始化线程和必要的读取位置，无业务I/O。', '各模块还需要自己的start/init与反馈检查。'),
        api('PWM_DT_SPEC_GET(DT_ALIAS(imu_heater))', '取得加热PWM设备、通道、周期和极性，交给ImuHeater::Config::pwm。MC02为heat通道4，Type-C为通道1；默认PWM周期20ms。', [['imu_heater','板级alias，不是自动温控算法。']], 'pwm_dt_spec值。', '初始化阶段，真实写PWM由ImuHeater所有者线程完成。', 'PWM spec存在不表示温控已启动或有效温度已就绪；缺alias无法编译。'),
        api('int regulator_enable(const device *dev)', 'MC02的power1/power2固定电源口默认regulator-boot-off，应用显式启用后才给相应XT30支路供电。', [['dev','DEVICE_DT_GET(DT_NODELABEL(power1)) 或power2，先device_is_ready。']], '0成功；负值由Zephyr regulator驱动返回。', '启动线程，不能假设所有板卡都有power1/power2；用DT_NODE_EXISTS条件适配。', '确认支路连接后启用；开启电源不等于Motor软件enable，仍须等待安全准备与反馈。'),
        api('app.overlay + prj.conf + src/board_config.hpp', '三种应用配置入口：overlay声明物理资源/alias；prj.conf打开模块；board_config提供真实协议参数与机械约定。', [['app.overlay','启用CAN/UART/传感器、设波特率/引脚、按应用绑定alias。'],['prj.conf','CAN、UART API、CPP、FPU、业务模块Kconfig选项。'],['board_config.hpp','电机地址/模式/零点/量程，速度电流力矩限幅，传输角色和connections_configured。']], '构建时配置，不是运行期接口。', '先把本应用连接参数填写完整，再构建；上车固件初始化应据connections_configured决定是否允许运动。', 'alias正确并不保证协议一致；修改overlay需pristine build。不要把电机型号/Group写成物理DTS驱动节点。')
      ],
      examples:[{title:'MC02：显式开启所需XT30，并取得物理CAN',language:'cpp',code:`#include <zephyr/device.h>
#include <zephyr/drivers/regulator.h>
#include <drivers/motor/can_bus.hpp>
#include <cerrno>

int initializeBoardPower() {
#if DT_NODE_EXISTS(DT_NODELABEL(power1))
  // 仅在确认本车电机供电确实使用XT30_1后调用。
  const device *power=DEVICE_DT_GET(DT_NODELABEL(power1));
  if (!device_is_ready(power)) return -ENODEV;
  const int r=regulator_enable(power);
  if (r<0) return r;
#endif
  const device *can=DEVICE_DT_GET(DT_NODELABEL(can1));
  return device_is_ready(can) ? 0 : -ENODEV;
}

// 后续业务初始化仍需：构造长寿命Motor/CanBus，attach，start，
// 等ready，enable，等active；不能因电源已开启直接写运动目标。
`,notes:'此例沿用现有DM和恢复样例的Zephyr调用；连接确认由应用负责。Type-C无power1节点时跳过MC02专有逻辑。board device pointer与CanBus/Motor对象生命周期是两个层次。'},{title:'构建入口与物理外设配置',language:'cpp',code:`// C++取设备；这是node label，始终选择物理can1。
const device *can = DEVICE_DT_GET(DT_NODELABEL(can1));
// UART按应用alias取；可能由app.overlay重新映射。
const device *telemetry = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
// MC02: telemetry=USART1，remote=UART5，console/shell=USART10。
// Type-C: telemetry=USART6，remote=USART3，console/shell=USART1。
`,notes:'板目标：dm_mc02/stm32h723xx、rm_typec。示例构建命令：west build -p -b dm_mc02/stm32h723xx -d build/dual_imu samples/imu/dual_imu。此任务仅展示命令，不运行固件构建或烧录。'}],
      lifecycle:['确定板卡与实际连接，选board目标。','overlay绑定物理设备和alias；prj.conf纳入模块并选择UART/FPU/缓存策略。','应用取得device并检查ready；按实际供电需要显式开启MC02电源支路。','依赖资源构造长寿命业务对象并按模块生命周期启动。','机械方向、零点、量程、通信身份与控制增益确认后才解除应用connections_configured门槛。'],
      pitfalls:['MC02有can1/2/3，Type-C有can1/2；不能照抄第三路CAN或RS485 alias。','当前Type-C DTS的console/shell是USART1，telemetry是USART6；alias usart1也指USART6，不能从名字猜物理设备。','MC02的XT30电源默认关闭；电机离线可能是供电未显式启用而非CAN解码问题。','H7 DMA缓存一致性需__nocache缓冲和CONFIG_NOCACHE_MEMORY；业务对象不一定全部放nocache，DMA实际字节存储需按接口要求放置。','不同UART API/console/VOFA/协议收发器不能各自同时独占同一物理UART。','DTS只负责硬件描述，Group、Motor型号和电机限幅仍由C++应用配置。'],
      config:[{name:'dm_mc02/stm32h723xx',description:'STM32H723，DTS CPU480MHz；CAN1/2/3，BMI088 SPI2，RS485 USART2/3，默认console USART10。'},{name:'rm_typec',description:'STM32F407，DTS CPU168MHz；CAN1/2，BMI088，UART1/3/6，console USART1、telemetry USART6。'},{name:'SKYWALKER_OPENOCD_PROBE',description:'board.cmake支持cmsis-dap、stlink、stlink-hla；烧录探针依板实际连接。'},{name:'CONFIG_NOCACHE_MEMORY / CONFIG_FPU_SHARING',description:'MC02默认启用nocache区；含浮点工作线程的样例显式配置FPU共享。'},{name:'connections_configured',description:'应用/云台样例中的接线确认开关；应和真实硬件/协议/机械限制一致，不能用一次编译成功代替确认。'}]
    }
  ]);
})();
