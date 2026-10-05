/* 接口来自 include/ 与 lib/ 的现行实现；示例中的 board_config 由对应样例提供。 */
window.SKYWALKER_MODULES = (window.SKYWALKER_MODULES || []).concat([
  {
    id: 'uart', title: '异步 UART 与 DMA', category: '通信与输入',
    summary: '把 UART 回调产生的字节复制进有界队列，业务线程按接收时刻解析；同一设备只有一个所有者。',
    responsibility: 'AsyncUart 负责双 RX 缓冲、整批 TX 复制、断流代次与重试。它不识别 DR16、裁判、视觉或板间消息，也不判断业务在线。',
    status: 'ready', statusNote: '已用于遥控、裁判、视觉和板间 UART/RS485；实际设备、DMA 通道与 nocache 区域仍由板级配置确定。',
    source: [{label: 'AsyncUart 公共接口', path: 'include/communication/async_uart.hpp'}, {label: 'UART/DMA 实现', path: 'lib/communication/async_uart.cpp'}],
    docs: [{label: 'UART DMA 配置', path: 'docs/guides/uart-dma.md'}, {label: '通信模块', path: 'docs/modules/communication/communication.md'}],
    depends: ['boards'],
    interfaces: [
      {signature: 'AsyncUart(const device *uart, DmaBuffers &buffers)', description: '绑定独占 UART 与外部 DMA 存储；构造不注册回调。DmaBuffers 含两个 128 字节 RX 缓冲和一个 256 字节 TX 缓冲，均按 32 字节对齐。', parameters: [{name: 'uart', meaning: 'Zephyr UART 设备，支持 async API，且没有被 console、VOFA 或其他接收器占用'}, {name: 'buffers', meaning: '实例专属、静态存活的 __nocache DmaBuffers；只有缓冲区放 nocache'}], returns: '构造对象；不可复制。', context: '构造后由一个通信线程拥有 init/service/read/send；对象和缓冲区持续存活至重启。', errors: '构造不检查设备；实际错误由 init 返回。'},
      {signature: 'int AsyncUart::init()', description: '注册 UART 回调并开启 RX；只成功注册一次。回调只复制字节、维护缓冲与完成状态。', parameters: [], returns: '0：RX 启动成功；负 errno：初始化或驱动错误。', context: '唯一通信线程调用；不是 ISR 接口。', errors: '-ENODEV：空设备或设备未就绪；-EALREADY：回调已注册；其他值来自 uart_callback_set/uart_rx_enable。回调已注册但 RX 启动失败后，继续 service，不再次 init。'},
      {signature: 'int AsyncUart::service(std::uint64_t now_ms)', description: '持续推进接收重启与发送超时中止。即使没有字节或发生错误也要周期调用；RX 重试间隔为 100 ms。', parameters: [{name: 'now_ms', meaning: '本机单调时间，单位 ms，例如 k_uptime_get()'}], returns: '0：当前 RX 可服务；-EAGAIN：等待重试；负值：错误。', context: '唯一通信线程调用，不能阻塞其他实时控制线程。', errors: '-EACCES：尚未 init；其他值来自重新开启 RX。TX busy 仅在完成/中止回调到达后释放。'},
      {signature: 'int AsyncUart::read(RxChunk &out)', description: '非阻塞复制最多 64 字节及原始接收时间；队列深度为 8。断流或溢出时先通知调用者，淘汰旧代次字节。', parameters: [{name: 'out', meaning: '成功时写入 bytes、size、timestamp_ms 和 generation'}], returns: '0：取到一个块；-EAGAIN：队列空；-EOVERFLOW：接收代次变化。', context: '与 service 同一个通信线程；按块原始 timestamp_ms 交给解析器。', errors: '-EACCES：未初始化。-EOVERFLOW 后必须丢弃上层 parser 半帧；不能把溢出前后字节拼成一帧。'},
      {signature: 'int AsyncUart::send(const std::uint8_t *bytes, std::size_t size, std::uint32_t timeout_ms = 20)', description: '先复制整个批次到实例 TX 缓冲，再启动异步发送；一次只允许一个批次。返回成功后调用者可以复用原始 bytes。', parameters: [{name: 'bytes', meaning: '有效字节数组'}, {name: 'size', meaning: '1～256 字节'}, {name: 'timeout_ms', meaning: '整批发送期限，1～1000 ms，默认 20 ms'}], returns: '0：驱动接受发送；不保证远端接收或执行。', context: '唯一通信线程调用；不要在 busy 时积压运动命令。', errors: '-EINVAL：未初始化、空指针、长度或期限非法；-EAGAIN：TX 忙；其他负值来自 UART 驱动。'},
      {signature: 'bool AsyncUart::txBusy() const; int AsyncUart::txError() const; atomic_val_t AsyncUart::droppedChunks() const', description: '读取发送占用、最近发送结果与累计丢块数，供通信诊断。', parameters: [], returns: 'busy 为真表示 DMA 仍占有 TX；txError 在 busy 变 false 后稳定；droppedChunks 返回累计计数。', context: '原子读；I/O 的生命周期仍由唯一通信线程推进。', errors: '发送中止时 txError=-ECANCELED；初始化/发送成功不等于业务在线。'}
    ],
    examples: [{title: '线程中读取 DR16 字节并正确处理断流', language: 'cpp', code: '#include <communication/async_uart.hpp>\n#include <communication/remote/remote_service.hpp>\n#include "board_config.hpp"\nusing namespace skywalker::communication;\n\nstatic AsyncUart::DmaBuffers dma __nocache;\nstatic AsyncUart uart(board_config::remote_uart, dma);\nstatic RemoteService parser({}, {});\n\n// 此函数只由一个常驻通信线程调用。\nvoid communicationTask() {\n    int ret = uart.init();\n    for (;;) {\n        const auto now = static_cast<std::uint64_t>(k_uptime_get());\n        ret = uart.service(now);\n        if (ret == -EACCES) ret = uart.init();\n        if (ret == 0) {\n            AsyncUart::RxChunk chunk{};\n            for (unsigned budget = 0; budget < 8; ++budget) {\n                const int rr = uart.read(chunk);\n                if (rr == -EOVERFLOW) { parser.discardPartial(); continue; }\n                if (rr != 0) break;\n                parser.processBytes(chunk.bytes, chunk.size, chunk.timestamp_ms);\n            }\n        }\n        parser.processBytes(nullptr, 0, now);\n        k_sleep(K_MSEC(1));\n    }\n}', notes: 'board_config::remote_uart 可参考 DR16 样例。上车通常直接使用 RemoteReceiver，不额外创建第二个 AsyncUart。错误日志与跨线程快照可按应用补齐。'}],
    lifecycle: ['静态分配设备所有者及实例专属 __nocache DMA 缓冲。', '通信线程 init 一次，然后持续 service → read → parser；send 与这些操作由同一个线程调用。', '遇到 -EOVERFLOW 立即 discardPartial；即使没数据，也用零长度输入推进解析超时。', '不销毁、不移动对象；没有 stop 或热替换设备接口。'],
    pitfalls: ['同一 UART 不能同时交给 AsyncUart 与 VOFA/console/shell。', '复制字节和快照时保留原始接收时间；读取时间不能冒充采样时间。', 'DCACHE 开启时需要 CONFIG_NOCACHE_MEMORY；不能仅靠 alignas 解决 DMA 缓存一致性。', 'TX 超时发起 abort 后仍须等待回调；不要复写还被 DMA 占用的缓冲。'],
    config: [{name: 'CONFIG_SKYWALKER_LIB_COMMUNICATION / CONFIG_SKYWALKER_UART_TRANSPORT', description: '启用通信库及 UART async transport；后者选择 UART_ASYNC_API。'}, {name: '设备树与 DMA', description: '配置 UART 引脚、波特率、独占 DMA 通道和 nocache 区域；遥控、视觉和板间串口各自独立。'}, {name: '时间单位', description: 'AsyncUart 时间是 ms；视觉 core::TimeUs 是 μs，适配时乘 1000。'}]
  },
  {
    id: 'remote', title: 'DR16 遥控输入', category: '通信与输入',
    summary: '从 18 字节 DR16 帧发布摇杆、开关、鼠标和键盘快照；脱机时保持数据但撤销 online。',
    responsibility: 'Dr16Decoder 解码单帧，RemoteService 负责流式对齐和超时，RemoteReceiver 拥有 UART worker。遥控输入如何变成运动模式由命令层决定。',
    status: 'ready', statusNote: 'DR16 receiver、独立样例和命令来源适配器已实现；sentry_gimbal 已接入 RemoteSource，硬件接线门禁仍需配置。',
    source: [{label: '接收器接口', path: 'include/communication/remote/remote_receiver.hpp'}, {label: '流式服务', path: 'include/communication/remote/remote_service.hpp'}, {label: '单帧解码', path: 'include/communication/remote/dr16_decoder.hpp'}, {label: '消息字段', path: 'include/robotics/messages/remote.hpp'}],
    docs: [{label: 'DR16 接线与样例', path: 'samples/communication/dr16/README.md'}, {label: '命令来源服务', path: 'docs/modules/robotics/command-service.md'}],
    depends: ['uart'],
    interfaces: [
      {signature: 'RemoteReceiver(const device *uart, AsyncUart::DmaBuffers &dma, const Config &config)', description: '创建独占 UART 的遥控接收器，构造不启动 I/O。Config 包含 decoder 与 remote 两组配置。', parameters: [{name: 'uart', meaning: 'DR16 所接 UART 设备'}, {name: 'dma', meaning: '专属、静态 __nocache 缓冲'}, {name: 'config', meaning: '通道合法范围、中心死区、帧间间隔与离线超时'}], returns: '不可复制的接收器。', context: '接收器及 DMA 需持续存活至重启，包括 start 出错以后。', errors: '配置由 start 校验，设备初始化结果由 Snapshot 报告。'},
      {signature: 'int RemoteReceiver::start()', description: '创建后台线程。0 只表示调度成功，不能据此认定遥控器已在线。', parameters: [], returns: '0：worker 已调度；负 errno：启动失败。', context: '线程上下文，只启动一次；若交给 RemoteSource，则由 CommandManager 启动，不要预先手动 start。', errors: '-EINVAL：通道范围/死区/超时配置非法；-EALREADY：重复启动。UART 错误异步记录在 snapshot.state/uart_error。'},
      {signature: 'int RemoteReceiver::snapshot(Snapshot &out)', description: '复制完整遥控与诊断信息，并按当前时刻重新判定 online。读者需要保留自己的已初始化 out。', parameters: [{name: 'out', meaning: '读者独有的 Snapshot{}；包含 remote、state、rx_chunks、resets、dropped、uart_error'}], returns: '0：复制成功，但可能离线；-EAGAIN：暂时取不到锁，保留原值并仍使旧 online 过期。', context: '线程上下文，允许多个读者；不是消费队列。', errors: '不得把 -EAGAIN 当作一条新帧；检查 out.remote.online 和原始 stamp，不只检查返回值。'},
      {signature: 'int RemoteService::processBytes(const std::uint8_t *bytes, std::size_t size, std::uint64_t now_ms); int RemoteService::snapshot(std::uint64_t now_ms, robotics::RemoteState &out) const', description: '自行管理通信线程时使用：流式累积 18 字节，坏帧滑动一个字节重同步；snapshot 复制最近合法帧并按 timeout 判断在线。', parameters: [{name: 'bytes / size', meaning: '原始字节；nullptr 与 size=0 用于推进半帧超时'}, {name: 'now_ms', meaning: '处理字节时传原始接收时间；读快照时传当前单调 ms'}, {name: 'out', meaning: '遥控状态输出'}], returns: 'processBytes：0 或 -EINVAL；snapshot：0 在线、-EAGAIN 尚无合法帧、-ESTALE 已过期（仍复制 out）。', context: '同一个 owner 线程调用；跨线程前另行发布值副本。', errors: '非空长度配空指针返回 -EINVAL；UART 溢出后调用 discardPartial()。'},
      {signature: 'int Dr16Decoder::decodeFrame(const std::uint8_t *bytes, std::size_t size, std::uint64_t timestamp_ms, robotics::RemoteState &out); int Dr16Decoder::reset()', description: '解码固定 18 字节帧，通道减去中心并应用死区，生成单调递增本地序号；reset 清零统计与序号。', parameters: [{name: 'bytes / size', meaning: '完整 DR16 帧，size 必须等于 18'}, {name: 'timestamp_ms', meaning: '接收时刻 ms'}, {name: 'out', meaning: '成功时写入 RemoteState，鼠标按有符号 16 位解析'}], returns: '0：合法；负 errno：拒绝且不替换 out。', context: '解码器单线程所有；不直接接受任意 UART 分块。', errors: '-EINVAL：参数或配置非法；-EBADMSG：通道、开关或鼠标按键字段非法。'}
    ],
    examples: [{title: '独立启动并读取遥控快照', language: 'cpp', code: '#include <communication/remote/remote_receiver.hpp>\n#include "board_config.hpp"\nusing namespace skywalker::communication;\nstatic AsyncUart::DmaBuffers dma __nocache;\nstatic RemoteReceiver remote(board_config::remote_uart, dma, {});\n\nint main() {\n    const int ret = remote.start();\n    if (ret < 0) return ret;\n    RemoteReceiver::Snapshot frame{}; // 每个读者自己保留一份\n    for (;;) {\n        const int copied = remote.snapshot(frame);\n        if (copied == 0 && frame.remote.online) {\n            const auto right_x = frame.remote.analog.right_x;\n            // right_x 是中心化原始值；运动映射交给命令层。\n            (void)right_x;\n        }\n        k_sleep(K_MSEC(10));\n    }\n}', notes: '使用 samples/communication/dr16 的 board_config 和 UART 配置。正式命令服务通过 RemoteSource 读取，不直接在控制线程解析 DR16。'}],
    lifecycle: ['准备 UART、DMA 与 RemoteReceiver；或把 receiver 引用交给 RemoteSource。', 'start 创建 worker，观察 State::Running 与 remote.online，二者分别代表串口服务和有效输入。', '后台每约 1 ms 处理有界字节预算；执行线程只读快照。', '断流时撤销 online；重连后使用新帧和新的执行授权，不凭遗留摇杆值重新使能。'],
    pitfalls: ['bytes 16～17 默认保留，decode_wheel=false；只有明确确认接收机提供第五通道才打开。', 'RemoteState 的摇杆是原始中心化值，不能直接当 m/s 或 rad/s。', '读取快照不消费数据、不延长有效期；0 不等于在线。', 'RemoteReceiver 不支持 stop、重复 start 或运行中析构。'],
    config: [{name: 'Dr16Decoder::Config', description: '默认 center=1024、min=364、max=1684、center_deadband=10；decode_wheel 默认 false。'}, {name: 'RemoteService::Config', description: '默认 offline_timeout_ms=100、assembly_gap_ms=10；全部使用 ms。'}, {name: 'CONFIG_SKYWALKER_REMOTE_DR16 / CONFIG_SKYWALKER_REMOTE_RECEIVER', description: '分别启用纯解码服务与后台接收器；接收器依赖 UART_TRANSPORT。'}, {name: 'CONFIG_SKYWALKER_REMOTE_RX_STACK_SIZE / PRIORITY', description: '默认栈 3072 字节、线程优先级 6；UART 参数按接收机和板级配置提供。'}]
  },
  {
    id: 'referee', title: '裁判许可与功率预算', category: '通信与输入',
    summary: '校验版本化裁判帧，分别记录机构输出许可、功率限额和缓冲能量的有效期。',
    responsibility: 'RefereeParser 负责 CRC 与已支持命令，RefereeService 计算在线状态，RefereeReceiver 用 poll 推进 UART。裁判许可约束运动命令，不能与遥控或视觉竞争控制来源。',
    status: 'ready', statusNote: '当前实现 profile 为 Rm2026V1_3，识别 0x0201/0x0202；sentry_gimbal 的板级配置仍是 Unspecified，需要明确配置后才有有效许可。',
    source: [{label: '轮询接收器', path: 'include/communication/referee/referee_receiver.hpp'}, {label: '流式解析器', path: 'include/communication/referee/referee_parser.hpp'}, {label: '支持的版本与线格式', path: 'include/communication/referee/referee_protocol.hpp'}, {label: '许可与功率消息', path: 'include/robotics/messages/referee.hpp'}, {label: '解析实现', path: 'lib/communication/referee.cpp'}],
    docs: [{label: '裁判样例', path: 'samples/communication/referee/README.md'}, {label: '命令与许可', path: 'docs/modules/robotics/command-service.md'}, {label: '通信模块', path: 'docs/modules/communication/communication.md'}],
    depends: ['uart'],
    interfaces: [
      {signature: 'RefereeReceiver(const device *device, AsyncUart::DmaBuffers &dma, RefereeVersion version, std::uint32_t timeout_ms = 500)', description: '绑定裁判 UART、DMA 与明确版本。构造不做 I/O，不按 payload 长度猜版本。', parameters: [{name: 'device', meaning: '裁判串口设备，独占'}, {name: 'dma', meaning: '静态 __nocache 缓冲'}, {name: 'version', meaning: '当前可用 profile：RefereeVersion::Rm2026V1_3；Unspecified 不能获得有效许可'}, {name: 'timeout_ms', meaning: '接收状态离线阈值，默认 500 ms'}], returns: '轮询式接收器；没有独立 start()。', context: '对象和缓冲持续存活至重启；一个线程拥有 poll/error。', errors: '实际驱动与解析错误由 poll 后 error() 报告。'},
      {signature: 'robotics::RefereeState RefereeReceiver::poll(std::uint64_t now_ms); int RefereeReceiver::error() const', description: '懒初始化、持续恢复 UART、最多读取 8 个块、丢弃断流半帧，并返回按当前时刻判断的裁判值副本。', parameters: [{name: 'now_ms', meaning: '当前本机单调时间 ms'}], returns: 'poll 返回 RefereeState，可能 online=false 或 stamp 无效；error 返回最近 UART/解析错误。', context: '唯一接收线程；若由 RefereePermissionSource 驱动，则不要再在应用中直接 poll 同一对象。', errors: '初始化失败约 100 ms 后重试；-EOVERFLOW 丢弃半帧；error=0 仅表示 I/O 没有当前错误，不能替代许可有效性检查。'},
      {signature: 'int RefereeParser::reset(); int RefereeParser::consume(const std::uint8_t *bytes, std::size_t size, std::uint64_t timestamp_ms)', description: '纯解析入口：reset 检查版本并清状态；consume 校验帧头、CRC8、长度与 CRC16，接受支持命令并记录统计。', parameters: [{name: 'bytes / size', meaning: '字节流；零长度用于推进半帧超时'}, {name: 'timestamp_ms', meaning: '数据接收的原始 ms 时间'}], returns: '0 或负 errno；state()/stats() 提供 owner 线程内引用。', context: '单线程解析；不得把内部引用跨线程保存。', errors: '未指定或不支持的版本会返回配置错误；坏 CRC/长度/未知命令由 Stats 记录并跳过，不把坏帧伪装成新的许可。'},
      {signature: 'int RefereeService::processBytes(const std::uint8_t *bytes, std::size_t size, std::uint64_t now_ms); int RefereeService::snapshot(std::uint64_t now_ms, robotics::RefereeState &out) const', description: '自定义输入线程的服务封装；snapshot 根据整体 stamp 判在线，但机器人许可与功率字段各有自己的 stamp。', parameters: [{name: 'bytes / size', meaning: '带原始接收时间的裁判字节'}, {name: 'now_ms', meaning: 'processBytes 的接收时间，或 snapshot 的当前时间'}, {name: 'out', meaning: '裁判输出副本'}], returns: 'snapshot：0 在线；-EAGAIN 尚无合法消息；-ESTALE 过期但仍写出 out。', context: '唯一 owner 调用；其他线程读另行发布的副本。', errors: '调用 discardPartial() 丢弃断流半帧；整体 online 不能证明每个许可或功率字段新鲜。'}
    ],
    examples: [{title: '使用明确版本持续轮询裁判', language: 'cpp', code: '#include <communication/referee/referee_receiver.hpp>\n#include "board_config.hpp"\nusing namespace skywalker;\nstatic communication::AsyncUart::DmaBuffers dma __nocache;\nstatic communication::RefereeReceiver referee(\n    board_config::referee_uart, dma, communication::RefereeVersion::Rm2026V1_3);\n\nvoid refereeTask() {\n    for (;;) {\n        const auto now = static_cast<std::uint64_t>(k_uptime_get());\n        const auto state = referee.poll(now);\n        const auto &permit = state.robot.gimbal_output;\n        const bool gimbal_allowed = permit.valid && permit.enabled &&\n            robotics::isFresh(permit.stamp, now, 300);\n        // 在短锁内发布 state 副本；正式应用交给命令仲裁。\n        (void)gimbal_allowed;\n        k_sleep(K_MSEC(1));\n    }\n}', notes: 'board_config::referee_uart 来自对应样例/应用。正式 CommandManager 会通过 RefereePermissionSource 调用 poll，因此不另启第二个 owner。'}],
    lifecycle: ['按实际协议选择明确版本、UART 与 DMA。', '唯一线程不断 poll；断线时也继续轮询以推进恢复。', '许可进入命令仲裁，power.limit_stamp 与 power.stamp 分别进入底盘预算判断。', '每个机构读取自己的许可有效性；禁止用任何新裁判帧刷新所有字段的年龄。'],
    pitfalls: ['当前 parser 只实现列出的 profile 与两个命令；仓库存在新版协议 PDF 不等于 parser 自动兼容该版本。', '0x0201 更新功率限额，不刷新实际功率/缓冲能量 stamp。', '当前 profile 保留功率字段不会被当成实测 chassis_power_w；功率模型需要应用侧标定。', '裁判缺失、过期或禁止时，require_referee_for_motion=true 的命令链撤销相应运动许可。'],
    config: [{name: 'CONFIG_SKYWALKER_REFEREE / CONFIG_SKYWALKER_UART_TRANSPORT', description: '启用纯 parser 与 UART 接收能力；纯 parser 不要求 Receiver。'}, {name: 'RefereeVersion', description: '必须显式绑定实际兼容版本；sentry_gimbal 当前 Unspecified 是待配置项。'}, {name: 'timeout_ms 与 permission_timeout_ms', description: '接收器默认离线 500 ms；仲裁器默认按机构许可 300 ms 判断，两者职责不同。'}]
  },
  {
  "id": "interboard",
  "title": "双主控板间通信",
  "category": "通信与输入",
  "summary": "统一 v4 字节契约，七类消息、boot/生产序号与原年龄，UART/RS485/CAN 三后端。",
  "responsibility": "Transport 管物理批次；Endpoint 唯一通信线程推进收发，其他线程复制输入/状态，不根据电机 ready 决定目标发送。",
  "status": "ready",
  "statusNote": "v4 已实现，旧版本直接拒绝；恢复授权 generation 删除。真实两板应使用同版固件，整车默认 USART1 UART。",
  "source": [
    {
      "label": "interboard_protocol.hpp",
      "path": "include/communication/interboard/interboard_protocol.hpp"
    },
    {
      "label": "interboard_endpoint.hpp",
      "path": "include/communication/interboard/interboard_endpoint.hpp"
    },
    {
      "label": "interboard.hpp",
      "path": "include/robotics/messages/interboard.hpp"
    },
    {
      "label": "interboard_endpoint.cpp",
      "path": "lib/communication/interboard_endpoint.cpp"
    },
    {
      "label": "interboard_codec.cpp",
      "path": "lib/communication/interboard_codec.cpp"
    }
  ],
  "docs": [
    {
      "label": "communication.md",
      "path": "docs/modules/communication/communication.md"
    },
    {
      "label": "interboard-transports.md",
      "path": "docs/modules/communication/interboard-transports.md"
    }
  ],
  "depends": [
    "uart",
    "boards"
  ],
  "interfaces": [
    {
      "signature": "ConfiguredInterBoardTransport(const Config &, AsyncUart::DmaBuffers *dma = nullptr)",
      "description": "一次选择 UART/RS485/CAN；对象和所选设备独占，纯 CAN 不需要 DMA。",
      "parameters": [],
      "returns": "构造不启动，后续 service/poll 推进。",
      "errors": "后端未编译/电气配置不支持 -ENOTSUP。",
      "context": "唯一通信线程使用，static 存活。"
    },
    {
      "signature": "InterBoardEndpoint(InterBoardTransport &, const Config &); void poll(uint64_t now_ms)",
      "description": "唯一通信 owner 约 1 ms 推进；命令/心跳/status 默认 100 ms，TX 默认 60 ms。",
      "parameters": [],
      "returns": "void；服务、解析和在线诊断由 snapshot 返回。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "poll 仅一个通信线程，其他 API 交换短锁值副本。"
    },
    {
      "signature": "void submit(const ChassisCommand &); void setReferee(const RefereeState &); void setStatus(const RunStatus &); void submitBigYaw(const BigYawRequest &); void setBigYawFeedback(const BigYawFeedback &)",
      "description": "复制原生产值与年龄；反复提交/重发同 producer 不续期，状态 stamp 由执行 owner 生产。",
      "parameters": [],
      "returns": "void；发送机会由 poll 安排，接受不代表远端执行。",
      "errors": "目标不会等待 ready/armed；真实 boot/link、原输入和数值仍需有效。",
      "context": "应用线程可发布值副本，不能修改源 stamp。"
    },
    {
      "signature": "int submitOperatorControl(const OperatorControl &); Snapshot snapshot() const",
      "description": "独立操作管理传 run_allowed/estop/clear event 与 boot/原年龄；snapshot 含七类消息、online/error/transport/parser_stats。",
      "parameters": [],
      "returns": "submit 0=复制；snapshot 为值副本。",
      "errors": "非云台 producer -EACCES，矛盾运行/停止/清除请求 -EINVAL；重复事件不改绑新 boot。",
      "context": "线程上下文，短锁交换。"
    },
    {
      "signature": "int InterBoardTransport::service(uint64_t); int read(RxChunk &); int send(const uint8_t *, size_t, uint32_t timeout_ms = 60)",
      "description": "批次完整复制，背压不积压运动目标；原始接收时刻随 chunk 保留。",
      "parameters": [],
      "returns": "0 成功；无数据/背压 -EAGAIN，断流 -EOVERFLOW。",
      "errors": "非法 -EINVAL，过大 -EMSGSIZE，未就绪 -EACCES；溢出丢半帧。",
      "context": "唯一通信 owner，不在电机执行 ISR 调用。"
    }
  ],
  "examples": [
    {
      "title": "独立通信与生产者",
      "language": "cpp",
      "code": "// 执行线程：RunStatus.stamp 来自本次实际 update。\nendpoint.setStatus(status);\nendpoint.submitBigYaw(fresh_request);\n// 通信线程：\nendpoint.poll(k_uptime_get());\nconst auto rx = endpoint.snapshot();\n// rx.online 只代表心跳，原输入和状态年龄分别检查。",
      "notes": "应用周期片段，构造与实物配置以链接源码为准。"
    }
  ],
  "lifecycle": [
    "两端同版 v4，选择同后端并绑定独占资源。",
    "静态构造 Transport/Endpoint 与 DMA，持续 poll。",
    "执行/管理 owner 发布保留源年龄的目标与状态。",
    "按真实 boot/link 更新上下文；Motor 恢复不改变输入身份。",
    "状态停止生产后撤销旧 ready/armed，通信心跳可继续。"
  ],
  "pitfalls": [
    "v4 没有 resume_generation；不要用旧恢复教程。",
    "frame sequence 是发送次数，不是 producer sequence。",
    "RS485 主动 IMU 协议不是板间轮询封装。",
    "底盘电机占满 CAN1/2/3，默认板间 UART；不能共享控制器给 CAN 后端。"
  ],
  "config": [
    {
      "name": "kInterBoardProtocolVersion",
      "description": "4；最大 payload 128、frame 142 B，CRC16、小端；七类消息。"
    },
    {
      "name": "状态年龄",
      "description": "RunStatus/BigYawFeedback 原始生产年龄；反复发送不续期。"
    }
  ]
},
  {
  "id": "vision",
  "title": "视觉链路与 AB 协议",
  "category": "通信与输入",
  "summary": "接收独立的 yaw/pitch 目标请求，提供带参考系与有效期的值；反馈由应用复制姿态等实测数据。",
  "responsibility": "AbProtocol 定义 29 字节下行/43 字节上行线协议；VisionLink 校验参考、时间与反馈；VisionReceiver 驱动独占 UART。模块不持有 IMU，不仲裁机器人命令，不操作电机和发射机构。",
  "status": "ready",
  "statusNote": "AB 接收已接入整车可选观察/执行阶段；自动执行要求头部参考匹配。弹速/弹数来源与显式视觉会话待补，反馈 TX 默认关闭。",
  "source": [
    {
      "label": "VisionReceiver",
      "path": "include/communication/vision/vision_receiver.hpp"
    },
    {
      "label": "VisionLink",
      "path": "include/communication/vision/vision_link.hpp"
    },
    {
      "label": "AB codec",
      "path": "include/communication/vision/ab_protocol.hpp"
    },
    {
      "label": "目标与反馈值类型",
      "path": "include/communication/vision/vision_types.hpp"
    },
    {
      "label": "链路校验实现",
      "path": "lib/communication/vision_link.cpp"
    }
  ],
  "docs": [
    {
      "label": "AB 字节布局与坐标约定",
      "path": "docs/modules/communication/vision.md"
    },
    {
      "label": "视觉台架样例",
      "path": "samples/communication/vision/README.md"
    },
    {
      "label": "视觉/IMU 流程场景",
      "path": "tests/vision_imu/README.md"
    }
  ],
  "depends": [
    "uart"
  ],
  "interfaces": [
    {
      "signature": "VisionReceiver(const device *uart, AsyncUart::DmaBuffers &dma, VisionProtocol &protocol, const Config &config); int VisionReceiver::start()",
      "description": "绑定 UART、DMA 与长生命周期 codec；start 初始化 Link 并启动独立 worker。反馈周期默认 0，仅收包。",
      "parameters": [
        {
          "name": "uart",
          "meaning": "专属视觉串口"
        },
        {
          "name": "dma",
          "meaning": "专属静态 __nocache 缓冲"
        },
        {
          "name": "protocol",
          "meaning": "AbProtocol 或其他实现 VisionProtocol 的对象，必须持续存活"
        },
        {
          "name": "config",
          "meaning": "Link 的超时配置与 feedback_period_us"
        }
      ],
      "returns": "start：0 表示 worker 创建；UART 就绪由 Snapshot::state 与 uart_error 表示。",
      "context": "线程上下文，启动一次，无停止/销毁后重建 API；交给 VisionSource 时由 manager 启动。",
      "errors": "-EALREADY：已启动；其他值为 Link/codec 的配置错误。线程创建成功也可能随后 InitFailed 并重试 UART。"
    },
    {
      "signature": "VisionReceiver::Snapshot VisionReceiver::snapshot() const",
      "description": "读取命令、协议统计和 RX/TX 诊断；在读者侧按当前时间重新计算 aim_fresh。",
      "parameters": [],
      "returns": "值副本：link.aim、link.aim_fresh、state、uart_error、dropped、resets、tx_frames、tx_skipped、tx_error。",
      "context": "多个线程可读，内部短锁；读取不产生新命令、不刷新时间。",
      "errors": "必须同时检查 aim_fresh、aim.stamp.valid 与 aim.value.control_requested；合法 stop 可能是新鲜消息，但不能控制电机。"
    },
    {
      "signature": "int VisionReceiver::setFeedback(const Feedback &feedback); int VisionLink::setFeedback(const Feedback &feedback)",
      "description": "复制应用提供的反馈，保留姿态、gyro、弹速、计数各自的原始时间；不填造任何缺失测量。",
      "parameters": [
        {
          "name": "feedback",
          "meaning": "mode、reference、Hamilton wxyz 四元数（B→W）、body-frame gyro(rad/s)、弹速(m/s)、累计计数，均带各自 stamp"
        }
      ],
      "returns": "0：保存副本；负 errno：拒绝。",
      "context": "Link 初始化后其他线程可设置；实际编码与 UART 发送由唯一 worker 进行。",
      "errors": "-EACCES：Link 尚未初始化；-EINVAL：未知 mode、未来时间、非法参考/非有限值、无效四元数或负弹速。"
    },
    {
      "signature": "int VisionLink::init(); int VisionLink::processRxBytes(const std::uint8_t *bytes, std::size_t size, core::TimeUs rx_us); void VisionLink::discardPartial()",
      "description": "不使用 Receiver 时的纯链路入口；init 校验配置并清状态，processRxBytes 流式解析，零长度推进半帧超时，断流时 discardPartial。",
      "parameters": [
        {
          "name": "bytes / size",
          "meaning": "原始字节流，nullptr 只允许 size=0"
        },
        {
          "name": "rx_us",
          "meaning": "接收时间 μs；AsyncUart timestamp_ms 乘 1000"
        }
      ],
      "returns": "0 或负 errno；命令更新为新的 Measurement<AimCommand> 本地序号。",
      "context": "processRxBytes/discardPartial/encodeFeedback 只允许一个 owner；snapshot/setFeedback 可以由其他线程调用。",
      "errors": "-EALREADY：重复 init；-EACCES：未 init；-EINVAL：配置、空指针、未来时间或目标非法；-ESTALE：接收时间回退。"
    },
    {
      "signature": "int VisionLink::encodeFeedback(std::uint8_t *out, std::size_t capacity)",
      "description": "在当前时刻判断各测量有效期和姿态/gyro 时间差，再交给 codec 编码；AB 输出完整 43 字节。",
      "parameters": [
        {
          "name": "out",
          "meaning": "输出字节数组"
        },
        {
          "name": "capacity",
          "meaning": "AB 至少 43 字节"
        }
      ],
      "returns": "正数：编码字节数；负 errno：此次不能发送。",
      "context": "唯一协议 owner 编码；worker 忙时跳过本周期，下一周期取最新反馈。",
      "errors": "-EACCES：未初始化；-EINVAL：空输出或值非法；-EMSGSIZE：容量不足；-ENODATA：必要字段无效/过期；-ESTALE：参考不一致或姿态/gyro 不同步；-ERANGE：ZYX pitch 接近 ±π/2。"
    },
    {
      "signature": "AbProtocol(const Config &config); int AbProtocol::validateConfig() const; int AbProtocol::consume(const std::uint8_t *bytes, std::size_t size, core::TimeUs rx_us, CommandSink &sink)",
      "description": "AB 下行 mode=0 停止、1 控制、2 控制并请求开火；yaw/pitch 角度 rad、速度 rad/s、加速度 rad/s²，float32 小端，帧经 CRC 校验后调用 sink。",
      "parameters": [
        {
          "name": "config.command_reference",
          "meaning": "固定本地约定 {frame_id, epoch}，默认 {1,1}；AB 线上没有此字段"
        },
        {
          "name": "config.assembly_timeout_us",
          "meaning": "半帧装配超时，默认 20000 μs"
        },
        {
          "name": "sink",
          "meaning": "完整目标请求的接受者，VisionLink 内部实现"
        }
      ],
      "returns": "0 或负 errno；statistics() 给出 frames/crc_errors/invalid_frames/assembly_timeouts。",
      "context": "单 owner 的有状态流式 codec；statistics 不跨线程直接读。",
      "errors": "未知 mode/非有限 active 目标/坏 CRC 会拒绝；AB 没有会话号，不能把本地序号当远端采样序号。"
    }
  ],
  "examples": [
    {
      "title": "独立接收视觉请求并正确识别停止",
      "language": "cpp",
      "code": "#include <communication/vision/vision_receiver.hpp>\n#include <communication/vision/ab_protocol.hpp>\n#include \"board_config.hpp\"\nnamespace vision = skywalker::communication::vision;\nstatic skywalker::communication::AsyncUart::DmaBuffers dma __nocache;\nstatic vision::AbProtocol protocol({.command_reference = {1, 1}});\nstatic vision::VisionReceiver receiver(bench::vision_uart, dma, protocol, {});\n\nint main() {\n    const int ret = receiver.start();\n    if (ret < 0) return ret;\n    for (;;) {\n        const auto frame = receiver.snapshot();\n        const bool requested = frame.link.aim_fresh &&\n                               frame.link.aim.value.control_requested;\n        if (requested) {\n            const float yaw_rad = frame.link.aim.value.yaw.angle_rad;\n            // 目标先进入 VisionSource/CommandArbiter；这里不驱动电机。\n            (void)yaw_rad;\n        }\n        k_sleep(K_MSEC(10));\n    }\n}",
      "notes": "bench::vision_uart 来自 samples/communication/vision/src/board_config.hpp。这是 RX-only 示例；上行需设 feedback_period_us>0 并持续 setFeedback 传真实数据。"
    },
    {
      "title": "应用把实测反馈送给 receiver",
      "language": "cpp",
      "code": "// 参数必须是保存原始时间的实测值；由应用/IMU owner 提供。\nint publishVisionFeedback(vision::VisionReceiver &receiver,\n                          const vision::Feedback &measurements) {\n    return receiver.setFeedback(measurements);\n}\n// measurements.orientation 与 gyro_rad_s 的 stamp 不改成发送时刻。\n// 无效/过期字段保留无效状态，AB 编码器会拒绝不完整反馈。",
      "notes": "正式架构需由应用桥接 IMU → vision::Feedback，接收器本身不拥有 IMU。"
    }
  ],
  "lifecycle": [
    "静态构造 codec、DMA 和 Receiver，并统一应用/视觉的参考系约定。",
    "start 初始化 Link 后启动 UART worker；通过快照区分串口状态和请求新鲜度。",
    "VisionSource 把 Measurement 原样送仲裁，不更改时间；mode=0 清空旧目标。",
    "需要回传时，应用复制 IMU 和发射机构实测值，设 feedback_period_us 并调用 setFeedback。",
    "执行层仍检查本地位置参考与授权；fire_requested 只是电平请求。"
  ],
  "pitfalls": [
    "aim_fresh 只说明消息年龄；停止帧也可以 fresh，必须同时检查 control_requested。",
    "AB 线上不携带 frame_id、epoch、valid bits 或远端 sequence；参考一致性是双方明确约定。",
    "body-frame gyro.z/gyro.y 在任意姿态下不等于 Euler yaw_vel/pitch_vel；codec 负责转换。",
    "没有反馈测量时不能发送伪造弹速或姿态；默认 feedback_period_us=0。",
    "连续快照可能读到同一 fire_requested，不能把每次读取当成一次开火事件。"
  ],
  "config": [
    {
      "name": "CONFIG_SKYWALKER_VISION / VISION_AB / VISION_RECEIVER",
      "description": "分别启用独立 Link、AB codec、UART worker；与 REFEREE 可独立编译。"
    },
    {
      "name": "VisionLink::Config",
      "description": "aim=100000 μs；orientation/gyro=20000 μs；bullet_speed/count=1000000 μs；max_feedback_skew=20000 μs。"
    },
    {
      "name": "VisionReceiver::Config::feedback_period_us",
      "description": "默认 0，只接收；正值启用周期回传，TX 忙时跳过。"
    },
    {
      "name": "CONFIG_SKYWALKER_VISION_RX_STACK_SIZE / PRIORITY",
      "description": "默认栈 4096 字节、优先级 6；AB 参考样例使用 115200、8N1。"
    }
  ]
},
  {
  "id": "command",
  "title": "命令仲裁与后台服务",
  "category": "机器人与仲裁",
  "summary": "按 Safe/Manual/Auto、输入新鲜度、参考系、人工接管和裁判许可发布完整机器人命令。",
  "responsibility": "CommandArbiter 是同步策略核心；CommandManager 注册静态来源、启动来源并周期采样/仲裁，发布非消费快照。执行器负责本地电机反馈、使能与恢复。",
  "status": "ready",
  "statusNote": "注册来源与后台服务已实现；整车手动默认 Remote，裁判/视觉按 VEHICLE_* 阶段配置注册，不改变原输入年龄。",
  "source": [
    {
      "label": "后台服务 API",
      "path": "include/robotics/command/command_manager.hpp"
    },
    {
      "label": "仲裁核心 API",
      "path": "include/robotics/command/command_arbiter.hpp"
    },
    {
      "label": "决策与原因位",
      "path": "include/robotics/command/command_inputs.hpp"
    },
    {
      "label": "后台发布实现",
      "path": "lib/robotics/command_manager.cpp"
    },
    {
      "label": "RobotCommand 单位与字段",
      "path": "include/robotics/messages/command.hpp"
    }
  ],
  "docs": [
    {
      "label": "注册来源与线程契约",
      "path": "docs/modules/robotics/command-service.md"
    },
    {
      "label": "命令与执行恢复",
      "path": "docs/modules/robotics/command-recovery.md"
    },
    {
      "label": "三源台架",
      "path": "samples/robotics/command_manager/README.md"
    }
  ],
  "depends": [
    "command-sources"
  ],
  "interfaces": [
    {
      "signature": "explicit CommandManager(const Config &config); int CommandManager::registerSource(ICommandSource &source)",
      "description": "构造策略服务并在启动前注册来源。一个 Operator 必须存在，Aim 可选，每个角色最多一个，总容量两个。",
      "parameters": [
        {
          "name": "config",
          "meaning": "CommandArbiter::Config 的别名，包含限速、输入超时、视觉参考、许可策略"
        },
        {
          "name": "source",
          "meaning": "静态来源对象，role 在整个生命周期固定"
        }
      ],
      "returns": "registerSource：0 注册成功；不启动来源。",
      "context": "同一个启动线程完成注册/绑定/start；所有公共调用只能在线程上下文。",
      "errors": "-EWOULDBLOCK：ISR；-EBUSY：已尝试启动；-EINVAL：非法 role；-EEXIST：对象或角色重复；-ENOSPC：容量已满。注册顺序不代表优先级。"
    },
    {
      "signature": "int CommandManager::bindPermissions(IPermissionSource &source)",
      "description": "绑定单独的裁判许可来源。许可是对目标的约束，不是第三个运动目标角色。",
      "parameters": [
        {
          "name": "source",
          "meaning": "长生命周期 IPermissionSource，例如 RefereePermissionSource"
        }
      ],
      "returns": "0：绑定成功。",
      "context": "启动前同一个启动线程调用。",
      "errors": "-EWOULDBLOCK：ISR；-EBUSY：已尝试 start；-EEXIST：已有许可来源。"
    },
    {
      "signature": "int CommandManager::start()",
      "description": "校验策略与必要来源，依次启动来源和许可源，最后创建后台 worker；一次启动尝试后不能重启对象。",
      "parameters": [],
      "returns": "0：worker 已创建，尚不保证发布/输入在线；负 errno：配置或来源启动错误。",
      "context": "线程上下文，只尝试一次；manager、来源、receiver、codec 与 DMA 均静态存活，即使启动中途失败。",
      "errors": "-EWOULDBLOCK：ISR；-EALREADY：重复尝试；-EINVAL：配置错误或缺少 Operator；-ENODEV：策略要求裁判但未绑定；其他值透传来源 start。失败时发布 invalid/Disabled 决策。"
    },
    {
      "signature": "int CommandManager::current(RobotCommand &out) const; int CommandManager::snapshot(CommandSnapshot &out) const",
      "description": "current 复制最终命令；snapshot 同时复制 observed、decision 与 remote/vision/permission 诊断，供执行和观测读者独立使用。",
      "parameters": [
        {
          "name": "out",
          "meaning": "由当前读者拥有的输出值"
        }
      ],
      "returns": "0：复制当前值，可能仍是同一序号；-EAGAIN：尚未首次发布；错误不修改 out。",
      "context": "多个线程可读，短锁保护；非消费、不刷新 stamp，ISR 返回 -EWOULDBLOCK。",
      "errors": "读取 0 不代表命令 Active；应检查模式、decision.error、原因位和 command.stamp。执行器继续强制过期，防止 worker 停滞时重用旧目标。"
    },
    {
      "signature": "explicit CommandArbiter(const Config &config); int CommandArbiter::configError() const; CommandDecision CommandArbiter::update(const CommandInputs &inputs); void CommandArbiter::reset()",
      "description": "同步入口适合应用自行采集/调度。update 生成 requested 候选与按裁判裁剪后的 command，保留最终采用视觉的完整 selected_vision；reset 清仲裁历史，不操作硬件。",
      "parameters": [
        {
          "name": "inputs.now_us",
          "meaning": "本次仲裁时刻，μs"
        },
        {
          "name": "inputs.remote / vision / referee",
          "meaning": "保留来源原始 stamp 的完整值副本"
        }
      ],
      "returns": "configError：0 或 -EINVAL；update：完整 CommandDecision，error 表示错误，reasons 给出各机构的禁止/限幅原因。",
      "context": "单写入者；update/reset 由调用者串行化，不适用于 ISR。",
      "errors": "时间倒退可产生 -ESTALE/ClockRegression；非法配置/输入产生 -EINVAL；来源离线、Safe、视觉缺失和许可撤销可正常返回 error=0 但命令 Disabled/Hold。"
    }
  ],
  "examples": [
    {
      "title": "真实后台服务：注册三源、读取同帧诊断",
      "language": "cpp",
      "code": "#include <robotics/command/command_manager.hpp>\n#include <robotics/command/receiver_sources.hpp>\nusing namespace skywalker;\n\n// remote / vision / referee 需按各模块接口静态构造。\nint startService(robotics::CommandManager &manager,\n                 robotics::RemoteSource &remote_source,\n                 robotics::VisionSource &vision_source,\n                 robotics::RefereePermissionSource &permission_source) {\n    int ret = manager.registerSource(remote_source);\n    if (ret == 0) ret = manager.registerSource(vision_source);\n    if (ret == 0) ret = manager.bindPermissions(permission_source);\n    if (ret == 0) ret = manager.start();\n    return ret;\n}\n\nvoid observeCommand(const robotics::CommandManager &manager) {\n    robotics::CommandSnapshot frame{};\n    if (manager.snapshot(frame) == 0) {\n        const auto &command = frame.decision.command;\n        const auto reasons = frame.decision.reasons();\n        // command 是最终授权结果；requested 仅供诊断。\n        (void)command; (void)reasons;\n    }\n}",
      "notes": "完整静态对象与参数见 samples/robotics/command_manager/src/main.cpp 和 board_config.hpp；所有引用对象需要覆盖 worker 生命周期。"
    },
    {
      "title": "直接使用同步仲裁核心",
      "language": "cpp",
      "code": "#include <core/clock.hpp>\n#include <robotics/command/command_arbiter.hpp>\nusing namespace skywalker;\n\nrobotics::CommandDecision arbitrate(\n    robotics::CommandArbiter &arbiter, robotics::CommandInputs inputs) {\n    inputs.now_us = core::monotonicTimeUs();\n    // 只设置本次仲裁时钟，不改 remote/vision/referee 的 stamp。\n    return arbiter.update(inputs);\n}\n// 构造后先检查 arbiter.configError()；所有调用必须串行。",
      "notes": "CommandManager 不接受 CommandInputs，也没有 update、configError、reset 或 stop；这些同步入口属于 CommandArbiter。"
    }
  ],
  "lifecycle": [
    "静态构造 receiver/codec/source/manager；策略与参考系先确定。",
    "一个启动线程 registerSource(Operator)，按需注册 Aim，bindPermissions，然后 start 一次。",
    "worker 按周期 sample → 原始时间有效性 → CommandArbiter::update → 发布整帧。",
    "执行线程 current/snapshot 后检查目标模式和命令年龄，交本地执行器。",
    "遥控 Safe、输入过期、参考不匹配或裁判许可撤销都会约束命令；本地恢复还需新的有效目标与恢复上下文。"
  ],
  "pitfalls": [
    "不要使用已移除的 GlobalSafetyManager/GimbalLocalSafety/ChassisLocalSafety；开发指南中的拟议接口不代表当前 API。",
    "decision.requested 是未按裁判裁剪的候选，电机只能消费 decision.command。",
    "启动成功不代表输入上线；命令被禁止也可能 error=0，需看模式和原因位。",
    "Auto 进入或人工接管释放后需要新的视觉目标；旧视觉快照不能跨新自动上下文直接采用。",
    "shooting 命令类型已存在，但当前 sentry 应用没有完整发射执行链。"
  ],
  "config": [
    {
      "name": "CONFIG_SKYWALKER_LIB_ROBOTICS / ROBOTICS_COMMAND / COMMAND_SERVICE",
      "description": "启用同步仲裁与后台注册服务；来源另外启用对应接收模块。"
    },
    {
      "name": "CONFIG_SKYWALKER_COMMAND_PERIOD_MS / PRIORITY / STACK_SIZE",
      "description": "默认周期 10 ms、优先级 5、栈 6144 字节；服务发布与执行控制周期分开。"
    },
    {
      "name": "新鲜度与参考",
      "description": "input_timeout_ms=100、permission_timeout_ms=300、vision_timeout_us=100000，expected_vision_reference={1,1}。"
    },
    {
      "name": "模式与人工接管",
      "description": "require_referee_for_motion=true、allow_auto=true；override_enter_norm=0.15、exit=0.05、释放安静期 200000 μs。应用没有视觉时设 allow_auto=false。"
    },
    {
      "name": "目标上限",
      "description": "默认底盘 vx/vy=3 m/s、wz=6 rad/s；云台 yaw=3/pitch=2 rad/s；视觉加速度上限 30/20 rad/s²；射速请求 5 Hz。"
    }
  ]
},
  {
  "id": "command-sources",
  "title": "来源适配与手动映射",
  "category": "机器人与仲裁",
  "summary": "把接收器快照接到固定 Operator/Aim 角色；裁判单独作为 Permission，保留原始时间与诊断。",
  "responsibility": "ICommandSource/IPermissionSource 是扩展入口；内置 RemoteSource、VisionSource、RefereePermissionSource 适配接收器。ManualCommandMapper 把遥控值变成归一化操作意图，不输出电机电流。",
  "status": "ready",
  "statusNote": "注册来源与后台服务已实现；整车手动默认 Remote，裁判/视觉按 VEHICLE_* 阶段配置注册，不改变原输入年龄。",
  "source": [
    {
      "label": "来源契约与快照",
      "path": "include/robotics/command/command_source.hpp"
    },
    {
      "label": "接收器适配器",
      "path": "include/robotics/command/receiver_sources.hpp"
    },
    {
      "label": "适配实现",
      "path": "lib/robotics/receiver_sources.cpp"
    },
    {
      "label": "手动映射",
      "path": "include/robotics/command/manual_command_mapper.hpp"
    },
    {
      "label": "映射实现",
      "path": "lib/robotics/command.cpp"
    }
  ],
  "docs": [
    {
      "label": "来源与后台服务说明",
      "path": "docs/modules/robotics/command-service.md"
    },
    {
      "label": "来源样例",
      "path": "samples/robotics/command_manager/README.md"
    }
  ],
  "depends": [
    "remote",
    "vision",
    "referee"
  ],
  "interfaces": [
    {
      "signature": "virtual SourceRole ICommandSource::role() const = 0; virtual int ICommandSource::start() = 0; virtual int ICommandSource::sample(SourceSample &out) = 0",
      "description": "输入扩展接口。role 固定为 Operator 或 Aim；SourceValue 是 RemoteState 或 Measurement<AimCommand> 的 variant，必须和角色相匹配。",
      "parameters": [
        {
          "name": "out",
          "meaning": "0 时写完整 value 与 diagnostics；value 即使离线/无效也应完整写出"
        }
      ],
      "returns": "start：0 已调度；sample：0 更新缓存、-EAGAIN 保留上次缓存、其他负值清空该来源有效值。",
      "context": "startup owner 只调用一次 start；只有 manager worker 调用 sample；采样应有界、不等待 I/O。",
      "errors": "角色/variant 不匹配由 manager 视为 -EINVAL；其他错误进入诊断，不能留旧值继续有效。"
    },
    {
      "signature": "virtual int IPermissionSource::start() = 0; virtual int IPermissionSource::sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics) = 0",
      "description": "单独的许可约束接口；不占据 Operator/Aim 来源槽。",
      "parameters": [
        {
          "name": "now_ms",
          "meaning": "manager 的本机当前 ms，供接收轮询和过期判断"
        },
        {
          "name": "out",
          "meaning": "完整裁判快照，保留字段时间"
        },
        {
          "name": "diagnostics",
          "meaning": "error、sample_error、state 与可用丢包计数"
        }
      ],
      "returns": "0 更新、-EAGAIN 保留、其他负值使许可缓存失效。",
      "context": "start 单次；sample 只有 manager worker 所有。",
      "errors": "缺少必需许可来源时 manager.start=-ENODEV；合法离线数据可返回 0，由仲裁按原始 stamp 决定。"
    },
    {
      "signature": "explicit RemoteSource(communication::RemoteReceiver &receiver); int RemoteSource::start(); int RemoteSource::sample(SourceSample &out)",
      "description": "固定 Operator；start 转发 receiver.start，sample 拷贝 RemoteState、UART 状态与 dropped 诊断。",
      "parameters": [
        {
          "name": "receiver",
          "meaning": "静态遥控接收器；不要先手动启动"
        },
        {
          "name": "out",
          "meaning": "variant 中的 RemoteState 与同次接收诊断"
        }
      ],
      "returns": "start 透传；sample 把 receiver 的 -EAGAIN 转成保留且已执行读者侧过期的 0，其余错误透传。",
      "context": "只给一个 manager 注册；receiver 与 source 持续存活。",
      "errors": "预先 start 接收器会导致 manager 再启动时 -EALREADY；dropped_available=true。"
    },
    {
      "signature": "explicit VisionSource(communication::vision::VisionReceiver &receiver); int VisionSource::start(); int VisionSource::sample(SourceSample &out)",
      "description": "固定 Aim；从 receiver.snapshot 提取原始 Measurement，不把读快照的时刻写到目标 stamp。",
      "parameters": [
        {
          "name": "receiver",
          "meaning": "静态视觉接收器及其 codec/DMA"
        },
        {
          "name": "out",
          "meaning": "variant 中的 Measurement<AimCommand> 与 UART/丢包诊断"
        }
      ],
      "returns": "start 透传；sample 返回 0，目标可以无效/过期，由仲裁器判断。",
      "context": "manager worker 采样；receiver 自己的 worker 接收字节。",
      "errors": "UART error 放在 diagnostics.error；样本存在不代表视觉请求正在控制；dropped_available=true。"
    },
    {
      "signature": "explicit RefereePermissionSource(communication::RefereeReceiver &receiver); int RefereePermissionSource::start(); int RefereePermissionSource::sample(std::uint64_t now_ms, RefereeState &out, SourceDiagnostics &diagnostics)",
      "description": "start 为 0，真正 I/O 由 sample 调用 receiver.poll；裁判没有单独 worker。",
      "parameters": [
        {
          "name": "receiver",
          "meaning": "唯一裁判 UART owner"
        },
        {
          "name": "now_ms",
          "meaning": "本机当前 ms"
        },
        {
          "name": "out / diagnostics",
          "meaning": "裁判值与 receiver.error"
        }
      ],
      "returns": "start/sample 当前均返回 0；在线、许可与接收错误在值/诊断中表达。",
      "context": "manager worker 独占 poll；应用不能并行轮询同一裁判 receiver。",
      "errors": "RefereeReceiver 未公开丢块数，因此 dropped_available=false，不伪造为可用计数。"
    },
    {
      "signature": "explicit ManualCommandMapper(const Config &config); int ManualCommandMapper::map(const RemoteState &remote, OperatorIntent &out) const",
      "description": "将中心化遥控值变为 [-1,1] 意图。左开关 Down=Safe/Middle=Manual/Up=Auto；右开关 Up=KeyboardMouse，其余合法档=Remote。",
      "parameters": [
        {
          "name": "remote",
          "meaning": "合法遥控快照，带在线与 stamp"
        },
        {
          "name": "out",
          "meaning": "模式、来源、归一化速度、摩擦轮/射击请求及原始 stamp"
        }
      ],
      "returns": "0：映射；离线输入映射成 Safe/零值；-EINVAL：配置或开关非法。",
      "context": "纯值映射，无 I/O；后续 CommandArbiter 应用物理限速和许可。",
      "errors": "mapper 不自行按当前时刻判断 stamp 年龄，仲裁器必须先检查新鲜度。默认 DR16 不解 wheel，底盘旋转请求因此可能为 0。"
    }
  ],
  "examples": [
    {
      "title": "用内置适配器连接现有接收器",
      "language": "cpp",
      "code": "#include <robotics/command/receiver_sources.hpp>\n#include <robotics/command/command_manager.hpp>\nusing namespace skywalker;\n\n// 示例调用位置：接收器构造后，manager.start 之前。\nint connectInputs(robotics::CommandManager &manager,\n                  robotics::ICommandSource &operator_source,\n                  robotics::ICommandSource &aim_source,\n                  robotics::IPermissionSource &permissions) {\n    int ret = manager.registerSource(operator_source);\n    if (ret == 0) ret = manager.registerSource(aim_source);\n    if (ret == 0) ret = manager.bindPermissions(permissions);\n    return ret;\n}\n// 实际静态对象声明：\n// robotics::RemoteSource operator_source(remote);\n// robotics::VisionSource aim_source(vision);\n// robotics::RefereePermissionSource permissions(referee);",
      "notes": "source 对象及依赖长期存活，所有注册完成后统一 manager.start，不单独调用内置 source.sample。"
    },
    {
      "title": "解释遥控映射，而后交仲裁器限速",
      "language": "cpp",
      "code": "#include <robotics/command/manual_command_mapper.hpp>\nusing namespace skywalker::robotics;\n\nint inspectIntent(const RemoteState &remote, OperatorIntent &intent) {\n    const ManualCommandMapper mapper({});\n    return mapper.map(remote, intent);\n}\n// intent.chassis_vx_norm 为归一化值，不能直接送电机。\n// CommandArbiter 会将它乘 max_chassis_vx_m_s 并施加裁判许可。",
      "notes": "遥控 left_y→vx，-left_x→vy，wheel→wz，-right_x→yaw_rate，right_y→pitch_rate；键鼠 W/S、A/D 与鼠标按固定映射提供意图。"
    }
  ],
  "lifecycle": [
    "先构造接收器，再构造内置 source 引用。",
    "固定来源 role，在同一启动线程注册并绑定许可。",
    "由 manager.start 启动来源；之后只有 manager worker sample。",
    "采样复制原始值与时间，按明确返回契约替换/保留/失效缓存。"
  ],
  "pitfalls": [
    "不要在 sample 中无限等待串口或重新刷新旧值时间；后台周期不能因此停滞。",
    "SourceRole 只有 Operator/Aim；没有直接把所有消息源塞入同一优先级队列的接口。",
    "注册多个 Operator 或 Aim 返回 -EEXIST；来源最多两个。",
    "自定义来源不得发布与 role 不匹配的 variant；诊断的 unavailable 与数值 0 不同。"
  ],
  "config": [
    {
      "name": "ManualCommandMapper::Config",
      "description": "analog_deadband=0.03、channel_range=660、mouse_yaw_scale/mouse_pitch_scale=0.002。"
    },
    {
      "name": "来源编译条件",
      "description": "RemoteSource 需 REMOTE_RECEIVER，VisionSource 需 VISION_RECEIVER，RefereePermissionSource 需 REFEREE 与 UART_TRANSPORT。"
    },
    {
      "name": "对象所有权",
      "description": "source 只引用接收器；manager 只引用 source，不拥有它们，不负责释放。"
    }
  ]
},
  {
  "id": "gimbal",
  "title": "云台单轴与本地执行",
  "category": "机器人与仲裁",
  "summary": "机械单轴保存目标；公开双轴 GimbalExecutor 独立计算两轴并由应用统一提交总线。",
  "responsibility": "GimbalAxis 管机械范围与目标，PositionMotor 管本轴反馈计算；GimbalExecutor 管新鲜输入/许可和双轴调用，不持有 CAN。",
  "status": "ready",
  "statusNote": "正式云台已装配双轴与惯性适配框架，真实接线与安装确认默认关闭。",
  "source": [
    {
      "label": "gimbal_axis.hpp",
      "path": "include/robotics/gimbal/gimbal_axis.hpp"
    },
    {
      "label": "gimbal_executor.hpp",
      "path": "include/robotics/gimbal/gimbal_executor.hpp"
    },
    {
      "label": "gimbal_axis.cpp",
      "path": "lib/robotics/gimbal_axis.cpp"
    },
    {
      "label": "gimbal_executor.cpp",
      "path": "lib/robotics/gimbal_executor.cpp"
    }
  ],
  "docs": [
    {
      "label": "executors.md",
      "path": "docs/modules/robotics/executors.md"
    },
    {
      "label": "README.md",
      "path": "samples/robotics/gimbal_control/README.md"
    }
  ],
  "depends": [
    "motor-control",
    "command"
  ],
  "interfaces": [
    {
      "signature": "int GimbalAxis::validate() const; int GimbalAxis::begin(); Status GimbalAxis::poll(uint64_t now_ms)",
      "description": "begin 配置；poll 观测反馈/配置允许的参考，Status 仅 feedback_healthy/error，无 ready_for_enable。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int GimbalAxis::reset(); int GimbalAxis::update(const AxisCommand &, SafetyAction, float dt_s); int updateRate(float rate_rad_s, float dt_s)",
      "description": "明确 Hold/reset 获取本轴目标；Rate 时间轴不因驱动离线重置，机械限位保留。",
      "parameters": [
        {
          "name": "dt_s",
          "meaning": "实际 s"
        },
        {
          "name": "action",
          "meaning": "业务授权动作；不能代替 Motor disable"
        }
      ],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "GimbalExecutor(Motor &yaw, Motor &pitch, Group &, const PositionMotor::Config &, const GimbalAxisConfig &, const PositionMotor::Config &, const GimbalAxisConfig &, const Config &); int begin()",
      "description": "注入长期存活两轴与配置；应用先 attach/start。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "RunStatus GimbalExecutor::update(const GimbalExecutionInputs &, core::TimeUs now_us); RunStatus suspend(core::TimeUs, WaitReason, int error = 0, bool blocked = false)",
      "description": "source_stamp/permission 与命令分别过期；yaw_output_valid/pitch_output_valid 分别声明数学输入是否足够。update 不 commit CAN。",
      "parameters": [
        {
          "name": "now_us",
          "meaning": "单调本地 µs"
        },
        {
          "name": "inputs",
          "meaning": "command/source_stamp/permission、transport_ready、两轴输出有效性与急停"
        }
      ],
      "returns": "RunStatus 值副本，生产 stamp、requested 与各轴统计。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    }
  ],
  "examples": [
    {
      "title": "两轴持续更新，共享总线统一发布",
      "language": "cpp",
      "code": "const auto status = executor.update(inputs, now_us);\nconst auto published = bus.commit();\n// 若两轴跨物理 CAN，另一总线仍独立 commit；状态用于诊断。",
      "notes": "应用周期片段，构造与实物配置以链接源码为准。"
    }
  ],
  "lifecycle": [
    "确定 Continuous/Limited 和可信机械参考。",
    "应用 attach/start，axis/executor.begin 配置。",
    "每周期从真实新鲜输入持续更新目标。",
    "本轴缺反馈只作废本轴 computed effort，其他轴继续。",
    "每物理 CAN 周期末一次 commit；真实停止由 suspend/disable 处理。"
  ],
  "pitfalls": [
    "世界姿态须先经惯性适配，不能直接送编码器角。",
    "reset 是业务目标/历史操作，不是电机恢复时重采原点。",
    "软件停止报告不能替代机械停止测量。"
  ],
  "config": [
    {
      "name": "GimbalAxisConfig",
      "description": "Continuous/Limited、机械范围、max_rate、hold_on_zero_rate、Preserve/CalibratedFeedback。"
    },
    {
      "name": "GimbalExecutor::Config",
      "description": "来源/命令 100 ms、许可 300 ms、周期 20 ms；所有 stamp 保留原时刻。"
    }
  ]
},
  {
  "id": "chassis",
  "title": "舵轮底盘与功率缩放",
  "category": "机器人与仲裁",
  "summary": "四轮运动学与八轴独立有效性；公开 SwerveHardware/ChassisExecutor 持续暂存，应用统一 commit。",
  "responsibility": "SwerveKinematics 分目标，SwerveModule 独立转向/轮驱计算与翻转迟滞，SwerveHardware 映射八电机；功率执行使用可信预算与测量。",
  "status": "ready",
  "statusNote": "单舵轮最新提交记录基本功能调通；四轮/整车软件入口已有，真实几何与功率仍待标定。",
  "source": [
    {
      "label": "swerve_types.hpp",
      "path": "include/robotics/swerve/swerve_types.hpp"
    },
    {
      "label": "swerve_module.hpp",
      "path": "include/robotics/swerve/swerve_module.hpp"
    },
    {
      "label": "swerve_chassis.hpp",
      "path": "include/robotics/swerve/swerve_chassis.hpp"
    },
    {
      "label": "swerve_hardware.hpp",
      "path": "include/robotics/chassis/swerve_hardware.hpp"
    },
    {
      "label": "chassis_executor.hpp",
      "path": "include/robotics/chassis/chassis_executor.hpp"
    },
    {
      "label": "chassis_executor.cpp",
      "path": "lib/robotics/chassis_executor.cpp"
    }
  ],
  "docs": [
    {
      "label": "executors.md",
      "path": "docs/modules/robotics/executors.md"
    },
    {
      "label": "README.md",
      "path": "samples/robotics/swerve/README.md"
    },
    {
      "label": "README.md",
      "path": "samples/robotics/four_swerve/README.md"
    }
  ],
  "depends": [
    "pid",
    "motor-dji",
    "interboard"
  ],
  "interfaces": [
    {
      "signature": "int SwerveChassis::validate() const; int reset(const ChassisFeedback &); int step(const ChassisCommand &, const ChassisFeedback &, float dt_s, ChassisOutput &out)",
      "description": "持续解算，不等待全轮反馈；ModuleFeedback.steer_valid/drive_valid 与各轴 generation 独立决定 effort 有效性。",
      "parameters": [
        {
          "name": "command",
          "meaning": "vx/vy m/s、wz rad/s；+x 前、+y 左、+wz 逆时针"
        },
        {
          "name": "feedback",
          "meaning": "四模块 FL/FR/RL/RR；舵绝对角/速度，轮驱速度"
        }
      ],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非法输入/配置返回负 errno；本轴缺反馈以本轴 output_valid=false 表达。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int SwerveModule::step(const ModuleTarget &, const ModuleFeedback &, float dt_s, ModuleOutput &out)",
      "description": "最短转向、翻转迟滞、转向斜坡、Hold/Coast；无对齐门控或 cos 缩速，舵向和轮驱分别恢复。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int SwerveHardware::begin(); int read(ChassisFeedback &); int stage(const ChassisOutput &, float steer_scale, float drive_scale); void suspend()",
      "description": "应用注入八台 Motor 和批量 Group；stage 独立携带计算 enable generation，不 commit 总线。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "ChassisExecutor(SwerveHardware &, const SwerveChassis::Config &, const Config &); int begin(); RunStatus update(const ChassisExecutionInputs &, core::TimeUs now_us)",
      "description": "注入硬件，不绑定 Endpoint；消费原输入、权限、预算和真实功率。",
      "parameters": [
        {
          "name": "inputs",
          "meaning": "command/source_stamp/permission/power_budget/measured_power 与运行/急停条件"
        }
      ],
      "returns": "RunStatus 与输出/测量观察入口，电机等待不撤销其他轴目标。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int ChassisPowerLimiter::reset(); int step(const ChassisPowerInput &, float dt_s, ChassisPowerDecision &out)",
      "description": "标定后以 measured_power_w、预算 W、缓冲 J 计算 [0,1] 缩放；预算不可冒充测量。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "数值/周期非法 -EINVAL；模型未标定或测量过期由执行器阻断对应功率模式。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    }
  ],
  "examples": [
    {
      "title": "四轮与大 Yaw 分别推进",
      "language": "cpp",
      "code": "const auto wheels = chassis.update(chassis_inputs, now_us);\nconst auto yaw = big_yaw.update(yaw_inputs, now_us);\nconst auto steer_tx = steer_bus.commit();\nconst auto wheel_tx = wheel_bus.commit();\nconst auto yaw_tx = yaw_bus.commit();\n// 每条物理 CAN 一个 owner；一个失败不漏提交另外两条。",
      "notes": "应用周期片段，构造与实物配置以链接源码为准。"
    }
  ],
  "lifecycle": [
    "核对 FL/FR/RL/RR、方向、单圈零位、半径、减速比。",
    "应用 attach/start 各总线，hardware/executor.begin 校验。",
    "持续解算各轴目标；只用有效测量计算对应 effort。",
    "功率模式保留预算与测量时效；未标定台架使用独立电流上限。",
    "统一提交每物理 CAN；输入停止才明确批量撤销。"
  ],
  "pitfalls": [
    "ModuleFeedback 没有旧 steer_continuous_rad，舵向绝对角局部展开。",
    "单轮经验速度比例/等效半径不是整车实测几何。",
    "mode=Disabled 的运动学目标不替代 Motor.disable。",
    "电机独立恢复，Group 不形成故障传播。"
  ],
  "config": [
    {
      "name": "SwerveModule::Config",
      "description": "wheel_radius_m、两套控制环、steer_target_rate、翻转进入/退出阈值、IdleBehavior。"
    },
    {
      "name": "ChassisExecutor::Config",
      "description": "真实功率/预算年龄、模型确认、estimated 回退、舵/驱独立台架缩放与电流上限。"
    }
  ]
},
  {
  "id": "telemetry",
  "title": "VOFA、日志与一致快照",
  "category": "调试与观测",
  "summary": "从同一帧快照解释来源、目标、许可、反馈和错误；JustFloat 有界排队，主机离线不阻塞控制。",
  "responsibility": "VOFA 有界 JustFloat/调参行；Latest 与 SnapshotCache 交换值副本，应用观察线程不参与目标授权。",
  "status": "ready",
  "statusNote": "单舵轮 16 通道，M2006 速度环 USB CDC 12 通道与反馈/CAN 前后快照诊断；详细通道见各样例 README。",
  "source": [
    {
      "label": "vofa.h",
      "path": "include/lib/vofa/vofa.h"
    },
    {
      "label": "latest.hpp",
      "path": "include/latest.hpp"
    },
    {
      "label": "snapshot_cache.hpp",
      "path": "include/robotics/execution/snapshot_cache.hpp"
    },
    {
      "label": "telemetry.cpp",
      "path": "samples/robotics/command_manager/src/telemetry.cpp"
    },
    {
      "label": "main.cpp",
      "path": "samples/motor/m2006_speed_control/src/main.cpp"
    }
  ],
  "docs": [
    {
      "label": "调试指南",
      "path": "docs/guides/debugging.md"
    },
    {
      "label": "命令观测通道",
      "path": "samples/robotics/command_manager/README.md"
    },
    {
      "label": "视觉 VOFA 示例",
      "path": "samples/communication/vision/README.md"
    },
    {
      "label": "README.md",
      "path": "samples/motor/m2006_speed_control/README.md"
    },
    {
      "label": "README.md",
      "path": "samples/robotics/swerve/README.md"
    }
  ],
  "depends": [],
  "interfaces": [
    {
      "signature": "int vofa_init(Vofa *vofa, const struct device *uart)",
      "description": "一次性绑定支持 IRQ/FIFO API 的独占 USART 或 USB CDC ACM；不等待主机连接。",
      "parameters": [
        {
          "name": "vofa",
          "meaning": "零初始化、静态或等同生命周期的 Vofa 实例"
        },
        {
          "name": "uart",
          "meaning": "就绪设备，不能与 console/shell/AsyncUart 分享回调"
        }
      ],
      "returns": "0：初始化成功；负 errno：失败，失败后不能 send。",
      "context": "线程中调用一次；初始化后不能复制、移动或重新初始化；USB 栈由应用/Zephyr 启动。",
      "errors": "-EINVAL：参数非法；-ENODEV：设备未就绪；或 UART 回调注册错误。"
    },
    {
      "signature": "int vofa_send(Vofa *vofa, const float *data, uint8_t num)",
      "description": "非阻塞复制完整 JustFloat 帧进入深度 4 的队列；成功后可立即复用 data，多生产者可共享实例。",
      "parameters": [
        {
          "name": "vofa",
          "meaning": "成功初始化的实例"
        },
        {
          "name": "data",
          "meaning": "num 个 float 的数组"
        },
        {
          "name": "num",
          "meaning": "1～16 个通道，VOFA_MAX_FLOATS=16"
        }
      ],
      "returns": "0：完整帧已入队，不表示主机接收；负 errno：整帧拒绝。",
      "context": "可在线程或普通 ISR 中调用；无等待 USB/DTR。",
      "errors": "-ENOBUFS：队列满，丢本次完整帧；-EINVAL：参数非法；-ENODEV/驱动错误：设备发送不可用。"
    },
    {
      "signature": "int vofa_set_handler(Vofa *vofa, uint8_t *rx_buf, size_t rx_buf_size, vofa_cmd_handler on_cmd)",
      "description": "设置一次接收行缓冲与 key=value handler，自动启用 IRQ 接收，支持 LF/CRLF。",
      "parameters": [
        {
          "name": "rx_buf / rx_buf_size",
          "meaning": "持续存活的接收缓冲，至少 2 字节"
        },
        {
          "name": "on_cmd",
          "meaning": "void (*)(const char *key, float val)，key 只在回调期间有效"
        }
      ],
      "returns": "0：handler 已设置；负 errno：失败。",
      "context": "线程配置一次；handler 在 UART 驱动回调运行，可能 ISR/工作队列，不能阻塞。要执行控制变更时先入队给应用线程。",
      "errors": "-EINVAL/-ENODEV/-EALREADY；超长或含 NUL 的行丢弃至下一换行，不并发配置或热替换 handler。"
    },
    {
      "signature": "int Latest<T>::put(const T &value); int Latest<T>::get(T &value)",
      "description": "用 K_NO_WAIT mutex 拷贝完整最新值，没有 I/O 和回调；get 非消费且不保证新的序号。",
      "parameters": [
        {
          "name": "value",
          "meaning": "put 的输入或 get 的读者输出；跨线程传值，不保存内部引用"
        }
      ],
      "returns": "0：完整复制；-EAGAIN：锁忙。初始内部值为 T{}。",
      "context": "线程之间交换适当大小的快照；mutex 不用于 ISR。",
      "errors": "Latest 本身没有 have_value/时间有效性标志；读者必须检查值里的 stamp.valid/sequence，-EAGAIN 时保留旧值仍要过期。"
    },
    {
      "signature": "int bench::Telemetry::start(const device *vofa_uart); void bench::Telemetry::emit(const skywalker::robotics::CommandSnapshot &frame)",
      "description": "命令 sample 私有封装：日志约每 100 ms 输出同帧的模式/来源/年龄/原因，VOFA 可选每 20 ms 输出最终命令与拆分原因位。",
      "parameters": [
        {
          "name": "vofa_uart",
          "meaning": "观测专属设备"
        },
        {
          "name": "frame",
          "meaning": "manager.snapshot 得到的同一次发布，主通道使用 decision.command"
        }
      ],
      "returns": "start 返回 VOFA 初始化结果（未编译 VOFA 时为 0）；emit 不返回，记录拒绝帧计数。",
      "context": "sample 观测线程调用；不是 lib 的通用机器人 telemetry 服务。",
      "errors": "VOFA 背压不会阻塞 manager；telemetry 不应反复重发旧运动命令或在控制线程打印高频长日志。"
    }
  ],
  "examples": [
    {
      "title": "将同一次发布的最终命令送到 VOFA",
      "language": "cpp",
      "code": "#include <lib/vofa/vofa.h>\n#include <robotics/command/command_manager.hpp>\nusing namespace skywalker;\n\nint telemetryLoop(const device *telemetry_uart,\n                  const robotics::CommandManager &manager) {\n    static Vofa vofa{};\n    const int init = vofa_init(&vofa, telemetry_uart);\n    if (init < 0) return init;\n    robotics::CommandSnapshot frame{};\n    for (;;) {\n        if (manager.snapshot(frame) == 0) {\n            const auto &c = frame.decision.command;\n            const float values[] = {c.chassis.vx_m_s, c.chassis.vy_m_s,\n                                    c.chassis.wz_rad_s, c.gimbal.yaw_rate_rad_s,\n                                    float(frame.decision.error)};\n            const int ret = vofa_send(&vofa, values, 5);\n            // ret==-ENOBUFS 时记录丢观测帧，继续下一周期。\n            (void)ret;\n        }\n        k_sleep(K_MSEC(20));\n    }\n}",
      "notes": "本函数仅调用一次，telemetry_uart 与遥控/裁判/视觉/板间设备独立。通道单位和顺序在主机侧对应配置。"
    }
  ],
  "lifecycle": [
    "选独占观测 UART/CDC，静态构造 Vofa 并 vofa_init 一次。",
    "采集一份完整 CommandSnapshot/执行快照；只在观测线程组装通道和低频日志。",
    "vofa_send 复制整帧，有背压则丢本次观测，下一周期用新值。",
    "接收调参行时回调只复制请求，应用线程决定何时应用参数，保留控制所有权。"
  ],
  "pitfalls": [
    "浮点通道不要直接存完整 32 位 bitmask/大序号；需要精确原因位可像台架一样拆为低/高 16 位。",
    "data 入队成功不等于主机已显示；串口断开也不能卡住控制循环。",
    "Latest.get 的 0 不证明已经 publish；值类型要包含有效标志。",
    "对比仲裁观测和最终输出时用同一个 CommandSnapshot，避免分别读导致跨帧误判。"
  ],
  "config": [
    {
      "name": "CONFIG_SKYWALKER_LIB_VOFA",
      "description": "依赖 SERIAL 与 SERIAL_SUPPORT_INTERRUPT，选择 UART_INTERRUPT_DRIVEN。"
    },
    {
      "name": "VOFA_MAX_FLOATS / VOFA_TX_QUEUE_DEPTH",
      "description": "16 个 float 通道、4 帧有界队列；不是无限数据缓存。"
    },
    {
      "name": "CONFIG_COMMAND_MANAGER_VOFA",
      "description": "三源 sample 的可选观测开关；串口选择和通道顺序见该 sample 配置。"
    },
    {
      "name": "日志与观测周期",
      "description": "控制/仲裁线程持续运行，观测采用更低频率；当前命令台架 log=100 ms、VOFA=20 ms。"
    }
  ]
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
  "source": [
    {
      "label": "inertial_gimbal.hpp",
      "path": "include/robotics/gimbal/inertial_gimbal.hpp"
    },
    {
      "label": "inertial_gimbal.cpp",
      "path": "lib/robotics/inertial_gimbal.cpp"
    },
    {
      "label": "main.cpp",
      "path": "samples/robotics/inertial_gimbal/src/main.cpp"
    }
  ],
  "docs": [
    {
      "label": "executors.md",
      "path": "docs/modules/robotics/executors.md"
    },
    {
      "label": "dual-controller.md",
      "path": "docs/applications/dual-controller.md"
    }
  ],
  "interfaces": [
    {
      "signature": "InertialGimbalAdapter(const Config &); InertialGimbalOutput update(const InertialGimbalInputs &, core::TimeUs now_us)",
      "description": "输入头部 Snapshot、两轴 MotorSnapshot、command/source_stamp/prerequisites_ready，输出关节 command、原 stamp、两轴 output_valid、stabilization_valid 与误差。",
      "parameters": [],
      "returns": "值副本；不写电机、不持有总线。",
      "errors": "IMU/参考/质量不足时相应数学输出无效；视觉参考须匹配头部 frame_id/epoch。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    }
  ],
  "examples": [
    {
      "title": "头部惯性云台适配 周期",
      "language": "cpp",
      "code": "const auto adapted = adapter.update(inputs, now_us);\nmechanical_inputs.command = adapted.command;\nmechanical_inputs.source_stamp = adapted.source_stamp;\nmechanical_inputs.yaw_output_valid = adapted.yaw_output_valid;\nmechanical_inputs.pitch_output_valid = adapted.pitch_output_valid;\nconst auto status = gimbal.update(mechanical_inputs, now_us);",
      "notes": "应用周期片段，构造与实物配置以链接源码为准。"
    }
  ],
  "lifecycle": [
    "静态构造与校验实物配置。",
    "保留原输入/测量时间及坐标身份。",
    "唯一执行 owner 持续 update；应用统一物理 CAN commit。",
    "真实输入停止才撤销；电机等待仅影响本轴计算。"
  ],
  "pitfalls": [
    "API 已存在不等于实物标定和闭环通过。",
    "状态生产时间不能被读者/通信刷新。"
  ],
  "config": [
    {
      "name": "InertialGimbalAdapter::Config",
      "description": "头部超时20ms、机械50ms、输入100ms；pitch_locked=true，方向/安装/姿态质量必须确认。"
    }
  ]
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
  "source": [
    {
      "label": "yaw_centering.hpp",
      "path": "include/robotics/gimbal/yaw_centering.hpp"
    },
    {
      "label": "big_yaw_executor.hpp",
      "path": "include/robotics/execution/big_yaw_executor.hpp"
    },
    {
      "label": "yaw_centering.cpp",
      "path": "lib/robotics/yaw_centering.cpp"
    },
    {
      "label": "big_yaw_executor.cpp",
      "path": "lib/robotics/big_yaw_executor.cpp"
    }
  ],
  "docs": [
    {
      "label": "executors.md",
      "path": "docs/modules/robotics/executors.md"
    },
    {
      "label": "dual-controller.md",
      "path": "docs/applications/dual-controller.md"
    }
  ],
  "interfaces": [
    {
      "signature": "YawCenteringOutput YawCenteringController::update(const YawCenteringInputs &, core::TimeUs now_us); void reset()",
      "description": "使用独立标定中心、关节采样时刻、头部稳定与新鲜许可，产生带死区/迟滞/限速/斜坡的角速度。",
      "parameters": [],
      "returns": "enabled、velocity_rad_s、center_error_rad、stamp 与 reason。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "BigYawExecutor(Motor &, const VelocityMotor::Config &, const Config &); int begin(); RunStatus update(const BigYawExecutionInputs &, core::TimeUs now_us); BigYawFeedback feedback() const",
      "description": "消费 v4 request、实际 local_boot_id、peer_online、transport_ready 和输入撤销条件；只暂存，不 commit。",
      "parameters": [],
      "returns": "RunStatus / BigYawFeedback 值副本；actual_rate_rad_s 仅 valid 时使用。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    }
  ],
  "examples": [
    {
      "title": "大 Yaw 回中与独立速度环 周期",
      "language": "cpp",
      "code": "const auto follow = centering.update(center_inputs, now_us);\nrequest.mode = follow.enabled ? BigYawMode::FollowCenter : BigYawMode::Disabled;\nrequest.target_rate_rad_s = follow.velocity_rad_s;\nrequest.stamp = follow.stamp;\n// 同时保留 source_sequence、来源/命令/权限原年龄。\nendpoint.submitBigYaw(request);",
      "notes": "应用周期片段，构造与实物配置以链接源码为准。"
    }
  ],
  "lifecycle": [
    "静态构造与校验实物配置。",
    "保留原输入/测量时间及坐标身份。",
    "唯一执行 owner 持续 update；应用统一物理 CAN commit。",
    "真实输入停止才撤销；电机等待仅影响本轴计算。"
  ],
  "pitfalls": [
    "API 已存在不等于实物标定和闭环通过。",
    "状态生产时间不能被读者/通信刷新。"
  ],
  "config": [
    {
      "name": "center_rad",
      "description": "独立实测小 Yaw 机械中心，不能用编码器零点替代。"
    },
    {
      "name": "v4",
      "description": "目标保留 boot/producer/age，无 resume_generation，发送不等待电机 ready。"
    }
  ]
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
  "source": [
    {
      "label": "shooter_executor.hpp",
      "path": "include/robotics/shooter/shooter_executor.hpp"
    },
    {
      "label": "shooter_executor.cpp",
      "path": "lib/robotics/shooter_executor.cpp"
    },
    {
      "label": "shooter_bench.hpp",
      "path": "samples/robotics/common/shooter_bench.hpp"
    }
  ],
  "docs": [
    {
      "label": "executors.md",
      "path": "docs/modules/robotics/executors.md"
    },
    {
      "label": "dual-controller.md",
      "path": "docs/applications/dual-controller.md"
    }
  ],
  "interfaces": [
    {
      "signature": "ShooterExecutor(Motor &left, Motor &right, Motor &dial, Group &friction, Group &feed, const VelocityMotor::Config &, const PositionMotor::Config &, const Config &); int begin()",
      "description": "注入三电机、两批量组与环参数；应用先 attach/start，不在执行器内 commit。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "ShooterStatus update(const ShooterExecutionInputs &, core::TimeUs now_us); ShooterStatus suspend(core::TimeUs, WaitReason, int error = 0)",
      "description": "输入原始命令/事件、source_stamp、热量/许可、拨盘参考、云台状态、allow_feed 与急停；单发去重，忙碌/过期/恢复中旧事件丢弃。",
      "parameters": [],
      "returns": "friction/feed RunStatus、摩擦就绪、拨盘 busy/jammed、last_event_id、软件 shots 与 reserved_heat。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    }
  ],
  "examples": [
    {
      "title": "摩擦轮与拨盘发射执行 周期",
      "language": "cpp",
      "code": "const auto status = shooter.update(inputs, now_us);\n// 小云台/摩擦/拨盘共享 DJI CAN1，所有机构 stage 后统一 commit。\nconst auto published = dji_bus.commit();",
      "notes": "应用周期片段，构造与实物配置以链接源码为准。"
    }
  ],
  "lifecycle": [
    "静态构造与校验实物配置。",
    "保留原输入/测量时间及坐标身份。",
    "唯一执行 owner 持续 update；应用统一物理 CAN commit。",
    "真实输入停止才撤销；电机等待仅影响本轴计算。"
  ],
  "pitfalls": [
    "API 已存在不等于实物标定和闭环通过。",
    "状态生产时间不能被读者/通信刷新。"
  ],
  "config": [
    {
      "name": "真实供弹来源",
      "description": "IShooterHeatSource、IDialHomeSource 默认 Pending 无效；不能将当前点伪造有载原点。"
    },
    {
      "name": "软件 shots",
      "description": "软件执行统计不是可信裁判弹数，不直接用作 AB 实测反馈。"
    }
  ]
}
]);
