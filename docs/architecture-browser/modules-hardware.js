/* Hardware and control API reference. Paths are repository relative. */
(() => {
  const api = (signature, description, parameters, returns, context, errors) => ({
    signature, description, parameters: parameters.map(([name, meaning]) => ({ name, meaning })),
    returns, context, errors
  });
  const thread = '应用线程调用；把同一总线的 setter/update → commit 交给一个控制周期所有者。CAN 中断回调只入队/记录完成，解码、超时和真正发送由 CanBus 的 I/O 线程处理。';
  window.SKYWALKER_MODULES = (window.SKYWALKER_MODULES || []).concat([
    {
  "id": "motor-dji",
  "title": "DJI 电机与共享 CAN 总线",
  "category": "驱动与感知",
  "summary": "统一 Motor/CanBus 持续接收目标，设备独立自动恢复；effort 单位 A。",
  "responsibility": "Motor 保存运行意图、最新命令和真实反馈；CanBus 处理物理 I/O、限频协议重试和独立恢复；Group 只做显式批量操作。",
  "status": "ready",
  "statusNote": "源码与台架入口已有；模式、ID、方向、范围和机械参数按实物确认。设备掉线不撤销其他轴。",
  "source": [
    {
      "label": "dji_motor.hpp",
      "path": "include/drivers/motor/dji_motor.hpp"
    },
    {
      "label": "dji_protocol.hpp",
      "path": "include/drivers/motor/dji_protocol.hpp"
    },
    {
      "label": "motor.cpp",
      "path": "drivers/motor/motor.cpp"
    },
    {
      "label": "can_bus.cpp",
      "path": "drivers/motor/can_bus.cpp"
    },
    {
      "label": "main.cpp",
      "path": "samples/motor/dji_unified/src/main.cpp"
    }
  ],
  "docs": [
    {
      "label": "DJI Markdown",
      "path": "docs/modules/drivers/motor-dji.md"
    },
    {
      "label": "电机操作顺序",
      "path": "docs/guides/motor-workflow.md"
    }
  ],
  "depends": [
    "boards"
  ],
  "interfaces": [
    {
      "signature": "dji::Config dji::gm6020(const Gm6020Options &options); dji::Config dji::m3508(const M3508Options &options); dji::Config dji::m2006(const M2006Options &options)",
      "description": "构建不同型号的值配置，然后传给 Motor；工厂不创建设备、不发 CAN、不验证全部选项。",
      "parameters": [
        {
          "name": "options.id",
          "meaning": "GM6020 为 1–7，其余为 1–8。"
        },
        {
          "name": "options.current_limit_a",
          "meaning": "软件电流上限：GM6020 ≤3 A，M3508 ≤20 A，M2006 ≤10 A，均须 >0。"
        },
        {
          "name": "options.gear_ratio",
          "meaning": "M3508 默认 19，M2006 默认 36；电机轴/输出轴转速比，必须 >0。"
        },
        {
          "name": "GM6020 专有选项",
          "meaning": "encoder_zero_ticks 0–8191 定义绝对零点；current_mode_confirmed 须确认固件电流模式后置 true。"
        },
        {
          "name": "options.timing",
          "meaning": "反馈、命令、稳定恢复、使能超时，单位 ms，全部须 >0。"
        }
      ],
      "returns": "返回 dji::Config；Motor(dji::Config) 保存一份配置。",
      "context": "初始化阶段纯函数；构造后的 Motor 不可拷贝/移动。",
      "errors": "配置错误在 CanBus::start() 暴露；GM6020 未确认电流模式为 -EINVAL。"
    },
    {
      "signature": "int dji::describe(const Config &config, Descriptor &out)",
      "description": "计算反馈 CAN ID、命令帧 ID 和帧内槽位。GM6020 反馈 0x204+id，命令 0x1FE/0x2FE；M3508/M2006 反馈 0x200+id，命令 0x200/0x1FF。",
      "parameters": [
        {
          "name": "config",
          "meaning": "DJI 配置。"
        },
        {
          "name": "out",
          "meaning": "成功时写入描述；command_slot 为 0–3，同一帧最多四电机。"
        }
      ],
      "returns": "0=成功；描述不代替完整配置验证。",
      "context": "无 I/O 的纯调用；通常应用不必自己映射 ID。",
      "errors": "-ERANGE：型号/电机 ID 不合法。不同协议 TX/RX 冲突最终由总线 start() 检查。"
    },
    {
      "signature": "bool dji::decodeFeedback(const can_frame &frame, RawFeedback &out); int dji::buildCommandFrame(can_frame &frame, uint16_t command_id, const int16_t command_raw[4])",
      "description": "底层协议工具：解码编码器/RPM/原始电流/温度；把四个原始电流槽位编码成命令帧。正常应用使用 Motor，避免绕过权限和限幅。",
      "parameters": [
        {
          "name": "frame",
          "meaning": "标准 CAN 数据帧。"
        },
        {
          "name": "out",
          "meaning": "成功解码后的原始协议数据，不是 SI 单位输出轴反馈。"
        },
        {
          "name": "command_id",
          "meaning": "DJI 的合法组帧 ID。"
        },
        {
          "name": "command_raw",
          "meaning": "四个槽位的 int16 原始值，按高字节在前编码。"
        }
      ],
      "returns": "decodeFeedback 返回是否帧合法；buildCommandFrame 返回 0 或负 errno。",
      "context": "纯编解码，不执行 CAN 发送；驱动内部在 I/O 线程调用。",
      "errors": "非法帧 decode 为 false；不由这些工具维护 freshness/Group/命令超时。"
    },
    {
      "signature": "int Motor::setCurrent(float ampere); int Motor::setCurrent(float ampere, uint64_t sampled_enable_generation)",
      "description": "设置 DJI 电流目标；反馈计算结果必须携带计算时 snapshot.enable_generation，不能计算后重新贴新版本。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非有限 -EINVAL，明确范围 -ERANGE，模式不支持 -ENOTSUP，producer 不匹配 -EACCES；离线/非 Active 不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "MotorInfo Motor::info() const; MotorSnapshot Motor::snapshot() const; bool Motor::active() const",
      "description": "查询能力、参数和状态值副本；enabled_requested 是运行意图，output_permitted/feedback_fresh 是实际执行条件，retry_count 与 last_fault 是诊断。",
      "parameters": [],
      "returns": "能力/状态值副本或实际 active 布尔值；快照读取不生产新反馈。",
      "errors": "检查 feedback.valid 和原始 timestamp；实际状态不能替代运行意图。",
      "context": "允许多读取线程，短锁保护；有状态输出只有一个生产者。"
    },
    {
      "signature": "int Motor::enable(); int Motor::disable()",
      "description": "enable 幂等保存运行意图，离线也可接受；disable 在真→假时取消旧命令和旧协议操作，停止后重复 disable 不清掉之后新提交的目标。Group 成员也可直接操作。",
      "parameters": [],
      "returns": "0 请求已接受，不证明机械执行或制动。",
      "errors": "-EACCES：CAN 未启动；disable 版本耗尽 -EOVERFLOW。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int Motor::reseedPosition(double known_position_rad)",
      "description": "用可信机械依据建立本轴连续坐标；不向固件保存零点，不影响其他轴。",
      "parameters": [
        {
          "name": "known_position_rad",
          "meaning": "已确认输出轴坐标 rad，不能凭断电前角度猜圈数。"
        }
      ],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非法/越界/能力或参考条件错误返回负 errno；见 Motor 实现。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int Motor::invalidateComputedEffort()",
      "description": "仅作废计算 effort，保留运行意图；反馈不足时应使旧计算输出失效。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "已绑定控制器 producer 时普通调用可能 -EACCES；封装内部使用其 producer。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "CanBus(const device *can, BusOptions options = {}); int CanBus::attach(Motor &motor); int CanBus::start()",
      "description": "每物理 CAN 一个 owner。初始化 attach 全部端点，再 start I/O 工作线程；DJI/DM 可共享。",
      "parameters": [
        {
          "name": "can",
          "meaning": "实际独占 Zephyr CAN 控制器"
        },
        {
          "name": "options",
          "meaning": "TX timeout 默认 2 ms，恢复重试默认 100 ms"
        }
      ],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "设备、拓扑、ID、参数错误由 attach/start 返回；不可在运行中增加成员。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "CommitResult CanBus::commit(); BusStatus CanBus::status() const",
      "description": "commit 发布完整总线最新快照，Recovering 时继续接受。status 含 latest_submitted_sequence、last_tx、rx_overflows、rx_invalid_frames、superseded_batches 和 last_recovery。",
      "parameters": [],
      "returns": "CommitResult.error/sequence 表示发布结果；CAN TX 完成另看 last_tx。",
      "errors": "未启动/永久配置错误返回调用错误；成功不代表远端收到或执行。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "Group(Motor &first, Others &...others); Group(std::span<Motor *const> members); int Group::enable(); void Group::disable(); GroupStatus Group::status() const",
      "description": "固定成员批量启停和统计，无故障传播、就绪屏障、成员所有权或独立恢复状态。",
      "parameters": [],
      "returns": "enable 返回首个调用错误并继续其他成员；status 为成员/意图/实际/离线/故障计数。",
      "errors": "固定列表配置错误或 Motor 调用错误；没有 ready/active/clearFault 公共组接口。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    }
  ],
  "examples": [
    {
      "title": "持续运行意图与最新目标",
      "language": "cpp",
      "code": "// 先 bus.attach(motor)、bus.start()，电机固件模式匹配。\nif (run_requested) {\n    const int enabled = motor.enable();\n    const int accepted = motor.setCurrent(0.1f);\n    // 分别记录调用错误，设备离线不终止周期。\n} else {\n    (void)motor.disable();\n}\nconst auto published = bus.commit();\nconst auto actual = motor.snapshot();",
      "notes": "直接协议目标片段；effort=A。绑定 VelocityMotor/PositionMotor 后应改用 axis.update，不混用 producer。"
    }
  ],
  "lifecycle": [
    "按真实模式/ID/量程静态构造 Motor、CanBus。",
    "attach 所有端点，再 start；不等待设备全部上线。",
    "运行意图有效时每周期 enable + setter/update + commit。",
    "各电机独立探测、清错、使能，反馈恢复后执行最新未过期目标。",
    "主动停止取消旧目标；停止进展单独观测，机械刹停不是 TX 成功。"
  ],
  "pitfalls": [
    "没有公共 Motor::ready/clearFault；Group 不传播成员故障。",
    "setter 接受、commit 发布、CAN TX 完成、机械执行是不同结果。",
    "共享 DJI 帧只把不可执行槽置零，其他槽保留目标。",
    "连续参考丢失不能猜圈数；可信 reseed 仅影响本轴。",
    "历史 last_fault/last_recovery 不等于当前仍在故障。"
  ],
  "config": [
    {
      "name": "Timing",
      "description": "具名反馈/命令/使能/重试期限；原目标时间不由恢复或 commit 续期。"
    },
    {
      "name": "Effort 单位",
      "description": "A；应用限幅与控制输出必须使用同单位。"
    },
    {
      "name": "物理总线 owner",
      "description": "每 CAN 一个 CanBus，所有机构先 stage，统一每周期 commit 一次。"
    }
  ]
},
    {
  "id": "motor-dm",
  "title": "达妙电机：MIT / 速度 / 位置速度",
  "category": "驱动与感知",
  "summary": "统一 Motor/CanBus 持续接收目标，设备独立自动恢复；effort 单位 N·m。",
  "responsibility": "Motor 保存运行意图、最新命令和真实反馈；CanBus 处理物理 I/O、限频协议重试和独立恢复；Group 只做显式批量操作。",
  "status": "ready",
  "statusNote": "源码与台架入口已有；模式、ID、方向、范围和机械参数按实物确认。设备掉线不撤销其他轴。",
  "source": [
    {
      "label": "dm_motor.hpp",
      "path": "include/drivers/motor/dm_motor.hpp"
    },
    {
      "label": "dm_protocol.hpp",
      "path": "include/drivers/motor/dm_protocol.hpp"
    },
    {
      "label": "motor.cpp",
      "path": "drivers/motor/motor.cpp"
    },
    {
      "label": "can_bus.cpp",
      "path": "drivers/motor/can_bus.cpp"
    },
    {
      "label": "main.cpp",
      "path": "samples/motor/dm_mit_control/src/main.cpp"
    }
  ],
  "docs": [
    {
      "label": "DM Markdown",
      "path": "docs/modules/drivers/motor-dm.md"
    },
    {
      "label": "三模式台架说明",
      "path": "samples/motor/DM_J4310_EXAMPLES.md"
    }
  ],
  "depends": [
    "boards"
  ],
  "interfaces": [
    {
      "signature": "dm::Config dm::j4310Mit(const J4310Options &options); dm::Config dm::j4310Velocity(const J4310Options &options); dm::Config dm::j4310PositionVelocity(const J4310Options &options)",
      "description": "构建 MIT、速度或位置速度模式配置；主控工厂不会把驱动器固件自动切换模式。",
      "parameters": [
        {
          "name": "options.id",
          "meaning": "电机 ID 1–15。"
        },
        {
          "name": "options.master_id",
          "meaning": "驱动器反馈的标准 CAN ID 0–0x7FF；不能与总线 TX ID 冲突。"
        },
        {
          "name": "position_max_rad / velocity_max_rad_s / torque_max_nm",
          "meaning": "真实固件 PMAX/VMAX/TMAX，须为正且匹配工具中已保存的值。"
        },
        {
          "name": "torque_limit_nm",
          "meaning": "应用软件力矩上限 >0 且 ≤TMAX。"
        },
        {
          "name": "timing",
          "meaning": "feedback_timeout_ms / command_timeout_ms / enable_timeout_ms / retry_interval_ms；DJI 20/10/100/100，J4310 50/20/3000/100 ms。"
        }
      ],
      "returns": "返回 dm::Config；交给 Motor(dm::Config)。",
      "context": "初始化阶段纯配置；对象全固件生命周期存活。",
      "errors": "全部配置验证在 CanBus::start()；不支持型号/模式 -EINVAL；范围错误 -ERANGE。"
    },
    {
      "signature": "int dm::describe(const Config &config, Descriptor &out); int dm::controlFrameId(ControlMode mode, uint16_t motor_id, uint16_t &out)",
      "description": "计算三模式控制帧 ID：MIT=id，位置速度=0x100+id，速度=0x200+id；反馈 ID 来自 master_id，并用帧内 motor_id 分流。",
      "parameters": [
        {
          "name": "config / mode",
          "meaning": "必须与固件持久化模式相同。"
        },
        {
          "name": "motor_id",
          "meaning": "电机地址1–15。"
        },
        {
          "name": "out",
          "meaning": "成功时写描述或控制帧 ID。"
        }
      ],
      "returns": "0=成功，负 errno=参数错误。",
      "context": "纯调用，无 CAN I/O。",
      "errors": "-EINVAL/-ERANGE：型号、模式或地址不合法。CanBus 允许不同 DM 电机共用反馈 master_id，但帧内 motor_id 必须不同。"
    },
    {
      "signature": "int dm::buildMitFrame(uint16_t motor_id, const Limits &limits, const MitCommand &command, can_frame &out)",
      "description": "MIT 量化编码工具；位置16位，速度/kp/kd/力矩各12位。正常应用直接使用 Motor::setMit。",
      "parameters": [
        {
          "name": "motor_id",
          "meaning": "电机地址。"
        },
        {
          "name": "limits",
          "meaning": "匹配固件的 PMAX/VMAX/TMAX。"
        },
        {
          "name": "command",
          "meaning": "MIT五字段。"
        },
        {
          "name": "out",
          "meaning": "编码后的标准 CAN 帧。"
        }
      ],
      "returns": "0=完成编帧，负 errno=无效/超范围；不发送。",
      "context": "驱动 I/O 线程内部使用的纯编码。",
      "errors": "协议范围不等于应用 torque_limit_nm；绕过 Motor 会绕开生命周期和软件力矩限幅。"
    },
    {
      "signature": "int dm::buildVelocityFrame(uint16_t motor_id, float velocity_rad_s, can_frame &out); int dm::buildPositionVelocityFrame(uint16_t motor_id, float position_rad, float velocity_rad_s, can_frame &out)",
      "description": "编码速度/位置速度模式的浮点命令帧；仅构造字节，不建立整车权限。",
      "parameters": [
        {
          "name": "motor_id",
          "meaning": "电机地址。"
        },
        {
          "name": "position_rad / velocity_rad_s",
          "meaning": "对应物理量；应用层额外限幅由 Motor 执行。"
        },
        {
          "name": "out",
          "meaning": "目标CAN帧。"
        }
      ],
      "returns": "0=完成编帧。",
      "context": "纯调用，无 I/O。",
      "errors": "地址/非有限输入错误返回负 errno；主控应调用 Motor setter，以免漏 PMAX/VMAX 校验。"
    },
    {
      "signature": "int dm::buildSpecialFrame(ControlMode mode, uint16_t motor_id, SpecialCommand command, can_frame &out); int dm::decodeFeedback(const can_frame &frame, uint8_t expected_motor_id, const Limits &limits, DecodedFeedback &out); bool dm::isFaultStatus(DriveStatus status)",
      "description": "特殊命令 Enable/Disable/ClearError/SaveZero、反馈状态和温度解码。SaveZero 是底层协议能力，统一 Motor 不公开保存硬件零点操作。",
      "parameters": [
        {
          "name": "mode / motor_id",
          "meaning": "真实固件模式和地址。"
        },
        {
          "name": "command",
          "meaning": "特殊协议命令枚举。"
        },
        {
          "name": "frame / expected_motor_id / limits",
          "meaning": "标准反馈帧、预期帧内电机地址和真实量程。"
        },
        {
          "name": "out / status",
          "meaning": "编码/解码输出或待分类驱动状态。"
        }
      ],
      "returns": "构帧/解码返回0或负errno；isFaultStatus 返回故障判断。",
      "context": "纯编解码；驱动将回调入队后在I/O线程解析。",
      "errors": "不能仅以 CAN TX完成认为已禁用：DM stop 的 DriveConfirmed 才表示后续反馈确认禁用；Unreachable 表示无法确认。"
    },
    {
      "signature": "int Motor::setTorque(float newton_meter); int Motor::setTorque(float newton_meter, uint64_t sampled_enable_generation)",
      "description": "MIT 纯力矩目标，computed effort 使用计算快照执行版本。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非有限 -EINVAL，明确范围 -ERANGE，模式不支持 -ENOTSUP，producer 不匹配 -EACCES；离线/非 Active 不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int Motor::setMit(const dm::MitCommand &command)",
      "description": "MIT 位置/速度/kp/kd/前馈目标，量化范围须匹配真实固件 PMAX/VMAX/TMAX。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非有限 -EINVAL，明确范围 -ERANGE，模式不支持 -ENOTSUP，producer 不匹配 -EACCES；离线/非 Active 不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int Motor::setVelocity(float rad_s); int Motor::setPositionVelocity(float rad, float max_rad_s)",
      "description": "固件原生速度或位置速度模式；主控模式配置不自动修改固件模式。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非有限 -EINVAL，明确范围 -ERANGE，模式不支持 -ENOTSUP，producer 不匹配 -EACCES；离线/非 Active 不拒绝合法目标。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "MotorInfo Motor::info() const; MotorSnapshot Motor::snapshot() const; bool Motor::active() const",
      "description": "查询能力、参数和状态值副本；enabled_requested 是运行意图，output_permitted/feedback_fresh 是实际执行条件，retry_count 与 last_fault 是诊断。",
      "parameters": [],
      "returns": "能力/状态值副本或实际 active 布尔值；快照读取不生产新反馈。",
      "errors": "检查 feedback.valid 和原始 timestamp；实际状态不能替代运行意图。",
      "context": "允许多读取线程，短锁保护；有状态输出只有一个生产者。"
    },
    {
      "signature": "int Motor::enable(); int Motor::disable()",
      "description": "enable 幂等保存运行意图，离线也可接受；disable 在真→假时取消旧命令和旧协议操作，停止后重复 disable 不清掉之后新提交的目标。Group 成员也可直接操作。",
      "parameters": [],
      "returns": "0 请求已接受，不证明机械执行或制动。",
      "errors": "-EACCES：CAN 未启动；disable 版本耗尽 -EOVERFLOW。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int Motor::reseedPosition(double known_position_rad)",
      "description": "用可信机械依据建立本轴连续坐标；不向固件保存零点，不影响其他轴。",
      "parameters": [
        {
          "name": "known_position_rad",
          "meaning": "已确认输出轴坐标 rad，不能凭断电前角度猜圈数。"
        }
      ],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非法/越界/能力或参考条件错误返回负 errno；见 Motor 实现。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int Motor::invalidateComputedEffort()",
      "description": "仅作废计算 effort，保留运行意图；反馈不足时应使旧计算输出失效。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "已绑定控制器 producer 时普通调用可能 -EACCES；封装内部使用其 producer。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "CanBus(const device *can, BusOptions options = {}); int CanBus::attach(Motor &motor); int CanBus::start()",
      "description": "每物理 CAN 一个 owner。初始化 attach 全部端点，再 start I/O 工作线程；DJI/DM 可共享。",
      "parameters": [
        {
          "name": "can",
          "meaning": "实际独占 Zephyr CAN 控制器"
        },
        {
          "name": "options",
          "meaning": "TX timeout 默认 2 ms，恢复重试默认 100 ms"
        }
      ],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "设备、拓扑、ID、参数错误由 attach/start 返回；不可在运行中增加成员。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "CommitResult CanBus::commit(); BusStatus CanBus::status() const",
      "description": "commit 发布完整总线最新快照，Recovering 时继续接受。status 含 latest_submitted_sequence、last_tx、rx_overflows、rx_invalid_frames、superseded_batches 和 last_recovery。",
      "parameters": [],
      "returns": "CommitResult.error/sequence 表示发布结果；CAN TX 完成另看 last_tx。",
      "errors": "未启动/永久配置错误返回调用错误；成功不代表远端收到或执行。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "Group(Motor &first, Others &...others); Group(std::span<Motor *const> members); int Group::enable(); void Group::disable(); GroupStatus Group::status() const",
      "description": "固定成员批量启停和统计，无故障传播、就绪屏障、成员所有权或独立恢复状态。",
      "parameters": [],
      "returns": "enable 返回首个调用错误并继续其他成员；status 为成员/意图/实际/离线/故障计数。",
      "errors": "固定列表配置错误或 Motor 调用错误；没有 ready/active/clearFault 公共组接口。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    }
  ],
  "examples": [
    {
      "title": "持续运行意图与最新目标",
      "language": "cpp",
      "code": "// 先 bus.attach(motor)、bus.start()，电机固件模式匹配。\nif (run_requested) {\n    const int enabled = motor.enable();\n    const int accepted = motor.setTorque(0.1f);\n    // 分别记录调用错误，设备离线不终止周期。\n} else {\n    (void)motor.disable();\n}\nconst auto published = bus.commit();\nconst auto actual = motor.snapshot();",
      "notes": "直接协议目标片段；effort=N·m。绑定 VelocityMotor/PositionMotor 后应改用 axis.update，不混用 producer。"
    }
  ],
  "lifecycle": [
    "按真实模式/ID/量程静态构造 Motor、CanBus。",
    "attach 所有端点，再 start；不等待设备全部上线。",
    "运行意图有效时每周期 enable + setter/update + commit。",
    "各电机独立探测、清错、使能，反馈恢复后执行最新未过期目标。",
    "主动停止取消旧目标；停止进展单独观测，机械刹停不是 TX 成功。"
  ],
  "pitfalls": [
    "没有公共 Motor::ready/clearFault；Group 不传播成员故障。",
    "setter 接受、commit 发布、CAN TX 完成、机械执行是不同结果。",
    "共享 DJI 帧只把不可执行槽置零，其他槽保留目标。",
    "连续参考丢失不能猜圈数；可信 reseed 仅影响本轴。",
    "历史 last_fault/last_recovery 不等于当前仍在故障。"
  ],
  "config": [
    {
      "name": "Timing",
      "description": "具名反馈/命令/使能/重试期限；原目标时间不由恢复或 commit 续期。"
    },
    {
      "name": "Effort 单位",
      "description": "N·m；应用限幅与控制输出必须使用同单位。"
    },
    {
      "name": "物理总线 owner",
      "description": "每 CAN 一个 CanBus，所有机构先 stage，统一每周期 commit 一次。"
    }
  ]
},
    {
  "id": "imu",
  "title": "IMU：独立采集、姿态与加热",
  "category": "驱动与感知",
  "summary": "BMI088 和达妙 RS485 共用带时间戳的快照，不同来源独立采集与独立诊断。",
  "responsibility": "ImuSource 负责某种真实输入；ImuState 按字段发布测量与新鲜度；ImuReceiver 独占采集循环并可管理 ImuHeater。BMI088 可挂 QuaternionEkf 估计姿态；RS485 直接使用设备主动上报的四元数。",
  "status": "partial",
  "statusNote": "板载与外置统一采集已实现，整车云台装配头部外置 IMU 与惯性适配；imu_mounting_confirmed 默认 false，姿态质量与参考仍须实测。",
  "source": [
    {
      "label": "统一测量类型",
      "path": "include/drivers/imu/imu_types.hpp"
    },
    {
      "label": "输入源接口",
      "path": "include/drivers/imu/imu.hpp"
    },
    {
      "label": "接收线程API",
      "path": "include/drivers/imu/imu_receiver.hpp"
    },
    {
      "label": "BMI088输入",
      "path": "drivers/imu/bmi088_imu.cpp"
    },
    {
      "label": "RS485输入",
      "path": "drivers/imu/dm_imu_rs485.cpp"
    },
    {
      "label": "加热保护",
      "path": "drivers/imu/imu_heater.cpp"
    },
    {
      "label": "双IMU样例",
      "path": "samples/imu/dual_imu/src/main.cpp"
    }
  ],
  "docs": [
    {
      "label": "IMU Markdown",
      "path": "docs/modules/drivers/imu.md"
    },
    {
      "label": "双IMU台架",
      "path": "samples/imu/dual_imu/README.md"
    }
  ],
  "depends": [
    "boards",
    "kalman",
    "pid",
    "uart"
  ],
  "interfaces": [
    {
      "signature": "virtual int ImuSource::init(); virtual int ImuSource::service(); virtual Snapshot ImuSource::snapshot() const",
      "description": "所有输入源的最小契约：init 初始化；service 驱动一次采集/解析；snapshot 读取副本。构造不执行I/O。",
      "parameters": [],
      "returns": "init：0成功；service：0有进展/-EAGAIN无新数据/其他负errno；snapshot：测量、状态、capabilities、fresh_mask和诊断。",
      "context": "只有一个所有者调用init/service；任何线程可copy snapshot；已注册回调的对象与依赖须一直存活。",
      "errors": "Running 不表示每个字段都新鲜；检查 fresh_mask 和 Measurement.stamp.valid。"
    },
    {
      "signature": "Bmi088Imu::Bmi088Imu(const device *accel, const device *gyro, const Config &c, control::QuaternionEkf *estimator = nullptr); int Bmi088Imu::init(); int Bmi088Imu::service()",
      "description": "通过Zephyr传感器API读取加速度/角速度/温度；可选独立EKF生成姿态，sensor_to_body 将传感器轴旋转到机器人机体轴。",
      "parameters": [
        {
          "name": "accel / gyro",
          "meaning": "加速度和陀螺仪device，通常来自accel0/gyro0 alias。"
        },
        {
          "name": "c",
          "meaning": "采样周期默认1250us，温度周期10000us，最大输入时间偏差2000us，参考{id,epoch}、安装旋转与freshness。"
        },
        {
          "name": "estimator",
          "meaning": "可选长寿命 QuaternionEkf；交给该source独占，source.init会初始化它。"
        }
      ],
      "returns": "init 0成功；service 0已处理/-EAGAIN尚未到采样时间。",
      "context": "所有者线程，会做SPI/sensor I/O；若ImuReceiver接管，其他线程不得直接再init/service或更新estimator。",
      "errors": "-EALREADY：重复init；-EINVAL：周期/安装旋转不合法；-ENODEV：传感器不可用；-ENOTSUP：传入EKF但未编入；-EACCES：未初始化；-ESTALE：加速度/陀螺仪时间差超限；透传sensor错误。"
    },
    {
      "signature": "DmImuRs485Source::DmImuRs485Source(const device *uart, communication::AsyncUart::DmaBuffers &dma, const Config &c); int DmImuRs485Source::init(); int DmImuRs485Source::service(); int DmImuRs485Source::resetReference()",
      "description": "消费DM-IMU-L1主动串行帧，应用缩放和安装旋转；遇UART故障可内部重试。resetReference在已知设备复位/清零/标定后递增epoch并撤销旧姿态，不向设备发送配置命令。",
      "parameters": [
        {
          "name": "uart / dma",
          "meaning": "独占UART与长寿命DMA存储；H7使用__nocache。"
        },
        {
          "name": "c.protocol",
          "meaning": "设备id、半帧组装超时。"
        },
        {
          "name": "c.reference / sensor_to_body",
          "meaning": "独立参考身份与传感器→机体安装旋转。"
        },
        {
          "name": "c.device_quaternion_is_world_to_sensor",
          "meaning": "若固件上报世界→传感器，则先共轭成传感器→世界。"
        },
        {
          "name": "c.acceleration_scale / angular_velocity_scale",
          "meaning": "正数缩放系数，转换为m/s²和rad/s；取决于设备已保存输出单位。"
        }
      ],
      "returns": "init/service：0成功或负errno；resetReference：0完成本地重置。",
      "context": "一个所有者调用；UART ISR只推进AsyncUart事件，不在ISR解析或算姿态。接收线程运行时不得外部直接resetReference。",
      "errors": "-EINVAL：配置错误；-EALREADY：重复init；-EACCES：尚未初始化；-EAGAIN：无新帧/等待恢复；传输溢出丢掉半帧并记录transportGap；CRC/帧错误在diagnostics报告。"
    },
    {
      "signature": "ImuReceiver::ImuReceiver(ImuSource &source, const Config &config, ImuHeater *heater = nullptr); int ImuReceiver::start()",
      "description": "启动一个源的专有工作线程，在线程内部异步init并持续service；可选加热器与此线程同所有者。",
      "parameters": [
        {
          "name": "source",
          "meaning": "生命周期覆盖工作线程的输入源。"
        },
        {
          "name": "config.poll_interval_us",
          "meaning": "轮询间隔，默认1000us；具体source决定真正采样频率。"
        },
        {
          "name": "config.priority",
          "meaning": "合法抢占优先级；默认CONFIG_SKYWALKER_IMU_RX_PRIORITY。"
        },
        {
          "name": "heater",
          "meaning": "可选长寿命加热器；由worker init/update管理。"
        }
      ],
      "returns": "0=worker已启动，不等于source初始化成功或姿态可用。",
      "context": "一次启动；无stop/restart或运行中销毁支持。不能再从别的线程调用source.init/service或heater.init/update/disable。",
      "errors": "-EINVAL：轮询/优先级无效；-ENOTSUP：加热器未编入；-EALREADY：已经start。"
    },
    {
      "signature": "Snapshot ImuReceiver::snapshot() const; ImuReceiver::Status ImuReceiver::status() const",
      "description": "snapshot直接让source按读取时刻算freshness；status分别报告异步初始化、service以及heater错误/占空比。没有第二份样本缓存。",
      "parameters": [],
      "returns": "独立副本；init_complete/init_error 判断初始化结果；service_error保留最近非-EAGAIN返回。",
      "context": "允许多个读线程；snapshot与status彼此不是原子组合。",
      "errors": "start成功后立即读到Uninitialized属于正常；业务要等姿态字段新鲜且quality达到要求。"
    },
    {
      "signature": "int ImuState::init(uint32_t capabilities, core::OrientationReference reference); int ImuState::publish(const Update &update); Snapshot ImuState::snapshot() const",
      "description": "具体source共享的字段发布器，Update.updated_mask指示本次真正更新的字段；其余字段保留独立时间戳。",
      "parameters": [
        {
          "name": "capabilities",
          "meaning": "Accel/Gyro/Orientation/Temperature能力位。"
        },
        {
          "name": "reference",
          "meaning": "参考系身份和重置epoch；身份不同不能直接混用姿态。"
        },
        {
          "name": "update",
          "meaning": "已转换为机体轴、SI单位的测量与更新位。"
        }
      ],
      "returns": "init/publish返回0或负errno；snapshot按Freshness逐字段计算fresh_mask。",
      "context": "写者由source所有者串行；自旋锁保护多读者副本。",
      "errors": "无效/非有限更新不应视作新测量；diagnostics.invalid_updates报告被拒绝内容。"
    },
    {
      "signature": "int DmImuParser::init(); int DmImuParser::consume(const uint8_t *bytes, size_t size, core::TimeUs now, DmImuSink &sink); void DmImuParser::discardPartial(); Statistics DmImuParser::statistics() const",
      "description": "主动帧字节组装、CRC与更新投递；consume(nullptr,0,now,sink)也可推进组装超时；传输缺口要discardPartial，避免拼接缺失帧。",
      "parameters": [
        {
          "name": "bytes / size",
          "meaning": "任意分片，size=0允许空指针。"
        },
        {
          "name": "now",
          "meaning": "当前单调时间us。"
        },
        {
          "name": "sink",
          "meaning": "同线程接收解码Update的对象。"
        }
      ],
      "returns": "consume返回成功接受帧数或负errno；statistics返回计数。",
      "context": "所有方法仅解析器所有者线程，无内部并发保证。",
      "errors": "CRC错误/无效帧/组装超时计数分别见Statistics；不执行配置写命令，不实现CAN。"
    },
    {
      "signature": "int ImuHeater::init(); int ImuHeater::update(const core::Measurement<float> &temperature, core::TimeUs now); int ImuHeater::disable(); ImuHeater::Snapshot ImuHeater::snapshot() const",
      "description": "独立温控PID与PWM安全管理。init明确写0占空比；温度过期、超上限或无效时尝试关闭；snapshot分别保留原因和关闭操作错误。",
      "parameters": [
        {
          "name": "temperature",
          "meaning": "带有效性与时间戳的摄氏温度。"
        },
        {
          "name": "now",
          "meaning": "单调时间us；不允许倒退。"
        },
        {
          "name": "Config",
          "meaning": "PWM设备/周期，目标默认50°C、最大65°C，温度超时200ms、控制周期100ms，输出PID限幅0–1。"
        }
      ],
      "returns": "0成功；update -EAGAIN未到周期；其他负errno为错误。",
      "context": "一个线程拥有init/update/disable；snapshot多读者。已附加ImuReceiver后全部控制由worker处理。",
      "errors": "-EACCES：未init；-EALREADY：重复init；-EINVAL/-ENODEV：配置/设备错；-ESTALE：过期/时间倒退；-ERANGE：≥最大温度、低于-40或非有限；PWM关断失败会返回底层错误并保留disable_error。"
    }
  ],
  "examples": [
    {
      "title": "板载BMI088 + EKF：异步采集，业务只读快照",
      "language": "cpp",
      "code": "#include <drivers/imu/bmi088_imu.hpp>\n#include <drivers/imu/imu_receiver.hpp>\n#include <core/attitude.hpp>\n#include <zephyr/kernel.h>\n#include <zephyr/sys/printk.h>\n#include <cerrno>\nnamespace imu = skywalker::imu;\n\nint main() {\n  static skywalker::control::QuaternionEkf ekf{\n      skywalker::control::QuaternionEkf::Config{}};\n  static imu::Bmi088Imu source{\n      DEVICE_DT_GET(DT_ALIAS(accel0)), DEVICE_DT_GET(DT_ALIAS(gyro0)),\n      imu::Bmi088Imu::Config{.reference={1,1}, .sensor_to_body={}}, &ekf};\n  static imu::ImuReceiver receiver{\n      source, imu::ImuReceiver::Config{.poll_interval_us=500, .priority=5}};\n  const int r = receiver.start();\n  if (r < 0) return r;\n  // start只启动线程；init与静止初始化由worker完成。\n  for (;;) {\n    const auto s = receiver.snapshot();\n    const auto diagnostic = receiver.status();\n    if (diagnostic.init_complete && diagnostic.init_error < 0)\n      return diagnostic.init_error;\n    if ((s.fresh_mask & imu::Orientation) &&\n        s.sample.attitude_quality == imu::AttitudeQuality::Tracking) {\n      const auto e = skywalker::core::euler(s.sample.orientation.value);\n      printk(\"roll=%d pitch=%d mrad\\n\", int(e.roll*1000), int(e.pitch*1000));\n    }\n    k_msleep(20);\n  }\n}",
      "notes": "安装旋转的单位四元数仅适用于传感器轴与机体轴一致；reference={1,1}是示例身份。启用IMU/BMI088/RECEIVER、ATTITUDE_EKF、SENSOR/BMI08X、FPU/FPU_SHARING以及CPP。初始化需稳定静止；加速度仅能校正倾斜，yaw会漂移。完整RS485+加热例见dual_imu样例。"
    }
  ],
  "lifecycle": [
    "构造输入源、可选EKF和heater；UART DMA、所有对象长期存活。",
    "调用receiver.start；worker负责heater.init和source.init，应用通过status等异步结果。",
    "单一worker周期service，source将各字段独立发布；姿态初始化阶段不可直接控制云台。",
    "控制/遥测线程只snapshot并检查fresh_mask、参考身份/epoch和姿态质量。",
    "温度过期或错误由同一worker关闭heater；源失败与加热关闭失败分别诊断。",
    "设备复位或参考变化必须撤销旧姿态并更新epoch；没有通用自动双IMU融合/切换逻辑。"
  ],
  "pitfalls": [
    "DM CAN 类保留纯虚init/service/snapshot，不能直接实例化；没有CAN接收器或对应Kconfig。",
    "两颗IMU不代表已融合。reference身份与epoch必须匹配消费者约定，不能随意相减不同零点的yaw。",
    "fresh_mask按字段变化；Running但Orientation不新鲜时不能使用上次四元数继续控制。",
    "ImuReceiver没有stop或析构协议，main栈上的短寿命对象不可交给它。",
    "H7的RS485 DMA缓冲需要__nocache和CONFIG_NOCACHE_MEMORY；避免CPU缓存看到旧字节。",
    "加热停机要看disable_error；软件目标占空比0并不自动证明PWM外设写入成功。"
  ],
  "config": [
    {
      "name": "CONFIG_SKYWALKER_IMU / BMI088 / RECEIVER",
      "description": "基础、板载传感器源、独立采集线程分别按需开启。"
    },
    {
      "name": "CONFIG_SKYWALKER_ATTITUDE_EKF",
      "description": "为BMI088生成姿态；不依赖线性Kalman或Matrix库。"
    },
    {
      "name": "CONFIG_SKYWALKER_IMU_DM_RS485",
      "description": "依赖UART_TRANSPORT并选择DM主动协议；设备单位、主动输出和速率需工具配置。"
    },
    {
      "name": "CONFIG_SKYWALKER_IMU_HEATER",
      "description": "需PWM，选择控制算法库；控制与source独立，但可由同一receiver持有。"
    },
    {
      "name": "Freshness",
      "description": "默认加速度/角速度/姿态20ms、温度1s；heater自己另外要求温度200ms内新鲜。"
    },
    {
      "name": "RX_STACK_SIZE / RX_PRIORITY",
      "description": "默认8192字节/优先级6；每个receiver独立线程，涉及浮点时按FPU配置分配寄存器上下文。"
    }
  ]
},
    {
      id:'kalman',title:'姿态 EKF 与通用 Kalman / Matrix',category:'控制算法',
      summary:'明确两条独立路线：BMI088 四元数 EKF；CMSIS-DSP 线性 Kalman 和矩阵工具。',
      responsibility:'QuaternionEkf维护每实例四元数、协方差、零偏和静止初始化。KalmanFilter提供通用线性预测/校正，调用者提供持久缓冲并显式Init，两者没有调用依赖。',
      status:'ready',statusNote:'独立QuaternionEkf已接BMI088输入与dual_imu样例；通用Kalman与Matrix为可选数学工具，不在当前BMI088姿态链中。整车仍需明确yaw外部参考、源选择和降级策略，EKF本身不提供绝对航向或双IMU融合。',
      source:[{label:'独立EKF API',path:'include/control/attitude_ekf.hpp'},{label:'EKF实现',path:'lib/control/attitude_ekf.cpp'},{label:'通用Kalman API',path:'include/control/kalman_filter.h'},{label:'通用Kalman实现',path:'lib/control/kalman_filter.c'},{label:'Matrix封装',path:'include/lib/matrix/matrix.h'},{label:'数学选项',path:'lib/Kconfig'}],docs:[{label:'Kalman与Matrix Markdown',path:'docs/modules/drivers/kalman-matrix.md'}],depends:[],
      interfaces:[
        api('QuaternionEkf::QuaternionEkf(const Config &c); int QuaternionEkf::init()', '构造保存配置，init验证全部正数/范围并重置每实例状态。不依赖通用线性Kalman，也不动态分配内存。', [['c','dt范围、重力和加速度门限、过程/测量噪声、创新门限、滤波/零偏时间常数及静止初始化样本数。']], 'init 0成功。', '单所有者；如果传给Bmi088Imu，则由该source.init初始化，不要先自行init。', '-EINVAL：非有限/非正配置、dt范围逆序、初始化样本数<2；-EALREADY：已初始化配置。'),
        api('int QuaternionEkf::update(const Input &input)', '输入加速度与角速度，静止初始化后用陀螺仪预测、重力方向校正倾斜；强加速度时保留预测并标记Degraded。yaw无可观测绝对约束。', [['input.accel_m_s2','同一传感器坐标的加速度，m/s²。'],['input.gyro_rad_s','角速度，rad/s。'],['input.time_us','两种输入对应的单调时间，us，必须严格递增。']], '0=有有效输出；-EAGAIN=初始化中/采样间隔过短/间隔过长触发重置；读取quality区分Tracking与Degraded。', '算法所有者线程；没有内部锁，不从别的线程并发update/reset/read。Bmi088消费者应通过source快照读取。', '-EACCES：未init；-EINVAL：非有限输入；-ESTALE：时间重复/倒退；-ERANGE：数值异常并重置。超过dt_max_s也会重置，generation递增。'),
        api('void QuaternionEkf::reset(); core::Quaternion QuaternionEkf::attitude() const; Quality QuaternionEkf::quality() const; uint32_t QuaternionEkf::generation() const', 'reset清除历史和零偏、进入Initializing、增加非零generation。attitude为q_WS，表示传感器坐标→局部重力对齐世界坐标的旋转。', [], '无返回码或值副本；Initializing时不得把默认四元数当有效姿态。', '仅所有者串行；source发布姿态时把estimator代次变化映射成OrientationReference.epoch。', 'reset不重新验证配置；Degraded仍有输出但加速度校正不足，需要消费者制定策略。'),
        api('int KalmanFilter_Init(KalmanFilter *kf, uint16_t n, uint16_t m, const KalmanBuffers *buffers)', '校验维度和七块缓冲容量后绑定调用者内存，设置F=I、P=1000I、Q=0.001I、R=I、H对角观测、X/K=0；无设备树和堆分配。重复调用覆盖模型及状态。', [['n / m','非零状态/观测维度。'],['buffers','七个KalmanBuffer的data与capacity，capacity以float元素计；数组互不重叠且覆盖使用周期。']], '0成功；失败不改变实例和缓冲。描述符只在调用中读取。', '同一实例由所有者线程串行初始化、配置、计算和读取；不用局部短寿命数组。', '-EINVAL：空指针/零维度；-ENOSPC：容量不足；-EOVERFLOW：矩阵索引或字节数范围溢出。'),
        api('void KalmanFilter_Predict(KalmanFilter *kf)', '通用线性预测：X=F·X，P=F·P·Fᵀ+Q。所有矩阵pData预先绑定，不由此函数分配持久内存。', [['kf','F/P/Q为n×n，X为n×1；有效非空缓冲。']], 'void；内部CMSIS运算返回状态未向调用方传播。', '同一滤波实例单线程所有者；按n²增长的临时栈数组，不宜在ISR使用。', '不会检查空指针、全部维度/数值或返回矩阵运算错误；调用者必须保证合法性，不能据void调用完成判定数值有效。'),
        api('void KalmanFilter_Correct(KalmanFilter *kf, const Matrix *z)', '线性校正：由P/H/R算K，再更新X/P；这是线性矩阵步骤，不是自动四元数EKF。', [['kf','H=m×n，R=m×m，K=n×m，F/P/Q/X与predict一致。'],['z','m×1观测矩阵，缓冲与维度正确；创新协方差须可逆。']], 'void，结果写入kf->X/P/K。', '单所有者线程；多块n²/m²/nm临时栈缓冲。', '当前实现不传播Matrix_Inverse失败；不可逆/错误维度/非有限数可污染状态，调用方须建立模型并保证条件。'),
        api('Matrix_Init(Matrix *matrix, uint16_t rows, uint16_t cols, float *buffer); void Matrix_Zero(Matrix *matrix); void Matrix_SetDiag(Matrix *matrix, float value)', 'Matrix是arm_matrix_instance_f32别名。Init绑定已有缓冲，Zero清零，SetDiag先清零再置对角元素；矩阵不拥有buffer。', [['matrix','待初始化/写入矩阵。'],['rows / cols','行列，buffer至少rows×cols个float。'],['buffer','调用者持有、存活时间覆盖全部使用。'],['value','对角值。']], 'void，修改结构体或缓冲。', '调用者管理内存与并发，不自动拷贝buffer。', '不提供空指针或容量检查；切勿把局部数组地址交给更长寿命对象。'),
        api('Matrix_Add / Matrix_Subtract / Matrix_Multiply / Matrix_Transpose / Matrix_Inverse', '宏别名分别对应CMSIS-DSP arm_mat_add_f32/sub_f32/mult_f32/trans_f32/inverse_f32；通过输入矩阵指针与已初始化目标矩阵运算。', [['输入 Matrix*','运算维度必须兼容。'],['目标 Matrix*','事先初始化正确尺寸与足量buffer；不要假设所有运算支持输入输出重叠。']], 'arm_status；调用者应处理维度不匹配、奇异矩阵等返回。', '共享buffer访问需要调用者串行/加锁。', '独立矩阵API有返回状态，但通用Kalman封装当前未传播。')
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
}`,notes:'此例只说明数学接口，不驱动电机。Bmi088Imu内由source调用init/update并发布已经变换到机体的姿态；不要在应用线程再操作同一个estimator。'},{title:'一维线性Kalman：静态缓冲与显式初始化',language:'cpp',code:`#include <control/kalman_filter.h>
#include <cmath>
#include <limits>

float smoothExample(float measurement) {
  const float invalid = std::numeric_limits<float>::quiet_NaN();
  if (!std::isfinite(measurement)) return invalid;
  static float f, h, r, x, p, q, gain;
  static KalmanFilter kf{};
  static const KalmanBuffers buffers{
    .F={&f,1}, .H={&h,1}, .R={&r,1}, .X={&x,1},
    .P={&p,1}, .Q={&q,1}, .K={&gain,1}};
  static bool initialized=false;
  if (!initialized) {
    const int rc = KalmanFilter_Init(&kf,1,1,&buffers);
    if (rc < 0) return invalid;
    initialized=true;
  }
  Matrix z{};
  Matrix_Init(&z,1,1,&measurement);
  KalmanFilter_Predict(&kf);
  KalmanFilter_Correct(&kf,&z);
  return kf.X.pData[0];
}`,notes:'示范平滑常量观测，Init默认R=1。此函数只能由一个线程串行调用；初始化失败或输入非有限返回NaN。开启CONFIG_SKYWALKER_LIB_KALMAN_FILTER，不需要设备树filter节点。void计算接口仍不传播内部矩阵错误。'}],
      lifecycle:['选定独立姿态EKF或通用线性模型，避免把二者串接成重复滤波。','QuaternionEkf：构造→init→稳定静止采样初始化→update→按quality使用；大时间缺口重置代次。','通用Kalman：分配全部F/H/R/X/P/Q/K与观测缓冲→Init并处理错误→设置模型→Predict→Correct。','重置姿态/参考系后，下游必须丢弃旧epoch的控制目标和历史。'],
      pitfalls:['没有磁力计/外部绝对观测的重力EKF不能提供绝对yaw；把yaw作为整车航向需要额外策略。','通用Kalman的void接口忽略内部矩阵错误，不能当作有错误恢复机制的通用安全封装。','CMSIS Matrix_Init只绑定内存，不分配、不复制；实例缓冲不可共享给独立滤波器。','线性Kalman已移除设备树入口；维度由调用者模型决定，不能把4/3维度等同于姿态EKF。'],
      config:[{name:'CONFIG_SKYWALKER_ATTITUDE_EKF',description:'独立姿态算法，配合控制库构建；默认dt 0.2–20ms、初始化100个静止样本。'},{name:'CONFIG_SKYWALKER_LIB_KALMAN_FILTER',description:'开启普通线性Kalman算法，自动选择CONTROL、Matrix和CMSIS-DSP；与BMI088独立EKF分别开启。'},{name:'QuaternionEkf::Config',description:'accel_gate_m_s2过滤明显非重力输入，innovation_gate控制测量校正，stationary_gyro_rad_s与initialization_samples控制静止初始化。'},{name:'CONFIG_SKYWALKER_LIB_MATRIX_STORAGE',description:'可选矩阵Flash存储，另需可用Flash/storage partition；不是姿态估计必需项。'}]
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
  "id": "motor-control",
  "title": "VelocityMotor / PositionMotor 硬件闭环",
  "category": "控制算法",
  "summary": "VelocityMotor / PositionMotor 保存最新目标，各轴独立反馈计算与自动重置历史。",
  "responsibility": "纯 C 控制算法 + producer 身份与计算版本；不管理运行许可，不创建线程，不发送 CAN。",
  "status": "ready",
  "statusNote": "现行 configure/update/telemetry 契约已实现，MotorSession、MotorSafety、ControlFailurePolicy 和 preflight 已移除。",
  "source": [
    {
      "label": "速度封装API",
      "path": "include/control/velocity_motor.hpp"
    },
    {
      "label": "位置封装API",
      "path": "include/control/position_motor.hpp"
    },
    {
      "label": "单位与安全配置",
      "path": "include/control/motor_common.hpp"
    },
    {
      "label": "闭环绑定实现",
      "path": "lib/control/motor_control.cpp"
    },
    {
      "label": "速度闭环完整样例",
      "path": "samples/motor/dji_speed_control/src/main.cpp"
    },
    {
      "label": "位置闭环样例",
      "path": "samples/motor/dji_position_control/src/main.cpp"
    }
  ],
  "docs": [
    {
      "label": "MotorControl Markdown",
      "path": "docs/modules/control/motor-control.md"
    },
    {
      "label": "调用顺序",
      "path": "docs/modules/call-examples.md"
    }
  ],
  "depends": [
    "motor-dji",
    "motor-dm",
    "pid"
  ],
  "interfaces": [
    {
      "signature": "VelocityMotor(Motor &, const Config &); PositionMotor(Motor &, const Config &); int configure()",
      "description": "configure 校验 PID、单位、输出限幅和反馈能力，绑定唯一 producer；按 attach/start/configure 顺序装配。",
      "parameters": [],
      "returns": "0 配置成功，重复 configure -EALREADY。",
      "errors": "非法 -EINVAL，输出限幅 -ERANGE，模式/能力不支持 -ENOTSUP，producer 冲突返回绑定错误。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int VelocityMotor::update(float target_rad_s, float dt_s); int PositionMotor::update(double target_position_rad, float dt_s)",
      "description": "合法目标已接受即返回 0，离线/参考等待时仍推进 target_sequence；output_valid 表示实际计算。恢复首可用周期重置本轴历史并提交带执行版本的零 effort。",
      "parameters": [
        {
          "name": "target",
          "meaning": "速度 rad/s 或目标位置 rad；PositionReference 决定参考"
        },
        {
          "name": "dt_s",
          "meaning": "实际周期 s；有限异常 dt 跳过本次积分，负数/非有限非法"
        }
      ],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "非法/超范围/未配置/producer 等调用错误；普通等待 error=0，原因由 telemetry.issue 观察。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "int VelocityMotor::reset(); int PositionMotor::reset()",
      "description": "清本轴控制历史、作废 computed effort；不能重定义 StartupRelative 原点。",
      "parameters": [],
      "returns": "0 为接受/配置成功；实际状态单独观测。",
      "errors": "未配置 -EACCES；下次有效周期自动初始化。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    },
    {
      "signature": "VelocityMotor::Telemetry telemetry() const; PositionMotor::Telemetry telemetry() const",
      "description": "值副本含 target_valid、target_sequence、output_valid、issue/error、同周期 MotorSnapshot 和计算输出。",
      "parameters": [],
      "returns": "一致遥测副本；目标被接受不表示已经产生 effort。",
      "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
      "context": "允许独立读取线程；configure/reset/update 由一个 owner 串行执行。"
    },
    {
      "signature": "enum class PositionReference { StartupRelative, DriverContinuous, AbsoluteNearest }",
      "description": "StartupRelative 保留首次可信原点，DriverContinuous 要求可信多圈坐标，AbsoluteNearest 用单圈绝对角本地展开。",
      "parameters": [],
      "returns": "参考策略；不是独立恢复授权。",
      "errors": "缺参考时本轴等待，不猜圈数、不重采启动零位。",
      "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
    }
  ],
  "examples": [
    {
      "title": "控制器每周期持续更新",
      "language": "cpp",
      "code": "if (run_requested) {\n    const int enabled = drive.enable();\n    const int accepted = axis.update(target_rad_s, dt_s);\n} else {\n    (void)drive.disable();\n}\nconst auto published = bus.commit();\nconst auto t = axis.telemetry();\n// t.target_valid/target_sequence 是接收；t.output_valid 是本周期输出。",
      "notes": "应用周期片段，构造与实物配置以链接源码为准。"
    }
  ],
  "lifecycle": [
    "静态构造，attach/start/configure。",
    "运行意图有效时持续 enable/update，每物理 CAN commit 一次。",
    "本轴反馈暂不可用时作废 computed effort，目标继续更新。",
    "enable/reference generation 变化后自动重置本轴控制历史。",
    "用户停止取消旧目标；StartupRelative 的首次可信原点保留。"
  ],
  "pitfalls": [
    "没有 ready/clearFault/MotorSafety/preflight 准入。",
    "DJI=A，DM MIT=N·m；原生 Velocity 模式不用主控力矩封装。",
    "同轴普通 setter 与封装 producer 不混用。",
    "反馈计算结果携带计算时 enable_generation；commit 不续期。"
  ],
  "config": [
    {
      "name": "Config.loop / effort_unit",
      "description": "PID、目标斜坡、请求目标/effort 限幅和 Ampere/NewtonMeter。"
    },
    {
      "name": "PositionReference",
      "description": "按机构选择可信坐标，恢复不改变首次 StartupRelative 原点。"
    }
  ]
},
    {
  "id": "boards",
  "title": "板级：MC02 / RoboMaster Type-C",
  "category": "工程与板级",
  "summary": "把物理引脚、外设、alias、时钟、DMA与电源连接交给Zephyr，业务参数保留在应用配置。",
  "responsibility": "DTS描述物理设备与连接，overlay选择应用实际使用的设备，Kconfig决定哪些实现进入构建，board_config.hpp提供电机ID/限幅/机械方向/链路身份。它们是四个不同层次。",
  "status": "ready",
  "statusNote": "两套板级存在；正式整车实物参数集中 calibration.hpp，connections_confirmed 默认 false；样例可另有 board_config。",
  "source": [
    {
      "label": "MC02设备树",
      "path": "boards/damiao/dm_mc02/dm_mc02.dts"
    },
    {
      "label": "MC02默认配置",
      "path": "boards/damiao/dm_mc02/dm_mc02_defconfig"
    },
    {
      "label": "MC02烧录配置",
      "path": "boards/damiao/dm_mc02/board.cmake"
    },
    {
      "label": "Type-C设备树",
      "path": "boards/rm_typec/rm_typec.dts"
    },
    {
      "label": "Type-C默认配置",
      "path": "boards/rm_typec/rm_typec_defconfig"
    },
    {
      "label": "双IMU板级参数示例",
      "path": "samples/imu/dual_imu/src/board_config.hpp"
    },
    {
      "label": "云台接线参数示例",
      "path": "samples/robotics/gimbal_control/src/board_config.hpp"
    }
  ],
  "docs": [
    {
      "label": "板级Markdown",
      "path": "docs/getting-started/boards.md"
    },
    {
      "label": "DMA与缓存",
      "path": "docs/guides/uart-dma.md"
    }
  ],
  "depends": [],
  "interfaces": [
    {
      "signature": "DEVICE_DT_GET(DT_NODELABEL(can1)); DEVICE_DT_GET(DT_ALIAS(remote_uart)); DEVICE_DT_GET(DT_ALIAS(accel0)); DEVICE_DT_GET(DT_ALIAS(gyro0))",
      "description": "取得Zephyr已描述设备的指针；node label指DTS节点标号，alias指应用连接名称。获取指针不启动业务驱动或允许电机输出。",
      "parameters": [
        {
          "name": "DT_NODELABEL(name)",
          "meaning": "直接选择节点label，例如物理can1/usart1。"
        },
        {
          "name": "DT_ALIAS(name)",
          "meaning": "选板或overlay约定的alias，DTS中横线在C宏里变下划线：remote-uart→remote_uart。"
        }
      ],
      "returns": "const device*；缺节点/未生成device通常是编译或链接错误，不是运行期空指针。",
      "context": "构造/初始化阶段使用；先device_is_ready，再交给Motor/UART/IMU等模块。",
      "errors": "Type-C的alias usart1指USART6，node label usart1指物理USART1，两者不可混淆；各应用overlay可能改alias。"
    },
    {
      "signature": "bool device_is_ready(const device *dev)",
      "description": "确认Zephyr驱动初始化成功，只证明设备层可用；不证明远端电机在线、UART协议正确或传感器姿态已初始化。",
      "parameters": [
        {
          "name": "dev",
          "meaning": "从DEVICE_DT_GET得到的设备。"
        }
      ],
      "returns": "true已就绪；false应用通常返回-ENODEV。",
      "context": "初始化线程和必要的读取位置，无业务I/O。",
      "errors": "各模块还需要自己的start/init与反馈检查。"
    },
    {
      "signature": "PWM_DT_SPEC_GET(DT_ALIAS(imu_heater))",
      "description": "取得加热PWM设备、通道、周期和极性，交给ImuHeater::Config::pwm。MC02为heat通道4，Type-C为通道1；默认PWM周期20ms。",
      "parameters": [
        {
          "name": "imu_heater",
          "meaning": "板级alias，不是自动温控算法。"
        }
      ],
      "returns": "pwm_dt_spec值。",
      "context": "初始化阶段，真实写PWM由ImuHeater所有者线程完成。",
      "errors": "PWM spec存在不表示温控已启动或有效温度已就绪；缺alias无法编译。"
    },
    {
      "signature": "int regulator_enable(const device *dev)",
      "description": "MC02的power1/power2固定电源口默认regulator-boot-off，应用显式启用后才给相应XT30支路供电。",
      "parameters": [
        {
          "name": "dev",
          "meaning": "DEVICE_DT_GET(DT_NODELABEL(power1)) 或power2，先device_is_ready。"
        }
      ],
      "returns": "0成功；负值由Zephyr regulator驱动返回。",
      "context": "启动线程，不能假设所有板卡都有power1/power2；用DT_NODE_EXISTS条件适配。",
      "errors": "确认支路连接后启用；开启电源不等于Motor软件enable，设备反馈与实际执行仍独立观测。"
    },
    {
      "signature": "app.overlay + prj.conf + src/board_config.hpp",
      "description": "三种应用配置入口：overlay声明物理资源/alias；prj.conf打开模块；board_config提供真实协议参数与机械约定。",
      "parameters": [
        {
          "name": "app.overlay",
          "meaning": "启用CAN/UART/传感器、设波特率/引脚、按应用绑定alias。"
        },
        {
          "name": "prj.conf",
          "meaning": "CAN、UART API、CPP、FPU、业务模块Kconfig选项。"
        },
        {
          "name": "board_config.hpp",
          "meaning": "电机地址/模式/零点/量程，速度电流力矩限幅，传输角色和connections_configured。"
        }
      ],
      "returns": "构建时配置，不是运行期接口。",
      "context": "先把本应用连接参数填写完整，再构建；上车固件初始化应据connections_configured决定是否允许运动。",
      "errors": "alias正确并不保证协议一致；修改overlay需pristine build。不要把电机型号/Group写成物理DTS驱动节点。"
    }
  ],
  "examples": [
    {
      "title": "MC02：显式开启所需XT30，并取得物理CAN",
      "language": "cpp",
      "code": "#include <zephyr/device.h>\n#include <zephyr/drivers/regulator.h>\n#include <drivers/motor/can_bus.hpp>\n#include <cerrno>\n\nint initializeBoardPower() {\n#if DT_NODE_EXISTS(DT_NODELABEL(power1))\n  // 仅在确认本车电机供电确实使用XT30_1后调用。\n  const device *power=DEVICE_DT_GET(DT_NODELABEL(power1));\n  if (!device_is_ready(power)) return -ENODEV;\n  const int r=regulator_enable(power);\n  if (r<0) return r;\n#endif\n  const device *can=DEVICE_DT_GET(DT_NODELABEL(can1));\n  return device_is_ready(can) ? 0 : -ENODEV;\n}\n\n// 后续业务初始化仍需：构造长寿命Motor/CanBus，attach，start，\n// 运行意图有效时持续enable/update/commit，设备独立恢复；不能因电源已开启直接写运动目标。\n",
      "notes": "此例沿用现有DM和恢复样例的Zephyr调用；连接确认由应用负责。Type-C无power1节点时跳过MC02专有逻辑。board device pointer与CanBus/Motor对象生命周期是两个层次。"
    },
    {
      "title": "构建入口与物理外设配置",
      "language": "cpp",
      "code": "// C++取设备；这是node label，始终选择物理can1。\nconst device *can = DEVICE_DT_GET(DT_NODELABEL(can1));\n// UART按应用alias取；可能由app.overlay重新映射。\nconst device *telemetry = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));\n// MC02: telemetry=USART1，remote=UART5，console/shell=USART10。\n// Type-C: telemetry=USART6，remote=USART3，console/shell=USART1。\n",
      "notes": "板目标：dm_mc02/stm32h723xx、rm_typec。示例构建命令：west build -p -b dm_mc02/stm32h723xx -d build/dual_imu samples/imu/dual_imu。此任务仅展示命令，不运行固件构建或烧录。"
    }
  ],
  "lifecycle": [
    "确定板卡与实际连接，选board目标。",
    "overlay绑定物理设备和alias；prj.conf纳入模块并选择UART/FPU/缓存策略。",
    "应用取得device并检查ready；按实际供电需要显式开启MC02电源支路。",
    "依赖资源构造长寿命业务对象并按模块生命周期启动。",
    "机械方向、零点、量程、通信身份与控制增益确认后才解除应用connections_configured门槛。"
  ],
  "pitfalls": [
    "MC02有can1/2/3，Type-C有can1/2；不能照抄第三路CAN或RS485 alias。",
    "当前Type-C DTS的console/shell是USART1，telemetry是USART6；alias usart1也指USART6，不能从名字猜物理设备。",
    "MC02的XT30电源默认关闭；电机离线可能是供电未显式启用而非CAN解码问题。",
    "H7 DMA缓存一致性需__nocache缓冲和CONFIG_NOCACHE_MEMORY；业务对象不一定全部放nocache，DMA实际字节存储需按接口要求放置。",
    "不同UART API/console/VOFA/协议收发器不能各自同时独占同一物理UART。",
    "DTS只负责硬件描述，Group、Motor型号和电机限幅仍由C++应用配置。"
  ],
  "config": [
    {
      "name": "dm_mc02/stm32h723xx",
      "description": "STM32H723，DTS CPU480MHz；CAN1/2/3，BMI088 SPI2，RS485 USART2/3，默认console USART10。"
    },
    {
      "name": "rm_typec",
      "description": "STM32F407，DTS CPU168MHz；CAN1/2，BMI088，UART1/3/6，console USART1、telemetry USART6。"
    },
    {
      "name": "SKYWALKER_OPENOCD_PROBE",
      "description": "board.cmake支持cmsis-dap、stlink、stlink-hla；烧录探针依板实际连接。"
    },
    {
      "name": "CONFIG_NOCACHE_MEMORY / CONFIG_FPU_SHARING",
      "description": "MC02默认启用nocache区；含浮点工作线程的样例显式配置FPU共享。"
    },
    {
      "name": "connections_configured",
      "description": "应用/云台样例中的接线确认开关；应和真实硬件/协议/机械限制一致，不能用一次编译成功代替确认。"
    }
  ]
}
  ]);
})();
