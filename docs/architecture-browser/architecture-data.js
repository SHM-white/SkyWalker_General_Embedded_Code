'use strict';
// Facts checked against main@99a97c9; historical entries retain their original context.
window.SKYWALKER_ARCHITECTURE = {
  "history": [
    {
      "hash": "fc75055",
      "date": "2026-09-22 19:15:29 +08:00",
      "title": "首次加入双主控交互式架构浏览器",
      "detail": "提交题为 docs: add interactive dual-controller architecture browser。index.html、styles.css、data.js、app.js 直接维护；原 README 说明浏览器是人工维护的架构快照，展示双板、模块关系与场景。Markdown 文档原本继续作为详细说明，没有被浏览器替代。"
    },
    {
      "hash": "4937c12",
      "date": "2026-09-27 01:30:17 +08:00",
      "title": "随多品牌电机重构增加电机详细链路",
      "detail": "该提交更新基础 data.js/app.js/index.html，并新增 motor-flow.js、motor-flow.css 和 run.sh。文档逐渐变成双板总览加独立电机交互视图；底层 Markdown 模块目录仍存在。"
    },
    {
      "hash": "32f07b9",
      "date": "2026-09-30 03:30:05 +08:00",
      "title": "新增 architecture-refresh.js 增量覆盖层",
      "detail": "提交题为 docs: add architecture browser v4 refresh，只新增 111 行刷新脚本。它继续使用基础 data.js/app.js 的全局对象，追加 IMU、视觉节点，并替换模块描述、场景和页面文字。"
    },
    {
      "hash": "d860163",
      "date": "2026-09-30 03:30:47 +08:00",
      "title": "HTML 启用覆盖层：改成当前叠加维护方式的时间点",
      "detail": "提交题为 docs: load architecture browser refresh。index.html 唯一变化是在 app.js 后、motor-flow.js 前加入 architecture-refresh.js，因此这是覆盖层实际进入浏览器执行的提交。"
    },
    {
      "hash": "a5054de",
      "date": "2026-09-30 03:31:07 +08:00",
      "title": "README 记录改动目的：同步视觉与独立 IMU",
      "detail": "提交说明刷新用于补入视觉 AB 通信与独立 IMU 架构，明确 data.js/app.js 仍为 v3 基线、刷新脚本为 v4 增量；并写明改动扩大时应将增量折回基础文件。Git 记录能够证明这个同步目的，无法证明某个作者另有取代 Markdown 的意图。"
    },
    {
      "hash": "cedab2a",
      "date": "2026-10-02 00:38:53 +08:00",
      "title": "在覆盖层继续追加新命令服务说明",
      "detail": "提交题为 docs: add command service documentation for registered command sources and arbitration service。新增注册来源、CommandManager worker、CommandArbiter 与本地执行器描述，并覆盖旧 GlobalSafetyManager/CommandRouter 的名称。旧基线和多次覆盖并存，需要读者区分多代接口；本次重建将当前事实与目标蓝图放入明确的数据结构。"
    },
    {
      "hash": "65a3de9",
      "date": "2026-10-02 01:13:51 +08:00",
      "title": "板间通信已扩展为 UART、RS485、CAN 三后端",
      "detail": "当前应用通过 ConfiguredInterBoardTransport 选择后端，默认 UART。这个新接入能力在新版架构中明确展示，并保留 Endpoint 的业务协议、boot ID、恢复代次与原始消息有效期语义。"
    },
    {
      "hash": "9dacbea",
      "date": "2026-10-05（提交顺序）",
      "title": "持续目标与每轴独立自动恢复",
      "detail": "移除 MotorSession、公共 ready/clearFault、Group 恢复授权和 resume_generation。控制器接受最新合法目标，各 Motor/CAN 独立恢复；板间只接受 v4。"
    },
    {
      "hash": "99a97c9",
      "date": "2026-10-05 21:17:49 +08:00",
      "title": "M2006 诊断与单舵轮基本功能调试",
      "detail": "本次代码提交更新 M2006 速度样例，增加前后快照、时间/周期/输出、原始反馈、CAN TX/恢复/TEC/REC 诊断。提交说明记录单舵轮基本功能调试完成，不扩展为四轮整车验收。"
    }
  ],
  "current": {
    "summary": "当前两个 sentry 入口共用 vehicle_bench。云台板运行命令服务、头部 IMU 惯性双轴、发射框架及小 Yaw 回中外环；底盘板运行四舵轮和大 Yaw 独立速度环。板间 v4，电机持续目标与逐轴恢复已实现；中央连接/安装确认关闭，真实功率/热量/拨盘原点和视觉上行来源仍待补，已有代码不等于实机闭环验收。",
    "lanes": [
      {
        "title": "云台板：输入、惯性双轴与统一发布",
        "steps": [
          {
            "title": "DR16 与可选裁判/视觉",
            "moduleId": "command-sources",
            "note": "默认 RemoteSource；VEHICLE_REFEREE、VISION_OBSERVE/EXECUTE 按阶段接入真实来源，保留原年龄。",
            "status": "ready"
          },
          {
            "title": "后台仲裁快照",
            "moduleId": "command",
            "note": "CommandManager worker → CommandArbiter → CommandSnapshot，多个消费者非消费式读取。",
            "status": "ready"
          },
          {
            "title": "头部 IMU 惯性适配",
            "moduleId": "inertial",
            "note": "外置头部 IMU 与两轴机械反馈转换成 Rate；两轴 output_valid 独立，imu_mounting_confirmed=false。",
            "status": "partial"
          },
          {
            "title": "公开双轴执行器",
            "moduleId": "gimbal",
            "note": "GimbalExecutor 持续更新小 Yaw/Pitch；输入/许可/急停检查保留，电机恢复不重采目标。",
            "status": "ready"
          },
          {
            "title": "共享 DJI 与 Pitch CAN",
            "moduleId": "motor-dji",
            "note": "小 Yaw、摩擦轮和拨盘 CAN1；Pitch DM CAN2。所有机构 stage 后每物理 CAN 一次 commit。",
            "status": "partial"
          }
        ]
      },
      {
        "title": "底盘板：轮控、大 Yaw 与独立恢复",
        "steps": [
          {
            "title": "板间 v4 输入",
            "moduleId": "interboard",
            "note": "七类消息保留 boot、producer sequence、原年龄；无恢复授权 generation，状态生产时效独立。",
            "status": "ready"
          },
          {
            "title": "四舵轮执行器",
            "moduleId": "chassis",
            "note": "ChassisExecutor 注入 SwerveHardware，FL/FR/RL/RR 逐轴有效性；CAN1 舵向，CAN3 轮驱，底盘 2ms 绝对节拍。",
            "status": "partial"
          },
          {
            "title": "大 Yaw 速度内环",
            "moduleId": "big-yaw",
            "note": "云台独立小 Yaw 中心外环请求 → 底盘 BigYawExecutor → DM MIT 速度控制，CAN2；不需固定绝对零点。",
            "status": "partial"
          },
          {
            "title": "电机独立自动恢复",
            "moduleId": "motor-control",
            "note": "运行意图有效时持续更新；掉线/协议恢复不形成全组就绪屏障，反馈计算携带本轴 enable generation。",
            "status": "ready"
          },
          {
            "title": "状态与真实测量回传",
            "moduleId": "interboard",
            "note": "RunStatus.stamp 与 BigYawFeedback 生产年龄保留；车体速度/实测功率仍未填入 ChassisFeedbackSummary。",
            "status": "partial"
          }
        ]
      },
      {
        "title": "阶段接入与剩余业务来源",
        "steps": [
          {
            "title": "摩擦轮与拨盘",
            "moduleId": "shooter",
            "note": "公开 ShooterExecutor 已装配；热量/可信原点 Pending 无效，有载供弹不可执行。",
            "status": "partial"
          },
          {
            "title": "视觉观察/执行阶段",
            "moduleId": "vision",
            "note": "观察不参与执行；执行须头部参考一致。真实弹速/弹数和视觉会话待补，AB 反馈 TX 默认关闭。",
            "status": "partial"
          },
          {
            "title": "功率测量与标定",
            "moduleId": "chassis",
            "note": "预算不是实测功率；IPowerMeasurementSource 默认 Pending，无效测量阻断功率模式。",
            "status": "partial"
          },
          {
            "title": "搜索与导航",
            "moduleId": "application",
            "note": "尚只有搜索规划与未装配的车体里程计/导航；不能标成已实现。",
            "status": "planned"
          }
        ]
      }
    ],
    "gaps": [
      {
        "title": "实物接线和标定确认关闭",
        "detail": "calibration.hpp 的 connections_confirmed、imu_mounting_confirmed、power_model_calibrated、shooter_constraints_confirmed 默认 false；ID/方向/零点/范围、安装旋转和机械几何须逐项验证。单舵轮经验比例不证明整车几何。",
        "source": "include/robotics/vehicle/calibration.hpp"
      },
      {
        "title": "真实功率源仍为 Pending",
        "detail": "run 支持注入 IPowerMeasurementSource；默认 PendingPowerSource 无效。裁判预算不能冒充实测功率，功率控制标定与估计回退默认不开放。",
        "source": "samples/robotics/common/vehicle_bench.hpp"
      },
      {
        "title": "有载供弹缺热量与可信原点",
        "detail": "IShooterHeatSource 与 IDialHomeSource 默认 Pending 无效；旧单发事件不能在恢复后重放，shots 只是软件执行统计。",
        "source": "include/robotics/shooter/shooter_executor.hpp"
      },
      {
        "title": "视觉反馈字段与参考会话未补齐",
        "detail": "视觉目标已能按阶段进入惯性适配；AB 线上没有 epoch 元数据。弹速/弹数缺真实来源，反馈 TX 默认禁用，不填占位数据。",
        "source": "samples/robotics/common/vehicle_bench.hpp"
      },
      {
        "title": "状态年龄已补，车体测量尚缺",
        "detail": "RunStatus.stamp 由执行线程产生，Endpoint 能识别执行停更；wireFeedback 的车体 vx/vy/wz/power 有效位仍未由实际测量填充。",
        "source": "include/robotics/execution/run_status.hpp"
      },
      {
        "title": "资源重分配与物理急停",
        "detail": "整车 CAN1/2/3 均用于底盘电机，默认板间 USART1；头部 IMU USART2、裁判 USART3，不能直接改成同口板间 RS485。裁判 TTL 到板载 RS485 需真实电气适配；物理急停仍按实物接入。",
        "source": "samples/robotics/common/vehicle_mc02.overlay"
      },
      {
        "title": "搜索/里程计/导航与整车验收",
        "detail": "搜索为设计记录，车体速度估计/导航未装配；最新提交单舵轮调通不扩展到双板惯性保持、四轮功率或有载发射。",
        "source": "docs/dev/哨兵空闲巡航与搜索模式规划.md"
      }
    ]
  },
  "target": {
    "summary": "最终上车在现有双板框架上完成真实接线、IMU 安装、机械/功率/发射标定和来源闭环；绿色表示软件模块已有，待接来源与行为扩展明确标为未完成。惯性双轴、大 Yaw 回中与 ShooterExecutor 已有实现，搜索/导航仍为目标。",
    "lanes": [
      {
        "title": "可信输入与机械标定",
        "steps": [
          {
            "title": "头部 IMU 安装与参考",
            "moduleId": "imu",
            "note": "真实安装变换、姿态质量和 frame_id/epoch；视觉目标使用同参考。",
            "status": "partial"
          },
          {
            "title": "真实裁判/功率/热量",
            "moduleId": "referee",
            "note": "选协议 profile、绑定电气接口和独立测量原时间；不能伪造许可或预算。",
            "status": "partial"
          },
          {
            "title": "拨盘 home/index 来源",
            "moduleId": "shooter",
            "note": "真实原点来源后开放有载供弹；摩擦稳速、热量、头部稳定持续检查。",
            "status": "partial"
          }
        ]
      },
      {
        "title": "现有双板执行框架",
        "steps": [
          {
            "title": "后台仲裁快照",
            "moduleId": "command",
            "note": "CommandManager worker → CommandArbiter → CommandSnapshot，多个消费者非消费式读取。",
            "status": "ready"
          },
          {
            "title": "头部 IMU 惯性适配",
            "moduleId": "inertial",
            "note": "外置头部 IMU 与两轴机械反馈转换成 Rate；两轴 output_valid 独立，imu_mounting_confirmed=false。",
            "status": "partial"
          },
          {
            "title": "公开双轴执行器",
            "moduleId": "gimbal",
            "note": "GimbalExecutor 持续更新小 Yaw/Pitch；输入/许可/急停检查保留，电机恢复不重采目标。",
            "status": "ready"
          },
          {
            "title": "四舵轮执行器",
            "moduleId": "chassis",
            "note": "ChassisExecutor 注入 SwerveHardware，FL/FR/RL/RR 逐轴有效性；CAN1 舵向，CAN3 轮驱，底盘 2ms 绝对节拍。",
            "status": "partial"
          },
          {
            "title": "大 Yaw 速度内环",
            "moduleId": "big-yaw",
            "note": "云台独立小 Yaw 中心外环请求 → 底盘 BigYawExecutor → DM MIT 速度控制，CAN2；不需固定绝对零点。",
            "status": "partial"
          }
        ]
      },
      {
        "title": "反馈闭环与扩展",
        "steps": [
          {
            "title": "AB 真实反馈闭环",
            "moduleId": "vision",
            "note": "补弹速/弹数与明确参考会话后才启用上行。",
            "status": "partial"
          },
          {
            "title": "车体速度与里程计",
            "moduleId": "chassis",
            "note": "由轮测量/姿态估计可信机体速度，有效测量再置位。",
            "status": "planned"
          },
          {
            "title": "搜索行为与导航",
            "moduleId": "application",
            "note": "行为层生产有单位/有时效目标，经过命令权限，不绕过本地执行。",
            "status": "planned"
          },
          {
            "title": "急停与实机逐级验收",
            "moduleId": "boards",
            "note": "接真实 GPIO/动力切断，记录停输出延迟、资源/栈余量和电气恢复。",
            "status": "partial"
          }
        ]
      }
    ],
    "requirements": [
      {
        "title": "目标与原输入年龄",
        "detail": "CommandManager 读取、板间重发和 CAN commit 不给旧输入续期；ms/us/s 按 API 区分。"
      },
      {
        "title": "机构独立执行与故障范围",
        "detail": "Group 仅批量启停，电机恢复不撤销其他机构目标；一个物理 CAN 的状态仍影响其成员。"
      },
      {
        "title": "数学参考与运行意图",
        "detail": "IMU 身份与机械参考决定对应计算有效性，反馈 ready 不形成重新授权边界；不能猜连续圈数。"
      },
      {
        "title": "物理总线唯一 owner",
        "detail": "应用先 stage 全部机构后统一 commit；底盘 CAN 已占满，传输后端更换先重分配资源。"
      },
      {
        "title": "状态生产与测量时效",
        "detail": "执行停更时旧 ready/armed 到期，不以通信心跳掩盖；目标值不能当实测速度/功率。"
      },
      {
        "title": "真实发射来源与离散事件",
        "detail": "热量/原点/许可/稳定状态真实且新鲜；旧单发消费丢弃，软件 shots 不是实测弹数。"
      },
      {
        "title": "逐级验收",
        "detail": "单轴、惯性保持、双 Yaw 回中、四舵轮、真实功率、受约束发射逐级记录，编译/软件入口不能替代实机。"
      }
    ]
  },
  "scenarios": [
    {
      "id": "manual-gimbal",
      "title": "遥控如何驱动惯性双轴",
      "summary": "当前公共整车框架调用链，实物确认默认关闭。",
      "steps": [
        {
          "title": "DR16 与可选裁判/视觉",
          "moduleId": "command-sources",
          "detail": "默认 RemoteSource；VEHICLE_REFEREE、VISION_OBSERVE/EXECUTE 按阶段接入真实来源，保留原年龄。"
        },
        {
          "title": "后台仲裁快照",
          "moduleId": "command",
          "detail": "CommandManager worker → CommandArbiter → CommandSnapshot，多个消费者非消费式读取。"
        },
        {
          "title": "头部 IMU 惯性适配",
          "moduleId": "inertial",
          "detail": "外置头部 IMU 与两轴机械反馈转换成 Rate；两轴 output_valid 独立，imu_mounting_confirmed=false。"
        },
        {
          "title": "公开双轴执行器",
          "moduleId": "gimbal",
          "detail": "GimbalExecutor 持续更新小 Yaw/Pitch；输入/许可/急停检查保留，电机恢复不重采目标。"
        },
        {
          "title": "共享 DJI 与 Pitch CAN",
          "moduleId": "motor-dji",
          "detail": "小 Yaw、摩擦轮和拨盘 CAN1；Pitch DM CAN2。所有机构 stage 后每物理 CAN 一次 commit。"
        }
      ],
      "sources": [
        {
          "label": "vehicle_bench.hpp",
          "path": "samples/robotics/common/vehicle_bench.hpp"
        },
        {
          "label": "inertial_gimbal.cpp",
          "path": "lib/robotics/inertial_gimbal.cpp"
        },
        {
          "label": "gimbal_executor.cpp",
          "path": "lib/robotics/gimbal_executor.cpp"
        }
      ]
    },
    {
      "id": "vision-auto",
      "title": "视觉目标如何进入自动瞄准",
      "summary": "已有可选观察/执行阶段，真实参考会话与反馈来源尚未完成。",
      "steps": [
        {
          "title": "AB 目标下行",
          "moduleId": "vision",
          "detail": "29B 下行有限值/CRC 校验，AimMeasurement 保留原时间。"
        },
        {
          "title": "按阶段注册 VisionSource",
          "moduleId": "command-sources",
          "detail": "VEHICLE_VISION_EXECUTE 注册 Aim 并启用 Auto；观察配置不执行。"
        },
        {
          "title": "Auto 仲裁",
          "moduleId": "command",
          "detail": "需要操作者 Auto、新目标和有效许可；失效 Hold，人工可接管。"
        },
        {
          "title": "头部参考匹配与惯性适配",
          "moduleId": "inertial",
          "detail": "selected_vision.reference 与 head.sample.reference 匹配后生成机械双轴 Rate。"
        },
        {
          "title": "机械执行与真实反馈缺口",
          "moduleId": "gimbal",
          "detail": "GimbalExecutor 更新双轴；弹速/弹数未接，AB 反馈 TX 禁用。"
        }
      ],
      "sources": [
        {
          "label": "vehicle_bench.hpp",
          "path": "samples/robotics/common/vehicle_bench.hpp"
        },
        {
          "label": "vision.md",
          "path": "docs/modules/communication/vision.md"
        },
        {
          "label": "command.cpp",
          "path": "lib/robotics/command.cpp"
        }
      ]
    },
    {
      "id": "interboard-chassis",
      "title": "云台命令如何驱动轮控和大 Yaw",
      "summary": "v4 输入身份/年龄与真实状态生产时效；电机反馈不门控目标发送。",
      "steps": [
        {
          "title": "板间 v4 输入",
          "moduleId": "interboard",
          "detail": "七类消息保留 boot、producer sequence、原年龄；无恢复授权 generation，状态生产时效独立。"
        },
        {
          "title": "四舵轮执行器",
          "moduleId": "chassis",
          "detail": "ChassisExecutor 注入 SwerveHardware，FL/FR/RL/RR 逐轴有效性；CAN1 舵向，CAN3 轮驱，底盘 2ms 绝对节拍。"
        },
        {
          "title": "大 Yaw 速度内环",
          "moduleId": "big-yaw",
          "detail": "云台独立小 Yaw 中心外环请求 → 底盘 BigYawExecutor → DM MIT 速度控制，CAN2；不需固定绝对零点。"
        },
        {
          "title": "电机独立自动恢复",
          "moduleId": "motor-control",
          "detail": "运行意图有效时持续更新；掉线/协议恢复不形成全组就绪屏障，反馈计算携带本轴 enable generation。"
        },
        {
          "title": "状态与真实测量回传",
          "moduleId": "interboard",
          "detail": "RunStatus.stamp 与 BigYawFeedback 生产年龄保留；车体速度/实测功率仍未填入 ChassisFeedbackSummary。"
        }
      ],
      "sources": [
        {
          "label": "interboard_endpoint.cpp",
          "path": "lib/communication/interboard_endpoint.cpp"
        },
        {
          "label": "chassis_executor.cpp",
          "path": "lib/robotics/chassis_executor.cpp"
        },
        {
          "label": "big_yaw_executor.cpp",
          "path": "lib/robotics/big_yaw_executor.cpp"
        }
      ]
    },
    {
      "id": "imu-feedback",
      "title": "IMU 如何进入惯性控制与视觉反馈",
      "summary": "头部 IMU 惯性控制已装配，AB 反馈的非 IMU 字段仍待补。",
      "steps": [
        {
          "title": "头部外置采集",
          "moduleId": "imu",
          "detail": "云台 USART2 主动 RS485 source → ImuReceiver，保留字段原 stamp/quality/reference。"
        },
        {
          "title": "IMU 与机械快照",
          "moduleId": "inertial",
          "detail": "惯性适配 owner 同周期读取头部和两轴快照；缺少数学测量只作废对应轴。"
        },
        {
          "title": "参考变化处理",
          "moduleId": "inertial",
          "detail": "frame_id/epoch 是坐标身份，不能用旧视觉目标匹配新参考。"
        },
        {
          "title": "视觉反馈完整来源",
          "moduleId": "vision",
          "detail": "orientation/gyro 可由真实 IMU 组装；弹速/弹数和显式会话仍待补，上行默认关闭。"
        }
      ],
      "sources": [
        {
          "label": "imu_types.hpp",
          "path": "include/drivers/imu/imu_types.hpp"
        },
        {
          "label": "inertial_gimbal.cpp",
          "path": "lib/robotics/inertial_gimbal.cpp"
        },
        {
          "label": "vehicle_bench.hpp",
          "path": "samples/robotics/common/vehicle_bench.hpp"
        }
      ]
    },
    {
      "id": "stop-recovery",
      "title": "掉线与主动停止的不同恢复路径",
      "summary": "最新持续控制契约：输入有效时每轴自动恢复，真正停止取消旧命令。",
      "steps": [
        {
          "title": "持续原输入与目标",
          "moduleId": "command",
          "detail": "输入有效时持续提交，快照读取不续期。"
        },
        {
          "title": "每轴目标接受",
          "moduleId": "motor-control",
          "detail": "update=0 表示接受，缺反馈/参考时 output_valid=false；其他轴继续。"
        },
        {
          "title": "Motor/CAN 独立协议恢复",
          "moduleId": "motor-dm",
          "detail": "限频探测、清错/使能，最新未过期目标保留；Group 不传播故障。"
        },
        {
          "title": "反馈计算版本",
          "moduleId": "motor-control",
          "detail": "恢复首可用周期自动重建本轴 PID，computed effort 携带计算 enable_generation。"
        },
        {
          "title": "主动停止取消",
          "moduleId": "motor-dji",
          "detail": "用户停止/急停/输入过期撤销运行意图，旧目标与协议操作失效；停止不被电机上线复活。"
        },
        {
          "title": "状态与真实板身份",
          "moduleId": "interboard",
          "detail": "执行状态 stamp 过期撤销旧 ready/armed；v4 没有 resume_generation，真实 boot/link 与原年龄仍校验。"
        }
      ],
      "sources": [
        {
          "label": "motor-workflow.md",
          "path": "docs/guides/motor-workflow.md"
        },
        {
          "label": "command-recovery.md",
          "path": "docs/modules/robotics/command-recovery.md"
        },
        {
          "label": "motor.cpp",
          "path": "drivers/motor/motor.cpp"
        }
      ]
    },
    {
      "id": "shooter",
      "title": "发射请求如何成为供弹动作",
      "summary": "执行框架已有，有载真实来源默认无效。",
      "steps": [
        {
          "title": "仲裁请求与离散事件",
          "moduleId": "command",
          "detail": "请求与事件源时间独立，单发 ID 去重，旧事件不在恢复后重放。"
        },
        {
          "title": "真实业务条件",
          "moduleId": "shooter",
          "detail": "真实许可/热量/拨盘原点/摩擦稳定/头部状态全部满足；默认 Pending 测量无效。"
        },
        {
          "title": "摩擦与拨盘独立目标",
          "moduleId": "shooter",
          "detail": "两速度轴与一位置轴持续更新，卡滞/热量业务策略沿用。"
        },
        {
          "title": "共享 DJI 提交",
          "moduleId": "motor-dji",
          "detail": "小云台与发射全部 stage 后统一 CAN1 commit。"
        }
      ],
      "sources": [
        {
          "label": "shooter_executor.cpp",
          "path": "lib/robotics/shooter_executor.cpp"
        },
        {
          "label": "vehicle_bench.hpp",
          "path": "samples/robotics/common/vehicle_bench.hpp"
        }
      ]
    }
  ],
  "startup": [
    {
      "title": "1 · 填真实中央标定与资源",
      "detail": "calibration.hpp、vehicle_mc02.overlay、阶段 prj.conf；connections/IMU/power/shooter 的确认标志分别核对。"
    },
    {
      "title": "2 · 静态构造与启动",
      "detail": "输入接收器、来源、Transport/Endpoint、Motor/CanBus、Group 和快照存储长期存活；应用 attach/start，机构 begin。"
    },
    {
      "title": "3 · 注册来源并采样",
      "detail": "默认 Remote，视觉/裁判按阶段选；start 成功仅表示线程启动，数据新鲜度仍独立检查。"
    },
    {
      "title": "4 · 唯一执行 owner",
      "detail": "云台约 5ms、底盘2ms绝对节拍；各机构先更新，统一每物理 CAN commit。通信1ms poll只交换快照。"
    },
    {
      "title": "5 · 持续目标与逐轴恢复",
      "detail": "运行意图有效即持续 enable/update/commit，不等全体 ready；本轴缺反馈或参考只作废本轴计算。"
    },
    {
      "title": "6 · 真实输入撤销",
      "detail": "主动停止/急停/输入过期取消旧目标；电机上线不能复活停止前命令；真实 boot/link 变化按输入身份处理。"
    },
    {
      "title": "7 · 完成来源与验收",
      "detail": "功率/热量/拨盘原点/视觉上行不使用占位值；按单轴到整车记录，单舵轮基本功能与整车验收分开。"
    }
  ],
  "sources": [
    {
      "label": "README.md",
      "path": "docs/README.md"
    },
    {
      "label": "executors.md",
      "path": "docs/modules/robotics/executors.md"
    },
    {
      "label": "dual-controller.md",
      "path": "docs/applications/dual-controller.md"
    },
    {
      "label": "motor-workflow.md",
      "path": "docs/guides/motor-workflow.md"
    },
    {
      "label": "interboard-transports.md",
      "path": "docs/modules/communication/interboard-transports.md"
    },
    {
      "label": "vehicle_bench.hpp",
      "path": "samples/robotics/common/vehicle_bench.hpp"
    },
    {
      "label": "calibration.hpp",
      "path": "include/robotics/vehicle/calibration.hpp"
    },
    {
      "label": "README.md",
      "path": "samples/robotics/swerve/README.md"
    },
    {
      "label": "README.md",
      "path": "samples/motor/m2006_speed_control/README.md"
    }
  ],
  "baseline": {
    "branch": "main",
    "commit": "99a97c91e7e8ed684b9758e0edf6b12bf73b0d79",
    "date": "2026-10-05"
  }
};
