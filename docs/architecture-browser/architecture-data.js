'use strict';
// Documentation snapshot of workspace 48984ed (2026-10-25); historical entries retain their context.
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
    "summary": "两个 sentry 最终工程各自独立装配并运行 main：云台为手动双轴惯性/大 Yaw 回中与行走发送，底盘为八电机轮控/大 Yaw 执行；上车工程共用 installed 配置与纯 profile。当前未装配视觉、裁判、发射或实测功率闭环。27 个相关工程已编译，尚未实板验证；连接/IMU/发射确认现值 true、功率模型 false，配置值不等于现场通过。",
    "lanes": [
      {
        "title": "云台板：固定手动输入、CAN3 惯性双轴与统一提交",
        "steps": [
          {
            "title": "DR16 与真实 RC 解锁",
            "moduleId": "command-sources",
            "note": "UART5 RemoteSource；Down/Down 归中 500ms 后左 Middle 解锁，输入失效/peer 重启撤权，CommandManager 外部门控保留原年龄。",
            "status": "ready"
          },
          {
            "title": "后台仲裁快照",
            "moduleId": "command",
            "note": "当前只注册 RemoteSource；CommandManager worker → CommandSnapshot，执行循环非消费式读取。",
            "status": "ready"
          },
          {
            "title": "CAN3 头部惯性适配",
            "moduleId": "inertial",
            "note": "DmImuCanSource 主动四元数 + 两轴机械反馈 → Rate；安装确认现值 true，实车安装/质量/参考待确认。",
            "status": "partial"
          },
          {
            "title": "独立双轴执行器",
            "moduleId": "gimbal",
            "note": "main 每 5ms 更新 GimbalExecutor；小 Yaw CAN1、Pitch CAN2，每周期各提交一次；不创建发射机构。",
            "status": "ready"
          },
          {
            "title": "DAPLink 单口诊断",
            "moduleId": "telemetry",
            "note": "UART7，diagnostic_output 选择 Log/Vofa 后重编译；启动失败继续报告状态，队列满丢观测并计数。",
            "status": "ready"
          }
        ]
      },
      {
        "title": "底盘板：轮控、大 Yaw 与独立恢复",
        "steps": [
          {
            "title": "板间 v4 输入",
            "moduleId": "interboard",
            "note": "云台 USART10 / 底盘 USART1，460800；main 推进 endpoint，真实 OperatorControl/boot/原年龄分别检查。",
            "status": "ready"
          },
          {
            "title": "四舵轮执行器",
            "moduleId": "chassis",
            "note": "main 独立装配 SwerveHardware/ChassisExecutor，CAN1 舵向、CAN2 轮驱，2ms 绝对节拍；当前仅低电流手动机械链。",
            "status": "partial"
          },
          {
            "title": "大 Yaw 速度内环",
            "moduleId": "big-yaw",
            "note": "小 Yaw 中心外环 → v4 关节 rad/s → 底盘 BigYawExecutor/DM CAN3；原始年龄、boot 与本轴反馈独立检查。",
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
        "title": "后续能力：当前最终应用未装配",
        "steps": [
          {
            "title": "摩擦轮与拨盘",
            "moduleId": "shooter",
            "note": "ShooterExecutor 库已有，当前最终云台不创建发射电机；真实热量/许可/归位未接，有载未完成。",
            "status": "planned"
          },
          {
            "title": "视觉观察/执行阶段",
            "moduleId": "vision",
            "note": "当前最终应用未创建视觉；UART7 诊断独占，无 vision-uart alias。需独立资源、参考会话及真实反馈后接入。",
            "status": "planned"
          },
          {
            "title": "功率测量与标定",
            "moduleId": "chassis",
            "note": "当前低电流机械策略不启用功率闭环；真实测量、预算与模型标定需独立接入，预算不能冒充实测。",
            "status": "planned"
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
        "title": "实物接线、确认值与标定记录",
        "detail": "calibration.hpp 连接/IMU 安装/发射约束现值 true，功率模型 false；不代表本次已完成实板核对。ID/方向/零点/限位、IMU 安装与机械几何仍需逐项记录。",
        "source": "include/robotics/vehicle/calibration.hpp"
      },
      {
        "title": "实测功率与模型尚未接入最终应用",
        "detail": "当前最终底盘为低电流手动机械链，不启用实测功率闭环或估计回退。历史 PendingPowerSource 仍不是有效测量。",
        "source": "include/robotics/vehicle/chassis_profile.hpp"
      },
      {
        "title": "有载供弹未装配，真实来源待补",
        "detail": "当前最终云台不创建发射电机。库 API 保留，真实热量/许可与拨盘可信原点仍需独立接入和机构标定。",
        "source": "include/robotics/shooter/shooter_executor.hpp"
      },
      {
        "title": "视觉资源与参考会话未补齐",
        "detail": "当前最终应用未装配视觉。新共享配置无 vision-uart，UART7 由诊断独占；AB 参考会话、真实弹速/弹数需补齐。",
        "source": "config/vehicle/installed/README.md"
      },
      {
        "title": "状态年龄已保留，车体测量尚缺",
        "detail": "RunStatus.stamp 由执行循环生产，Endpoint 识别执行停更；车体速度/功率有效位仍未由真实测量填入。",
        "source": "include/robotics/execution/run_status.hpp"
      },
      {
        "title": "DAPLink 接线与物理资源待现场核对",
        "detail": "云台 USART10 板间/CAN3 IMU，底盘 USART1 板间/CAN1舵向/CAN2轮驱/CAN3大Yaw；两板候选 UART7 诊断，DAPLink 实际接线未实查。",
        "source": "config/vehicle/installed/README.md"
      },
      {
        "title": "编译记录与实板结论分开",
        "detail": "27 个相关工程编译完成，未实板运行；方向、停止时限和组合性能仍按独立上车阶段记录。搜索/里程计/导航尚未装配。",
        "source": "samples/bringup/README.md"
      }
    ]
  },
  "target": {
    "summary": "后续在独立工程和共享实车配置上逐级完成接线、安装、方向与机构记录，再接视觉、真实功率与有载发射来源。库实现、最终应用装配、工程编译和硬件通过分别登记；搜索/导航仍为目标。",
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
            "note": "CAN3 主动头部 IMU 与两轴机械反馈转换成 Rate；安装确认现值 true，实车安装/质量/参考仍待验证。",
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
            "note": "ChassisExecutor 注入应用持有的 SwerveHardware；CAN1 舵向、CAN2 轮驱，底盘 2ms 绝对节拍，当前为低电流机械策略。",
            "status": "partial"
          },
          {
            "title": "大 Yaw 速度内环",
            "moduleId": "big-yaw",
            "note": "小 Yaw 中心外环 → 底盘 BigYawExecutor → DM CAN3 关节速度内环；boot/来源年龄/反馈独立检查。",
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
      "summary": "固定云台 main 的真实手动调用链；独立工程已编译，未进行实板动作。",
      "steps": [
        {
          "title": "DR16 与真实 RC 解锁",
          "moduleId": "command-sources",
          "detail": "固定手动入口只注册 UART5 RemoteSource，真实 RC 归中解锁接入外部门控；保留原输入年龄。"
        },
        {
          "title": "后台仲裁快照",
          "moduleId": "command",
          "detail": "CommandManager worker → CommandArbiter → CommandSnapshot，多个消费者非消费式读取。"
        },
        {
          "title": "头部 IMU 惯性适配",
          "moduleId": "inertial",
          "detail": "CAN3 主动头部 IMU 与两轴机械反馈转换成 Rate；安装确认现值 true，实车安装/质量/参考仍待验证。"
        },
        {
          "title": "公开双轴执行器",
          "moduleId": "gimbal",
          "detail": "GimbalExecutor 持续更新小 Yaw/Pitch；输入/许可/急停检查保留，电机恢复不重采目标。"
        },
        {
          "title": "小 Yaw CAN1 与 Pitch CAN2",
          "moduleId": "motor-dji",
          "detail": "当前最终云台只装配小 Yaw CAN1 与 Pitch DM CAN2，每周期各提交一次；发射机构未装配。"
        }
      ],
      "sources": [
        {
          "label": "sentry_gimbal/main.cpp",
          "path": "applications/sentry_gimbal/src/main.cpp"
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
      "summary": "后续接入路线；库与历史样例已有，当前两个最终应用未装配视觉。",
      "steps": [
        {
          "title": "AB 目标下行",
          "moduleId": "vision",
          "detail": "29B 下行有限值/CRC 校验，AimMeasurement 保留原时间。"
        },
        {
          "title": "按阶段注册 VisionSource",
          "moduleId": "command-sources",
          "detail": "历史宏样例曾注册 VisionSource；新最终应用未注册，后续需独立观察/执行工程与专属串口。"
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
          "detail": "后续在参考一致和真实反馈来源具备后接入机械执行；当前最终应用未启用视觉或 AB 上行。"
        }
      ],
      "sources": [
        {
          "label": "历史 vehicle_integration 说明",
          "path": "samples/robotics/vehicle_integration/README.md"
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
      "summary": "固定两应用传输 v4 输入身份/原年龄与真实执行状态；云台 USART10、底盘 USART1。",
      "steps": [
        {
          "title": "板间 v4 输入",
          "moduleId": "interboard",
          "detail": "七类消息保留 boot、producer sequence、原年龄；无恢复授权 generation，状态生产时效独立。"
        },
        {
          "title": "四舵轮执行器",
          "moduleId": "chassis",
          "detail": "ChassisExecutor 注入应用持有的 SwerveHardware；CAN1 舵向、CAN2 轮驱，底盘 2ms 绝对节拍，当前为低电流机械策略。"
        },
        {
          "title": "大 Yaw 速度内环",
          "moduleId": "big-yaw",
          "detail": "小 Yaw 中心外环 → 底盘 BigYawExecutor → DM CAN3 关节速度内环；boot/来源年龄/反馈独立检查。"
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
          "label": "sentry_chassis/main.cpp",
          "path": "applications/sentry_chassis/src/main.cpp"
        },
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
      "summary": "当前 CAN3 头部 IMU 已进入惯性云台；视觉反馈属于未接入的后续能力。",
      "steps": [
        {
          "title": "头部外置采集",
          "moduleId": "imu",
          "detail": "云台 CAN3 DmImuCanSource 主动四元数 → ImuReceiver，保留原 stamp/quality/reference；不是 USART2 RS485。"
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
          "detail": "后续视觉可使用真实 orientation/gyro；当前未装配视觉，仍缺显式会话与弹速/弹数来源。"
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
          "label": "gimbal_profile.hpp",
          "path": "include/robotics/vehicle/gimbal_profile.hpp"
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
      "summary": "后续有载发射路线；库/历史样例保留，当前最终云台未创建发射电机。",
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
          "detail": "后续组合需所有 DJI 机构暂存后由唯一 owner 提交；当前最终云台只提交小 Yaw 与 Pitch。"
        }
      ],
      "sources": [
        {
          "label": "shooter_executor.cpp",
          "path": "lib/robotics/shooter_executor.cpp"
        },
        {
          "label": "历史发射样例",
          "path": "samples/robotics/shooter_bench/README.md"
        }
      ]
    }
  ],
  "startup": [
    {
      "title": "1 · 选择固定工程与共享配置",
      "detail": "按 samples/bringup 目录选择阶段；installed overlay/conf 固定资源，calibration/profile 共用标定。确认值 true 仍需实车记录。"
    },
    {
      "title": "2 · 先诊断，再装配业务对象",
      "detail": "Diagnostics 先发布身份与资源；每个 main 自行创建长期来源/Endpoint/Motor/CanBus/Group，attach/start 后 begin；初始化失败保留诊断。"
    },
    {
      "title": "3 · 真实 RC 授权与原始输入",
      "detail": "固定手动云台只注册 RemoteSource；Down/Down 归中 500ms 后左 Middle 解锁，底盘接收原始授权；start 成功不代表测量有效。"
    },
    {
      "title": "4 · 唯一执行 owner",
      "detail": "云台 5ms、底盘 2ms 绝对节拍，main 内推进 endpoint；各机构 stage 后每物理 CAN 一次 commit，诊断 worker 只发送观测。"
    },
    {
      "title": "5 · 持续目标与逐轴恢复",
      "detail": "当前输入和许可有效时持续更新；Motor/CanBus 推进本轴恢复，不能伪造新的 RC 解锁或板间 boot。"
    },
    {
      "title": "6 · 原始年龄与真实停止",
      "detail": "输入撤销/过期、实际 peer boot/link 改变撤权；v4 保留原始年龄与状态生产年龄，重发不延长旧运动。"
    },
    {
      "title": "7 · 单口诊断与实车记录",
      "detail": "只改 diagnostic_output 后重编译烧录选择 DAPLink Log/Vofa；27 工程仅编译完成，实板逐级确认，视觉/功率/有载发射继续独立接入。"
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
    },
    {
      "label": "独立上车工程",
      "path": "samples/bringup/README.md"
    },
    {
      "label": "实车共享配置",
      "path": "config/vehicle/installed/README.md"
    },
    {
      "label": "板间 v4 协议契约",
      "path": "docs/protocols/interboard-v4.md"
    },
    {
      "label": "sentry_gimbal/main.cpp",
      "path": "applications/sentry_gimbal/src/main.cpp"
    },
    {
      "label": "sentry_chassis/main.cpp",
      "path": "applications/sentry_chassis/src/main.cpp"
    }
  ],
  "baseline": {
    "branch": "工作区",
    "commit": "48984ed90898726e8a9d451ac07bdc88c4663f11",
    "date": "2026-10-25"
  }
};
