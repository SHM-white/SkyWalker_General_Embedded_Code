'use strict';
// Current source baseline: main@99a97c9 (2026-10-05).
window.SKYWALKER_CATEGORIES = [
  "工程与板级",
  "驱动与感知",
  "控制算法",
  "通信与输入",
  "机器人与仲裁",
  "调试与观测"
];
window.SKYWALKER_GRAPH_COLUMNS = [
  {
    "title": "设备与测量",
    "ids": [
      "boards",
      "imu",
      "kalman",
      "uart"
    ]
  },
  {
    "title": "协议与输入",
    "ids": [
      "remote",
      "referee",
      "vision",
      "interboard"
    ]
  },
  {
    "title": "决策与编排",
    "ids": [
      "command-sources",
      "command",
      "application",
      "telemetry"
    ]
  },
  {
    "title": "机构执行",
    "ids": [
      "gimbal",
      "inertial",
      "chassis",
      "big-yaw",
      "shooter"
    ]
  },
  {
    "title": "算法与电机",
    "ids": [
      "pid",
      "motor-control",
      "motor-dji",
      "motor-dm"
    ]
  }
];
window.SKYWALKER_MODULES = [
  {
    "id": "application",
    "title": "双主控应用与执行器",
    "category": "机器人与仲裁",
    "status": "partial",
    "summary": "两个 sentry 入口共用 vehicle_bench：云台惯性双轴/发射/回中，底盘四舵轮/大 Yaw，各机构统一物理 CAN 发布。",
    "responsibility": "公共运行时持有物理设备、长期对象、输入/通信/观测线程与执行周期；公开执行器保存目标和消费新鲜输入，Motor/CAN 独立恢复。",
    "statusNote": "软件框架已有，connections_confirmed、imu_mounting_confirmed 等中央确认默认关闭；功率、热量、拨盘原点与视觉反馈测量待接。",
    "depends": [
      "command",
      "interboard",
      "inertial",
      "gimbal",
      "chassis",
      "big-yaw",
      "shooter",
      "boards"
    ],
    "source": [
      {
        "label": "main.cpp",
        "path": "applications/sentry_gimbal/src/main.cpp"
      },
      {
        "label": "main.cpp",
        "path": "applications/sentry_chassis/src/main.cpp"
      },
      {
        "label": "vehicle_bench.hpp",
        "path": "samples/robotics/common/vehicle_bench.hpp"
      },
      {
        "label": "vehicle_mc02.overlay",
        "path": "samples/robotics/common/vehicle_mc02.overlay"
      },
      {
        "label": "calibration.hpp",
        "path": "include/robotics/vehicle/calibration.hpp"
      }
    ],
    "docs": [
      {
        "label": "dual-controller.md",
        "path": "docs/applications/dual-controller.md"
      },
      {
        "label": "executors.md",
        "path": "docs/modules/robotics/executors.md"
      },
      {
        "label": "README.md",
        "path": "samples/robotics/vehicle_integration/README.md"
      }
    ],
    "interfaces": [
      {
        "signature": "int samples::vehicle::run(IPowerMeasurementSource *power_source = nullptr, IShooterHeatSource *heat_source = nullptr, IDialHomeSource *home_source = nullptr)",
        "description": "共用整车入口按 VEHICLE_CHASSIS_ROLE 选择角色，持有硬件并进入唯一机构执行循环。",
        "parameters": [
          {
            "name": "power_source",
            "meaning": "真实功率源"
          },
          {
            "name": "heat_source",
            "meaning": "真实热量源"
          },
          {
            "name": "home_source",
            "meaning": "真实拨盘 home/index 原点源"
          }
        ],
        "returns": "初始化结果或运行循环；默认 Pending 来源无效，不伪造测量。",
        "errors": "非法输入/配置返回负 errno；普通设备等待不拒绝合法目标。",
        "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
      },
      {
        "signature": "RunStatus::stamp / SnapshotCache<T>::publish / snapshot",
        "description": "状态由执行 owner 生产，通信/观测只复制；Endpoint 使用状态年龄撤销陈旧 ready/armed。",
        "parameters": [],
        "returns": "生产时间和一致值副本；状态不是目标生产门禁。",
        "errors": "缺数据、争锁或过期分别处理，不能重盖时间。",
        "context": "单一执行线程拥有状态；对象与引用须覆盖 worker 生命周期。"
      }
    ],
    "examples": [
      {
        "title": "正式应用共用入口",
        "language": "cpp",
        "code": "#include \"../../../samples/robotics/common/vehicle_bench.hpp\"\nint main() {\n    return skywalker::samples::vehicle::run();\n}",
        "notes": "两个正式应用的当前 main.cpp；阶段配置和资源所有权在公共运行时。"
      }
    ],
    "lifecycle": [
      "中央 calibration 与 overlay 填实物参数；保持未完成确认为 false。",
      "静态构造 Motor/CanBus/输入来源/快照缓存。",
      "attach/start 全部物理总线，再 begin 各执行器和命令服务。",
      "通信线程 1 ms poll；云台约 5 ms，底盘 2 ms 绝对节拍执行。",
      "各机构先暂存，统一每物理 CAN commit 一次；错误诊断不撤销其他机构目标。",
      "按手动、视觉观察、视觉执行、功率和有载发射逐级接入真实来源。"
    ],
    "pitfalls": [
      "旧 applications/src/board_config 与私有 executor 已删除。",
      "电机反馈等待不撤销用户运行意图，输入本身过期仍撤销。",
      "sample 物理遥控与 applications 键鼠/console 操作 profile 有区别。",
      "底盘 CAN1/3/2 已全部用于电机，板间 CAN 后端需重新分配资源。"
    ],
    "config": [
      {
        "name": "calibration.hpp",
        "description": "connections_confirmed/imu_mounting_confirmed/power_model_calibrated/shooter_constraints_confirmed 默认 false。"
      },
      {
        "name": "VEHICLE_*",
        "description": "CHASSIS_ROLE、REFEREE、POWER_BUDGET、VISION_OBSERVE/EXECUTE、SHOOTING、LOCK_PITCH；默认手动台架，不等于比赛整车授权。"
      }
    ]
  }
];
