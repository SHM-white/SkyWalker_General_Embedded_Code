'use strict';
// Current assembly reviewed at dd3f673 (2026-11-15); target is future design.
window.SKYWALKER_VEHICLE_MAP = {
  "current": {
    "height": 770,
    "nodes": [
      {
        "id": "remote",
        "title": "DR16 / 物理遥控",
        "detail": "UART5 · 真实归中解锁",
        "moduleId": "remote",
        "x": 20,
        "y": 4,
        "w": 175,
        "h": 90,
        "status": "ready",
        "zone": "input"
      },
      {
        "id": "referee",
        "title": "裁判原始帧 / 许可",
        "detail": "底盘 USART10 → v4 → 云台",
        "moduleId": "referee",
        "x": 210,
        "y": 4,
        "w": 190,
        "h": 90,
        "status": "ready",
        "zone": "input"
      },
      {
        "id": "imu",
        "title": "CAN3 头部 IMU + BMI088",
        "detail": "头部姿态 / 载体航向",
        "moduleId": "imu",
        "x": 420,
        "y": 4,
        "w": 200,
        "h": 90,
        "status": "partial",
        "zone": "input"
      },
      {
        "id": "host",
        "title": "视觉 AB（USB CDC）",
        "detail": "allow_auto=true · 10 ms 姿态回传",
        "moduleId": "vision",
        "x": 640,
        "y": 4,
        "w": 200,
        "h": 90,
        "status": "partial",
        "zone": "host"
      },
      {
        "id": "navigation",
        "title": "导航 / 巡航来源",
        "detail": "USART1 · 导航档与掉线保持",
        "moduleId": "command-sources",
        "x": 860,
        "y": 4,
        "w": 215,
        "h": 90,
        "status": "ready",
        "zone": "host"
      },
      {
        "id": "command",
        "title": "CommandManager",
        "detail": "Remote/Aim/Navigation + 裁判",
        "moduleId": "command",
        "x": 55,
        "y": 175,
        "w": 275,
        "h": 88,
        "status": "ready",
        "zone": "gimbal"
      },
      {
        "id": "glink",
        "title": "云台 Endpoint v4",
        "detail": "USART10 · main 5ms poll",
        "moduleId": "interboard",
        "x": 380,
        "y": 175,
        "w": 245,
        "h": 88,
        "status": "ready",
        "zone": "gimbal"
      },
      {
        "id": "clink",
        "title": "底盘 Endpoint v4",
        "detail": "USART1 · boot / 原年龄",
        "moduleId": "interboard",
        "x": 770,
        "y": 175,
        "w": 245,
        "h": 88,
        "status": "ready",
        "zone": "chassis"
      },
      {
        "id": "inertial",
        "title": "双 Yaw 分配 + 头部惯性",
        "detail": "分配器关节轨迹 · 视觉姿态外环",
        "moduleId": "inertial",
        "x": 55,
        "y": 320,
        "w": 275,
        "h": 88,
        "status": "partial",
        "zone": "gimbal"
      },
      {
        "id": "gimbal",
        "title": "小 Yaw / Pitch 执行",
        "detail": "两轴计算有效性 · 持续目标",
        "moduleId": "gimbal",
        "x": 55,
        "y": 465,
        "w": 275,
        "h": 88,
        "status": "partial",
        "zone": "gimbal"
      },
      {
        "id": "shooter",
        "title": "摩擦轮 / 拨盘",
        "detail": "已装配 · 未要求热量/许可",
        "moduleId": "shooter",
        "x": 380,
        "y": 465,
        "w": 245,
        "h": 88,
        "status": "partial",
        "zone": "gimbal"
      },
      {
        "id": "gmotor",
        "title": "云台电机 / 共享提交",
        "detail": "CAN1 小 Yaw/发射 · CAN2 Pitch",
        "moduleId": "motor-dji",
        "x": 55,
        "y": 650,
        "w": 275,
        "h": 88,
        "status": "partial",
        "zone": "gimbal"
      },
      {
        "id": "centering",
        "title": "双 Yaw 分配器",
        "detail": "关节轨迹 + 大 Yaw 世界意图",
        "moduleId": "big-yaw",
        "x": 380,
        "y": 320,
        "w": 245,
        "h": 88,
        "status": "partial",
        "zone": "gimbal"
      },
      {
        "id": "chassis",
        "title": "ChassisExecutor",
        "detail": "云台系平移转换 · 无功率闭环",
        "moduleId": "chassis",
        "x": 770,
        "y": 320,
        "w": 245,
        "h": 88,
        "status": "partial",
        "zone": "chassis"
      },
      {
        "id": "wheel",
        "title": "四舵向 + 四轮驱",
        "detail": "CAN1 / CAN2 · 2ms 绝对节拍",
        "moduleId": "motor-dji",
        "x": 770,
        "y": 465,
        "w": 245,
        "h": 88,
        "status": "partial",
        "zone": "chassis"
      },
      {
        "id": "bigyaw",
        "title": "大 Yaw 独立速度环",
        "detail": "BigYawExecutor · DM CAN3",
        "moduleId": "big-yaw",
        "x": 770,
        "y": 650,
        "w": 245,
        "h": 88,
        "status": "partial",
        "zone": "chassis"
      }
    ],
    "edges": [
      {
        "to": "command",
        "label": "原输入",
        "kind": "command",
        "from": "remote"
      },
      {
        "to": "command",
        "label": "裁判许可",
        "kind": "permission",
        "from": "referee",
        "points": [
          {
            "x": 305,
            "y": 94
          },
          {
            "x": 305,
            "y": 132
          },
          {
            "x": 280,
            "y": 132
          },
          {
            "x": 280,
            "y": 175
          }
        ],
        "labelPosition": {
          "x": 300,
          "y": 123
        }
      },
      {
        "to": "command",
        "label": "视觉 Aim",
        "kind": "command",
        "from": "host",
        "points": [
          {
            "x": 740,
            "y": 94
          },
          {
            "x": 740,
            "y": 112
          },
          {
            "x": 350,
            "y": 112
          },
          {
            "x": 350,
            "y": 197
          },
          {
            "x": 330,
            "y": 197
          }
        ],
        "labelPosition": {
          "x": 680,
          "y": 103
        }
      },
      {
        "to": "inertial",
        "label": "头部姿态 / 载体角速度",
        "kind": "feedback",
        "from": "imu",
        "points": [
          {
            "x": 520,
            "y": 94
          },
          {
            "x": 520,
            "y": 140
          },
          {
            "x": 350,
            "y": 140
          },
          {
            "x": 350,
            "y": 345
          },
          {
            "x": 330,
            "y": 345
          }
        ],
        "labelPosition": {
          "x": 420,
          "y": 131
        }
      },
      {
        "to": "inertial",
        "label": "云台命令",
        "kind": "command",
        "from": "command"
      },
      {
        "to": "gimbal",
        "label": "机械 Rate",
        "kind": "command",
        "from": "inertial"
      },
      {
        "to": "gmotor",
        "label": "stage / 一次 commit",
        "kind": "command",
        "from": "gimbal"
      },
      {
        "to": "gmotor",
        "label": "共享 CAN1 stage",
        "kind": "command",
        "from": "shooter",
        "points": [
          {
            "x": 502,
            "y": 553
          },
          {
            "x": 502,
            "y": 600
          },
          {
            "x": 290,
            "y": 600
          },
          {
            "x": 290,
            "y": 650
          }
        ],
        "labelPosition": {
          "x": 440,
          "y": 591
        }
      },
      {
        "to": "shooter",
        "label": "发射请求",
        "kind": "command",
        "from": "command",
        "points": [
          {
            "x": 300,
            "y": 263
          },
          {
            "x": 300,
            "y": 290
          },
          {
            "x": 665,
            "y": 290
          },
          {
            "x": 665,
            "y": 510
          },
          {
            "x": 625,
            "y": 510
          }
        ],
        "labelPosition": {
          "x": 650,
          "y": 406
        }
      },
      {
        "to": "glink",
        "label": "轮控 / 操作",
        "kind": "command",
        "from": "command",
        "points": [
          {
            "x": 330,
            "y": 218
          },
          {
            "x": 380,
            "y": 218
          }
        ]
      },
      {
        "to": "clink",
        "label": "v4 / 原年龄",
        "kind": "command",
        "from": "glink",
        "points": [
          {
            "x": 625,
            "y": 207
          },
          {
            "x": 770,
            "y": 207
          }
        ]
      },
      {
        "to": "glink",
        "label": "心跳 / 状态",
        "kind": "feedback",
        "from": "clink",
        "points": [
          {
            "x": 770,
            "y": 244
          },
          {
            "x": 625,
            "y": 244
          }
        ]
      },
      {
        "to": "chassis",
        "label": "输入快照",
        "kind": "command",
        "from": "clink"
      },
      {
        "to": "wheel",
        "label": "四轮目标 / stage",
        "kind": "command",
        "from": "chassis"
      },
      {
        "to": "centering",
        "label": "头部世界意图",
        "kind": "command",
        "from": "command",
        "points": [
          {
            "x": 300,
            "y": 263
          },
          {
            "x": 300,
            "y": 288
          },
          {
            "x": 450,
            "y": 288
          },
          {
            "x": 450,
            "y": 320
          }
        ],
        "labelPosition": {
          "x": 390,
          "y": 280
        }
      },
      {
        "to": "glink",
        "label": "0x0402 世界速度",
        "kind": "command",
        "from": "centering",
        "points": [
          {
            "x": 502,
            "y": 320
          },
          {
            "x": 502,
            "y": 263
          }
        ]
      },
      {
        "to": "bigyaw",
        "label": "大 Yaw 请求",
        "kind": "command",
        "from": "clink",
        "points": [
          {
            "x": 1015,
            "y": 235
          },
          {
            "x": 1058,
            "y": 235
          },
          {
            "x": 1058,
            "y": 694
          },
          {
            "x": 1015,
            "y": 694
          }
        ],
        "labelPosition": {
          "x": 1045,
          "y": 575
        }
      },
      {
        "to": "chassis",
        "label": "八轴反馈",
        "kind": "feedback",
        "from": "wheel",
        "points": [
          {
            "x": 1015,
            "y": 507
          },
          {
            "x": 1038,
            "y": 507
          },
          {
            "x": 1038,
            "y": 365
          },
          {
            "x": 1015,
            "y": 365
          }
        ]
      },
      {
        "to": "gimbal",
        "label": "机械反馈",
        "kind": "feedback",
        "from": "gmotor",
        "points": [
          {
            "x": 55,
            "y": 694
          },
          {
            "x": 25,
            "y": 694
          },
          {
            "x": 25,
            "y": 510
          },
          {
            "x": 55,
            "y": 510
          }
        ]
      },
      {
        "to": "clink",
        "label": "原生角 / 速度 / 状态年龄",
        "kind": "feedback",
        "from": "bigyaw",
        "points": [
          {
            "x": 1015,
            "y": 715
          },
          {
            "x": 1080,
            "y": 715
          },
          {
            "x": 1080,
            "y": 249
          },
          {
            "x": 1015,
            "y": 249
          }
        ],
        "labelPosition": {
          "x": 1070,
          "y": 410
        }
      },
      {
        "from": "navigation",
        "to": "command",
        "label": "Navigation / 巡航",
        "kind": "command",
        "points": [
          {
            "x": 967,
            "y": 94
          },
          {
            "x": 967,
            "y": 158
          },
          {
            "x": 365,
            "y": 158
          },
          {
            "x": 365,
            "y": 242
          },
          {
            "x": 330,
            "y": 242
          }
        ],
        "labelPosition": {
          "x": 705,
          "y": 150
        }
      },
      {
        "from": "centering",
        "to": "inertial",
        "label": "可行参考 / 关节轨迹",
        "kind": "command",
        "points": [
          {
            "x": 380,
            "y": 380
          },
          {
            "x": 330,
            "y": 380
          }
        ],
        "labelPosition": {
          "x": 355,
          "y": 420
        }
      }
    ]
  },
  "target": {
    "height": 770,
    "nodes": [
      {
        "id": "remote",
        "title": "DR16 / 键鼠",
        "detail": "Remote · 原始输入",
        "moduleId": "remote",
        "x": 35,
        "y": 24,
        "w": 175,
        "h": 70,
        "status": "ready",
        "zone": "input"
      },
      {
        "id": "referee",
        "title": "裁判 / 真实测量",
        "detail": "可选权限 · 功率 / 热量",
        "moduleId": "referee",
        "x": 230,
        "y": 24,
        "w": 175,
        "h": 70,
        "status": "partial",
        "zone": "input"
      },
      {
        "id": "imu",
        "title": "头部 IMU",
        "detail": "确认安装 / 质量 / 参考会话",
        "moduleId": "imu",
        "x": 470,
        "y": 24,
        "w": 185,
        "h": 70,
        "status": "partial",
        "zone": "input"
      },
      {
        "id": "host",
        "title": "视觉 AB 主机",
        "detail": "真实姿态 / 弹速 / 弹数闭环",
        "moduleId": "vision",
        "x": 765,
        "y": 24,
        "w": 250,
        "h": 70,
        "status": "partial",
        "zone": "host"
      },
      {
        "id": "command",
        "title": "CommandManager",
        "detail": "遥控 / 视觉 / 导航 / 巡航仲裁",
        "moduleId": "command",
        "x": 55,
        "y": 175,
        "w": 275,
        "h": 88,
        "status": "partial",
        "zone": "gimbal"
      },
      {
        "id": "glink",
        "title": "云台 Endpoint v4",
        "detail": "USART10 · main 5ms poll",
        "moduleId": "interboard",
        "x": 380,
        "y": 175,
        "w": 245,
        "h": 88,
        "status": "ready",
        "zone": "gimbal"
      },
      {
        "id": "clink",
        "title": "底盘 Endpoint v4",
        "detail": "USART1 · boot / 原年龄",
        "moduleId": "interboard",
        "x": 770,
        "y": 175,
        "w": 245,
        "h": 88,
        "status": "ready",
        "zone": "chassis"
      },
      {
        "id": "inertial",
        "title": "头部惯性适配",
        "detail": "测量 / 参考 → 双轴 Rate",
        "moduleId": "inertial",
        "x": 55,
        "y": 320,
        "w": 275,
        "h": 88,
        "status": "partial",
        "zone": "gimbal"
      },
      {
        "id": "gimbal",
        "title": "小 Yaw / Pitch 执行",
        "detail": "两轴计算有效性 · 持续目标",
        "moduleId": "gimbal",
        "x": 55,
        "y": 465,
        "w": 275,
        "h": 88,
        "status": "partial",
        "zone": "gimbal"
      },
      {
        "id": "shooter",
        "title": "摩擦轮 / 拨盘执行",
        "detail": "接真实热量 / 原点 · 受约束供弹",
        "moduleId": "shooter",
        "x": 380,
        "y": 465,
        "w": 245,
        "h": 88,
        "status": "partial",
        "zone": "gimbal"
      },
      {
        "id": "gmotor",
        "title": "小云台与发射 / CAN",
        "detail": "DJI CAN1 · Pitch DM CAN2",
        "moduleId": "motor-dji",
        "x": 55,
        "y": 650,
        "w": 275,
        "h": 88,
        "status": "partial",
        "zone": "gimbal"
      },
      {
        "id": "centering",
        "title": "小 Yaw 中心外环",
        "detail": "双轴输出有效 · 关节 rad/s",
        "moduleId": "big-yaw",
        "x": 380,
        "y": 320,
        "w": 245,
        "h": 88,
        "status": "partial",
        "zone": "gimbal"
      },
      {
        "id": "chassis",
        "title": "ChassisExecutor",
        "detail": "标定功率 · 补车体速度 / 里程计",
        "moduleId": "chassis",
        "x": 770,
        "y": 320,
        "w": 245,
        "h": 88,
        "status": "partial",
        "zone": "chassis"
      },
      {
        "id": "wheel",
        "title": "四舵向 + 四轮驱",
        "detail": "CAN1 / CAN2 · 2ms 绝对节拍",
        "moduleId": "motor-dji",
        "x": 770,
        "y": 465,
        "w": 245,
        "h": 88,
        "status": "partial",
        "zone": "chassis"
      },
      {
        "id": "bigyaw",
        "title": "大 Yaw 独立速度环",
        "detail": "BigYawExecutor · DM CAN3",
        "moduleId": "big-yaw",
        "x": 770,
        "y": 650,
        "w": 245,
        "h": 88,
        "status": "partial",
        "zone": "chassis"
      }
    ],
    "edges": [
      {
        "to": "command",
        "label": "原输入",
        "kind": "command",
        "from": "remote"
      },
      {
        "to": "command",
        "label": "可选许可",
        "kind": "permission",
        "from": "referee",
        "points": [
          {
            "x": 317,
            "y": 94
          },
          {
            "x": 317,
            "y": 132
          },
          {
            "x": 280,
            "y": 132
          },
          {
            "x": 280,
            "y": 175
          }
        ],
        "labelPosition": {
          "x": 300,
          "y": 123
        }
      },
      {
        "to": "command",
        "label": "可选 Aim",
        "kind": "command",
        "from": "host",
        "points": [
          {
            "x": 890,
            "y": 94
          },
          {
            "x": 890,
            "y": 112
          },
          {
            "x": 350,
            "y": 112
          },
          {
            "x": 350,
            "y": 197
          },
          {
            "x": 330,
            "y": 197
          }
        ],
        "labelPosition": {
          "x": 700,
          "y": 103
        }
      },
      {
        "to": "inertial",
        "label": "姿态 / 参考",
        "kind": "feedback",
        "from": "imu",
        "points": [
          {
            "x": 562,
            "y": 94
          },
          {
            "x": 562,
            "y": 140
          },
          {
            "x": 350,
            "y": 140
          },
          {
            "x": 350,
            "y": 345
          },
          {
            "x": 330,
            "y": 345
          }
        ],
        "labelPosition": {
          "x": 420,
          "y": 131
        }
      },
      {
        "to": "inertial",
        "label": "云台命令",
        "kind": "command",
        "from": "command"
      },
      {
        "to": "gimbal",
        "label": "机械 Rate",
        "kind": "command",
        "from": "inertial"
      },
      {
        "to": "gmotor",
        "label": "stage / 一次 commit",
        "kind": "command",
        "from": "gimbal"
      },
      {
        "to": "gmotor",
        "label": "共享 DJI stage",
        "kind": "command",
        "from": "shooter",
        "points": [
          {
            "x": 502,
            "y": 553
          },
          {
            "x": 502,
            "y": 600
          },
          {
            "x": 290,
            "y": 600
          },
          {
            "x": 290,
            "y": 650
          }
        ],
        "labelPosition": {
          "x": 440,
          "y": 591
        }
      },
      {
        "to": "shooter",
        "label": "发射请求",
        "kind": "command",
        "from": "command",
        "points": [
          {
            "x": 300,
            "y": 263
          },
          {
            "x": 300,
            "y": 290
          },
          {
            "x": 665,
            "y": 290
          },
          {
            "x": 665,
            "y": 510
          },
          {
            "x": 625,
            "y": 510
          }
        ],
        "labelPosition": {
          "x": 650,
          "y": 406
        }
      },
      {
        "to": "glink",
        "label": "轮控 / 操作",
        "kind": "command",
        "from": "command",
        "points": [
          {
            "x": 330,
            "y": 218
          },
          {
            "x": 380,
            "y": 218
          }
        ]
      },
      {
        "to": "clink",
        "label": "v4 / 原年龄",
        "kind": "command",
        "from": "glink",
        "points": [
          {
            "x": 625,
            "y": 207
          },
          {
            "x": 770,
            "y": 207
          }
        ]
      },
      {
        "to": "glink",
        "label": "心跳 / 状态",
        "kind": "feedback",
        "from": "clink",
        "points": [
          {
            "x": 770,
            "y": 244
          },
          {
            "x": 625,
            "y": 244
          }
        ]
      },
      {
        "to": "chassis",
        "label": "输入快照",
        "kind": "command",
        "from": "clink"
      },
      {
        "to": "wheel",
        "label": "四轮目标 / stage",
        "kind": "command",
        "from": "chassis"
      },
      {
        "to": "centering",
        "label": "输出有效 / Active",
        "kind": "feedback",
        "from": "inertial"
      },
      {
        "to": "glink",
        "label": "0x0402 世界速度",
        "kind": "command",
        "from": "centering",
        "points": [
          {
            "x": 502,
            "y": 320
          },
          {
            "x": 502,
            "y": 263
          }
        ]
      },
      {
        "to": "bigyaw",
        "label": "大 Yaw 请求",
        "kind": "command",
        "from": "clink",
        "points": [
          {
            "x": 1015,
            "y": 235
          },
          {
            "x": 1058,
            "y": 235
          },
          {
            "x": 1058,
            "y": 694
          },
          {
            "x": 1015,
            "y": 694
          }
        ],
        "labelPosition": {
          "x": 1045,
          "y": 575
        }
      },
      {
        "to": "chassis",
        "label": "八轴反馈",
        "kind": "feedback",
        "from": "wheel",
        "points": [
          {
            "x": 1015,
            "y": 507
          },
          {
            "x": 1038,
            "y": 507
          },
          {
            "x": 1038,
            "y": 365
          },
          {
            "x": 1015,
            "y": 365
          }
        ]
      },
      {
        "to": "gimbal",
        "label": "机械反馈",
        "kind": "feedback",
        "from": "gmotor",
        "points": [
          {
            "x": 55,
            "y": 694
          },
          {
            "x": 25,
            "y": 694
          },
          {
            "x": 25,
            "y": 510
          },
          {
            "x": 55,
            "y": 510
          }
        ]
      },
      {
        "to": "clink",
        "label": "真实速度 / 状态年龄",
        "kind": "feedback",
        "from": "bigyaw",
        "points": [
          {
            "x": 1015,
            "y": 715
          },
          {
            "x": 1080,
            "y": 715
          },
          {
            "x": 1080,
            "y": 249
          },
          {
            "x": 1015,
            "y": 249
          }
        ],
        "labelPosition": {
          "x": 1070,
          "y": 410
        }
      },
      {
        "to": "host",
        "label": "实测弹速 / 弹数",
        "kind": "planned",
        "from": "gmotor",
        "points": [
          {
            "x": 330,
            "y": 694
          },
          {
            "x": 690,
            "y": 694
          },
          {
            "x": 690,
            "y": 125
          },
          {
            "x": 920,
            "y": 125
          },
          {
            "x": 920,
            "y": 94
          }
        ],
        "labelPosition": {
          "x": 690,
          "y": 618
        }
      }
    ]
  }
};
