'use strict';
// Physical ownership: main@99a97c9, current and remaining integration targets.
window.SKYWALKER_VEHICLE_MAP = {
  "current": {
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
        "detail": "已装配 · 安装待确认",
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
        "detail": "可选观察 / 执行 · 上行待补",
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
        "detail": "Remote / 可选 Aim / Permission",
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
        "detail": "目标 / 原年龄 · 1ms poll",
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
        "detail": "真实 boot · 状态生产年龄",
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
        "detail": "热量 / 原点 Pending 无效",
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
        "detail": "头部稳定 · 角速度 / 原年龄",
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
        "detail": "八轴有效性 · 功率模式待测量",
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
        "detail": "CAN1 / CAN3 · 2ms 绝对节拍",
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
        "detail": "BigYawExecutor · DM CAN2",
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
        "label": "头部稳定",
        "kind": "feedback",
        "from": "inertial"
      },
      {
        "to": "glink",
        "label": "回中请求",
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
        "detail": "可扩展搜索 / 导航目标 · 待实现",
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
        "detail": "目标 / 原年龄 · 1ms poll",
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
        "detail": "真实 boot · 状态生产年龄",
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
        "detail": "头部稳定 · 角速度 / 原年龄",
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
        "detail": "CAN1 / CAN3 · 2ms 绝对节拍",
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
        "detail": "BigYawExecutor · DM CAN2",
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
        "label": "头部稳定",
        "kind": "feedback",
        "from": "inertial"
      },
      {
        "to": "glink",
        "label": "回中请求",
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
