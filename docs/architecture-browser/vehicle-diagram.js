'use strict';

// Physical ownership and parallel data paths. Coordinates use viewBox 0 0 1100 560.
// Explicit routes keep arrows outside node rectangles. Detached current sensing
// nodes are intentional: sentry_gimbal has not assembled IMU or vision receivers.
window.SKYWALKER_VEHICLE_MAP = {
  current: {
    nodes: [
      { id: 'current-remote', title: 'DR16 / 键鼠', detail: '已接命令来源 · 原始时间戳', moduleId: 'remote', x: 35, y: 24, w: 175, h: 62, status: 'ready', zone: 'input' },
      { id: 'current-referee', title: '裁判系统', detail: '权限 / 功率 · 配置待完成', moduleId: 'referee', x: 230, y: 24, w: 175, h: 62, status: 'partial', zone: 'input' },
      { id: 'current-imu', title: 'IMU（待装配）', detail: '模块已有 · 正式应用未接', moduleId: 'imu', x: 470, y: 24, w: 185, h: 62, status: 'partial', zone: 'input' },
      { id: 'current-host', title: '视觉主机（待装配）', detail: 'AB 模块已有 · 未参与仲裁', moduleId: 'vision', x: 765, y: 24, w: 250, h: 62, status: 'partial', zone: 'host' },
      { id: 'current-command', title: '命令来源 → CommandManager', detail: 'Remote + Permission · 后台仲裁', moduleId: 'command', x: 55, y: 175, w: 275, h: 76, status: 'ready', zone: 'gimbal' },
      { id: 'current-gimbal-link', title: '云台 InterBoardEndpoint', detail: 'UART 默认 · 可选 RS485 / CAN', moduleId: 'interboard', x: 380, y: 175, w: 245, h: 76, status: 'ready', zone: 'gimbal' },
      { id: 'current-chassis-link', title: '底盘 InterBoardEndpoint', detail: '接收命令 / 约束 · 回传状态', moduleId: 'interboard', x: 770, y: 175, w: 245, h: 76, status: 'ready', zone: 'chassis' },
      { id: 'current-gimbal-executor', title: 'GimbalExecutor → 单 Yaw', detail: '本地恢复 / GimbalAxis / 控制环', moduleId: 'gimbal', x: 55, y: 320, w: 275, h: 76, status: 'partial', zone: 'gimbal' },
      { id: 'current-yaw-motor', title: 'GM6020 Yaw / CAN1', detail: '模式 / 零点 / 连接门禁待配置', moduleId: 'motor-dji', x: 55, y: 485, w: 275, h: 60, status: 'partial', zone: 'gimbal' },
      { id: 'current-chassis-executor', title: 'ChassisExecutor', detail: 'boot / generation / 年龄 / 功率门控', moduleId: 'application', x: 770, y: 290, w: 245, h: 70, status: 'partial', zone: 'chassis' },
      { id: 'current-swerve', title: 'Swerve + 功率限制', detail: '四轮目标分配 · 模型尚未标定', moduleId: 'chassis', x: 770, y: 405, w: 245, h: 70, status: 'partial', zone: 'chassis' },
      { id: 'current-chassis-motors', title: '4 舵向 + 4 驱动电机', detail: 'Group / 1～2 条 CanBus', moduleId: 'motor-dji', x: 770, y: 495, w: 245, h: 50, status: 'partial', zone: 'chassis' }
    ],
    edges: [
      { from: 'current-remote', to: 'current-command', label: '操作者输入', kind: 'command', points: [{ x: 122, y: 86 }, { x: 122, y: 175 }] },
      { from: 'current-referee', to: 'current-command', label: '输出许可', kind: 'permission', points: [{ x: 317, y: 86 }, { x: 317, y: 128 }, { x: 272, y: 128 }, { x: 272, y: 175 }] },
      { from: 'current-command', to: 'current-gimbal-executor', label: '本地云台命令', kind: 'command', points: [{ x: 192, y: 251 }, { x: 192, y: 320 }] },
      { from: 'current-gimbal-executor', to: 'current-yaw-motor', label: '控制输出 / commit', kind: 'command', points: [{ x: 192, y: 396 }, { x: 192, y: 485 }] },
      { from: 'current-yaw-motor', to: 'current-gimbal-executor', label: '角度 / 速度反馈', kind: 'feedback', points: [{ x: 55, y: 515 }, { x: 26, y: 515 }, { x: 26, y: 357 }, { x: 55, y: 357 }] },
      { from: 'current-command', to: 'current-gimbal-link', label: '底盘命令', kind: 'command', points: [{ x: 330, y: 198 }, { x: 380, y: 198 }] },
      { from: 'current-command', to: 'current-gimbal-link', label: '裁判约束', kind: 'permission', points: [{ x: 330, y: 234 }, { x: 380, y: 234 }] },
      { from: 'current-gimbal-link', to: 'current-chassis-link', label: '命令 + 权限 / 功率', kind: 'command', points: [{ x: 625, y: 195 }, { x: 770, y: 195 }] },
      { from: 'current-chassis-link', to: 'current-gimbal-link', label: '心跳 / ready / 代次', kind: 'feedback', points: [{ x: 770, y: 235 }, { x: 625, y: 235 }] },
      { from: 'current-chassis-link', to: 'current-chassis-executor', label: '端点快照', kind: 'command', points: [{ x: 892, y: 251 }, { x: 892, y: 290 }] },
      { from: 'current-chassis-executor', to: 'current-chassis-link', label: '执行状态摘要', kind: 'feedback', points: [{ x: 770, y: 321 }, { x: 746, y: 321 }, { x: 746, y: 238 }, { x: 770, y: 238 }] },
      { from: 'current-chassis-executor', to: 'current-swerve', label: '机体速度目标', kind: 'command', points: [{ x: 892, y: 360 }, { x: 892, y: 405 }] },
      { from: 'current-swerve', to: 'current-chassis-motors', label: '电流目标', kind: 'command', points: [{ x: 892, y: 475 }, { x: 892, y: 495 }] },
      { from: 'current-chassis-motors', to: 'current-chassis-executor', label: '8 电机真实反馈', kind: 'feedback', points: [{ x: 1015, y: 522 }, { x: 1056, y: 522 }, { x: 1056, y: 325 }, { x: 1015, y: 325 }] }
    ]
  },
  target: {
    nodes: [
      { id: 'target-remote', title: 'DR16 / 键鼠', detail: 'Manual / Auto / Safe / 接管', moduleId: 'remote', x: 35, y: 24, w: 165, h: 62, status: 'ready', zone: 'input' },
      { id: 'target-referee', title: '裁判系统', detail: '独立权限、功率与热量来源', moduleId: 'referee', x: 225, y: 24, w: 180, h: 62, status: 'partial', zone: 'input' },
      { id: 'target-imu', title: 'IMU / 姿态', detail: '采集 / EKF / 安装参考 / 时效', moduleId: 'imu', x: 455, y: 24, w: 180, h: 62, status: 'partial', zone: 'input' },
      { id: 'target-host', title: '视觉计算机 ↔ AB 收发', detail: '目标下行 · 姿态 / 弹速 / 计数上行', moduleId: 'vision', x: 765, y: 24, w: 250, h: 62, status: 'partial', zone: 'host' },
      { id: 'target-command', title: '来源 + 仲裁 + 行为目标', detail: 'CommandManager · 跟踪 / 搜索装配', moduleId: 'command', x: 55, y: 175, w: 275, h: 76, status: 'partial', zone: 'gimbal' },
      { id: 'target-gimbal-link', title: '云台 InterBoardEndpoint', detail: '命令 / 权限 / 功率与双向上下文', moduleId: 'interboard', x: 380, y: 175, w: 245, h: 76, status: 'ready', zone: 'gimbal' },
      { id: 'target-chassis-link', title: '底盘 InterBoardEndpoint', detail: '心跳 / 状态年龄 / 可信机体反馈', moduleId: 'interboard', x: 770, y: 175, w: 245, h: 76, status: 'partial', zone: 'chassis' },
      { id: 'target-gimbal-executor', title: '姿态适配 + 双轴执行', detail: 'Yaw / Pitch · 按机构加大小 Yaw', moduleId: 'gimbal', x: 55, y: 330, w: 275, h: 76, status: 'planned', zone: 'gimbal' },
      { id: 'target-gimbal-motors', title: '云台 DJI / DM 电机', detail: 'GimbalAxis / 控制环 / Group / CAN', moduleId: 'motor-dji', x: 55, y: 485, w: 275, h: 60, status: 'partial', zone: 'gimbal' },
      { id: 'target-shooter', title: '发射执行器（按任务装配）', detail: '摩擦轮 / 拨弹 / 热量 / 卡弹恢复', moduleId: 'application', x: 380, y: 330, w: 245, h: 76, status: 'planned', zone: 'extras' },
      { id: 'target-chassis-control', title: '底盘执行 / Swerve / 功率', detail: '本地恢复 + 标定 + 可信速度估计', moduleId: 'chassis', x: 770, y: 330, w: 245, h: 76, status: 'partial', zone: 'chassis' },
      { id: 'target-chassis-motors', title: '4 舵向 + 4 驱动电机', detail: 'Group / 独占 CanBus / 测量闭环', moduleId: 'motor-dji', x: 770, y: 485, w: 245, h: 60, status: 'partial', zone: 'chassis' }
    ],
    edges: [
      { from: 'target-remote', to: 'target-command', label: '操作者输入', kind: 'command', points: [{ x: 117, y: 86 }, { x: 117, y: 175 }] },
      { from: 'target-referee', to: 'target-command', label: '输出许可', kind: 'permission', points: [{ x: 315, y: 86 }, { x: 315, y: 142 }, { x: 260, y: 142 }, { x: 260, y: 175 }] },
      { from: 'target-host', to: 'target-command', label: '视觉目标 → Aim 来源', kind: 'planned', points: [{ x: 890, y: 86 }, { x: 890, y: 111 }, { x: 352, y: 111 }, { x: 352, y: 187 }, { x: 330, y: 187 }] },
      { from: 'target-imu', to: 'target-host', label: '真实姿态 / 角速度反馈', kind: 'planned', points: [{ x: 635, y: 55 }, { x: 765, y: 55 }] },
      { from: 'target-imu', to: 'target-gimbal-executor', label: '姿态 / 参考适配', kind: 'planned', points: [{ x: 545, y: 86 }, { x: 545, y: 138 }, { x: 363, y: 138 }, { x: 363, y: 361 }, { x: 330, y: 361 }] },
      { from: 'target-command', to: 'target-gimbal-executor', label: '授权后的云台目标', kind: 'planned', points: [{ x: 170, y: 251 }, { x: 170, y: 330 }] },
      { from: 'target-gimbal-executor', to: 'target-gimbal-motors', label: '机械轴目标 / 控制输出', kind: 'planned', points: [{ x: 192, y: 406 }, { x: 192, y: 485 }] },
      { from: 'target-gimbal-motors', to: 'target-gimbal-executor', label: '编码器 / 速度反馈', kind: 'feedback', points: [{ x: 55, y: 515 }, { x: 26, y: 515 }, { x: 26, y: 368 }, { x: 55, y: 368 }] },
      { from: 'target-command', to: 'target-shooter', label: '射击请求 + 许可', kind: 'planned', points: [{ x: 287, y: 251 }, { x: 287, y: 287 }, { x: 502, y: 287 }, { x: 502, y: 330 }] },
      { from: 'target-shooter', to: 'target-host', label: '真实弹速 / 计数', kind: 'planned', points: [{ x: 625, y: 367 }, { x: 660, y: 367 }, { x: 660, y: 96 }, { x: 824, y: 96 }, { x: 824, y: 86 }] },
      { from: 'target-command', to: 'target-gimbal-link', label: '底盘命令', kind: 'command', points: [{ x: 330, y: 205 }, { x: 380, y: 205 }] },
      { from: 'target-command', to: 'target-gimbal-link', label: '裁判约束', kind: 'permission', points: [{ x: 330, y: 235 }, { x: 380, y: 235 }] },
      { from: 'target-gimbal-link', to: 'target-chassis-link', label: '命令 + 权限 / 功率', kind: 'command', points: [{ x: 625, y: 195 }, { x: 770, y: 195 }] },
      { from: 'target-chassis-link', to: 'target-gimbal-link', label: '状态 / 代次 / 机体反馈', kind: 'feedback', points: [{ x: 770, y: 235 }, { x: 625, y: 235 }] },
      { from: 'target-chassis-link', to: 'target-chassis-control', label: '有时效的命令 / 约束', kind: 'command', points: [{ x: 892, y: 251 }, { x: 892, y: 330 }] },
      { from: 'target-chassis-control', to: 'target-chassis-link', label: '状态年龄 / 实测反馈', kind: 'planned', points: [{ x: 770, y: 367 }, { x: 746, y: 367 }, { x: 746, y: 238 }, { x: 770, y: 238 }] },
      { from: 'target-chassis-control', to: 'target-chassis-motors', label: '限功率的轮端输出', kind: 'command', points: [{ x: 892, y: 406 }, { x: 892, y: 485 }] },
      { from: 'target-chassis-motors', to: 'target-chassis-control', label: '8 电机真实反馈', kind: 'feedback', points: [{ x: 1015, y: 515 }, { x: 1056, y: 515 }, { x: 1056, y: 368 }, { x: 1015, y: 368 }] }
    ]
  }
};
