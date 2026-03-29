# robot_skills — XLeRobot 统一机器人技能库

> 双臂 + 底盘 + 云台 + 双目 + 笛卡尔IK 控制, 适配 Agent / Skills / MCP 调用

## 硬件拓扑

```
ttyACM0 (9 舵机)                   ttyACM1 (8 舵机)
┌─────────────────────────┐        ┌─────────────────────────┐
│ 左臂从臂 (位置模式)     │        │ 右臂从臂 (位置模式)     │
│  ID1 shoulder_pan 左右旋 │        │  ID1 shoulder_pan 左右旋 │
│  ID2 shoulder_lift 俯仰 │        │  ID2 shoulder_lift 俯仰 │
│  ID3 elbow_flex    俯仰 │        │  ID3 elbow_flex    俯仰 │
│  ID4 wrist_flex    俯仰 │        │  ID4 wrist_flex    俯仰 │
│  ID5 wrist_roll    旋转 │        │  ID5 wrist_roll    旋转 │
│  ID6 gripper       夹爪 │        │  ID6 gripper       夹爪 │
├─────────────────────────┤        ├─────────────────────────┤
│ 底盘三轮 (速度模式)     │        │ 头部云台 (位置模式)     │
│  ID7 base_left_wheel    │        │  ID7 head_pan  水平旋转 │
│  ID8 base_back_wheel    │        │  ID8 head_tilt 垂直俯仰 │
│  ID9 base_right_wheel   │        └─────────────────────────┘
└─────────────────────────┘
ttyACM2 (6) 左臂主臂(遥操)        ttyACM3 (6) 右臂主臂(遥操)

Camera A: /dev/video0 (头部)       Camera B: /dev/video2 (右腕)
```

**共 29 个 STS3215 舵机, 全部 1Mbps 串口通信**

## 文件结构

```
robot_skills/
├── __init__.py      # 统一导出
├── __main__.py      # python -m robot_skills → 自检菜单
├── config.py        # 舵机ID / 端口 / 运动学常量
├── bus_utils.py     # FeetechMotorsBus 连接工具 (带重试)
├── arm.py           # ArmController + ArmState (速度控制)
├── ik.py            # SO101Kinematics2D + ArmIKController (笛卡尔控制)
├── head.py          # HeadController (双轴云台)
├── base.py          # BaseController + BaseVelocity (三轮全向)
├── camera.py        # CameraManager (双目)
├── robot.py         # RobotSkills (统一入口)
└── README.md        # 本文件
```

## 核心概念

### 速度控制 (3 层防护)

所有运动接口默认安全速度, Agent 可放心调用:

| 层级 | 方式 | 参数 | 说明 |
|------|------|------|------|
| 硬件 | `Acceleration` 寄存器 | `acceleration=20` | 舵机加减速梯度, 1=最慢 254=最快 |
| 硬件 | `Goal_Time` 寄存器 | `duration_ms=1000` | 舵机到达目标的时间 (ms) |
| 软件 | 插值轨迹 | `duration=2.0, hz=20` | 关节/笛卡尔空间分步插值 |

### 坐标系 (IK 模块)

```
      z (上)
      │
      │   x (前)
      │  /
      │ /
      └───── y (左)
     臂基座
```

- **2D 平面**: `plane_r`(前伸) + `plane_z`(高度), 由 `shoulder_lift` + `elbow_flex` 控制
- **3D 笛卡尔**: `x`(前) + `y`(左) + `z`(上), 由 `shoulder_pan` 扩展水平旋转
- **wrist_flex 耦合**: 自动补偿 `= -lift - elbow + pitch`, 保持末端水平

### 归一化值 ↔ 角度

```
STS3215: 4096 步 = 360°
归一化 [-100, 100] ↔ [-180°, 180°]
1 归一化单位 = 1.8°
```

## API 速查

### RobotSkills (统一入口)

```python
from robot_skills import RobotSkills

with RobotSkills() as robot:
    robot.head.set_angle(pan=30, tilt=-10)
    robot.left_arm.set_gripper(80, duration_ms=500)
    robot.base.move_for(x=0.1, duration=2)
    robot.cameras.save_snapshot("cam_a")
    obs = robot.get_observation()
    robot.emergency_stop()
```

选择性启用: `RobotSkills(enable_base=False, enable_cameras=False)`

### ArmController (关节空间)

```python
from robot_skills import ArmController, PORT_LEFT_FOLLOWER

arm = ArmController(PORT_LEFT_FOLLOWER, "left", acceleration=20)
arm.connect()

arm.read_position()                                # → ArmState
arm.read_position_raw()                            # → {name: 0-4095}

arm.move_to({"shoulder_pan": 10.0}, duration_ms=1000)  # 1 秒到达
arm.move_smooth({"shoulder_pan": 10.0}, duration=2.0)  # 2 秒插值
arm.set_gripper(100, duration_ms=500)                   # 0.5 秒开夹爪

arm.set_acceleration(30)                           # 调整加速度
arm.disable_torque()                               # 释放关节
arm.disconnect()
```

### ArmIKController (笛卡尔空间)

```python
from robot_skills import ArmController, ArmIKController, PORT_LEFT_FOLLOWER

arm = ArmController(PORT_LEFT_FOLLOWER, "left")
arm.connect()
ik = ArmIKController(arm)

pose = ik.get_cartesian_pose()                     # → CartesianPose(x, y, z, pitch, roll, gripper)

ik.move_cartesian(0.18, 0.0, 0.10, duration=2.0)  # 笛卡尔直线移动, 2 秒
ik.move_cartesian_delta(dx=0.02, dz=-0.01, duration=1.5)  # 增量移动
ik.move_to_home(duration=3.0)                      # 回零位

ws = ik.get_workspace_info()                       # 工作空间参数
```

### HeadController

```python
head.set_angle(pan=30, tilt=-10)       # 正=左/上, 负=右/下
head.look_center()                     # 渐进回中
head.scan(angle_range=60, steps=5)     # 水平扫描
```

### BaseController

```python
base.move(x=0.2)                       # 前进
base.move(y=0.1)                       # 左移
base.move(theta=30)                    # 逆时针旋转
base.move_for(x=0.1, duration=2)       # 前进 2 秒
base.stop()
```

### CameraManager

```python
cam.capture("cam_a")                   # → numpy BGR
cam.capture_all()                      # → {"cam_a": ndarray, "cam_b": ndarray}
cam.save_snapshot("cam_a", "out.jpg")
```

## 总线共享

同一串口不能被两个 bus 同时打开。`RobotSkills` 自动处理:

```
ttyACM0: left_arm bus (ID 1-9) ← base 共享此 bus
ttyACM1: right_arm bus (ID 1-8) ← head 共享此 bus
```

独立使用 HeadController / BaseController 时, 不能同时使用同端口的 ArmController。

## 校准

```bash
conda activate new-lerobot

# 左臂
python -m lerobot.scripts.lerobot_measure_feetech_ranges \
    --port /dev/ttyACM0 --save --robot-id left

# 右臂
python -m lerobot.scripts.lerobot_measure_feetech_ranges \
    --port /dev/ttyACM1 --save --robot-id right

# 硬件自检
python -m robot_skills
```
