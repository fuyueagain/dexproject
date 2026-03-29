# XLeRobot 机器人技能库 (`robot_skills.py`)

> 基于 `servo_map.json` 交互式校准的实测数据

## 1. 硬件拓扑 (实测确认)

```
┌──────────────────────────────────────────────────────┐
│           Jetson Orin NX Super (8GB)                  │
│           JetPack R36.4.3 / CUDA 12.6                │
├──────────────────────────────────────────────────────┤
│                                                       │
│  4 条 USB 串口总线, 共 29 个 STS3215 舵机             │
│                                                       │
│  ┌───────────────────────────────────────┐            │
│  │ /dev/ttyACM0 — 左臂从臂 + 底盘 (9个) │            │
│  │   ID 1: shoulder_pan  (底座左右旋转)  │            │
│  │   ID 2: shoulder_lift (肩部上下俯仰)  │            │
│  │   ID 3: elbow_flex    (肘部上下俯仰)  │            │
│  │   ID 4: wrist_flex    (腕部上下俯仰)  │            │
│  │   ID 5: wrist_roll    (腕部左右旋转)  │            │
│  │   ID 6: gripper       (夹爪开合)      │            │
│  │   ID 7: base_left_wheel  (底盘轮1)   │ ← 底盘     │
│  │   ID 8: base_back_wheel  (底盘轮2)   │   三轮     │
│  │   ID 9: base_right_wheel (底盘轮3)   │   三角布局 │
│  └───────────────────────────────────────┘            │
│  ┌───────────────────────────────────────┐            │
│  │ /dev/ttyACM1 — 右臂从臂 + 云台 (8个) │            │
│  │   ID 1: shoulder_pan  (底座左右旋转)  │            │
│  │   ID 2: shoulder_lift (肩部上下俯仰)  │            │
│  │   ID 3: elbow_flex    (肘部上下俯仰)  │            │
│  │   ID 4: wrist_flex    (腕部上下俯仰)  │            │
│  │   ID 5: wrist_roll    (腕部左右旋转)  │            │
│  │   ID 6: gripper       (夹爪开合)      │            │
│  │   ID 7: head_pan  (云台水平旋转)     │ ← Camera A │
│  │   ID 8: head_tilt (云台垂直俯仰)     │   云台     │
│  └───────────────────────────────────────┘            │
│  ┌───────────────────────────────────────┐            │
│  │ /dev/ttyACM2 — 左臂主臂 (6个, 遥操)  │            │
│  │   ID 1-6: 与从臂相同关节命名          │            │
│  └───────────────────────────────────────┘            │
│  ┌───────────────────────────────────────┐            │
│  │ /dev/ttyACM3 — 右臂主臂 (6个, 遥操)  │            │
│  │   ID 1-6: 与从臂相同关节命名          │            │
│  └───────────────────────────────────────┘            │
│                                                       │
│  USB 摄像头                                           │
│  ┌───────────────────────────────────────┐            │
│  │ /dev/video0 — Camera A (头部云台上)    │            │
│  │ /dev/video2 — Camera B (右腕)         │            │
│  └───────────────────────────────────────┘            │
└──────────────────────────────────────────────────────┘
```

---

## 2. 总线共享机制

同一串口不能被两个 bus 实例同时打开。`RobotSkills` 自动处理：

```
RobotSkills.connect()
  ├── left_arm.connect(extra_motors={底盘 ID7/8/9})
  │     └── 创建 bus(ttyACM0, ID 1-9)
  ├── base._attach_shared_bus(left_arm._bus)
  │     └── 复用 ttyACM0, 底盘切速度模式
  ├── right_arm.connect(extra_motors={云台 ID7/8})
  │     └── 创建 bus(ttyACM1, ID 1-8)
  └── head._attach_shared_bus(right_arm._bus)
        └── 复用 ttyACM1
```

---

## 3. 环境准备

```bash
conda activate new-lerobot
cd ~/lerobot-MakerMods && pip install -e ".[feetech]"
sudo chmod 666 /dev/ttyACM{0,1,2,3}
```

---

## 4. 校准

```bash
# 左臂从臂
python -m lerobot.scripts.lerobot_measure_feetech_ranges \
    --port /dev/ttyACM0 --save --robot-id left

# 右臂从臂
python -m lerobot.scripts.lerobot_measure_feetech_ranges \
    --port /dev/ttyACM1 --save --robot-id right
```

---

## 5. API 速查

### 5.1 统一入口 `RobotSkills`

```python
from robot_skills import RobotSkills

# 上下文管理器
with RobotSkills() as robot:
    robot.head.set_angle(pan=30, tilt=-10)
    robot.left_arm.set_gripper(100)
    robot.base.move_for(x=0.1, duration=2)
    robot.cameras.save_snapshot("cam_a")

# 选择性启用
robot = RobotSkills(enable_base=False, enable_cameras=False)
robot.connect()
```

### 5.2 单臂 `ArmController`

```python
from robot_skills import ArmController, PORT_LEFT_FOLLOWER

arm = ArmController(PORT_LEFT_FOLLOWER, "left_arm")
arm.connect()

pos = arm.read_position()            # ArmState (归一化)
raw = arm.read_position_raw()        # dict (0-4095)
arm.move_to({"shoulder_pan": 10.0})
arm.move_to({"elbow_flex": -20}, wait=True)
arm.set_gripper(100)                 # 张开
arm.set_gripper(0)                   # 闭合
arm.disable_torque()                 # 可手动拖动
arm.enable_torque()
arm.disconnect()
```

### 5.3 头部云台 `HeadController`

```python
from robot_skills import HeadController

head = HeadController()   # 默认 ttyACM1
head.connect()

head.read_angle()                    # {"head_pan": 15.2, "head_tilt": -3.1}
head.set_angle(pan=30)               # 左转 30°
head.set_angle(tilt=-15)             # 下压 15°
head.set_angle(pan=30, tilt=10)      # 同时两轴
head.set_angle(pan=0, wait=True)     # 回中并等待
head.look_center()                   # 快速回中
head.scan(angle_range=60, steps=5)   # 水平扫描

head.disconnect()
```

### 5.4 底盘 `BaseController`

```python
from robot_skills import BaseController

base = BaseController()   # 默认 ttyACM0
base.connect()

base.move(x=0.2)                     # 前进
base.move(y=0.1)                     # 左移
base.move(theta=30)                  # 逆时针旋转
base.move_for(x=0.2, duration=2)     # 前进 2 秒后停
base.stop()

base.disconnect()
```

底盘坐标系:
```
     x+ (前进)
      ↑
 y+ ←─┼─→ y-
      ↓
     x- (后退)      theta+ 逆时针
```

### 5.5 摄像头 `CameraManager`

```python
from robot_skills import CameraManager

cam = CameraManager()
cam.connect()

frame = cam.capture("cam_a")         # numpy BGR
frames = cam.capture_all()           # {"cam_a": ..., "cam_b": ...}
cam.save_snapshot("cam_a", "out.jpg")

cam.disconnect()
```

### 5.6 紧急停止

```python
robot.emergency_stop()   # 底盘停 + 双臂释放扭矩
```

---

## 6. 信息流

### 6.1 臂控制

```
arm.move_to({"shoulder_pan": 10.0})
  → bus.sync_write("Goal_Position", {"shoulder_pan": 10.0})
  → scservo_sdk → USB → ttyACM0/1 → STS3215 舵机
```

### 6.2 头部云台

```
head.set_angle(pan=30, tilt=10)
  → degrees_to_raw(30) ≈ 2388
  → bus.sync_write("Goal_Position", {"head_pan": 2388, "head_tilt": 2161})
  → ttyACM1 → ID 7 水平转 + ID 8 俯仰
  → Camera A 朝向改变
```

### 6.3 底盘

```
base.move(x=0.2, y=0, theta=0)
  → 三轮全向运动学 → 三个轮速指令
  → bus.sync_write("Goal_Velocity", {轮1, 轮2, 轮3})
  → ttyACM0 → ID 7/8/9 速度模式
```

---

## 7. 自检

```bash
conda activate new-lerobot
cd ~/dexproject
python robot_skills.py
```

```
[1] 摄像头
[2] 左臂从臂 (ACM0)
[3] 右臂从臂 (ACM1)
[4] 头部云台 (ACM1 ID7/8)
[5] 底盘三轮 (ACM0 ID7/8/9)
[6] 左臂主臂 (ACM2)
[7] 右臂主臂 (ACM3)
[8] 左臂校准
[9] 右臂校准
[a] 全部测试
[q] 退出
```

---

## 8. 示例

### 头部扫描拍照

```python
from robot_skills import RobotSkills

with RobotSkills(enable_base=False) as robot:
    for i, angle in enumerate([-60, -30, 0, 30, 60]):
        robot.head.set_angle(pan=angle, wait=True)
        robot.cameras.save_snapshot("cam_a", f"scan_{i}.jpg")
    robot.head.look_center()
```

### 双臂协同

```python
from robot_skills import RobotSkills
import time

with RobotSkills(enable_base=False, enable_cameras=False) as robot:
    robot.left_arm.set_gripper(100)
    robot.right_arm.set_gripper(100)
    time.sleep(1)
    robot.left_arm.set_gripper(10)
    robot.right_arm.set_gripper(10)
```

### 底盘巡逻

```python
from robot_skills import RobotSkills

with RobotSkills(enable_arms=False) as robot:
    robot.base.move_for(x=0.15, duration=2)
    robot.cameras.save_snapshot("cam_a", "p1.jpg")
    robot.base.move_for(theta=45, duration=2)
    robot.cameras.save_snapshot("cam_a", "p2.jpg")
```

### 手动示教录制

```python
from robot_skills import ArmController, PORT_LEFT_FOLLOWER
import time, json

arm = ArmController(PORT_LEFT_FOLLOWER, "left")
arm.connect()
arm.disable_torque()

traj = []
for _ in range(100):
    traj.append({"t": time.time(), **arm.read_position_raw()})
    time.sleep(0.05)

arm.enable_torque()
arm.disconnect()
json.dump(traj, open("traj.json", "w"), indent=2)
```

### 轨迹回放

```python
from robot_skills import ArmController, PORT_LEFT_FOLLOWER
import time, json

arm = ArmController(PORT_LEFT_FOLLOWER, "left")
arm.connect()
traj = json.load(open("traj.json"))

for i, f in enumerate(traj):
    arm.move_to_raw({k: v for k, v in f.items() if k != "t"})
    if i < len(traj) - 1:
        time.sleep(max(0, traj[i+1]["t"] - f["t"]))

arm.disconnect()
```
