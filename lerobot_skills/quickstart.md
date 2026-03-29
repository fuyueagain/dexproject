# LeRobot 底层 API 快速上手

> 本文档展示如何用 lerobot 的底层 API (`FeetechMotorsBus`) 控制这台 XLeRobot。
> 所有示例已在真实硬件上验证。

## 前置条件

```bash
conda activate new-lerobot
cd ~/dexproject
```

## 1. 最简示例: 读取关节位置

```python
import sys
sys.path.insert(0, "/home/makermods/lerobot-MakerMods/src")

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# 定义电机
motors = {
    "shoulder_pan":  Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex":    Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex":    Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll":    Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    "gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}

# 连接
bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors)
bus.connect()

# 写入默认校准 (跳过交互式校准!)
cal = {}
for name, m in motors.items():
    cal[name] = MotorCalibration(
        id=m.id, drive_mode=0, homing_offset=0,
        range_min=0, range_max=4095,
    )
bus.write_calibration(cal)

# 读取
positions = bus.sync_read("Present_Position")
print(positions)
# {"shoulder_pan": -1.2, "shoulder_lift": 45.3, ...}

bus.disconnect()
```

## 2. 推荐方式: 使用 robot_skills

上面的底层代码比较冗长。`robot_skills` 已经封装好了连接、校准、容错等逻辑：

```python
from robot_skills import ArmController, PORT_LEFT_FOLLOWER

arm = ArmController(PORT_LEFT_FOLLOWER, "left", acceleration=20)
arm.connect()

# 读取
state = arm.read_position()
print(state)  # ArmState(shoulder_pan=0.2, shoulder_lift=-1.3, ...)

# 移动 (带速度控制)
arm.move_to({"shoulder_pan": 10.0}, duration_ms=1000)

# 夹爪
arm.set_gripper(100, duration_ms=500)

arm.disconnect()
```

## 3. 全机器人控制

```python
from robot_skills import RobotSkills

with RobotSkills() as robot:
    # get_observation() — 对应 lerobot 的 Robot.get_observation()
    obs = robot.get_observation()
    # obs = {
    #   "left_shoulder_pan.pos": -1.2,    # 左臂关节
    #   "right_shoulder_pan.pos": 3.5,    # 右臂关节
    #   "head_pan.pos": 0.0,              # 头部角度
    #   "cam_a": ndarray(480,640,3),      # 头部相机 BGR
    #   "cam_b": ndarray(480,640,3),      # 右腕相机 BGR
    # }

    # send_action() — 对应 lerobot 的 Robot.send_action()
    robot.send_action({
        "left_shoulder_pan.pos": 10.0,
        "right_gripper.pos": 80.0,
        "head_pan.pos": 20.0,
    })

    # 底盘
    robot.base.move_for(x=0.1, duration=1.5)

    # 紧急停止
    # robot.emergency_stop()
```

## 4. 模型推理循环

```python
import time
from robot_skills import RobotSkills

FPS = 20

with RobotSkills(enable_base=False) as robot:
    while True:
        t0 = time.perf_counter()

        obs = robot.get_observation()

        # 在这里调用你的模型
        # action = model.select_action(obs)

        # 示例: 维持当前位置
        action = {k: v for k, v in obs.items()
                  if isinstance(v, float) and k.endswith(".pos")}

        robot.send_action(action)

        time.sleep(max(0, 1.0/FPS - (time.perf_counter() - t0)))
```

## 5. 底层 API vs 高层 API 对比

| 操作 | lerobot 高层 (SO100Follower) | lerobot 底层 (FeetechMotorsBus) | robot_skills |
|------|------|------|------|
| 连接 | `robot.connect()` ← 触发交互校准 | `bus.connect()` + 手动写校准 | `arm.connect()` ← 自动处理 |
| 读取 | `robot.get_observation()` | `bus.sync_read("Present_Position")` | `arm.read_position()` |
| 写入 | `robot.send_action(action)` | `bus.sync_write("Goal_Position", ...)` | `arm.move_to(target)` |
| 速度控制 | 仅 `max_relative_target` | `bus.write("Acceleration", ...)` | `arm.set_acceleration()` |
| 力矩 | `robot.disconnect()` 时 | `bus.enable/disable_torque()` | `arm.enable/disable_torque()` |
| 夹爪 | action dict 中设值 | `sync_write("Goal_Position", {"gripper": x})` | `arm.set_gripper(x)` |
| 相机 | `OpenCVCamera` 类 | `cv2.VideoCapture` | `CameraManager` |

## 6. 为什么 SO100Follower 在这台机器上不好用

```
SO100Follower.connect()
  ↓
bus.connect()  ← OK
  ↓
is_calibrated == False  ← 没有校准文件
  ↓
calibrate()  ← 进入交互模式!
  ↓
"Move SO100Follower to the middle of its range..."  ← 需要手动操作
  ↓
record_ranges_of_motion()  ← 卡死在 sync_read 循环
  ↓
KeyboardInterrupt  ← 用户只能 Ctrl+C
```

`robot_skills` 的做法:
```
make_bus(port, motors)
  ↓
bus.connect()  ← OK (带重试)
  ↓
is_calibrated == False
  ↓
write_calibration(默认校准 0~4095)  ← 直接写入, 不需交互
  ↓
configure_motors()  ← 设置 PID, 位置模式
  ↓
enable_torque()  ← 完成, 可以控制了
```
