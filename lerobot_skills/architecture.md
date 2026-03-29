# LeRobot 在 XLeRobot 上的架构与信息流

> **重要**: 本文档描述的是 lerobot 底层 API 在这台 XLeRobot 上的**实际使用方式**，
> 对应 `robot_skills/` 中已验证可工作的代码。

## 1. 实际硬件拓扑

```
ttyACM0 (1Mbps, 9 个 STS3215)              ttyACM1 (1Mbps, 8 个 STS3215)
┌──────────────────────────────┐            ┌──────────────────────────────┐
│ 左臂从臂 (位置模式)          │            │ 右臂从臂 (位置模式)          │
│  ID1 shoulder_pan   底座旋转 │            │  ID1 shoulder_pan   底座旋转 │
│  ID2 shoulder_lift  肩部俯仰 │            │  ID2 shoulder_lift  肩部俯仰 │
│  ID3 elbow_flex     肘部弯曲 │            │  ID3 elbow_flex     肘部弯曲 │
│  ID4 wrist_flex     腕部弯曲 │            │  ID4 wrist_flex     腕部弯曲 │
│  ID5 wrist_roll     腕部旋转 │            │  ID5 wrist_roll     腕部旋转 │
│  ID6 gripper        夹爪开合 │            │  ID6 gripper        夹爪开合 │
├──────────────────────────────┤            ├──────────────────────────────┤
│ 底盘三轮 (速度模式)          │            │ 头部云台 (位置模式)          │
│  ID7 base_left_wheel         │            │  ID7 head_pan   水平旋转     │
│  ID8 base_back_wheel         │            │  ID8 head_tilt  垂直俯仰     │
│  ID9 base_right_wheel        │            └──────────────────────────────┘
└──────────────────────────────┘

ttyACM2 (6 个) 左臂主臂 (遥操)             ttyACM3 (6 个) 右臂主臂 (遥操)
Camera A: /dev/video0 (头部)                Camera B: /dev/video2 (右腕)
```

**共 29 个 STS3215 舵机，全部 1Mbps 串口**

## 2. 为什么不能直接用 SO100Follower

`robot_skills/` 没有使用 lerobot 的 `SO100Follower` 高层类，而是直接使用底层的 `FeetechMotorsBus`。原因：

| 问题 | SO100Follower | robot_skills 做法 |
|------|---------------|-------------------|
| 校准 | `connect()` 触发**交互式校准**：要求手动晃动手臂+按Enter | `make_bus()` 写入默认校准 (0~4095)，跳过交互 |
| 共享总线 | 每个 SO100Follower 独占一个串口，只管 6 个关节 | 一个 bus 同时管理 臂(ID1-6) + 底盘/云台(ID7-9) |
| 容错 | `bus.connect()` 任何舵机无响应就抛异常 | 带重试，部分舵机无响应可跳过继续 |
| 相机 | 走 lerobot 的 `OpenCVCamera` 类 | 直接用 `cv2.VideoCapture` + V4L2 + MJPEG |

## 3. 实际使用的 LeRobot API 层次

`robot_skills` 使用的是 lerobot 的**底层电机总线 API**，不是高层 Robot 类：

```
lerobot 包结构:
├── robots/              # 高层 Robot 类 (SO100Follower 等) ← 不使用
├── motors/              # 底层电机总线 API ← 实际使用
│   ├── motors_bus.py    #   MotorsBus 抽象基类
│   ├── feetech/         #   FeetechMotorsBus (STS3215 舵机)
│   └── __init__.py      #   Motor, MotorCalibration, MotorNormMode
├── cameras/             # 相机 API ← 相机部分可选
└── teleoperators/       # 遥操作 ← 不使用
```

## 4. 信息流: robot_skills 如何用 lerobot API 控制机器人

### 4.1 连接阶段 (make_bus)

```
bus_utils.make_bus(port, motors)
│
├── 1. FeetechMotorsBus(port, motors, calibration)
│   └── 创建 bus 对象, 还未打开串口
│
├── 2. bus.connect()                        # 打开串口
│   ├── port_handler.openPort()             # pyserial 打开 /dev/ttyACM0
│   ├── _handshake()                        # ping 各电机, 校验型号
│   └── (如果舵机无响应 → 重试或跳过检查)
│
├── 3. 校准处理 (跳过交互式校准!)
│   └── bus.write_calibration({            # 直接写入默认校准
│         "shoulder_pan": MotorCalibration(id=1, drive_mode=0,
│             homing_offset=0, range_min=0, range_max=4095),
│         ...
│       })
│
└── 返回已连接的 bus 对象
```

### 4.2 读取关节位置

```
bus.sync_read("Present_Position")
│
├── GroupSyncRead.txRxPacket()              # 广播读取所有电机
│   └── 串口: 发送 Sync Read 指令 → 各电机依次应答原始编码器值
│
├── _decode_sign()                          # 解码有符号值
│
├── _normalize()                            # 编码器原始值 → 归一化
│   ├── shoulder_pan:  RANGE_M100_100 → [-100, 100]
│   ├── shoulder_lift: RANGE_M100_100 → [-100, 100]
│   ├── ...
│   └── gripper:       RANGE_0_100    → [0, 100]
│
└── 返回 {"shoulder_pan": -12.5, "shoulder_lift": 45.3, ..., "gripper": 65.2}
```

### 4.3 发送目标位置

```
bus.sync_write("Goal_Position", {"shoulder_pan": 10.0, "gripper": 50.0})
│
├── _unnormalize()                          # 归一化值 → 编码器原始值
│   └── 例: 10.0 → (10.0+100)/200 * 4095 + 0 = 2252
│
├── _encode_sign()                          # 编码有符号值
│
├── _serialize_data()                       # int → 字节列表
│
├── GroupSyncWrite.addParam(id, data)       # 每个电机的目标数据
│
└── GroupSyncWrite.txPacket()               # 一次性广播写入 (无应答, 速度快)
```

### 4.4 相机读取

```
cv2.VideoCapture(0, cv2.CAP_V4L2)
│
├── set FOURCC = MJPG                       # 降低 USB 带宽
├── set 640x480 @ 30fps
│
└── cap.read()
    └── 返回 BGR numpy.ndarray (480, 640, 3) uint8
```

## 5. robot_skills 与 lerobot 的 get_observation / send_action 对比

### robot_skills.RobotSkills.get_observation()

```python
obs = {}

# 左臂关节位置 (通过 bus.sync_read)
left_pos = left_arm._bus.sync_read("Present_Position")
# → {"shoulder_pan": -12.5, ...}
# 加前缀 → obs["left_shoulder_pan.pos"] = -12.5

# 右臂关节位置
right_pos = right_arm._bus.sync_read("Present_Position")
# 加前缀 → obs["right_shoulder_pan.pos"] = -12.5

# 头部角度 (同一个 bus 的 sync_read, 只读 head_pan/head_tilt)
head_pos = bus.sync_read("Present_Position", ["head_pan", "head_tilt"])
# → obs["head_pan.pos"] = 5.3

# 相机 (直接 cv2)
frame = cap.read()
# → obs["cam_a"] = numpy BGR (480, 640, 3)

return obs
```

### robot_skills.RobotSkills.send_action()

```python
# 解析 action 字典中的 key 前缀
action = {
    "left_shoulder_pan.pos": 10.0,    → left_arm.move_to({"shoulder_pan": 10.0})
    "right_gripper.pos": 80.0,        → right_arm.move_to({"gripper": 80.0})
    "head_pan.pos": 20.0,             → head.set_angle(pan=20.0)
    "x.vel": 0.1,                     → base.move(x=0.1)
}
```

## 6. 归一化值 ↔ 物理量

### STS3215 编码器

```
原始值范围: 0 ~ 4095 (12-bit)
中位值: 2047
一圈: 4096 步 = 360°
```

### 归一化公式

```
# RANGE_M100_100 (身体关节):
normalized = ((raw - range_min) / (range_max - range_min)) * 200 - 100
# range_min=0, range_max=4095 时:
# raw=2047 → normalized ≈ 0.0
# raw=0    → normalized = -100.0
# raw=4095 → normalized = +100.0
# 1 归一化单位 ≈ 1.8°

# RANGE_0_100 (夹爪):
normalized = ((raw - range_min) / (range_max - range_min)) * 100
# 0=全闭, 100=全开
```

## 7. 关键代码路径

| 功能 | robot_skills 路径 | 对应 lerobot 底层 API |
|------|-------------------|----------------------|
| 总线连接 | `bus_utils.make_bus()` | `FeetechMotorsBus(port, motors).connect()` |
| 读关节 | `arm.read_position()` | `bus.sync_read("Present_Position")` |
| 写关节 | `arm.move_to()` | `bus.sync_write("Goal_Position", ...)` |
| 速度控制 | `arm.set_acceleration()` | `bus.write("Acceleration", motor, val)` |
| 到达时间 | `arm.set_goal_time()` | `bus.write("Goal_Time", motor, ms)` |
| 力矩 | `arm.enable/disable_torque()` | `bus.enable/disable_torque()` |
| 相机 | `camera.CameraManager` | `cv2.VideoCapture(idx, CAP_V4L2)` |
