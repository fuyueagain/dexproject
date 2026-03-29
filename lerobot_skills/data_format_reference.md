# 数据格式参考 — XLeRobot 实际使用

## 1. robot_skills.get_observation() 返回值

```python
from robot_skills import RobotSkills

with RobotSkills() as robot:
    obs = robot.get_observation()
```

### 完整结构

```python
obs = {
    # ── 左臂 (ttyACM0, 归一化值) ──
    "left_shoulder_pan.pos":  -1.2,     # float, [-100, 100]
    "left_shoulder_lift.pos":  45.3,
    "left_elbow_flex.pos":     23.1,
    "left_wrist_flex.pos":    -5.7,
    "left_wrist_roll.pos":     0.0,
    "left_gripper.pos":        65.2,    # float, [0, 100]

    # ── 右臂 (ttyACM1, 归一化值) ──
    "right_shoulder_pan.pos":  3.5,
    "right_shoulder_lift.pos": -12.0,
    "right_elbow_flex.pos":    10.0,
    "right_wrist_flex.pos":   -2.3,
    "right_wrist_roll.pos":    0.0,
    "right_gripper.pos":       0.0,

    # ── 头部 (ttyACM1 ID7/8, 角度 度) ──
    "head_pan.pos":  5.3,               # float, 正=左转
    "head_tilt.pos": -2.1,              # float, 正=上抬

    # ── 相机 (BGR numpy) ──
    "cam_a": ndarray(480, 640, 3),      # uint8, BGR, 头部相机
    "cam_b": ndarray(480, 640, 3),      # uint8, BGR, 右腕相机
}
```

## 2. send_action() 输入格式

```python
robot.send_action({
    # 左臂关节 (去掉前缀后传给 arm.move_to)
    "left_shoulder_pan.pos": 10.0,
    "left_gripper.pos": 80.0,

    # 右臂关节
    "right_shoulder_pan.pos": -5.0,

    # 头部 (提取 head_pan / head_tilt)
    "head_pan.pos": 20.0,
    "head_tilt.pos": -10.0,

    # 底盘 (以 .vel 结尾)
    "x.vel": 0.1,       # 前进 m/s
    "y.vel": 0.0,       # 左移 m/s
    "theta.vel": 0.0,   # 逆时针 deg/s
})
```

## 3. 底层 bus.sync_read 返回值

```python
# 归一化模式 (默认)
pos = bus.sync_read("Present_Position")
# → {"shoulder_pan": -1.2, "shoulder_lift": 45.3, ..., "gripper": 65.2}
#   类型: dict[str, float]

# 原始模式
raw = bus.sync_read("Present_Position", normalize=False)
# → {"shoulder_pan": 2024, "shoulder_lift": 3070, ..., "gripper": 2700}
#   类型: dict[str, int], 范围 0~4095
```

## 4. 归一化值范围

### 身体关节 (RANGE_M100_100)

| 归一化值 | 编码器原始值 | 角度 |
|----------|-------------|------|
| -100.0 | 0 | -180° |
| 0.0 | 2047 | 0° (中位) |
| +100.0 | 4095 | +180° |

**1 归一化单位 ≈ 1.8°**

### 夹爪 (RANGE_0_100)

| 归一化值 | 状态 |
|----------|------|
| 0 | 全闭 |
| 50 | 半开 |
| 100 | 全开 |

### 头部角度

头部使用**角度** (度)，不使用归一化值:
```python
raw_value = bus.sync_read("Present_Position", ["head_pan"], normalize=False)["head_pan"]
angle_deg = (raw_value - 2047) * 360.0 / 4096.0
```

## 5. 相机图像格式

| 属性 | 值 |
|------|-----|
| 数据类型 | `numpy.ndarray` |
| 形状 | `(480, 640, 3)` |
| dtype | `uint8` |
| 值范围 | 0~255 |
| 颜色空间 | **BGR** (OpenCV 原生) |
| 编码 | MJPEG 传输, 解码后为 BGR |

### 与 lerobot 的区别

| | robot_skills | lerobot OpenCVCamera |
|---|---|---|
| 颜色空间 | **BGR** | **RGB** (自动转换) |
| 后端 | `cv2.CAP_V4L2` | 默认后端 |
| 编码 | MJPEG | 无强制 |

### 颜色转换

```python
frame_bgr = obs["cam_a"]              # robot_skills 返回 BGR

# → RGB
frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

# → PyTorch tensor (C, H, W) float [0, 1]
import torch
frame_tensor = torch.from_numpy(frame_rgb).permute(2, 0, 1).float() / 255.0

# → PIL Image
from PIL import Image
pil_img = Image.fromarray(frame_rgb)
```

## 6. 电机映射表

### 左臂从臂 (ttyACM0)

| 关节名 | 电机 ID | 归一化模式 |
|--------|---------|-----------|
| shoulder_pan | 1 | RANGE_M100_100 |
| shoulder_lift | 2 | RANGE_M100_100 |
| elbow_flex | 3 | RANGE_M100_100 |
| wrist_flex | 4 | RANGE_M100_100 |
| wrist_roll | 5 | RANGE_M100_100 |
| gripper | 6 | RANGE_0_100 |

### 底盘 (ttyACM0, 同一总线)

| 轮子名 | 电机 ID | 控制模式 |
|--------|---------|---------|
| base_left_wheel | 7 | 速度模式 |
| base_back_wheel | 8 | 速度模式 |
| base_right_wheel | 9 | 速度模式 |

### 右臂从臂 (ttyACM1)

与左臂相同 (ID 1-6)

### 头部云台 (ttyACM1, 同一总线)

| 关节名 | 电机 ID | 控制模式 |
|--------|---------|---------|
| head_pan | 7 | 位置模式 |
| head_tilt | 8 | 位置模式 |

### 编码器规格 (STS3215)

| 参数 | 值 |
|------|-----|
| 分辨率 | 4096 步 (12 bit) |
| 波特率 | 1,000,000 bps |
| 协议 | Feetech SCS/STS |
| 可读寄存器 | Present_Position, Present_Speed, Present_Load, Present_Voltage, Present_Temperature |
| 可写寄存器 | Goal_Position, Goal_Velocity, Acceleration, Goal_Time, Torque_Enable, Operating_Mode, P/I/D_Coefficient |
