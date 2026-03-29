"""
实测确认的舵机映射与硬件配置 (基于 servo_map.json)

硬件拓扑 (4 总线 29 个 STS3215):
    左臂从臂(ID1-6) + 底盘三轮(ID7/8/9) = 9 舵机
    右臂从臂(ID1-6) + 头部云台(ID7/8)   = 8 舵机
    左臂主臂(ID1-6, 遥操)               = 6 舵机
    右臂主臂(ID1-6, 遥操)               = 6 舵机

⚠ USB 端口号在重启/重插后会变化! 通过环境变量或 detect_ports.py 确认。
"""

import os

# ── 臂关节 (左右臂相同，各自独立总线) ──
ARM_MOTOR_IDS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}
ARM_MOTOR_NAMES = list(ARM_MOTOR_IDS.keys())

# ── 头部云台 (与右臂同一总线) ──
HEAD_MOTOR_IDS = {
    "head_pan": 7,
    "head_tilt": 8,
}

# ── 底盘三轮 (与左臂同一总线) ──
BASE_MOTOR_IDS = {
    "base_left_wheel": 7,
    "base_back_wheel": 8,
    "base_right_wheel": 9,
}

# ── 端口 (可通过环境变量覆盖, 重启/重插后端口号可能变化) ──
# 通过探测确认: ACM0=左从臂+底盘, ACM1=右从臂+云台,
# ACM2=左主臂, ACM3=右主臂
PORT_LEFT_FOLLOWER = os.environ.get("LEFT_PORT", "/dev/ttyACM0")
PORT_RIGHT_FOLLOWER = os.environ.get("RIGHT_PORT", "/dev/ttyACM1")
PORT_LEFT_LEADER = os.environ.get("LEFT_LEADER_PORT", "/dev/ttyACM2")
PORT_RIGHT_LEADER = os.environ.get("RIGHT_LEADER_PORT", "/dev/ttyACM3")

# ── 摄像头 ──
CAMERA_A_INDEX = 0   # /dev/video0 头部
CAMERA_B_INDEX = 2   # /dev/video2 右腕

# ── 底盘运动学 ──
WHEEL_RADIUS = 0.05
BASE_RADIUS = 0.125
WHEEL_ANGLES_DEG = [240.0, 0.0, 120.0]
