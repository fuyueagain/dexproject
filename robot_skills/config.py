"""
实测确认的舵机映射与硬件配置 (基于 servo_map.json)

硬件拓扑 (4 总线 29 个 STS3215):
    左臂从臂(ID1-6) + 底盘三轮(ID7/8/9) = 9 舵机
    右臂从臂(ID1-6) + 头部云台(ID7/8)   = 8 舵机
    左臂主臂(ID1-6, 遥操)               = 6 舵机
    右臂主臂(ID1-6, 遥操)               = 6 舵机

⚠ USB 端口号在重启/重插后会变化! 通过环境变量或 detect_ports.py 确认。
"""

import logging
import os

from .port_detection import detect_port_map

logger = logging.getLogger("robot_skills")

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
# 优先级:
#   1. 环境变量覆盖
#   2. 自动识别当前实际连接的串口
#   3. 若自动识别关闭, 才回退到文档中的静态默认值
_AUTO_DETECT_PORTS = os.environ.get("ROBOT_SKILLS_AUTO_DETECT_PORTS", "1") != "0"
_DEFAULT_PORTS = {
    "left_follower": "/dev/ttyACM0",
    "right_follower": "/dev/ttyACM1",
    "left_leader": "/dev/ttyACM2",
    "right_leader": "/dev/ttyACM3",
}

_DETECTED_PORTS = {}
if _AUTO_DETECT_PORTS:
    try:
        _DETECTED_PORTS = detect_port_map()
    except Exception as e:
        logger.warning(f"[ports] 自动识别串口失败，回退静态配置: {e}")

PORT_LEFT_FOLLOWER = os.environ.get("LEFT_PORT")
if PORT_LEFT_FOLLOWER is None:
    PORT_LEFT_FOLLOWER = _DETECTED_PORTS.get("left_follower")
    if PORT_LEFT_FOLLOWER is None and not _AUTO_DETECT_PORTS:
        PORT_LEFT_FOLLOWER = _DEFAULT_PORTS["left_follower"]

PORT_RIGHT_FOLLOWER = os.environ.get("RIGHT_PORT")
if PORT_RIGHT_FOLLOWER is None:
    PORT_RIGHT_FOLLOWER = _DETECTED_PORTS.get("right_follower")
    if PORT_RIGHT_FOLLOWER is None and not _AUTO_DETECT_PORTS:
        PORT_RIGHT_FOLLOWER = _DEFAULT_PORTS["right_follower"]

PORT_LEFT_LEADER = os.environ.get("LEFT_LEADER_PORT")
if PORT_LEFT_LEADER is None:
    PORT_LEFT_LEADER = _DETECTED_PORTS.get("left_leader")
    if PORT_LEFT_LEADER is None and not _AUTO_DETECT_PORTS:
        PORT_LEFT_LEADER = _DEFAULT_PORTS["left_leader"]

PORT_RIGHT_LEADER = os.environ.get("RIGHT_LEADER_PORT")
if PORT_RIGHT_LEADER is None:
    PORT_RIGHT_LEADER = _DETECTED_PORTS.get("right_leader")
    if PORT_RIGHT_LEADER is None and not _AUTO_DETECT_PORTS:
        PORT_RIGHT_LEADER = _DEFAULT_PORTS["right_leader"]

if _AUTO_DETECT_PORTS:
    logger.info(
        "[ports] 自动识别结果: left_follower=%s, right_follower=%s, left_leader=%s, right_leader=%s",
        PORT_LEFT_FOLLOWER or "未检测到",
        PORT_RIGHT_FOLLOWER or "未检测到",
        PORT_LEFT_LEADER or "未检测到",
        PORT_RIGHT_LEADER or "未检测到",
    )
    if PORT_RIGHT_FOLLOWER is None:
        logger.warning("[ports] 未检测到右臂从臂总线，已禁用右臂/云台控制，避免误控遥操作电机")

# ── 摄像头 ──
CAMERA_A_INDEX = 0   # /dev/video0 头部
CAMERA_B_INDEX = 2   # /dev/video2 右腕

# ── 底盘运动学 ──
WHEEL_RADIUS = 0.05
BASE_RADIUS = 0.125
WHEEL_ANGLES_DEG = [240.0, 0.0, 120.0]
