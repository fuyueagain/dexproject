"""
robot_skills - XLeRobot 统一机器人技能库

用法:
    from robot_skills import RobotSkills
    with RobotSkills() as robot:
        robot.head.set_angle(pan=30)
        robot.left_arm.set_gripper(100)
        robot.base.move(x=0.1)
"""

import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path.home() / "lerobot-MakerMods" / "src"))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)

from .config import *
from .arm import ArmController, ArmState
from .head import HeadController
from .base import BaseController, BaseVelocity
from .camera import CameraManager
from .robot import RobotSkills
from .ik import SO101Kinematics2D
from .ik import ArmIKController as ArmIKController2D
from .ik import CartesianPose as CartesianPose2D
from .kinematics import (
    SO101Kinematics, ArmIKController, CartesianPose,
    servo_deg_to_urdf, urdf_to_servo_deg, norm_to_urdf, urdf_to_norm,
)

__all__ = [
    "RobotSkills",
    "ArmController", "ArmState",
    "HeadController",
    "BaseController", "BaseVelocity",
    "CameraManager",
    # 新 5DOF 运动学 (推荐)
    "SO101Kinematics", "ArmIKController", "CartesianPose",
    "servo_deg_to_urdf", "urdf_to_servo_deg", "norm_to_urdf", "urdf_to_norm",
    # 旧 2D 运动学 (兼容)
    "SO101Kinematics2D", "ArmIKController2D", "CartesianPose2D",
    "PORT_LEFT_FOLLOWER", "PORT_RIGHT_FOLLOWER",
    "PORT_LEFT_LEADER", "PORT_RIGHT_LEADER",
]
