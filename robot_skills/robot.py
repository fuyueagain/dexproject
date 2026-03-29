"""RobotSkills: 统一控制接口

自动处理总线共享:
  ttyACM0: 左臂(ID1-6) + 底盘(ID7-9) → 一个 bus
  ttyACM1: 右臂(ID1-6) + 云台(ID7-8) → 一个 bus
"""

import logging
from typing import Any

from .arm import ArmController
from .base import BaseController
from .camera import CameraManager
from .config import (
    BASE_MOTOR_IDS, HEAD_MOTOR_IDS,
    PORT_LEFT_FOLLOWER, PORT_RIGHT_FOLLOWER,
)
from .head import HeadController

logger = logging.getLogger("robot_skills")


class RobotSkills:
    """
    典型用法:
        with RobotSkills() as robot:
            robot.head.set_angle(pan=30, tilt=-10)
            robot.left_arm.set_gripper(100)
            robot.base.move(x=0.1)
    """

    def __init__(self, enable_arms=True, enable_head=True,
                 enable_base=True, enable_cameras=True):
        self.left_arm = ArmController(PORT_LEFT_FOLLOWER, "left_arm") if enable_arms else None
        self.right_arm = ArmController(PORT_RIGHT_FOLLOWER, "right_arm") if enable_arms else None
        self.head = HeadController(PORT_RIGHT_FOLLOWER) if enable_head else None
        self.base = BaseController(PORT_LEFT_FOLLOWER) if enable_base else None
        self.cameras = CameraManager() if enable_cameras else None
        self._enable_head = enable_head
        self._enable_base = enable_base

    def connect(self) -> None:
        # ttyACM0: 左臂 + 底盘
        if self.left_arm:
            extra = BASE_MOTOR_IDS if self._enable_base else None
            self.left_arm.connect(extra_motors=extra)
        if self.base and self._enable_base:
            if self.left_arm and self.left_arm.is_connected:
                self.base._attach_shared_bus(self.left_arm._bus)
            else:
                self.base.connect()

        # ttyACM1: 右臂 + 云台
        if self.right_arm:
            extra = HEAD_MOTOR_IDS if self._enable_head else None
            self.right_arm.connect(extra_motors=extra)
        if self.head and self._enable_head:
            if self.right_arm and self.right_arm.is_connected:
                self.head._attach_shared_bus(self.right_arm._bus)
            else:
                self.head.connect()

        if self.cameras:
            self.cameras.connect()
        logger.info("=== 机器人就绪 ===")

    def disconnect(self) -> None:
        if self.base:
            self.base.disconnect()
        if self.head:
            self.head.disconnect()
        if self.left_arm:
            self.left_arm.disconnect()
        if self.right_arm:
            self.right_arm.disconnect()
        if self.cameras:
            self.cameras.disconnect()
        logger.info("=== 机器人已断开 ===")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.disconnect()

    def get_observation(self) -> dict[str, Any]:
        obs = {}
        if self.left_arm and self.left_arm.is_connected:
            for k, v in self.left_arm.read_position().to_dict().items():
                obs[f"left_{k}.pos"] = v
        if self.right_arm and self.right_arm.is_connected:
            for k, v in self.right_arm.read_position().to_dict().items():
                obs[f"right_{k}.pos"] = v
        if self.head and self.head.is_connected:
            for k, v in self.head.read_angle().items():
                obs[f"{k}.pos"] = v
        if self.cameras and self.cameras.is_connected:
            obs.update(self.cameras.capture_all())
        return obs

    def send_action(self, action: dict[str, float]) -> None:
        left, right, head_a, base_a = {}, {}, {}, {}
        for key, val in action.items():
            if key.startswith("left_") and key.endswith(".pos"):
                left[key.removeprefix("left_").removesuffix(".pos")] = val
            elif key.startswith("right_") and key.endswith(".pos"):
                right[key.removeprefix("right_").removesuffix(".pos")] = val
            elif key.startswith("head_") and key.endswith(".pos"):
                head_a[key.removesuffix(".pos")] = val
            elif key.endswith(".vel"):
                base_a[key] = val

        if left and self.left_arm and self.left_arm.is_connected:
            self.left_arm.move_to(left)
        if right and self.right_arm and self.right_arm.is_connected:
            self.right_arm.move_to(right)
        if head_a and self.head and self.head.is_connected:
            self.head.set_angle(pan=head_a.get("head_pan"),
                                tilt=head_a.get("head_tilt"))
        if base_a and self.base and self.base.is_connected:
            self.base.move(x=base_a.get("x.vel", 0),
                           y=base_a.get("y.vel", 0),
                           theta=base_a.get("theta.vel", 0))

    def emergency_stop(self) -> None:
        logger.warning("!!! 紧急停止 !!!")
        if self.base and self.base.is_connected:
            self.base.stop()
        if self.left_arm and self.left_arm.is_connected:
            self.left_arm.disable_torque()
        if self.right_arm and self.right_arm.is_connected:
            self.right_arm.disable_torque()
