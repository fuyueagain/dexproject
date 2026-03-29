"""三轮全向底盘控制器 (ttyACM0 ID7/8/9, 速度模式)"""

import logging
import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

from .bus_utils import make_bus, safe_disconnect
from .config import BASE_MOTOR_IDS, BASE_RADIUS, PORT_LEFT_FOLLOWER, WHEEL_ANGLES_DEG, WHEEL_RADIUS

logger = logging.getLogger("robot_skills")


@dataclass
class BaseVelocity:
    x: float = 0.0       # 前进 m/s
    y: float = 0.0       # 左移 m/s
    theta: float = 0.0   # 逆时针 deg/s


class BaseController:
    """与左臂共用 ttyACM0"""

    def __init__(self, port: str = PORT_LEFT_FOLLOWER):
        self.port = port
        self._bus: Optional[FeetechMotorsBus] = None
        self._shared = False

    @property
    def is_connected(self) -> bool:
        return self._bus is not None and self._bus.is_connected

    def _attach_shared_bus(self, bus: FeetechMotorsBus) -> None:
        self._bus = bus
        self._shared = True
        for n in BASE_MOTOR_IDS:
            try:
                self._bus.write("Torque_Enable", n, 0)
                self._bus.write("Operating_Mode", n, OperatingMode.VELOCITY.value)
                self._bus.write("Torque_Enable", n, 1)
            except Exception as e:
                logger.warning(f"[base] 切换 {n} 速度模式失败: {e}")
        logger.info("[base] 绑定共享总线, 速度模式就绪")

    def connect(self) -> None:
        if self.is_connected or self._shared:
            return
        motors = {n: Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
                  for n, mid in BASE_MOTOR_IDS.items()}
        self._bus = make_bus(self.port, motors)
        self._bus.disable_torque()
        self._bus.configure_motors()
        for n in BASE_MOTOR_IDS:
            self._bus.write("Operating_Mode", n, OperatingMode.VELOCITY.value)
        self._bus.enable_torque()
        logger.info(f"[base] 已连接 {self.port}")

    def disconnect(self) -> None:
        self.stop()
        if self._shared:
            self._bus = None
            self._shared = False
            return
        safe_disconnect(self._bus, True, "base")
        self._bus = None

    def move(self, x=0.0, y=0.0, theta=0.0) -> None:
        self._check()
        self._bus.sync_write("Goal_Velocity", self._body_to_wheel_raw(x, y, theta))

    def stop(self) -> None:
        if not self.is_connected:
            return
        try:
            self._bus.sync_write("Goal_Velocity", {n: 0 for n in BASE_MOTOR_IDS})
        except Exception:
            pass

    def move_for(self, x=0.0, y=0.0, theta=0.0, duration=1.0) -> None:
        self.move(x, y, theta)
        time.sleep(duration)
        self.stop()

    @staticmethod
    def _body_to_wheel_raw(x, y, theta, max_raw=3000) -> dict[str, int]:
        theta_rad = theta * (math.pi / 180.0)
        vel = np.array([x, y, theta_rad])
        angles = np.radians(np.array(WHEEL_ANGLES_DEG) - 90)
        m = np.array([[np.cos(a), np.sin(a), BASE_RADIUS] for a in angles])
        wheel_degps = (m.dot(vel) / WHEEL_RADIUS) * (180.0 / np.pi)
        steps_per_deg = 4096.0 / 360.0
        raw_abs = [abs(d) * steps_per_deg for d in wheel_degps]
        if max(raw_abs) > max_raw:
            wheel_degps = wheel_degps * (max_raw / max(raw_abs))
        names = list(BASE_MOTOR_IDS.keys())
        return {n: max(-0x8000, min(0x7FFF, int(round(d * steps_per_deg))))
                for n, d in zip(names, wheel_degps)}

    def _check(self):
        if not self.is_connected:
            raise RuntimeError("[base] 未连接")
