"""Camera A 双轴云台控制器 (ttyACM1 ID7 pan / ID8 tilt)"""

import logging
import time
from typing import Optional

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

from .bus_utils import make_bus, safe_disconnect
from .config import HEAD_MOTOR_IDS, PORT_RIGHT_FOLLOWER

logger = logging.getLogger("robot_skills")


class HeadController:
    """与右臂共用 ttyACM1。独立使用或通过 RobotSkills 共享 bus。"""

    def __init__(self, port: str = PORT_RIGHT_FOLLOWER):
        self.port = port
        self._bus: Optional[FeetechMotorsBus] = None
        self._shared = False

    @property
    def is_connected(self) -> bool:
        return self._bus is not None and self._bus.is_connected

    def _attach_shared_bus(self, bus: FeetechMotorsBus) -> None:
        self._bus = bus
        self._shared = True
        logger.info("[head] 绑定共享总线")

    # 云台运动参数 — 防止瞬间大电流触发过载保护
    _ACCEL = 10         # 加减速档位 (0=瞬时, 越小越柔和)
    _MAX_VELOCITY = 300  # 最大速度 (steps/s, 0=不限)
    _P = 10              # 比例增益 (越低越柔)
    _D = 64              # 微分增益 (越高阻尼越大)
    _TORQUE_LIMIT = 500  # 运行扭矩上限 (满量程 1000)

    def connect(self) -> None:
        if self.is_connected or self._shared:
            return
        if not self.port:
            raise RuntimeError("[head] 未检测到云台所在右臂从臂串口")
        motors = {n: Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
                  for n, mid in HEAD_MOTOR_IDS.items()}
        self._bus = make_bus(self.port, motors)
        self._ok_motors = []
        for n in HEAD_MOTOR_IDS:
            try:
                self._bus.write("Torque_Enable", n, 0)
                self._bus.write("Operating_Mode", n, OperatingMode.POSITION.value)
                self._bus.write("P_Coefficient", n, self._P)
                self._bus.write("D_Coefficient", n, self._D)
                self._bus.write("Acceleration", n, self._ACCEL)
                self._bus.write("Goal_Velocity", n, self._MAX_VELOCITY, normalize=False)
                self._bus.write("Torque_Limit", n, self._TORQUE_LIMIT, normalize=False)
                cur = self._bus.read("Present_Position", n, normalize=False)
                self._bus.write("Goal_Position", n, cur, normalize=False)
                self._bus.write("Torque_Enable", n, 1)
                self._ok_motors.append(n)
            except Exception as e:
                logger.warning(f"[head] {n}(ID{HEAD_MOTOR_IDS[n]}) 配置失败 "
                               f"(可能过载保护，需断电重启): {e}")
        if self._ok_motors:
            logger.info(f"[head] 已连接 {self.port}, 可用: {self._ok_motors}")
        else:
            logger.error(f"[head] 所有舵机均不可用! 请断电重启机器人")

    def disconnect(self) -> None:
        if self._shared:
            self._bus = None
            self._shared = False
            return
        safe_disconnect(self._bus, True, "head")
        self._bus = None

    # ── 读取 ──

    def read_angle_raw(self, quiet=False) -> dict[str, int]:
        """逐个读取可用舵机，故障舵机返回中位默认值"""
        self._check()
        ok = getattr(self, "_ok_motors", list(HEAD_MOTOR_IDS.keys()))
        result = {n: 2047 for n in HEAD_MOTOR_IDS}
        for name in ok:
            try:
                pos = self._bus.sync_read("Present_Position", [name], normalize=False)
                result[name] = pos[name]
            except Exception as e:
                if not quiet:
                    logger.warning(f"[head] 读取 {name} 失败: {e}")
        return result

    def read_angle(self) -> dict[str, float]:
        """返回角度 (度, 0=中位)"""
        raw = self.read_angle_raw()
        return {n: (v - 2047) * 360.0 / 4096.0 for n, v in raw.items()}

    # ── 写入 ──

    def set_angle(self, pan: Optional[float] = None,
                  tilt: Optional[float] = None,
                  wait=False, timeout=3.0) -> None:
        """pan: 正=左转  tilt: 正=上抬  (度, 0=中位)"""
        self._check()
        ok = getattr(self, "_ok_motors", list(HEAD_MOTOR_IDS.keys()))
        target = {}
        if pan is not None and "head_pan" in ok:
            target["head_pan"] = max(0, min(4095, int(pan * 4096 / 360 + 2047)))
        if tilt is not None and "head_tilt" in ok:
            target["head_tilt"] = max(0, min(4095, int(tilt * 4096 / 360 + 2047)))
        if not target:
            return
        for name, val in target.items():
            try:
                self._bus.sync_write("Goal_Position", {name: val}, normalize=False)
            except Exception as e:
                logger.warning(f"[head] 写入 {name} 失败: {e}")
        if wait:
            t0 = time.time()
            while time.time() - t0 < timeout:
                try:
                    pos = self.read_angle_raw(quiet=True)
                    if all(abs(pos.get(k, 2047) - v) < 40 for k, v in target.items()):
                        return
                except Exception:
                    pass
                time.sleep(0.1)

    def set_angle_raw(self, target: dict[str, int]) -> None:
        self._check()
        ok = getattr(self, "_ok_motors", list(HEAD_MOTOR_IDS.keys()))
        for name, val in target.items():
            if name not in ok:
                continue
            try:
                self._bus.sync_write("Goal_Position", {name: val}, normalize=False)
            except Exception as e:
                logger.warning(f"[head] 写入 {name} 失败: {e}")

    def look_center(self, steps=10, step_delay=0.25) -> None:
        """渐进回中 (多步插值 + 硬件加速度限制, 避免过载)"""
        ok = getattr(self, "_ok_motors", list(HEAD_MOTOR_IDS.keys()))
        raw = self.read_angle_raw()
        for i in range(1, steps + 1):
            interp = {n: int(raw.get(n, 2047) + (2047 - raw.get(n, 2047)) * i / steps)
                      for n in ok}
            self.set_angle_raw(interp)
            time.sleep(step_delay)

    def scan(self, angle_range=60.0, steps=5, pause=0.5) -> list[float]:
        """水平扫描"""
        half = angle_range / 2.0
        angles = [-half + i * angle_range / max(steps - 1, 1) for i in range(steps)]
        for a in angles:
            self.set_angle(pan=a)
            time.sleep(pause)
        return angles

    def _check(self):
        if not self.is_connected:
            raise RuntimeError("[head] 未连接")
