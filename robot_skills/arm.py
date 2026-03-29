"""SO-101 六轴臂控制器 (位置模式, 带速度控制)

安全策略:
    1. MAX_STEP_PER_CMD: 单次命令最大关节变化量 (归一化), 超过则自动拆成小步
    2. Acceleration 寄存器: 控制加减速梯度 (1=最慢, 254=最快)
    3. Goal_Time 寄存器: 指定到达目标的时间 (ms), 0=不限时
    4. 校准偏移: 从 calibration.json 加载零位, 保证 IK 角度有意义
"""

import json
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

from .bus_utils import build_arm_motors, make_bus, safe_disconnect
from .config import ARM_MOTOR_IDS, ARM_MOTOR_NAMES

logger = logging.getLogger("robot_skills")

DEFAULT_ACCELERATION = 10
MAX_ACCELERATION = 254
MAX_STEP_PER_CMD = 5.0

CALIBRATION_FILE = Path(__file__).parent / "calibration.json"


@dataclass
class ArmState:
    shoulder_pan: float = 0.0
    shoulder_lift: float = 0.0
    elbow_flex: float = 0.0
    wrist_flex: float = 0.0
    wrist_roll: float = 0.0
    gripper: float = 0.0

    def to_dict(self) -> dict[str, float]:
        return {n: getattr(self, n) for n in ARM_MOTOR_NAMES}

    @classmethod
    def from_dict(cls, d: dict[str, float]) -> "ArmState":
        return cls(**{k: d[k] for k in ARM_MOTOR_NAMES if k in d})

    def __repr__(self):
        vals = ", ".join(f"{n}={getattr(self, n):.1f}" for n in ARM_MOTOR_NAMES)
        return f"ArmState({vals})"


class ArmController:
    """SO-101 臂控制器

    所有运动方法默认使用安全速度. 通过 acceleration / duration 参数控制速度.
    """

    def __init__(self, port: str, name: str = "arm",
                 acceleration: int = DEFAULT_ACCELERATION):
        self.port = port
        self.name = name
        self._bus: Optional[FeetechMotorsBus] = None
        self._acceleration = max(1, min(MAX_ACCELERATION, acceleration))

    @property
    def is_connected(self) -> bool:
        return self._bus is not None and self._bus.is_connected

    def connect(self, extra_motors: Optional[dict[str, int]] = None) -> None:
        """连接臂。extra_motors 用于在同一 bus 上附加云台/底盘电机。"""
        if self.is_connected:
            return
        if not self.port:
            raise RuntimeError(f"[{self.name}] 未检测到对应串口，已跳过连接")
        from lerobot.motors import Motor, MotorNormMode
        motors = build_arm_motors()
        if extra_motors:
            for n, mid in extra_motors.items():
                motors[n] = Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
        self._bus = make_bus(self.port, motors)
        self._configure()
        logger.info(f"[{self.name}] 已连接 {self.port} (acceleration={self._acceleration})")

    def _configure(self) -> None:
        try:
            held_pos = self._bus.sync_read("Present_Position", normalize=False)
        except Exception:
            held_pos = None

        self._bus.disable_torque()
        self._bus.configure_motors()

        for motor in self._bus.motors:
            try:
                self._bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            except Exception as e:
                logger.warning(f"[{self.name}] {motor} Operating_Mode 写入失败: {e}")
            for reg, val in [("P_Coefficient", 16), ("I_Coefficient", 0), ("D_Coefficient", 32)]:
                try:
                    self._bus.write(reg, motor, val)
                except Exception as e:
                    logger.warning(f"[{self.name}] {motor} {reg} 写入失败 (已跳过): {e}")
            if motor == "gripper":
                for reg, val in [("Max_Torque_Limit", 500), ("Protection_Current", 250),
                                 ("Overload_Torque", 25)]:
                    try:
                        self._bus.write(reg, motor, val)
                    except Exception as e:
                        logger.warning(f"[{self.name}] gripper {reg} 写入失败 (已跳过): {e}")

        if held_pos:
            try:
                self._bus.sync_write("Goal_Position", held_pos, normalize=False)
            except Exception as e:
                logger.warning(f"[{self.name}] 写入保持位置失败: {e}")

        failed_motors = []
        for motor in self._bus.motors:
            try:
                self._bus.write("Torque_Enable", motor, 1)
            except Exception as e:
                failed_motors.append(motor)
                logger.warning(f"[{self.name}] {motor} 开扭矩失败 (可能处于保护状态): {e}")
        if failed_motors:
            logger.warning(f"[{self.name}] ⚠ 以下舵机不可用: {failed_motors}  → 需要断电重启机器人清除保护")
        self.set_acceleration(self._acceleration)

    def disconnect(self, disable_torque: bool = True) -> None:
        if not self.is_connected:
            return
        safe_disconnect(self._bus, disable_torque, self.name)
        self._bus = None
        logger.info(f"[{self.name}] 已断开")

    # ── 速度控制 ──

    def set_acceleration(self, value: int) -> None:
        """设置加速度 (1=最慢平滑, 254=最快冲击). 推荐 10~50."""
        self._check()
        self._acceleration = max(1, min(MAX_ACCELERATION, value))
        for motor in self._bus.motors:
            try:
                self._bus.write("Acceleration", motor, self._acceleration)
            except Exception as e:
                logger.warning(f"[{self.name}] {motor} Acceleration 写入失败: {e}")

    def set_goal_time(self, ms: int) -> None:
        """设置到达目标的时间 (毫秒). 0=不限时 (用 Acceleration 控制). 推荐 500~3000."""
        self._check()
        ms = max(0, min(65535, ms))
        for motor in self._bus.motors:
            if motor in ARM_MOTOR_NAMES:
                try:
                    self._bus.write("Goal_Time", motor, ms)
                except Exception as e:
                    logger.warning(f"[{self.name}] {motor} Goal_Time 写入失败: {e}")

    # ── 读取 ──

    def read_position(self, retries: int = 3) -> ArmState:
        self._check()
        for attempt in range(retries):
            try:
                pos = self._bus.sync_read("Present_Position")
                return ArmState.from_dict(pos)
            except Exception as e:
                if attempt == retries - 1:
                    raise
                logger.warning(f"[{self.name}] 读取失败 (重试 {attempt+1}/{retries}): {e}")
                time.sleep(0.05)

    def read_position_raw(self, retries: int = 3) -> dict[str, int]:
        self._check()
        for attempt in range(retries):
            try:
                return self._bus.sync_read("Present_Position", ARM_MOTOR_NAMES, normalize=False)
            except Exception as e:
                if attempt == retries - 1:
                    raise
                logger.warning(f"[{self.name}] 原始读取失败 (重试 {attempt+1}/{retries}): {e}")
                time.sleep(0.05)

    def is_near_home(self, threshold: float = 30.0) -> bool:
        """检查臂是否在零位附近 (归一化值偏差 < threshold)"""
        try:
            state = self.read_position()
            return all(abs(getattr(state, n)) < threshold for n in ARM_MOTOR_NAMES if n != "gripper")
        except Exception:
            return False

    # ── 移动 ──

    def move_to(self, target: dict[str, float],
                duration_ms: int = 0,
                wait: bool = False, timeout: float = 5.0) -> None:
        """安全移动到目标位置 (归一化值)

        如果任何关节变化超过 MAX_STEP_PER_CMD, 自动拆成多步.

        Args:
            target: 关节名 → 归一化值 [-100, 100]
            duration_ms: 到达时间 (毫秒), 0=由 Acceleration 控制
            wait: 是否阻塞等待到位
            timeout: 等待超时 (秒)
        """
        self._check()

        try:
            current = self.read_position().to_dict()
        except Exception:
            current = None

        if current is not None:
            max_delta = max(abs(target.get(k, current.get(k, 0)) - current.get(k, 0))
                           for k in target)
            if max_delta > MAX_STEP_PER_CMD:
                n_steps = int(max_delta / MAX_STEP_PER_CMD) + 1
                step_ms = max(100, duration_ms // n_steps) if duration_ms > 0 else 200
                keys = list(target.keys())
                start = np.array([current.get(k, 0.0) for k in keys])
                end = np.array([target[k] for k in keys])
                for i in range(1, n_steps + 1):
                    alpha = i / n_steps
                    interp = start + alpha * (end - start)
                    cmd = {k: float(v) for k, v in zip(keys, interp)}
                    self._safe_write_position(cmd, step_ms)
                    time.sleep(step_ms / 1000.0)
                if wait:
                    self._wait_until_reached(target, timeout)
                return

        self._safe_write_position(target, duration_ms)
        if wait:
            self._wait_until_reached(target, timeout)

    def _safe_write_position(self, target: dict[str, float], duration_ms: int = 0) -> None:
        """底层写入: Goal_Time + Goal_Position, 全部容错"""
        if duration_ms > 0:
            for motor in list(target.keys()):
                if motor in ARM_MOTOR_NAMES:
                    try:
                        self._bus.write("Goal_Time", motor, duration_ms)
                    except Exception:
                        pass
        try:
            self._bus.sync_write("Goal_Position", target)
        except Exception as e:
            logger.warning(f"[{self.name}] Goal_Position 写入失败: {e}")
        if duration_ms > 0:
            for motor in list(target.keys()):
                if motor in ARM_MOTOR_NAMES:
                    try:
                        self._bus.write("Goal_Time", motor, 0)
                    except Exception:
                        pass

    def move_to_raw(self, target: dict[str, int], duration_ms: int = 0) -> None:
        self._check()
        if duration_ms > 0:
            for motor in list(target.keys()):
                try:
                    self._bus.write("Goal_Time", motor, duration_ms)
                except Exception:
                    pass
        try:
            self._bus.sync_write("Goal_Position", target, normalize=False)
        except Exception as e:
            logger.warning(f"[{self.name}] 原始写入失败: {e}")
        if duration_ms > 0:
            for motor in list(target.keys()):
                try:
                    self._bus.write("Goal_Time", motor, 0)
                except Exception:
                    pass

    def move_smooth(self, target: dict[str, float],
                    duration: float = 2.0,
                    hz: float = 20.0) -> None:
        """软件插值平滑移动 (关节空间线性插值)

        Args:
            target: 目标位置 (归一化值)
            duration: 总时长 (秒)
            hz: 控制频率 (Hz)
        """
        self._check()
        current = self.read_position().to_dict()
        keys = list(target.keys())
        start_vals = np.array([current.get(k, 0.0) for k in keys])
        end_vals = np.array([target[k] for k in keys])

        steps = max(2, int(duration * hz))
        dt = duration / steps
        step_ms = max(50, int(dt * 1000))

        for i in range(1, steps + 1):
            alpha = i / steps
            interp = start_vals + alpha * (end_vals - start_vals)
            cmd = {k: float(v) for k, v in zip(keys, interp)}
            try:
                self.move_to(cmd, duration_ms=step_ms)
            except Exception as e:
                logger.warning(f"[{self.name}] 插值步 {i}/{steps} 写入失败: {e}")
            time.sleep(dt)

    def set_gripper(self, openness: float, duration_ms: int = 500) -> None:
        """0=闭合, 100=张开"""
        self._check()
        if duration_ms > 0:
            try:
                self._bus.write("Goal_Time", "gripper", duration_ms)
            except Exception:
                pass
        self._bus.sync_write("Goal_Position", {"gripper": openness})
        if duration_ms > 0:
            try:
                self._bus.write("Goal_Time", "gripper", 0)
            except Exception:
                pass

    # ── 扭矩 ──

    def enable_torque(self) -> None:
        self._check()
        self._bus.enable_torque()

    def disable_torque(self) -> None:
        self._check()
        self._bus.disable_torque()

    # ── 内部 ──

    def _wait_until_reached(self, target: dict[str, float], timeout: float) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            pos = self._bus.sync_read("Present_Position")
            if all(abs(pos.get(k, 0) - v) < 2.0 for k, v in target.items()):
                return
            time.sleep(0.05)

    def _check(self):
        if not self.is_connected:
            raise RuntimeError(f"[{self.name}] 未连接")
