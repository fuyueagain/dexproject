#!/usr/bin/env python3
"""
XLeRobot 统一机器人技能库

基于 servo_map.json 实测确认的舵机映射，封装双臂从臂、双臂主臂、
头部云台、三轮全向底盘、双目摄像头的控制接口。

硬件拓扑 (实测确认, 4 总线 29 个 STS3215 舵机):
    /dev/ttyACM0  →  左臂从臂 (ID 1-6) + 底盘三轮 (ID 7/8/9) = 9 舵机
    /dev/ttyACM1  →  右臂从臂 (ID 1-6) + 头部云台 (ID 7/8)   = 8 舵机
    /dev/ttyACM2  →  左臂主臂 (ID 1-6, 遥操作用)              = 6 舵机
    /dev/ttyACM3  →  右臂主臂 (ID 1-6, 遥操作用)              = 6 舵机
    /dev/video0   →  Camera A (头部云台上)
    /dev/video2   →  Camera B (右腕)

使用前提:
    conda activate new-lerobot
    cd ~/lerobot-MakerMods && pip install -e ".[feetech]"
"""

import glob
import logging
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np

sys.path.insert(0, str(Path.home() / "lerobot-MakerMods" / "src"))

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("robot_skills")

# ═══════════════════════════════════════════════════════════════════════════════
# 实测确认的舵机映射
# ═══════════════════════════════════════════════════════════════════════════════

ARM_MOTOR_IDS = {
    "shoulder_pan": 1,    # 关节1: 底座左右旋转
    "shoulder_lift": 2,   # 关节2: 肩部上下俯仰
    "elbow_flex": 3,      # 关节3: 肘部上下俯仰
    "wrist_flex": 4,      # 关节4: 腕部上下俯仰
    "wrist_roll": 5,      # 关节5: 腕部左右旋转
    "gripper": 6,         # 关节6: 夹爪开合
}
ARM_MOTOR_NAMES = list(ARM_MOTOR_IDS.keys())

HEAD_MOTOR_IDS = {
    "head_pan": 7,        # 云台水平旋转 (实测: ttyACM1 ID7)
    "head_tilt": 8,       # 云台垂直俯仰 (实测: ttyACM1 ID8)
}

BASE_MOTOR_IDS = {
    "base_left_wheel": 7,   # 底盘轮1 (实测: ttyACM0 ID7)
    "base_back_wheel": 8,   # 底盘轮2 (实测: ttyACM0 ID8)
    "base_right_wheel": 9,  # 底盘轮3 (实测: ttyACM0 ID9)
}

# ── 端口映射 (实测确认) ──
PORT_LEFT_FOLLOWER = "/dev/ttyACM0"   # 左臂从臂 + 底盘
PORT_RIGHT_FOLLOWER = "/dev/ttyACM1"  # 右臂从臂 + 头部云台
PORT_LEFT_LEADER = "/dev/ttyACM2"     # 左臂主臂 (遥操作)
PORT_RIGHT_LEADER = "/dev/ttyACM3"    # 右臂主臂 (遥操作)

CAMERA_A_INDEX = 0   # /dev/video0 头部
CAMERA_B_INDEX = 2   # /dev/video2 右腕

WHEEL_RADIUS = 0.05
BASE_RADIUS = 0.125
WHEEL_ANGLES_DEG = [240.0, 0.0, 120.0]


# ═══════════════════════════════════════════════════════════════════════════════
# 通用工具
# ═══════════════════════════════════════════════════════════════════════════════

def _make_bus(port: str, motors: dict[str, Motor],
              calibration: Optional[dict] = None) -> FeetechMotorsBus:
    bus = FeetechMotorsBus(port=port, motors=motors, calibration=calibration)
    bus.connect()
    if not bus.is_calibrated:
        cal = {}
        for name, motor in bus.motors.items():
            cal[name] = MotorCalibration(
                id=motor.id, drive_mode=0, homing_offset=0,
                range_min=0, range_max=4095,
            )
        bus.write_calibration(cal)
    return bus


def _build_arm_motors() -> dict[str, Motor]:
    motors = {
        n: Motor(ARM_MOTOR_IDS[n], "sts3215", MotorNormMode.RANGE_M100_100)
        for n in ARM_MOTOR_NAMES
    }
    motors["gripper"] = Motor(6, "sts3215", MotorNormMode.RANGE_0_100)
    return motors


# ═══════════════════════════════════════════════════════════════════════════════
# 数据类
# ═══════════════════════════════════════════════════════════════════════════════

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


@dataclass
class BaseVelocity:
    x: float = 0.0       # 前进 m/s
    y: float = 0.0       # 左移 m/s
    theta: float = 0.0   # 逆时针 deg/s


# ═══════════════════════════════════════════════════════════════════════════════
# 单臂控制器
# ═══════════════════════════════════════════════════════════════════════════════

class ArmController:
    """SO-101 六轴臂控制器 (位置模式)"""

    def __init__(self, port: str, name: str = "arm"):
        self.port = port
        self.name = name
        self._bus: Optional[FeetechMotorsBus] = None

    @property
    def is_connected(self) -> bool:
        return self._bus is not None and self._bus.is_connected

    def connect(self, extra_motors: Optional[dict[str, int]] = None) -> None:
        if self.is_connected:
            return
        motors = _build_arm_motors()
        if extra_motors:
            for n, mid in extra_motors.items():
                motors[n] = Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
        self._bus = _make_bus(self.port, motors)
        self._configure()
        logger.info(f"[{self.name}] 已连接 {self.port}")

    def _configure(self) -> None:
        self._bus.disable_torque()
        self._bus.configure_motors()
        for motor in self._bus.motors:
            self._bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            self._bus.write("P_Coefficient", motor, 16)
            self._bus.write("I_Coefficient", motor, 0)
            self._bus.write("D_Coefficient", motor, 32)
            if motor == "gripper":
                self._bus.write("Max_Torque_Limit", motor, 500)
                self._bus.write("Protection_Current", motor, 250)
                self._bus.write("Overload_Torque", motor, 25)
        self._bus.enable_torque()

    def disconnect(self, disable_torque: bool = True) -> None:
        if not self.is_connected:
            return
        try:
            self._bus.disconnect(disable_torque)
        except Exception as e:
            logger.warning(f"[{self.name}] 断开时异常 (已忽略): {e}")
        self._bus = None
        logger.info(f"[{self.name}] 已断开")

    def read_position(self) -> ArmState:
        self._check()
        pos = self._bus.sync_read("Present_Position")
        return ArmState.from_dict(pos)

    def read_position_raw(self) -> dict[str, int]:
        self._check()
        return self._bus.sync_read("Present_Position", ARM_MOTOR_NAMES, normalize=False)

    def move_to(self, target: dict[str, float], wait: bool = False,
                timeout: float = 5.0) -> None:
        self._check()
        self._bus.sync_write("Goal_Position", target)
        if wait:
            t0 = time.time()
            while time.time() - t0 < timeout:
                pos = self._bus.sync_read("Present_Position")
                if all(abs(pos.get(k, 0) - v) < 2.0 for k, v in target.items()):
                    return
                time.sleep(0.05)

    def move_to_raw(self, target: dict[str, int]) -> None:
        self._check()
        self._bus.sync_write("Goal_Position", target, normalize=False)

    def set_gripper(self, openness: float) -> None:
        self._check()
        self._bus.sync_write("Goal_Position", {"gripper": openness})

    def enable_torque(self) -> None:
        self._check()
        self._bus.enable_torque()

    def disable_torque(self) -> None:
        self._check()
        self._bus.disable_torque()

    def _check(self):
        if not self.is_connected:
            raise RuntimeError(f"[{self.name}] 未连接")


# ═══════════════════════════════════════════════════════════════════════════════
# 头部云台控制器 (ttyACM1 ID 7/8)
# ═══════════════════════════════════════════════════════════════════════════════

class HeadController:
    """Camera A 双轴云台: pan(ID7) + tilt(ID8), 与右臂共用 ttyACM1"""

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

    def connect(self) -> None:
        if self.is_connected:
            return
        if self._shared:
            return
        motors = {n: Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
                  for n, mid in HEAD_MOTOR_IDS.items()}
        self._bus = _make_bus(self.port, motors)
        self._bus.disable_torque()
        self._bus.configure_motors()
        for n in HEAD_MOTOR_IDS:
            self._bus.write("Operating_Mode", n, OperatingMode.POSITION.value)
            self._bus.write("P_Coefficient", n, 16)
            self._bus.write("D_Coefficient", n, 32)
        self._bus.enable_torque()
        logger.info(f"[head] 已连接 {self.port}")

    def disconnect(self) -> None:
        if self._shared:
            self._bus = None
            self._shared = False
            return
        if self._bus and self._bus.is_connected:
            try:
                self._bus.disconnect()
            except Exception as e:
                logger.warning(f"[head] 断开时异常 (已忽略): {e}")
                self._bus = None
        self._bus = None

    def read_angle_raw(self) -> dict[str, int]:
        """逐个读取避免一个舵机无响应时全部失败"""
        self._check()
        result = {}
        for name in HEAD_MOTOR_IDS:
            try:
                pos = self._bus.sync_read("Present_Position", [name], normalize=False)
                result[name] = pos[name]
            except Exception as e:
                logger.warning(f"[head] 读取 {name} 失败: {e}")
                result[name] = 2047
        return result

    def read_angle(self) -> dict[str, float]:
        raw = self.read_angle_raw()
        return {n: (v - 2047) * 360.0 / 4096.0 for n, v in raw.items()}

    def set_angle(self, pan: Optional[float] = None,
                  tilt: Optional[float] = None,
                  wait: bool = False, timeout: float = 3.0) -> None:
        """设置云台角度 (度, 0=中位)

        pan:  正=左转, 负=右转
        tilt: 正=上抬, 负=下压
        """
        self._check()
        target = {}
        if pan is not None:
            target["head_pan"] = max(0, min(4095, int(pan * 4096 / 360 + 2047)))
        if tilt is not None:
            target["head_tilt"] = max(0, min(4095, int(tilt * 4096 / 360 + 2047)))
        if not target:
            return
        # 逐个写入，避免一个舵机掉线拖累另一个
        for name, val in target.items():
            try:
                self._bus.sync_write("Goal_Position", {name: val}, normalize=False)
            except Exception as e:
                logger.warning(f"[head] 写入 {name} 失败: {e}")
        if wait:
            t0 = time.time()
            while time.time() - t0 < timeout:
                try:
                    pos = self.read_angle_raw()
                    if all(abs(pos.get(k, 2047) - v) < 40
                           for k, v in target.items()):
                        return
                except Exception:
                    pass
                time.sleep(0.1)

    def set_angle_raw(self, target: dict[str, int]) -> None:
        self._check()
        for name, val in target.items():
            try:
                self._bus.sync_write("Goal_Position", {name: val}, normalize=False)
            except Exception as e:
                logger.warning(f"[head] 写入 {name} 失败: {e}")

    def look_center(self) -> None:
        """渐进回中: 分步移动避免大幅跳变导致过载"""
        raw = self.read_angle_raw()
        steps = 5
        for i in range(1, steps + 1):
            interp = {}
            for name in HEAD_MOTOR_IDS:
                cur = raw.get(name, 2047)
                interp[name] = int(cur + (2047 - cur) * i / steps)
            self.set_angle_raw(interp)
            time.sleep(0.15)

    def scan(self, angle_range: float = 60.0, steps: int = 5,
             pause: float = 0.5) -> list[float]:
        half = angle_range / 2.0
        angles = [-half + i * angle_range / max(steps - 1, 1)
                  for i in range(steps)]
        for a in angles:
            self.set_angle(pan=a)
            time.sleep(pause)
        return angles

    def _check(self):
        if not self.is_connected:
            raise RuntimeError("[head] 未连接")


# ═══════════════════════════════════════════════════════════════════════════════
# 底盘控制器 (ttyACM0 ID 7/8/9)
# ═══════════════════════════════════════════════════════════════════════════════

class BaseController:
    """三轮全向底盘: 三角布局, 速度模式, 与左臂共用 ttyACM0"""

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
            self._bus.write("Torque_Enable", n, 0)
            self._bus.write("Operating_Mode", n, OperatingMode.VELOCITY.value)
            self._bus.write("Torque_Enable", n, 1)
        logger.info("[base] 绑定共享总线, 速度模式就绪")

    def connect(self) -> None:
        if self.is_connected or self._shared:
            return
        motors = {n: Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
                  for n, mid in BASE_MOTOR_IDS.items()}
        self._bus = _make_bus(self.port, motors)
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
        if self._bus and self._bus.is_connected:
            try:
                self._bus.disconnect()
            except Exception as e:
                logger.warning(f"[base] 断开时异常 (已忽略): {e}")
        self._bus = None

    def move(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self._check()
        wheel_cmds = self._body_to_wheel_raw(x, y, theta)
        self._bus.sync_write("Goal_Velocity", wheel_cmds)

    def stop(self) -> None:
        if not self.is_connected:
            return
        try:
            self._bus.sync_write("Goal_Velocity",
                                 {n: 0 for n in BASE_MOTOR_IDS})
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
        names = ["base_left_wheel", "base_back_wheel", "base_right_wheel"]
        return {n: max(-0x8000, min(0x7FFF, int(round(d * steps_per_deg))))
                for n, d in zip(names, wheel_degps)}

    def _check(self):
        if not self.is_connected:
            raise RuntimeError("[base] 未连接")


# ═══════════════════════════════════════════════════════════════════════════════
# 摄像头管理
# ═══════════════════════════════════════════════════════════════════════════════

class CameraManager:
    def __init__(self, cam_a_index=CAMERA_A_INDEX, cam_b_index=CAMERA_B_INDEX):
        self.indices = {"cam_a": cam_a_index, "cam_b": cam_b_index}
        self._caps: dict[str, cv2.VideoCapture] = {}

    @property
    def is_connected(self) -> bool:
        return len(self._caps) > 0 and all(c.isOpened() for c in self._caps.values())

    def connect(self, cameras: Optional[list[str]] = None) -> None:
        for name in (cameras or list(self.indices.keys())):
            idx = self.indices[name]
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                logger.warning(f"[camera] 无法打开 {name} (/dev/video{idx})")
                continue
            self._caps[name] = cap
            ret, frame = cap.read()
            if ret:
                logger.info(f"[camera] {name} 已连接 {frame.shape[1]}x{frame.shape[0]}")

    def disconnect(self) -> None:
        for cap in self._caps.values():
            cap.release()
        self._caps.clear()

    def capture(self, camera: str = "cam_a") -> Optional[np.ndarray]:
        cap = self._caps.get(camera)
        if cap is None or not cap.isOpened():
            return None
        ret, frame = cap.read()
        return frame if ret else None

    def capture_all(self) -> dict[str, np.ndarray]:
        return {n: f for n in self._caps if (f := self.capture(n)) is not None}

    def save_snapshot(self, camera="cam_a", path=None) -> Optional[str]:
        frame = self.capture(camera)
        if frame is None:
            return None
        if path is None:
            path = f"/home/makermods/dexproject/{camera}_{int(time.time())}.jpg"
        cv2.imwrite(path, frame)
        logger.info(f"[camera] 已保存 {path}")
        return path


# ═══════════════════════════════════════════════════════════════════════════════
# 统一机器人接口
# ═══════════════════════════════════════════════════════════════════════════════

class RobotSkills:
    """XLeRobot 统一控制接口

    自动处理总线共享:
      ttyACM0: 左臂(ID1-6) + 底盘(ID7-9) → 一个 bus
      ttyACM1: 右臂(ID1-6) + 云台(ID7-8) → 一个 bus

    典型用法:
        robot = RobotSkills()
        robot.connect()
        robot.head.set_angle(pan=30, tilt=-10)
        robot.left_arm.set_gripper(100)
        robot.base.move(x=0.1)
        robot.disconnect()
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
        # ttyACM0: 左臂 + 底盘共用 bus
        if self.left_arm:
            extra = BASE_MOTOR_IDS if self._enable_base else None
            self.left_arm.connect(extra_motors=extra)
        if self.base and self._enable_base:
            if self.left_arm and self.left_arm.is_connected:
                self.base._attach_shared_bus(self.left_arm._bus)
            else:
                self.base.connect()

        # ttyACM1: 右臂 + 云台共用 bus
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


# ═══════════════════════════════════════════════════════════════════════════════
# 校准辅助
# ═══════════════════════════════════════════════════════════════════════════════

def run_calibration(port: str, robot_id: str = "default") -> None:
    import subprocess
    cmd = [sys.executable, "-m",
           "lerobot.scripts.lerobot_measure_feetech_ranges",
           "--port", port, "--save", "--robot-id", robot_id]
    logger.info(f"运行校准: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)


# ═══════════════════════════════════════════════════════════════════════════════
# 自检 / 测试
# ═══════════════════════════════════════════════════════════════════════════════

def _test_cameras():
    print(f"\n{'='*60}\n测试: 摄像头\n{'='*60}")
    cam = CameraManager()
    cam.connect()
    if not cam.is_connected:
        print("  ✗ 没有可用摄像头"); return
    for n in cam._caps:
        p = cam.save_snapshot(n)
        print(f"  {'✓' if p else '✗'} {n} → {p or '失败'}")
    cam.disconnect()


def _test_arm(port, name):
    print(f"\n{'='*60}\n测试: {name} ({port})\n{'='*60}")
    arm = ArmController(port, name)
    try:
        arm.connect()
        print(f"  ✓ 连接成功")
        print(f"  位置: {arm.read_position()}")
        print(f"  原始: {arm.read_position_raw()}")
        print("  释放扭矩 3 秒...")
        arm.disable_torque()
        time.sleep(3)
        arm.enable_torque()
        print(f"  重新锁定: {arm.read_position()}")
    except Exception as e:
        print(f"  ✗ {e}")
    finally:
        arm.disconnect()


def _test_head(port):
    print(f"\n{'='*60}\n测试: 头部云台 ({port})\n{'='*60}")
    head = HeadController(port)
    try:
        head.connect()
        angles = head.read_angle()
        print(f"  ✓ 连接成功, 当前角度: {angles}")

        print("  渐进回中...")
        head.look_center()
        time.sleep(0.5)

        # 分别测试每个轴
        print("  --- 测试 Pan (水平) ---")
        for desc, p in [("左转20°", 20), ("右转20°", -20), ("回中", 0)]:
            print(f"    {desc}...")
            try:
                head.set_angle(pan=p, wait=True, timeout=2.0)
                time.sleep(0.3)
            except Exception as e:
                print(f"    ⚠ {e}")

        print("  --- 测试 Tilt (俯仰) ---")
        for desc, t in [("上抬15°", 15), ("下压15°", -15), ("回中", 0)]:
            print(f"    {desc}...")
            try:
                head.set_angle(tilt=t, wait=True, timeout=2.0)
                time.sleep(0.3)
            except Exception as e:
                print(f"    ⚠ {e}")

        print("  ✓ 云台测试完成")
    except Exception as e:
        print(f"  ✗ {e}")
    finally:
        head.disconnect()


def _test_base(port):
    print(f"\n{'='*60}\n测试: 底盘 ({port})\n{'='*60}")
    base = BaseController(port)
    try:
        base.connect()
        print("  ✓ 连接成功")
        for desc, x, y, th in [("前进", 0.1, 0, 0), ("左移", 0, 0.1, 0),
                                ("旋转", 0, 0, 30)]:
            print(f"  {desc} 1秒...")
            base.move_for(x=x, y=y, theta=th, duration=1.0)
        print("  ✓ 底盘测试完成")
    except Exception as e:
        print(f"  ✗ {e}")
    finally:
        base.disconnect()


def main():
    print(f"{'='*60}\nXLeRobot 硬件自检\n{'='*60}\n")
    ports = sorted(glob.glob("/dev/ttyACM*"))
    print(f"串口: {', '.join(ports)}\n")

    tests = {
        "1": ("摄像头", _test_cameras),
        "2": ("左臂从臂 (ACM0)", lambda: _test_arm(PORT_LEFT_FOLLOWER, "left_follower")),
        "3": ("右臂从臂 (ACM1)", lambda: _test_arm(PORT_RIGHT_FOLLOWER, "right_follower")),
        "4": ("头部云台 (ACM1 ID7/8)", lambda: _test_head(PORT_RIGHT_FOLLOWER)),
        "5": ("底盘三轮 (ACM0 ID7/8/9)", lambda: _test_base(PORT_LEFT_FOLLOWER)),
        "6": ("左臂主臂 (ACM2)", lambda: _test_arm(PORT_LEFT_LEADER, "left_leader")),
        "7": ("右臂主臂 (ACM3)", lambda: _test_arm(PORT_RIGHT_LEADER, "right_leader")),
        "8": ("左臂校准", lambda: run_calibration(PORT_LEFT_FOLLOWER, "left")),
        "9": ("右臂校准", lambda: run_calibration(PORT_RIGHT_FOLLOWER, "right")),
        "a": ("全部测试", None),
        "q": ("退出", None),
    }
    for k, (d, _) in tests.items():
        print(f"  [{k}] {d}")
    print()
    c = input("请选择: ").strip().lower()

    if c == "q":
        return
    elif c == "a":
        _test_cameras()
        _test_arm(PORT_LEFT_FOLLOWER, "left_follower")
        _test_arm(PORT_RIGHT_FOLLOWER, "right_follower")
        _test_head(PORT_RIGHT_FOLLOWER)
        _test_base(PORT_LEFT_FOLLOWER)
    elif c in tests and tests[c][1]:
        tests[c][1]()


if __name__ == "__main__":
    main()
