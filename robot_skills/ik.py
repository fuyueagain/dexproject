"""SO-101 臂运动学: 正/逆运动学 + 笛卡尔空间控制

坐标系定义 (臂基座坐标系):
    2D 平面 (shoulder_lift + elbow_flex 控制):
        plane_r: 臂在水平面上的投影距离 (前伸, 米)
        plane_z: 高度 (上为正, 米)

    3D 笛卡尔 (加上 shoulder_pan):
        x: 前方 (米)
        y: 左方 (米)
        z: 上方 (米)

连杆参数 (SO-101 实测):
    L1 = 0.1159 m  (大臂: shoulder → elbow)
    L2 = 0.1350 m  (小臂: elbow → wrist)

归一化值与角度关系:
    STS3215 舵机 4096 步 = 360°
    ArmController 归一化范围 [-100, 100] 对应 [-180°, 180°]
    因此: 1 归一化单位 = 1.8°
"""

import math
import logging
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .config import ARM_MOTOR_NAMES

logger = logging.getLogger("robot_skills")

L1 = 0.1159
L2 = 0.1350

NORM_TO_DEG = 1.8
DEG_TO_NORM = 1.0 / NORM_TO_DEG


def norm_to_deg(val: float) -> float:
    return val * NORM_TO_DEG


def deg_to_norm(val: float) -> float:
    return val * DEG_TO_NORM


class SO101Kinematics2D:
    """SO-101 二连杆平面运动学 (shoulder_lift + elbow_flex)

    在臂的矢状面内求解, 输入/输出均为度数 (与舵机角度一致).
    """

    def __init__(self, l1: float = L1, l2: float = L2):
        self.l1 = l1
        self.l2 = l2
        self._theta1_offset = math.atan2(0.028, 0.11257)
        self._theta2_offset = math.atan2(0.0052, 0.1349) + self._theta1_offset

    def forward_kinematics(self, shoulder_lift_deg: float, elbow_flex_deg: float) -> Tuple[float, float]:
        """FK: 关节角度(度) → 平面坐标(米)

        Returns:
            (plane_r, plane_z): 前伸距离和高度
        """
        joint2_rad = math.radians(90.0 - shoulder_lift_deg)
        joint3_rad = math.radians(elbow_flex_deg + 90.0)

        theta1 = joint2_rad - self._theta1_offset
        theta2 = joint3_rad - self._theta2_offset

        r_sq = self.l1 ** 2 + self.l2 ** 2 + 2.0 * self.l1 * self.l2 * math.cos(theta2)
        r = math.sqrt(max(0.0, r_sq))
        gamma = math.atan2(self.l2 * math.sin(theta2), self.l1 + self.l2 * math.cos(theta2))
        beta = theta1 - gamma

        plane_r = r * math.cos(beta)
        plane_z = r * math.sin(beta)
        return plane_r, plane_z

    def inverse_kinematics(self, plane_r: float, plane_z: float) -> Tuple[float, float]:
        """IK: 平面坐标(米) → 关节角度(度)

        Returns:
            (shoulder_lift_deg, elbow_flex_deg)
        """
        r = math.sqrt(plane_r ** 2 + plane_z ** 2)
        r_max = self.l1 + self.l2
        r_min = abs(self.l1 - self.l2)

        if r > r_max:
            scale = r_max / r
            plane_r *= scale
            plane_z *= scale
            r = r_max
        if r < r_min and r > 0:
            scale = r_min / r
            plane_r *= scale
            plane_z *= scale
            r = r_min

        cos_theta2 = -(r ** 2 - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))

        theta2 = math.pi - math.acos(cos_theta2)

        beta = math.atan2(plane_z, plane_r)
        gamma = math.atan2(self.l2 * math.sin(theta2), self.l1 + self.l2 * math.cos(theta2))
        theta1 = beta + gamma

        joint2 = theta1 + self._theta1_offset
        joint3 = theta2 + self._theta2_offset

        joint2 = max(-0.1, min(3.45, joint2))
        joint3 = max(-0.2, min(math.pi, joint3))

        shoulder_lift_deg = 90.0 - math.degrees(joint2)
        elbow_flex_deg = math.degrees(joint3) - 90.0

        return shoulder_lift_deg, elbow_flex_deg


@dataclass
class CartesianPose:
    """末端执行器笛卡尔位姿"""
    x: float = 0.0        # 前方 (米)
    y: float = 0.0        # 左方 (米)
    z: float = 0.0        # 上方 (米)
    pitch: float = 0.0    # 俯仰角 (度)
    roll: float = 0.0     # 横滚角 (度)
    gripper: float = 0.0  # 夹爪开合 (0~100)

    def distance_to(self, other: "CartesianPose") -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2)

    def __repr__(self):
        return (f"CartesianPose(x={self.x:.4f}, y={self.y:.4f}, z={self.z:.4f}, "
                f"pitch={self.pitch:.1f}°, roll={self.roll:.1f}°, gripper={self.gripper:.0f})")


class ArmIKController:
    """基于 2D IK + shoulder_pan 的笛卡尔空间控制器

    所有运动方法默认带 duration 参数, 在笛卡尔空间做直线插值, 确保安全可控.

    典型用法:
        ik = ArmIKController(arm)
        pose = ik.get_cartesian_pose()
        ik.move_cartesian(0.16, 0.0, 0.10, duration=2.0)
        ik.move_cartesian_delta(dx=0.02, duration=1.0)
    """

    DEFAULT_DURATION = 2.0
    DEFAULT_HZ = 20.0

    def __init__(self, arm_controller, auto_wrist_level: bool = True):
        """
        Args:
            arm_controller: ArmController 实例 (已连接)
            auto_wrist_level: 自动耦合 wrist_flex 保持末端水平
        """
        self.arm = arm_controller
        self.kin = SO101Kinematics2D()
        self.auto_wrist_level = auto_wrist_level

    def _read_degrees(self) -> dict[str, float]:
        """读取各关节的角度 (度)"""
        state = self.arm.read_position()
        return {name: norm_to_deg(getattr(state, name)) for name in state.to_dict()}

    def get_cartesian_pose(self) -> CartesianPose:
        """FK: 读取当前关节 → 笛卡尔坐标"""
        deg = self._read_degrees()

        plane_r, plane_z = self.kin.forward_kinematics(deg["shoulder_lift"], deg["elbow_flex"])

        pan_rad = math.radians(deg["shoulder_pan"])
        x = plane_r * math.cos(pan_rad)
        y = plane_r * math.sin(pan_rad)
        z = plane_z

        return CartesianPose(
            x=x, y=y, z=z,
            pitch=deg["wrist_flex"],
            roll=deg["wrist_roll"],
            gripper=deg["gripper"],
        )

    def _solve_ik_full(self, x: float, y: float, z: float,
                       pitch: Optional[float], roll: Optional[float],
                       gripper: Optional[float]) -> dict[str, float]:
        """IK 求解: 笛卡尔 → 关节角度 (度)"""
        pan_deg = math.degrees(math.atan2(y, x)) if abs(x) > 1e-6 or abs(y) > 1e-6 else 0.0
        plane_r = math.sqrt(x ** 2 + y ** 2)

        sl_deg, ef_deg = self.kin.inverse_kinematics(plane_r, z)

        if self.auto_wrist_level:
            wf_deg = -sl_deg - ef_deg + (pitch if pitch is not None else 0.0)
        elif pitch is not None:
            wf_deg = pitch
        else:
            wf_deg = norm_to_deg(self.arm.read_position().wrist_flex)

        target_deg = {
            "shoulder_pan": pan_deg,
            "shoulder_lift": sl_deg,
            "elbow_flex": ef_deg,
            "wrist_flex": wf_deg,
        }
        if roll is not None:
            target_deg["wrist_roll"] = roll
        if gripper is not None:
            target_deg["gripper"] = gripper
        return target_deg

    def move_cartesian(
        self,
        x: float, y: float, z: float,
        pitch: Optional[float] = None,
        roll: Optional[float] = None,
        gripper: Optional[float] = None,
        duration: float = DEFAULT_DURATION,
        hz: float = DEFAULT_HZ,
    ) -> dict[str, float]:
        """笛卡尔空间直线插值移动

        在当前位置和目标位置之间做直线插值, 每步重新求 IK, 保证末端走直线.

        Args:
            x, y, z: 目标位置 (米)
            pitch: 末端俯仰角 (度), None=自动保持水平
            roll: 末端横滚角 (度), None=保持当前值
            gripper: 夹爪开合 (0~100), None=保持当前值
            duration: 移动时长 (秒), 默认 2.0
            hz: 插值频率 (Hz), 默认 20

        Returns:
            最终目标关节角度 (度)
        """
        current_pose = self.get_cartesian_pose()

        target_deg = self._solve_ik_full(x, y, z, pitch, roll, gripper)
        target_norm = {k: deg_to_norm(v) for k, v in target_deg.items()}

        dist = math.sqrt((x - current_pose.x) ** 2 +
                         (y - current_pose.y) ** 2 +
                         (z - current_pose.z) ** 2)

        logger.info(f"[IK] 移动到({x:.4f}, {y:.4f}, {z:.4f}) "
                     f"距离={dist * 100:.1f}cm 时长={duration:.1f}s")

        if dist < 0.001 and duration <= 0:
            self.arm.move_to(target_norm)
            return target_deg

        steps = max(2, int(duration * hz))
        dt = duration / steps
        step_ms = max(50, int(dt * 1000))

        start = np.array([current_pose.x, current_pose.y, current_pose.z])
        end = np.array([x, y, z])

        pitch_start = current_pose.pitch
        pitch_end = pitch if pitch is not None else (0.0 if self.auto_wrist_level else pitch_start)
        roll_start = current_pose.roll
        roll_end = roll if roll is not None else roll_start
        grip_start = current_pose.gripper
        grip_end = gripper if gripper is not None else grip_start

        for i in range(1, steps + 1):
            alpha = i / steps

            pt = start + alpha * (end - start)
            p_interp = pitch_start + alpha * (pitch_end - pitch_start)
            r_interp = roll_start + alpha * (roll_end - roll_start)
            g_interp = grip_start + alpha * (grip_end - grip_start)

            step_deg = self._solve_ik_full(
                float(pt[0]), float(pt[1]), float(pt[2]),
                pitch=p_interp, roll=r_interp, gripper=g_interp,
            )
            step_norm = {k: deg_to_norm(v) for k, v in step_deg.items()}
            try:
                self.arm.move_to(step_norm, duration_ms=step_ms)
            except Exception as e:
                logger.warning(f"[IK] 插值步 {i}/{steps} 写入失败: {e}")
            time.sleep(dt)

        return target_deg

    def move_cartesian_delta(
        self,
        dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
        dpitch: float = 0.0,
        duration: float = DEFAULT_DURATION,
        **kwargs,
    ) -> dict[str, float]:
        """在当前位置上做增量移动

        Args:
            dx, dy, dz: 位移增量 (米)
            dpitch: 俯仰增量 (度)
            duration: 移动时长 (秒)
            **kwargs: 传递给 move_cartesian 的其他参数
        """
        pose = self.get_cartesian_pose()
        return self.move_cartesian(
            x=pose.x + dx,
            y=pose.y + dy,
            z=pose.z + dz,
            pitch=pose.pitch + dpitch,
            duration=duration,
            **kwargs,
        )

    def move_to_home(self, duration: float = 3.0) -> dict[str, float]:
        """回到零位 (所有关节归零)"""
        home = {n: 0.0 for n in ARM_MOTOR_NAMES}
        self.arm.move_smooth(home, duration=duration)
        return {n: 0.0 for n in ARM_MOTOR_NAMES}

    # ── 可达性分析 ──

    JOINT_NORM_LIMIT = 90.0

    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """判断笛卡尔位置是否在可达工作空间内

        检查条件:
          1. 平面距离在 [r_min, r_max] 范围内 (留 5% 余量)
          2. shoulder_pan 不超限
          3. shoulder_lift / elbow_flex / wrist_flex 不超限
        """
        pan_deg = math.degrees(math.atan2(y, x)) if abs(x) > 1e-6 or abs(y) > 1e-6 else 0.0
        if abs(deg_to_norm(pan_deg)) > self.JOINT_NORM_LIMIT:
            return False

        plane_r = math.sqrt(x ** 2 + y ** 2)
        r = math.sqrt(plane_r ** 2 + z ** 2)
        r_max = self.kin.l1 + self.kin.l2
        r_min = abs(self.kin.l1 - self.kin.l2)
        if r > r_max * 0.95 or (r < r_min * 1.1 and r_min > 0.001):
            return False

        sl_deg, ef_deg = self.kin.inverse_kinematics(plane_r, z)
        if abs(deg_to_norm(sl_deg)) > self.JOINT_NORM_LIMIT:
            return False
        if abs(deg_to_norm(ef_deg)) > self.JOINT_NORM_LIMIT:
            return False

        if self.auto_wrist_level:
            wf_deg = -sl_deg - ef_deg
            if abs(deg_to_norm(wf_deg)) > self.JOINT_NORM_LIMIT:
                return False

        return True

    def get_feasible_range(self, step: float = 0.002, max_steps: int = 150,
                           pose: Optional[CartesianPose] = None) -> dict:
        """计算当前位姿下各轴方向的可行增量范围

        沿 ±x, ±y, ±z 六个方向逐步探测, 找到每个方向的最大可行增量.

        Args:
            step: 探测步长 (米), 默认 2mm
            max_steps: 每个方向最大探测步数
            pose: 指定位姿, None=读取当前

        Returns:
            {
                "dx": (min, max),   # 可行的 dx 范围 (米)
                "dy": (min, max),
                "dz": (min, max),
                "current": CartesianPose,
                "description": str,  # 人类可读的摘要
            }
        """
        if pose is None:
            pose = self.get_cartesian_pose()

        axes = {
            "dx": (1.0, 0.0, 0.0),
            "dy": (0.0, 1.0, 0.0),
            "dz": (0.0, 0.0, 1.0),
        }

        result = {"current": pose}

        for axis_name, (ax, ay, az) in axes.items():
            neg_limit = 0.0
            pos_limit = 0.0

            for sign in (-1.0, +1.0):
                limit = 0.0
                for i in range(1, max_steps + 1):
                    d = sign * i * step
                    tx = pose.x + ax * d
                    ty = pose.y + ay * d
                    tz = pose.z + az * d
                    if self.is_reachable(tx, ty, tz):
                        limit = d
                    else:
                        break

                if sign < 0:
                    neg_limit = limit
                else:
                    pos_limit = limit

            result[axis_name] = (neg_limit, pos_limit)

        dx_neg, dx_pos = result["dx"]
        dy_neg, dy_pos = result["dy"]
        dz_neg, dz_pos = result["dz"]
        result["description"] = (
            f"当前位姿: x={pose.x:.4f}, y={pose.y:.4f}, z={pose.z:.4f}\n"
            f"可行增量范围 (米):\n"
            f"  dx: [{dx_neg:+.3f}, {dx_pos:+.3f}]  ({(dx_pos - dx_neg) * 100:.1f} cm)\n"
            f"  dy: [{dy_neg:+.3f}, {dy_pos:+.3f}]  ({(dy_pos - dy_neg) * 100:.1f} cm)\n"
            f"  dz: [{dz_neg:+.3f}, {dz_pos:+.3f}]  ({(dz_pos - dz_neg) * 100:.1f} cm)"
        )

        return result

    def check_delta(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                    pose: Optional[CartesianPose] = None) -> dict:
        """检查一个增量移动是否可行, 并返回详情

        Returns:
            {
                "reachable": bool,
                "target": (x, y, z),
                "target_joints_deg": dict or None,
                "max_joint_change_deg": float,   # 最大关节变化量
                "reason": str,                   # 不可达时的原因
            }
        """
        if pose is None:
            pose = self.get_cartesian_pose()

        tx, ty, tz = pose.x + dx, pose.y + dy, pose.z + dz
        info = {
            "target": (tx, ty, tz),
            "target_joints_deg": None,
            "max_joint_change_deg": 0.0,
            "reason": "",
        }

        if not self.is_reachable(tx, ty, tz):
            info["reachable"] = False
            plane_r = math.sqrt(tx ** 2 + ty ** 2)
            r = math.sqrt(plane_r ** 2 + tz ** 2)
            r_max = self.kin.l1 + self.kin.l2
            r_min = abs(self.kin.l1 - self.kin.l2)
            if r > r_max * 0.95:
                info["reason"] = f"超出最大伸展 (距离 {r:.4f}m > 上限 {r_max * 0.95:.4f}m)"
            elif r < r_min * 1.1:
                info["reason"] = f"低于最小伸展 (距离 {r:.4f}m < 下限 {r_min * 1.1:.4f}m)"
            else:
                info["reason"] = "关节角度超限"
            return info

        target_deg = self._solve_ik_full(tx, ty, tz, pitch=None, roll=None, gripper=None)
        info["target_joints_deg"] = target_deg

        current_deg = {
            "shoulder_pan": math.degrees(math.atan2(pose.y, pose.x)) if abs(pose.x) > 1e-6 or abs(pose.y) > 1e-6 else 0.0,
        }
        plane_r_cur = math.sqrt(pose.x ** 2 + pose.y ** 2)
        sl_cur, ef_cur = self.kin.inverse_kinematics(plane_r_cur, pose.z)
        current_deg["shoulder_lift"] = sl_cur
        current_deg["elbow_flex"] = ef_cur

        max_change = max(
            abs(target_deg.get(k, 0) - current_deg.get(k, 0))
            for k in ["shoulder_pan", "shoulder_lift", "elbow_flex"]
        )
        info["max_joint_change_deg"] = max_change
        info["reachable"] = True
        info["reason"] = "可达"
        if max_change > 45:
            info["reason"] = f"可达, 但关节变化较大 ({max_change:.1f}°), 建议增大 duration"

        return info

    def get_workspace_info(self) -> dict:
        """返回工作空间参数"""
        r_max = self.kin.l1 + self.kin.l2
        r_min = abs(self.kin.l1 - self.kin.l2)
        return {
            "l1": self.kin.l1,
            "l2": self.kin.l2,
            "reach_max": r_max,
            "reach_min": r_min,
            "description": (
                f"SO-101 工作空间:\n"
                f"  大臂 L1={self.kin.l1:.4f}m, 小臂 L2={self.kin.l2:.4f}m\n"
                f"  平面最大伸展: {r_max:.4f}m\n"
                f"  平面最小伸展: {r_min:.4f}m\n"
                f"  shoulder_pan 旋转范围: ±~100° (归一化 ±~56)\n"
            ),
        }
