"""SO-101 完整 5DOF 运动学 — 基于 URDF 变换链

从 xlerobot.urdf 提取的精确几何参数, 纯 numpy 实现, 零外部依赖。

运动链: Base → shoulder_pan → shoulder_lift → elbow_flex
         → wrist_flex → wrist_roll → Fixed_Jaw_tip

IK 始终以 6D 目标 (xyz + rpy) 求解, 通过 pos_weight / ori_weight
控制位置与姿态的优先级 (5DOF 不能完全独立满足 6D, DLS 自动折中).

工作坐标系: x=前方, y=左方, z=上方
"""

import math
import logging
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .config import ARM_MOTOR_NAMES

logger = logging.getLogger("robot_skills")

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# URDF 几何参数 (从 xlerobot.urdf 右臂提取)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

_PI = math.pi
_H = _PI / 2

_JOINT_DEFS = [
    ([0.0, -0.0452, 0.0165], [_H, 0, 0],  [0, -1, 0]),   # shoulder_pan
    ([0.0,  0.1025, 0.0306], [_H, 0, 0],  [-1, 0, 0]),   # shoulder_lift
    ([0.0,  0.11257, 0.028], [-_H, 0, 0], [1, 0, 0]),    # elbow_flex
    ([0.0,  0.0052, 0.1349], [-_H, 0, 0], [1, 0, 0]),    # wrist_flex
    ([0.0, -0.0601, 0.0],    [0, _H, 0],  [0, -1, 0]),   # wrist_roll
]

_EE_TIP_XYZ = np.array([0.01, -0.097, 0.0])

_JOINT_LIMITS = np.array([
    [-2.1,    2.1],      # shoulder_pan
    [-0.1,    3.45],     # shoulder_lift
    [-0.2,    _PI],      # elbow_flex
    [-1.8,    1.8],      # wrist_flex
    [-_PI,    _PI],      # wrist_roll
])

IK_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
                   "wrist_flex", "wrist_roll"]

NORM_TO_DEG = 1.8
DEG_TO_NORM = 1.0 / NORM_TO_DEG

# 工作坐标系 ↔ URDF Base 坐标系
#   work_x = -urdf_y (前), work_y = urdf_x (左), work_z = urdf_z (上)
_R_U2W = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
_R_W2U = _R_U2W.T


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 齐次变换工具
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    """RPY (弧度) → 3×3 旋转矩阵  (Rz·Ry·Rx)"""
    return _rz(y) @ _ry(p) @ _rx(r)


def _R_to_rpy(R: np.ndarray):
    """3×3 旋转矩阵 → (roll, pitch, yaw) 弧度"""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def _make_T(xyz, rpy=(0, 0, 0)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def _rot_axis_T(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues: 绕单位轴旋转 → 4×4"""
    a = axis / np.linalg.norm(axis)
    c, s = math.cos(angle), math.sin(angle)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) * c + (1 - c) * np.outer(a, a) + s * K
    T = np.eye(4)
    T[:3, :3] = R
    return T


def _rot_to_axis_angle(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 → 轴角向量 (3维, 模=角度)"""
    cos_a = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    angle = math.acos(cos_a)
    if abs(angle) < 1e-10:
        return np.zeros(3)
    if abs(angle - _PI) < 1e-6:
        _, vecs = np.linalg.eigh(R)
        return vecs[:, np.argmax(_)] * angle
    axis = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]]) / (2.0 * math.sin(angle))
    return axis * angle


def _slerp(R0: np.ndarray, R1: np.ndarray, t: float) -> np.ndarray:
    """球面线性插值: R0 → R1, t ∈ [0,1]"""
    R_rel = R0.T @ R1
    aa = _rot_to_axis_angle(R_rel)
    angle = np.linalg.norm(aa)
    if angle < 1e-10:
        return R0.copy()
    axis = aa / angle
    return R0 @ _rot_axis_T(axis, t * angle)[:3, :3]


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# Servo ↔ URDF 转换
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def servo_deg_to_urdf(servo_deg) -> np.ndarray:
    d = np.asarray(servo_deg, dtype=float)
    return np.array([
        -math.radians(d[0]),
         math.radians(90.0 - d[1]),
         math.radians(d[2] + 90.0),
         math.radians(d[3]),
        -math.radians(d[4]),
    ])


def urdf_to_servo_deg(q_urdf) -> np.ndarray:
    q = np.asarray(q_urdf, dtype=float)
    return np.array([
        -math.degrees(q[0]),
         90.0 - math.degrees(q[1]),
         math.degrees(q[2]) - 90.0,
         math.degrees(q[3]),
        -math.degrees(q[4]),
    ])


def norm_to_urdf(norm_values) -> np.ndarray:
    return servo_deg_to_urdf(np.asarray(norm_values, dtype=float) * NORM_TO_DEG)


def urdf_to_norm(q_urdf) -> np.ndarray:
    return urdf_to_servo_deg(q_urdf) * DEG_TO_NORM


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 工作坐标系 ↔ URDF 坐标系 (位置 + 旋转)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _T_urdf_to_work(T_urdf: np.ndarray) -> np.ndarray:
    """URDF 齐次变换 → 工作坐标系齐次变换"""
    Tw = np.eye(4)
    Tw[:3, :3] = _R_U2W @ T_urdf[:3, :3]
    Tw[:3, 3] = _R_U2W @ T_urdf[:3, 3]
    return Tw


def _T_work_to_urdf(T_work: np.ndarray) -> np.ndarray:
    """工作坐标系齐次变换 → URDF 齐次变换"""
    Tu = np.eye(4)
    Tu[:3, :3] = _R_W2U @ T_work[:3, :3]
    Tu[:3, 3] = _R_W2U @ T_work[:3, 3]
    return Tu


def pose6d_to_T(x, y, z, roll_deg, pitch_deg, yaw_deg) -> np.ndarray:
    """6D 位姿 (工作坐标系, 度) → 4×4 齐次变换 (工作坐标系)"""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_R(math.radians(roll_deg),
                           math.radians(pitch_deg),
                           math.radians(yaw_deg))
    T[:3, 3] = [x, y, z]
    return T


def T_to_pose6d(T: np.ndarray):
    """4×4 齐次变换 (工作坐标系) → (x, y, z, roll°, pitch°, yaw°)"""
    r, p, y = _R_to_rpy(T[:3, :3])
    return (*T[:3, 3], math.degrees(r), math.degrees(p), math.degrees(y))


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SO101 运动学核心
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class SO101Kinematics:
    """SO-101 五自由度运动学求解器 (URDF 坐标系)"""

    def __init__(self):
        self._statics = []
        for xyz, rpy, axis in _JOINT_DEFS:
            self._statics.append((_make_T(xyz, rpy), np.array(axis, dtype=float)))
        self._T_ee = np.eye(4)
        self._T_ee[:3, 3] = _EE_TIP_XYZ

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """FK: 5 URDF 弧度 → 4×4 齐次变换 (URDF 坐标系)"""
        T = np.eye(4)
        for i, (T_s, axis) in enumerate(self._statics):
            T = T @ T_s @ _rot_axis_T(axis, q[i])
        return T @ self._T_ee

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """几何 Jacobian 6×5 (URDF 坐标系)"""
        T = np.eye(4)
        z_list, p_list = [], []
        for i, (T_s, axis) in enumerate(self._statics):
            T = T @ T_s
            z_list.append(T[:3, :3] @ axis)
            p_list.append(T[:3, 3].copy())
            T = T @ _rot_axis_T(axis, q[i])

        p_ee = (T @ self._T_ee)[:3, 3]
        J = np.zeros((6, 5))
        for i in range(5):
            J[:3, i] = np.cross(z_list[i], p_ee - p_list[i])
            J[3:, i] = z_list[i]
        return J

    def inverse_kinematics(
        self,
        q_init: np.ndarray,
        T_target: np.ndarray,
        pos_weight: float = 1.0,
        ori_weight: float = 0.2,
        max_iter: int = 200,
        pos_tol: float = 5e-4,
        ori_tol: float = 1e-2,
        damping: float = 0.01,
        step_scale: float = 0.5,
    ) -> np.ndarray:
        """6D IK: Weighted DLS + 关节限位

        始终同时追踪位置和姿态. 通过 pos_weight / ori_weight 控制优先级.
        5DOF 无法完全满足 6D, DLS 自动找最优折中.

        Args:
            q_init:     初始关节角 (URDF 弧度, 5维)
            T_target:   目标 4×4 齐次变换 (URDF 坐标系)
            pos_weight: 位置误差权重 (默认 1.0)
            ori_weight: 姿态误差权重 (默认 0.2, 低于位置以保证位置优先)
            max_iter:   最大迭代
            pos_tol:    位置收敛门限 (米)
            ori_tol:    姿态收敛门限 (弧度)
            damping:    DLS 阻尼
            step_scale: 步长缩放

        Returns:
            q: 求解的关节角 (URDF 弧度, 5维)
        """
        q = np.array(q_init, dtype=float)
        W = np.diag([pos_weight] * 3 + [ori_weight] * 3)

        for _ in range(max_iter):
            T_cur = self.forward_kinematics(q)

            pos_err = T_target[:3, 3] - T_cur[:3, 3]
            R_err = T_target[:3, :3] @ T_cur[:3, :3].T
            ori_err = _rot_to_axis_angle(R_err)

            if (np.linalg.norm(pos_err) < pos_tol and
                    np.linalg.norm(ori_err) < ori_tol):
                break

            dx = np.concatenate([pos_err, ori_err])
            J = self.jacobian(q)

            Jw = W @ J
            dxw = W @ dx
            JJT = Jw @ Jw.T
            dq = Jw.T @ np.linalg.solve(JJT + damping ** 2 * np.eye(6), dxw)

            q = q + step_scale * dq
            np.clip(q, _JOINT_LIMITS[:, 0], _JOINT_LIMITS[:, 1], out=q)

        return q

    def get_ee_position(self, q: np.ndarray) -> np.ndarray:
        return self.forward_kinematics(q)[:3, 3].copy()

    def get_ee_pose(self, q: np.ndarray):
        T = self.forward_kinematics(q)
        return T[:3, 3].copy(), T[:3, :3].copy()


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# CartesianPose
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

@dataclass
class CartesianPose:
    """末端 6D 位姿 (工作坐标系: x=前, y=左, z=上)"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0     # 度
    pitch: float = 0.0    # 度
    yaw: float = 0.0      # 度
    gripper: float = 0.0  # 0~100

    def pos(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def rpy_deg(self) -> np.ndarray:
        return np.array([self.roll, self.pitch, self.yaw])

    def rpy_rad(self) -> np.ndarray:
        return np.radians(self.rpy_deg())

    def to_T(self) -> np.ndarray:
        """→ 4×4 齐次变换 (工作坐标系)"""
        return pose6d_to_T(self.x, self.y, self.z,
                           self.roll, self.pitch, self.yaw)

    @classmethod
    def from_T(cls, T: np.ndarray, gripper: float = 0.0) -> "CartesianPose":
        x, y, z, r, p, ya = T_to_pose6d(T)
        return cls(x=x, y=y, z=z, roll=r, pitch=p, yaw=ya, gripper=gripper)

    def distance_to(self, other: "CartesianPose") -> float:
        return float(np.linalg.norm(self.pos() - other.pos()))

    def __repr__(self):
        return (f"CartesianPose(x={self.x:.4f}, y={self.y:.4f}, z={self.z:.4f}, "
                f"rpy=({self.roll:.1f}°,{self.pitch:.1f}°,{self.yaw:.1f}°), "
                f"gripper={self.gripper:.0f})")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 高级控制器 (驱动真实机器人)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class ArmIKController:
    """6D 笛卡尔空间控制器 (xyz + rpy)

    所有位姿使用工作坐标系 (x=前, y=左, z=上), 角度为度数.
    IK 始终同时追踪位置+姿态, 通过权重控制优先级.

    典型用法:
        ik = ArmIKController(arm)
        pose = ik.get_cartesian_pose()

        # 绝对 6D 移动
        ik.move_to(x=0.35, y=0, z=0.2, roll=-90, pitch=90, yaw=0)

        # 增量 6D 移动
        ik.move_delta(dx=0.02, dz=0.03)            # 只移位置
        ik.move_delta(droll=10, dpitch=-5)          # 只转姿态
        ik.move_delta(dx=0.02, droll=10, dz=-0.01)  # 同时位置+姿态
    """

    DEFAULT_DURATION = 2.0
    DEFAULT_HZ = 20.0
    POSITION_THRESHOLD_M = 0.005
    ORIENTATION_THRESHOLD_DEG = 5.0

    def __init__(self, arm_controller,
                 pos_weight: float = 1.0,
                 ori_weight: float = 0.2):
        """
        Args:
            arm_controller: ArmController 实例 (已连接)
            pos_weight: IK 位置误差权重
            ori_weight: IK 姿态误差权重 (0=仅位置, 越大姿态越优先)
        """
        self.arm = arm_controller
        self.kin = SO101Kinematics()
        self.pos_weight = pos_weight
        self.ori_weight = ori_weight

    # ── 内部读写 ──

    def _read_q(self) -> np.ndarray:
        state = self.arm.read_position()
        norms = np.array([getattr(state, n) for n in IK_JOINT_NAMES])
        return norm_to_urdf(norms)

    def _write_q(self, q_urdf: np.ndarray, duration_ms: int = 0):
        norms = urdf_to_norm(q_urdf)
        target = {n: float(norms[i]) for i, n in enumerate(IK_JOINT_NAMES)}
        self.arm.move_to(target, duration_ms=duration_ms)

    def _solve_ik(self, T_target_urdf: np.ndarray,
                  q_init: np.ndarray, max_iter: int = 200) -> np.ndarray:
        return self.kin.inverse_kinematics(
            q_init, T_target_urdf,
            pos_weight=self.pos_weight,
            ori_weight=self.ori_weight,
            max_iter=max_iter)

    # ── FK ──

    def get_cartesian_pose(self) -> CartesianPose:
        """读取当前关节 → 6D 位姿 (工作坐标系)"""
        q = self._read_q()
        T_urdf = self.kin.forward_kinematics(q)
        T_work = _T_urdf_to_work(T_urdf)
        grip = getattr(self.arm.read_position(), "gripper", 0.0)
        return CartesianPose.from_T(T_work, gripper=grip)

    def fk_from_servo(self, servo_deg) -> CartesianPose:
        """纯计算: servo 度数 → 6D 位姿 (不读硬件)"""
        q = servo_deg_to_urdf(servo_deg)
        T_work = _T_urdf_to_work(self.kin.forward_kinematics(q))
        return CartesianPose.from_T(T_work)

    # ── 6D 绝对移动 ──

    def move_to(
        self,
        x: float, y: float, z: float,
        roll: float, pitch: float, yaw: float,
        gripper: Optional[float] = None,
        duration: float = DEFAULT_DURATION,
        hz: float = DEFAULT_HZ,
    ) -> dict[str, float]:
        """6D 绝对移动: 位置 (m) + 姿态 (度)

        末端沿直线移动, 姿态做 SLERP 球面插值, 每步重新 IK.

        Returns:
            最终关节角 (servo 度数)
        """
        q_cur = self._read_q()
        T_cur_urdf = self.kin.forward_kinematics(q_cur)

        T_target_work = pose6d_to_T(x, y, z, roll, pitch, yaw)
        T_target_urdf = _T_work_to_urdf(T_target_work)

        q_final = self._solve_ik(T_target_urdf, q_cur)

        T_cur_work = _T_urdf_to_work(T_cur_urdf)
        cur_pos = T_cur_work[:3, 3]
        tgt_pos = T_target_work[:3, 3]
        dist = np.linalg.norm(tgt_pos - cur_pos)
        logger.info(f"[IK6D] → ({x:.3f},{y:.3f},{z:.3f}|"
                    f"r{roll:.0f},p{pitch:.0f},y{yaw:.0f}) "
                    f"d={dist*100:.1f}cm t={duration:.1f}s")

        return self._execute_trajectory(
            T_cur_urdf, T_target_urdf, q_cur, q_final,
            gripper, duration, hz)

    # ── 6D 增量移动 ──

    def move_delta(
        self,
        dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
        droll: float = 0.0, dpitch: float = 0.0, dyaw: float = 0.0,
        gripper: Optional[float] = None,
        duration: float = DEFAULT_DURATION,
        hz: float = DEFAULT_HZ,
    ) -> dict[str, float]:
        """6D 增量移动: 位置增量 (m) + 姿态增量 (度)

        Returns:
            最终关节角 (servo 度数)
        """
        p = self.get_cartesian_pose()
        return self.move_to(
            x=p.x + dx, y=p.y + dy, z=p.z + dz,
            roll=p.roll + droll,
            pitch=p.pitch + dpitch,
            yaw=p.yaw + dyaw,
            gripper=gripper, duration=duration, hz=hz)

    # ── 兼容旧 API ──

    def move_cartesian(self, x, y, z,
                       roll=None, pitch=None, yaw=None, **kw):
        """兼容旧接口: None 表示保持当前姿态"""
        p = self.get_cartesian_pose()
        return self.move_to(
            x=x, y=y, z=z,
            roll=roll if roll is not None else p.roll,
            pitch=pitch if pitch is not None else p.pitch,
            yaw=yaw if yaw is not None else p.yaw, **kw)

    def move_cartesian_delta(self, dx=0.0, dy=0.0, dz=0.0,
                             droll=0.0, dpitch=0.0, dyaw=0.0, **kw):
        """兼容旧接口"""
        return self.move_delta(dx=dx, dy=dy, dz=dz,
                               droll=droll, dpitch=dpitch, dyaw=dyaw, **kw)

    # ── 轨迹执行 ──

    def _execute_trajectory(
        self,
        T_start_urdf: np.ndarray,
        T_end_urdf: np.ndarray,
        q_start: np.ndarray,
        q_end: np.ndarray,
        gripper: Optional[float],
        duration: float,
        hz: float,
    ) -> dict[str, float]:
        """直线位置 + SLERP 姿态插值, 逐步 IK"""
        steps = max(2, int(duration * hz))
        dt = duration / steps
        step_ms = max(50, int(dt * 1000))

        p0 = T_start_urdf[:3, 3]
        p1 = T_end_urdf[:3, 3]
        R0 = T_start_urdf[:3, :3]
        R1 = T_end_urdf[:3, :3]

        try:
            grip_s = getattr(self.arm.read_position(), "gripper", 0.0)
        except Exception:
            grip_s = 0.0
        grip_e = gripper if gripper is not None else grip_s

        q_prev = q_start.copy()

        for i in range(1, steps + 1):
            t = i / steps

            pos_i = p0 + t * (p1 - p0)
            R_i = _slerp(R0, R1, t)

            T_i = np.eye(4)
            T_i[:3, :3] = R_i
            T_i[:3, 3] = pos_i

            q_i = self._solve_ik(T_i, q_prev, max_iter=50)

            try:
                self._write_q(q_i, duration_ms=step_ms)
            except Exception as e:
                logger.warning(f"[IK6D] step {i}/{steps}: {e}")

            if gripper is not None:
                g = grip_s + t * (grip_e - grip_s)
                self.arm.set_gripper(g, duration_ms=step_ms)

            q_prev = q_i
            time.sleep(dt)

        return dict(zip(IK_JOINT_NAMES, urdf_to_servo_deg(q_end)))

    def move_to_home(self, duration: float = 3.0):
        home = {n: 0.0 for n in ARM_MOTOR_NAMES}
        self.arm.move_smooth(home, duration=duration)

    def _solve_target_pose(
        self,
        x: float, y: float, z: float,
        roll: float, pitch: float, yaw: float,
        q_init: Optional[np.ndarray] = None,
        max_iter: int = 100,
    ) -> dict:
        """求解目标位姿，并返回误差，供可达性和调试接口复用。"""
        if q_init is None:
            try:
                q_init = self._read_q()
            except Exception:
                q_init = servo_deg_to_urdf([0, 0, 0, 0, 0])

        T_work_target = pose6d_to_T(x, y, z, roll, pitch, yaw)
        T_urdf_target = _T_work_to_urdf(T_work_target)
        q_sol = self._solve_ik(T_urdf_target, q_init, max_iter=max_iter)

        T_urdf_sol = self.kin.forward_kinematics(q_sol)
        T_work_sol = _T_urdf_to_work(T_urdf_sol)

        pos_err_m = float(np.linalg.norm(T_work_sol[:3, 3] - T_work_target[:3, 3]))
        ori_err_deg = float(math.degrees(np.linalg.norm(
            _rot_to_axis_angle(T_work_target[:3, :3] @ T_work_sol[:3, :3].T)
        )))

        return {
            "q_sol": q_sol,
            "target_work": T_work_target,
            "solved_work": T_work_sol,
            "pos_err_m": pos_err_m,
            "ori_err_deg": ori_err_deg,
        }

    # ── 可达性 ──

    def is_reachable(self, x: float, y: float, z: float,
                     roll: float = 0, pitch: float = 0, yaw: float = 0,
                     q_init: Optional[np.ndarray] = None,
                     pos_threshold: float = POSITION_THRESHOLD_M,
                     ori_threshold_deg: Optional[float] = None) -> bool:
        solved = self._solve_target_pose(
            x=x, y=y, z=z,
            roll=roll, pitch=pitch, yaw=yaw,
            q_init=q_init, max_iter=100,
        )
        if solved["pos_err_m"] >= pos_threshold:
            return False
        if ori_threshold_deg is not None and solved["ori_err_deg"] >= ori_threshold_deg:
            return False
        return True

    def check_delta(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                    droll: float = 0.0, dpitch: float = 0.0, dyaw: float = 0.0,
                    pose: Optional[CartesianPose] = None) -> dict:
        """检查增量移动是否可行，兼容旧版调试接口。"""
        if pose is None:
            pose = self.get_cartesian_pose()

        tx = pose.x + dx
        ty = pose.y + dy
        tz = pose.z + dz
        tr = pose.roll + droll
        tp = pose.pitch + dpitch
        tyaw = pose.yaw + dyaw

        solved = self._solve_target_pose(
            x=tx, y=ty, z=tz,
            roll=tr, pitch=tp, yaw=tyaw,
            max_iter=100,
        )
        target_servo_deg = urdf_to_servo_deg(solved["q_sol"])
        current_servo_deg = urdf_to_servo_deg(self._read_q())
        max_change = float(np.max(np.abs(target_servo_deg - current_servo_deg)))

        info = {
            "reachable": False,
            "target": (tx, ty, tz),
            "target_rpy": (tr, tp, tyaw),
            "target_joints_deg": dict(zip(IK_JOINT_NAMES, target_servo_deg)),
            "max_joint_change_deg": max_change,
            "reason": "",
            "pos_err_mm": solved["pos_err_m"] * 1000.0,
            "ori_err_deg": solved["ori_err_deg"],
        }

        if solved["pos_err_m"] >= self.POSITION_THRESHOLD_M:
            info["reason"] = f"位置误差过大 ({solved['pos_err_m'] * 1000.0:.1f} mm)"
            return info

        if solved["ori_err_deg"] >= self.ORIENTATION_THRESHOLD_DEG:
            info["reason"] = f"姿态误差过大 ({solved['ori_err_deg']:.1f}°)"
            return info

        info["reachable"] = True
        info["reason"] = "可达"
        if max_change > 45.0:
            info["reason"] = f"可达, 但关节变化较大 ({max_change:.1f}°), 建议增大 duration"
        return info

    def get_feasible_range(self, step: float = 0.002, max_steps: int = 150,
                           pose: Optional[CartesianPose] = None) -> dict:
        """计算当前位姿下 xyz 三轴的可行增量范围，兼容旧版调试接口。"""
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
            for sign in (-1.0, 1.0):
                limit = 0.0
                for i in range(1, max_steps + 1):
                    delta = sign * i * step
                    check = self.check_delta(
                        dx=ax * delta,
                        dy=ay * delta,
                        dz=az * delta,
                        pose=pose,
                    )
                    if check["reachable"]:
                        limit = delta
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

    def get_workspace_info(self) -> dict:
        l1 = math.sqrt(0.11257 ** 2 + 0.028 ** 2)
        l2 = math.sqrt(0.1349 ** 2 + 0.0052 ** 2)
        return {
            "l1": l1, "l2": l2,
            "reach_max": l1 + l2,
            "reach_min": abs(l1 - l2),
            "description": (
                f"SO-101 5DOF (6D IK):\n"
                f"  大臂≈{l1:.4f}m, 小臂≈{l2:.4f}m\n"
                f"  最大伸展: {l1+l2:.4f}m\n"
                f"  关节: {', '.join(IK_JOINT_NAMES)}\n"
                f"  IK权重: pos={self.pos_weight}, ori={self.ori_weight}\n"),
        }


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 自检
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def verify():
    kin = SO101Kinematics()

    print("=" * 60)
    print("SO-101 5DOF 运动学验证 (6D IK)")
    print("=" * 60)

    # FK 测试
    q_home = servo_deg_to_urdf([0, 0, 0, 0, 0])
    T_home = kin.forward_kinematics(q_home)
    Tw = _T_urdf_to_work(T_home)
    home_6d = T_to_pose6d(Tw)
    print(f"\n零位 6D 位姿 (工作坐标系):")
    print(f"  pos = ({home_6d[0]:.4f}, {home_6d[1]:.4f}, {home_6d[2]:.4f}) m")
    print(f"  rpy = ({home_6d[3]:.1f}°, {home_6d[4]:.1f}°, {home_6d[5]:.1f}°)")

    # 6D IK: 位置+姿态
    print(f"\n{'─'*60}")
    print("6D IK 测试 (位置+姿态同时约束)")
    print(f"{'─'*60}")

    T_home_urdf = T_home.copy()
    home_pos = Tw[:3, 3].copy()
    home_R = Tw[:3, :3].copy()

    tests = [
        (0, 0, 0.03, 0, 0, 0, "上抬3cm, 保持姿态"),
        (0.02, 0, 0, 0, 0, 0, "前伸2cm, 保持姿态"),
        (0, 0, 0, 10, 0, 0, "roll+10°, 保持位置"),
        (0, 0, 0, 0, -10, 0, "pitch-10°, 保持位置"),
        (0.02, 0, 0.02, 0, -15, 0, "前伸+上抬+pitch-15°"),
    ]

    for dx, dy, dz, dr, dp, dya, desc in tests:
        target_pos = home_pos + np.array([dx, dy, dz])
        r0, p0, y0 = _R_to_rpy(home_R)
        target_R = _rpy_to_R(r0 + math.radians(dr),
                             p0 + math.radians(dp),
                             y0 + math.radians(dya))
        Tw_target = np.eye(4)
        Tw_target[:3, :3] = target_R
        Tw_target[:3, 3] = target_pos
        Tu_target = _T_work_to_urdf(Tw_target)

        q_sol = kin.inverse_kinematics(q_home, Tu_target,
                                       pos_weight=1.0, ori_weight=0.2)
        T_sol = kin.forward_kinematics(q_sol)
        Tw_sol = _T_urdf_to_work(T_sol)

        pos_err = np.linalg.norm(Tw_sol[:3, 3] - target_pos) * 1000
        R_err_aa = _rot_to_axis_angle(target_R @ Tw_sol[:3, :3].T)
        ori_err = math.degrees(np.linalg.norm(R_err_aa))

        sol_6d = T_to_pose6d(Tw_sol)
        servo = urdf_to_servo_deg(q_sol)

        print(f"\n  {desc}:")
        print(f"    目标pos: ({target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f})")
        print(f"    实际pos: ({sol_6d[0]:.4f}, {sol_6d[1]:.4f}, {sol_6d[2]:.4f})")
        print(f"    目标rpy: ({math.degrees(r0+math.radians(dr)):.1f}°, "
              f"{math.degrees(p0+math.radians(dp)):.1f}°, "
              f"{math.degrees(y0+math.radians(dya)):.1f}°)")
        print(f"    实际rpy: ({sol_6d[3]:.1f}°, {sol_6d[4]:.1f}°, {sol_6d[5]:.1f}°)")
        print(f"    位置误差: {pos_err:.2f} mm  |  姿态误差: {ori_err:.2f}°")
        print(f"    servo: [{', '.join(f'{v:.1f}' for v in servo)}]°")

    # 权重对比
    print(f"\n{'─'*60}")
    print("权重对比: 相同目标, 不同 ori_weight")
    print(f"{'─'*60}")

    target_pos = home_pos + np.array([0.02, 0, 0.02])
    r0, p0, y0 = _R_to_rpy(home_R)
    target_R = _rpy_to_R(r0, p0 + math.radians(-20), y0)
    Tw_t = np.eye(4); Tw_t[:3,:3] = target_R; Tw_t[:3,3] = target_pos
    Tu_t = _T_work_to_urdf(Tw_t)

    for ow in [0.0, 0.2, 0.5, 1.0]:
        q_s = kin.inverse_kinematics(q_home, Tu_t, pos_weight=1.0, ori_weight=ow)
        T_s = kin.forward_kinematics(q_s)
        Tw_s = _T_urdf_to_work(T_s)
        pe = np.linalg.norm(Tw_s[:3,3] - target_pos) * 1000
        oe = math.degrees(np.linalg.norm(_rot_to_axis_angle(target_R @ Tw_s[:3,:3].T)))
        print(f"  ori_weight={ow:.1f}: pos_err={pe:.2f}mm  ori_err={oe:.2f}°")


if __name__ == "__main__":
    verify()
