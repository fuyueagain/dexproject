"""
lerobot RobotKinematics 的实用封装

解决官方 inverse_kinematics() 单步收敛不足的问题:
  - 多步迭代 IK (solve 多次直到收敛)
  - 增量笛卡尔移动 (分步插值, 每步调一次 IK)
  - FK 直接使用官方代码

依赖:
  - placo (已安装)
  - XLeRobot URDF: /home/makermods/XLeRobot/simulation/Maniskill/assets/xlerobot/xlerobot.urdf
  - lerobot 的 RobotKinematics: lerobot.model.kinematics
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "/home/makermods/lerobot-MakerMods/src")

URDF_PATH = "/home/makermods/XLeRobot/simulation/Maniskill/assets/xlerobot/xlerobot.urdf"

ARMS = {
    "左臂": {
        "urdf_joints": ["Rotation_2", "Pitch_2", "Elbow_2",
                         "Wrist_Pitch_2", "Wrist_Roll_2", "Jaw_2"],
        "ee_frame": "Fixed_Jaw_tip_2",
    },
    "右臂": {
        "urdf_joints": ["Rotation", "Pitch", "Elbow",
                         "Wrist_Pitch", "Wrist_Roll", "Jaw"],
        "ee_frame": "Fixed_Jaw_tip",
    },
}

MOTOR_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll", "gripper"]


class XLeRobotKinematics:
    """基于 lerobot RobotKinematics (placo + URDF) 的运动学封装

    用法:
        kin = XLeRobotKinematics("左臂")

        # FK
        ee_pos = kin.forward_kinematics(joint_deg)  # (x, y, z) 米

        # IK (多步迭代, 保证收敛)
        new_deg = kin.inverse_kinematics(current_deg, target_xyz)

        # 增量轨迹 (笛卡尔空间插值)
        waypoints = kin.plan_cartesian_delta(current_deg, dx=0.03, steps=20)
    """

    def __init__(self, arm_name: str = "左臂", urdf_path: str = URDF_PATH):
        import placo

        info = ARMS[arm_name]
        self.joint_names = info["urdf_joints"]
        self.ee_frame = info["ee_frame"]
        self.arm_name = arm_name

        self.robot = placo.RobotWrapper(urdf_path)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)

        self.tip_task = self.solver.add_frame_task(self.ee_frame, np.eye(4))

    def _set_joints(self, joint_deg: np.ndarray):
        joint_rad = np.deg2rad(joint_deg[:len(self.joint_names)])
        for i, jn in enumerate(self.joint_names):
            self.robot.set_joint(jn, joint_rad[i])
        self.robot.update_kinematics()

    def _get_joints_deg(self) -> np.ndarray:
        return np.rad2deg([self.robot.get_joint(jn) for jn in self.joint_names])

    def forward_kinematics(self, joint_deg: np.ndarray) -> np.ndarray:
        """FK: 关节角度(度) → 末端位置 (x, y, z) 米

        也返回完整 4x4 变换矩阵存在 self.last_T 中
        """
        self._set_joints(joint_deg)
        self.last_T = self.robot.get_T_world_frame(self.ee_frame)
        return self.last_T[:3, 3].copy()

    def forward_kinematics_T(self, joint_deg: np.ndarray) -> np.ndarray:
        """FK: 关节角度(度) → 4x4 变换矩阵"""
        self._set_joints(joint_deg)
        return self.robot.get_T_world_frame(self.ee_frame).copy()

    def inverse_kinematics(
        self,
        current_deg: np.ndarray,
        target_xyz: np.ndarray,
        max_iters: int = 50,
        tol_mm: float = 1.0,
        orientation_weight: float = 1.0,
    ) -> np.ndarray:
        """IK: 当前关节 + 目标末端位置 → 新关节角度(度)

        保持当前姿态方向不变, 多步迭代直到收敛。
        orientation_weight=1.0 防止求解器跳到另一个构型。
        """
        self._set_joints(current_deg)

        T_current = self.robot.get_T_world_frame(self.ee_frame).copy()
        T_target = T_current.copy()
        T_target[:3, 3] = target_xyz

        self.tip_task.T_world_frame = T_target
        self.tip_task.configure(self.ee_frame, "soft", 1.0, orientation_weight)

        for _ in range(max_iters):
            self.solver.solve(True)
            self.robot.update_kinematics()
            ee = self.robot.get_T_world_frame(self.ee_frame)[:3, 3]
            if np.linalg.norm(ee - target_xyz) * 1000 < tol_mm:
                break

        result = self._get_joints_deg()
        n_input = len(current_deg)
        if n_input > len(self.joint_names):
            full = current_deg.copy()
            full[:len(self.joint_names)] = result
            return full
        return result

    def plan_cartesian_delta(
        self,
        current_deg: np.ndarray,
        dx: float = 0, dy: float = 0, dz: float = 0,
        steps: int = 20,
    ) -> list[np.ndarray]:
        """笛卡尔空间增量移动: 分步插值, 每步求解 IK

        返回关节角度序列 (度), 长度 = steps
        """
        ee_start = self.forward_kinematics(current_deg)
        ee_end = ee_start + np.array([dx, dy, dz])

        waypoints = []
        prev_deg = current_deg.copy()

        for i in range(1, steps + 1):
            alpha = i / steps
            target = ee_start + alpha * (ee_end - ee_start)
            new_deg = self.inverse_kinematics(prev_deg, target, max_iters=20)
            waypoints.append(new_deg)
            prev_deg = new_deg

        return waypoints
