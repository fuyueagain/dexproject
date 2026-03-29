"""技能层 — 封装 robot_skills 为 Agent 可调用的高级接口"""

import base64
import logging
import threading
import time
from typing import Optional

import cv2
import numpy as np

from .config import MAX_DELTA_PER_STEP, MAX_CARTESIAN_DELTA, OBSERVATION_RESIZE

logger = logging.getLogger("loop_agent.skills")


class CameraStreamer:
    """独立线程持续采集相机帧，供 MJPEG 推流和 Agent 观测共用"""

    def __init__(self, cam_indices: Optional[dict[str, int]] = None):
        self.cam_indices = cam_indices or {"cam_a": 0, "cam_b": 2}
        self._caps: dict[str, cv2.VideoCapture] = {}
        self._frames: dict[str, np.ndarray] = {}
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        if self._running:
            return
        MJPG = cv2.VideoWriter_fourcc(*"MJPG")
        for name, idx in self.cam_indices.items():
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, MJPG)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 30)
                for _ in range(5):
                    cap.read()
                self._caps[name] = cap
                logger.info(f"[camera_streamer] {name} (/dev/video{idx}) 已打开")
            else:
                logger.warning(f"[camera_streamer] {name} (/dev/video{idx}) 打开失败")

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        for cap in self._caps.values():
            cap.release()
        self._caps.clear()
        self._frames.clear()

    def _loop(self):
        while self._running:
            for name, cap in self._caps.items():
                ret, frame = cap.read()
                if ret:
                    with self._lock:
                        self._frames[name] = frame
            time.sleep(1 / 30)

    def get_frame(self, camera: str) -> Optional[np.ndarray]:
        with self._lock:
            f = self._frames.get(camera)
            return f.copy() if f is not None else None

    def get_frame_jpeg(self, camera: str, quality: int = 80) -> Optional[bytes]:
        frame = self.get_frame(camera)
        if frame is None:
            return None
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes()

    def get_frame_base64(self, camera: str, resize: Optional[tuple] = None,
                         quality: int = 70) -> Optional[str]:
        frame = self.get_frame(camera)
        if frame is None:
            return None
        if resize:
            frame = cv2.resize(frame, resize)
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return base64.b64encode(buf.tobytes()).decode("ascii")

    @property
    def available_cameras(self) -> list[str]:
        return list(self._caps.keys())


class RobotSkillsWrapper:
    """对 robot_skills.RobotSkills 的封装，提供 Agent 友好的接口"""

    def __init__(self):
        self._robot = None
        self.camera_streamer = CameraStreamer()
        self._connected = False
        self._prev_observation_frames: dict[str, np.ndarray] = {}
        self._ik_controllers: dict[str, "ArmIKController"] = {}

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> str:
        if self._connected:
            return "已连接"

        # 相机始终独立启动，不依赖舵机
        self.camera_streamer.start()

        import sys
        from pathlib import Path
        sys.path.insert(0, str(Path.home() / "dexproject"))

        # 自动修复串口权限 (ttyACM* 可能在重启后丢失权限)
        self._fix_serial_permissions()

        status_parts = []
        cam_count = len(self.camera_streamer.available_cameras)
        status_parts.append(f"相机: {cam_count} 个")

        # 尝试不同的连接策略，逐步降级
        connect_plans = [
            {"enable_arms": True, "enable_head": True, "enable_base": False,
             "enable_cameras": False, "label": "双臂+云台"},
            {"enable_arms": True, "enable_head": False, "enable_base": False,
             "enable_cameras": False, "label": "仅双臂"},
        ]

        for plan in connect_plans:
            label = plan.pop("label")
            try:
                # 清理上一次失败残留的实例
                if self._robot:
                    try:
                        self._robot.disconnect()
                    except Exception:
                        pass
                    self._robot = None

                from robot_skills import RobotSkills
                self._robot = RobotSkills(**plan)
                self._robot.connect()
                status_parts.append(f"{label}: OK")
                logger.info(f"连接成功: {label}")
                break
            except Exception as e:
                logger.warning(f"{label} 连接失败: {e}")
                if self._robot:
                    try:
                        self._robot.disconnect()
                    except Exception:
                        pass
                    self._robot = None
        else:
            status_parts.append("舵机: 全部离线")

        self._connected = True  # 相机可用就算部分连接
        result = " | ".join(status_parts)
        logger.info(f"连接结果: {result}")
        return result

    @staticmethod
    def _fix_serial_permissions():
        """修复 /dev/ttyACM* 权限 — 依次尝试 os.chmod → sudo chmod"""
        import glob
        import os
        import stat
        import subprocess
        need_fix = []
        for port in glob.glob("/dev/ttyACM*"):
            try:
                st = os.stat(port)
                if not (st.st_mode & stat.S_IROTH and st.st_mode & stat.S_IWOTH):
                    need_fix.append(port)
            except OSError:
                pass

        if not need_fix:
            return

        # 先尝试直接 chmod
        for port in list(need_fix):
            try:
                os.chmod(port, 0o666)
                logger.info(f"[权限修复] {port} → 666")
                need_fix.remove(port)
            except PermissionError:
                pass

        # 还有没修好的 → 尝试 sudo (免密时才行)
        if need_fix:
            try:
                cmd = ["sudo", "-n", "chmod", "666"] + need_fix
                r = subprocess.run(cmd, capture_output=True, timeout=3)
                if r.returncode == 0:
                    logger.info(f"[权限修复] sudo chmod 成功: {need_fix}")
                else:
                    logger.warning(
                        f"[权限修复] 以下端口需要手动修复权限:\n"
                        f"  sudo chmod 666 {' '.join(need_fix)}\n"
                        f"  或永久解决: sudo usermod -aG dialout $USER (需重新登录)"
                    )
            except Exception:
                logger.warning(
                    f"[权限修复] 请手动执行: sudo chmod 666 {' '.join(need_fix)}"
                )

    def disconnect(self) -> str:
        self._ik_controllers.clear()
        if self._robot:
            try:
                self._robot.disconnect()
            except Exception as e:
                logger.warning(f"断开异常: {e}")
            self._robot = None
        self.camera_streamer.stop()
        self._connected = False
        return "已断开"

    def get_arm_positions(self, arm: str) -> dict[str, float]:
        if not self._robot:
            return {}
        try:
            ctrl = self._robot.left_arm if arm == "left" else self._robot.right_arm
            if ctrl and ctrl.is_connected:
                return ctrl.read_position().to_dict()
        except Exception as e:
            logger.warning(f"读取 {arm} 位置失败: {e}")
        return {}

    def get_all_positions(self) -> dict:
        return {
            "left_arm": self.get_arm_positions("left"),
            "right_arm": self.get_arm_positions("right"),
            "head": self.get_head_state(),
        }

    def move_arm_delta(self, arm: str, deltas: dict[str, float]) -> dict:
        clamped = {k: max(-MAX_DELTA_PER_STEP, min(MAX_DELTA_PER_STEP, v))
                   for k, v in deltas.items()}

        if not self._robot:
            return {"success": False, "error": "未连接"}
        try:
            ctrl = self._robot.left_arm if arm == "left" else self._robot.right_arm
            if not ctrl or not ctrl.is_connected:
                return {"success": False, "error": f"{arm} 臂未连接"}

            current = ctrl.read_position().to_dict()
            target = {}
            for k, d in clamped.items():
                if k in current and k != "gripper":
                    target[k] = current[k] + d

            if not target:
                return {"success": True, "positions": current, "note": "无有效增量"}

            ctrl.move_to(target, duration_ms=0)
            time.sleep(0.8)

            new_pos = ctrl.read_position().to_dict()
            moved = {k: round(new_pos.get(k, 0) - current.get(k, 0), 2)
                     for k in target}
            logger.info(f"[{arm}] move_arm_delta: 目标={target}, 实际变化={moved}")
            return {"success": True, "positions": new_pos, "moved": moved}
        except Exception as e:
            logger.error(f"move_arm_delta 失败: {e}")
            return {"success": False, "error": str(e)}

    def set_gripper(self, arm: str, openness: float) -> dict:
        openness = max(0, min(100, openness))

        if not self._robot:
            return {"success": False, "error": "未连接"}
        try:
            ctrl = self._robot.left_arm if arm == "left" else self._robot.right_arm
            if not ctrl or not ctrl.is_connected:
                return {"success": False, "error": f"{arm} 臂未连接"}
            ctrl.set_gripper(openness, duration_ms=500)
            time.sleep(0.5)
            new_pos = ctrl.read_position()
            return {"success": True, "gripper": new_pos.gripper}
        except Exception as e:
            logger.error(f"set_gripper 失败: {e}")
            return {"success": False, "error": str(e)}

    def get_head_state(self) -> dict:
        if not self._robot or not self._robot.head:
            return {}
        try:
            if self._robot.head.is_connected:
                return self._robot.head.read_angle()
        except Exception as e:
            logger.warning(f"读取云台失败: {e}")
        return {}

    def move_head_delta(self, pan_delta: float = 0, tilt_delta: float = 0) -> dict:
        if not self._robot or not self._robot.head:
            return {"success": False, "error": "云台未连接"}
        if pan_delta == 0 and tilt_delta == 0:
            return {"success": True, "head": self.get_head_state(), "note": "无增量"}
        try:
            current = self._robot.head.read_angle()
            new_pan = current.get("head_pan", 0) + pan_delta
            new_tilt = current.get("head_tilt", 0) + tilt_delta
            self._robot.head.set_angle(
                pan=new_pan if pan_delta != 0 else None,
                tilt=new_tilt if tilt_delta != 0 else None,
                wait=True, timeout=2.0,
            )
            time.sleep(0.3)
            after = self._robot.head.read_angle()
            logger.info(f"[head] delta=({pan_delta},{tilt_delta}) → {after}")
            return {"success": True, "head": after}
        except Exception as e:
            logger.error(f"move_head_delta 失败: {e}")
            return {"success": False, "error": str(e)}

    def move_base(self, x: float = 0, y: float = 0, theta: float = 0,
                  duration: float = 0.5) -> dict:
        if not self._robot or not self._robot.base:
            return {"success": False, "error": "未连接"}
        try:
            self._robot.base.move_for(x=x, y=y, theta=theta, duration=duration)
            return {"success": True}
        except Exception as e:
            logger.error(f"move_base 失败: {e}")
            return {"success": False, "error": str(e)}

    def _get_ik_controller(self, arm: str):
        """懒初始化 ArmIKController 实例"""
        if arm in self._ik_controllers:
            return self._ik_controllers[arm]
        if not self._robot:
            return None
        ctrl = self._robot.left_arm if arm == "left" else self._robot.right_arm
        if not ctrl or not ctrl.is_connected:
            return None
        from robot_skills.kinematics import ArmIKController
        ik = ArmIKController(ctrl)
        self._ik_controllers[arm] = ik
        return ik

    def get_cartesian_pose(self, arm: str) -> dict:
        """获取指定臂的笛卡尔空间末端位姿 (工作坐标系: x=前, y=左, z=上)"""
        ik = self._get_ik_controller(arm)
        if ik is None:
            return {"success": False, "error": f"{arm} 臂未连接或 IK 不可用"}
        try:
            pose = ik.get_cartesian_pose()
            return {
                "success": True,
                "pose": {
                    "x": round(pose.x, 4), "y": round(pose.y, 4),
                    "z": round(pose.z, 4),
                    "roll": round(pose.roll, 1), "pitch": round(pose.pitch, 1),
                    "yaw": round(pose.yaw, 1),
                    "gripper": round(pose.gripper, 1),
                },
                "description": (
                    f"{arm}臂末端: "
                    f"x={pose.x:.4f}m(前), y={pose.y:.4f}m(左), z={pose.z:.4f}m(上), "
                    f"rpy=({pose.roll:.1f}°,{pose.pitch:.1f}°,{pose.yaw:.1f}°)"
                ),
            }
        except Exception as e:
            logger.error(f"get_cartesian_pose({arm}) 失败: {e}")
            return {"success": False, "error": str(e)}

    def move_arm_cartesian_delta(self, arm: str, dx: float = 0.0,
                                  dy: float = 0.0, dz: float = 0.0,
                                  duration: float = 2.0) -> dict:
        """笛卡尔空间增量移动 (米为单位, 工作坐标系)

        Args:
            arm: "left" 或 "right"
            dx: 前后增量 (正=前伸, 负=后缩), 单位: 米
            dy: 左右增量 (正=左移, 负=右移), 单位: 米
            dz: 上下增量 (正=上抬, 负=下压), 单位: 米
            duration: 移动时长 (秒)
        """
        for axis_name, val in [("dx", dx), ("dy", dy), ("dz", dz)]:
            if abs(val) > MAX_CARTESIAN_DELTA:
                val_clamped = max(-MAX_CARTESIAN_DELTA, min(MAX_CARTESIAN_DELTA, val))
                logger.warning(
                    f"[{arm}] {axis_name}={val:.4f} 超限, 裁剪为 {val_clamped:.4f}")
                if axis_name == "dx":
                    dx = val_clamped
                elif axis_name == "dy":
                    dy = val_clamped
                else:
                    dz = val_clamped

        ik = self._get_ik_controller(arm)
        if ik is None:
            return {"success": False, "error": f"{arm} 臂未连接或 IK 不可用"}

        try:
            before_pose = ik.get_cartesian_pose()
            ik.move_cartesian_delta(dx=dx, dy=dy, dz=dz, duration=duration)
            after_pose = ik.get_cartesian_pose()

            actual_dx = after_pose.x - before_pose.x
            actual_dy = after_pose.y - before_pose.y
            actual_dz = after_pose.z - before_pose.z

            logger.info(
                f"[{arm}] cartesian_delta: 请求=({dx:.4f},{dy:.4f},{dz:.4f}), "
                f"实际=({actual_dx:.4f},{actual_dy:.4f},{actual_dz:.4f})")
            return {
                "success": True,
                "before": {"x": round(before_pose.x, 4), "y": round(before_pose.y, 4),
                           "z": round(before_pose.z, 4)},
                "after": {"x": round(after_pose.x, 4), "y": round(after_pose.y, 4),
                          "z": round(after_pose.z, 4)},
                "actual_delta": {"dx": round(actual_dx, 4), "dy": round(actual_dy, 4),
                                 "dz": round(actual_dz, 4)},
            }
        except Exception as e:
            logger.error(f"move_arm_cartesian_delta({arm}) 失败: {e}")
            return {"success": False, "error": str(e)}

    def move_arm_cartesian(self, arm: str, x: float, y: float, z: float,
                           gripper: Optional[float] = None,
                           duration: float = 2.0) -> dict:
        """笛卡尔空间绝对位置移动 (米为单位, 工作坐标系)"""
        ik = self._get_ik_controller(arm)
        if ik is None:
            return {"success": False, "error": f"{arm} 臂未连接或 IK 不可用"}

        if not ik.is_reachable(x, y, z):
            return {"success": False,
                    "error": f"目标位置 ({x:.4f},{y:.4f},{z:.4f}) 不可达"}

        try:
            before_pose = ik.get_cartesian_pose()
            ik.move_cartesian(x=x, y=y, z=z, gripper=gripper, duration=duration)
            after_pose = ik.get_cartesian_pose()

            logger.info(
                f"[{arm}] cartesian_move: "
                f"({before_pose.x:.4f},{before_pose.y:.4f},{before_pose.z:.4f}) → "
                f"({after_pose.x:.4f},{after_pose.y:.4f},{after_pose.z:.4f})")
            return {
                "success": True,
                "before": {"x": round(before_pose.x, 4), "y": round(before_pose.y, 4),
                           "z": round(before_pose.z, 4)},
                "after": {"x": round(after_pose.x, 4), "y": round(after_pose.y, 4),
                          "z": round(after_pose.z, 4)},
            }
        except Exception as e:
            logger.error(f"move_arm_cartesian({arm}) 失败: {e}")
            return {"success": False, "error": str(e)}

    def get_feasible_range(self, arm: str) -> dict:
        """获取指定臂当前位姿下各轴可行的笛卡尔增量范围"""
        ik = self._get_ik_controller(arm)
        if ik is None:
            return {"success": False, "error": f"{arm} 臂未连接或 IK 不可用"}
        try:
            result = ik.get_feasible_range()
            return {
                "success": True,
                "current": {
                    "x": round(result["current"].x, 4),
                    "y": round(result["current"].y, 4),
                    "z": round(result["current"].z, 4),
                },
                "dx_range": [round(result["dx"][0], 4), round(result["dx"][1], 4)],
                "dy_range": [round(result["dy"][0], 4), round(result["dy"][1], 4)],
                "dz_range": [round(result["dz"][0], 4), round(result["dz"][1], 4)],
                "description": result["description"],
            }
        except Exception as e:
            logger.error(f"get_feasible_range({arm}) 失败: {e}")
            return {"success": False, "error": str(e)}

    def emergency_stop(self):
        if self._robot:
            self._robot.emergency_stop()

    @staticmethod
    def _build_comparison(before: np.ndarray, after: np.ndarray,
                          resize: Optional[tuple] = None) -> str:
        """生成 before|after(差异高亮) 拼接对比图，返回 base64。"""
        if resize:
            before = cv2.resize(before, resize)
            after = cv2.resize(after, resize)

        h, w = before.shape[:2]
        diff = cv2.absdiff(before, after)
        diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)

        after_vis = after.copy()
        mask = diff_gray > 20
        if mask.any():
            after_vis[mask, 2] = np.clip(
                after_vis[mask, 2].astype(np.int16) + 120, 0, 255
            ).astype(np.uint8)
            after_vis[mask, 0] = (after_vis[mask, 0] * 0.5).astype(np.uint8)
            after_vis[mask, 1] = (after_vis[mask, 1] * 0.5).astype(np.uint8)

        sep = 3
        canvas = np.zeros((h, w * 2 + sep, 3), dtype=np.uint8)
        canvas[:, :w] = before
        canvas[:, w:w + sep] = [40, 40, 60]
        canvas[:, w + sep:] = after_vis

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(canvas, "Before", (6, 18), font, 0.5,
                    (120, 230, 120), 1, cv2.LINE_AA)
        cv2.putText(canvas, "After", (w + sep + 6, 18), font, 0.5,
                    (120, 120, 255), 1, cv2.LINE_AA)

        _, buf = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 75])
        return base64.b64encode(buf.tobytes()).decode("ascii")

    _CAM_LABELS = {"cam_a": "头部相机", "cam_b": "右腕相机"}

    def get_observation_for_llm(self) -> tuple[str, list[dict], list[dict]]:
        """获取当前状态文本 + 相机图片 + 前后对比图。

        Returns:
            state_text: 关节状态描述
            images: [{"camera", "label", "base64"}] — 当前帧（喂给 LLM）
            comparisons: [{"camera", "label", "base64"}] — 前后帧对比
        """
        positions = self.get_all_positions()

        text_parts = ["## 当前机器人状态\n"]
        for part_name, part_data in positions.items():
            if part_data:
                text_parts.append(f"### {part_name}")
                for k, v in part_data.items():
                    text_parts.append(f"  {k}: {v:.1f}")
        state_text = "\n".join(text_parts)

        images: list[dict] = []
        comparisons: list[dict] = []

        for cam in ["cam_a", "cam_b"]:
            frame = self.camera_streamer.get_frame(cam)
            if frame is None:
                continue

            prev = self._prev_observation_frames.get(cam)
            if prev is not None:
                try:
                    comp_b64 = self._build_comparison(
                        prev, frame, resize=OBSERVATION_RESIZE
                    )
                    comparisons.append({
                        "camera": cam,
                        "label": f"{self._CAM_LABELS.get(cam, cam)} 前后对比",
                        "base64": comp_b64,
                    })
                except Exception as e:
                    logger.warning(f"生成对比图失败 ({cam}): {e}")

            self._prev_observation_frames[cam] = frame.copy()

            resized = cv2.resize(frame, OBSERVATION_RESIZE) if OBSERVATION_RESIZE else frame
            _, buf = cv2.imencode(".jpg", resized, [cv2.IMWRITE_JPEG_QUALITY, 70])
            images.append({
                "camera": cam,
                "label": self._CAM_LABELS.get(cam, cam),
                "base64": base64.b64encode(buf.tobytes()).decode("ascii"),
            })

        return state_text, images, comparisons
