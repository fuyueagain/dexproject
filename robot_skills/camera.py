"""双目摄像头管理"""

import logging
import time
from typing import Optional

import cv2
import numpy as np

from .config import CAMERA_A_INDEX, CAMERA_B_INDEX

logger = logging.getLogger("robot_skills")

_MJPG_FOURCC = cv2.VideoWriter_fourcc(*"MJPG")


class CameraManager:
    def __init__(self, cam_a_index=CAMERA_A_INDEX, cam_b_index=CAMERA_B_INDEX,
                 width=640, height=480, fps=30):
        self.indices = {"cam_a": cam_a_index, "cam_b": cam_b_index}
        self._width = width
        self._height = height
        self._fps = fps
        self._caps: dict[str, cv2.VideoCapture] = {}

    @property
    def is_connected(self) -> bool:
        return len(self._caps) > 0 and all(c.isOpened() for c in self._caps.values())

    def _open_cap(self, idx: int) -> Optional[cv2.VideoCapture]:
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        # MJPEG 大幅降低 USB 带宽，使多摄像头共用同一 USB 控制器成为可能
        cap.set(cv2.CAP_PROP_FOURCC, _MJPG_FOURCC)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS, self._fps)
        return cap

    def connect(self, cameras: Optional[list[str]] = None) -> None:
        for name in (cameras or list(self.indices.keys())):
            idx = self.indices[name]
            cap = self._open_cap(idx)
            if cap is None:
                logger.warning(f"[camera] 无法打开 {name} (/dev/video{idx})")
                continue
            # 预热：丢弃初始帧以让自动曝光稳定
            for _ in range(3):
                ret, frame = cap.read()
                if ret:
                    break
                time.sleep(0.1)
            if not ret:
                logger.warning(f"[camera] {name} (/dev/video{idx}) 已打开但无法读取帧")
                cap.release()
                continue
            self._caps[name] = cap
            logger.info(f"[camera] {name} 已连接 {frame.shape[1]}x{frame.shape[0]} MJPEG")

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
