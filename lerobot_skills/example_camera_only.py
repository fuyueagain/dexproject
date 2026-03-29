"""
相机测试示例 — 使用 robot_skills.CameraManager

与 lerobot 的 OpenCVCamera 不同, 这里用 V4L2 + MJPEG 降低 USB 带宽,
使多摄像头可以共用 USB 控制器。

用法:
    conda activate new-lerobot
    python example_camera_only.py
"""

import sys
import time

sys.path.insert(0, "/home/makermods/dexproject")

import cv2
import numpy as np

from robot_skills import CameraManager

# ────────────────────────────────────────────────────────────
# 说明
# ────────────────────────────────────────────────────────────
print(f"""
{'='*60}
  example_camera_only.py — 摄像头采集演示
{'='*60}

  cam_a: /dev/video0 (头部摄像头)
  cam_b: /dev/video2 (右腕摄像头)

  预期执行效果:
    1. [CameraManager] 打开两个摄像头 (V4L2 + MJPEG 640x480)
       - 各拍一张快照保存到 ~/dexproject/cam_X_时间戳.jpg
       - 连续采集 3 秒, 打印实际帧率 (预期 ~30 Hz)
    2. [直接 cv2] 分别打开 video0 和 video2
       - 各拍一帧保存到 /tmp/cam_X_test.jpg
    3. 如果某个摄像头不可用, 会打印失败信息 (不会崩溃)

  不涉及舵机, 无需担心机械臂运动。
{'='*60}
""")

input("按 Enter 开始执行 (Ctrl+C 取消)...")
print()

# ────────────────────────────────────────────────────────────
# 方式 1: 使用 robot_skills.CameraManager
# ────────────────────────────────────────────────────────────
print("=== 使用 CameraManager ===\n")
cam = CameraManager()   # 默认 cam_a=video0, cam_b=video2
cam.connect()

if cam.is_connected:
    frames = cam.capture_all()
    for name, frame in frames.items():
        print(f"  {name}: shape={frame.shape}, dtype={frame.dtype}")
        path = cam.save_snapshot(name)
        print(f"    保存到: {path}")

    # 连续采集测试
    print(f"\n连续采集 3 秒...")
    count = 0
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < 3.0:
        frames = cam.capture_all()
        count += 1
        time.sleep(1.0 / 30)
    elapsed = time.perf_counter() - t0
    print(f"  {count} 次采集 / {elapsed:.1f}s = {count/elapsed:.1f} Hz")
    cam.disconnect()
else:
    print("  没有可用摄像头")

# ────────────────────────────────────────────────────────────
# 方式 2: 直接用 cv2 (最底层)
# ────────────────────────────────────────────────────────────
print("\n=== 直接使用 cv2.VideoCapture ===\n")

MJPG = cv2.VideoWriter_fourcc(*"MJPG")

for idx, name in [(0, "cam_a"), (2, "cam_b")]:
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"  {name} (/dev/video{idx}): 打开失败")
        continue

    cap.set(cv2.CAP_PROP_FOURCC, MJPG)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    ret, frame = cap.read()
    if ret:
        print(f"  {name} (/dev/video{idx}): {frame.shape} BGR uint8")
        cv2.imwrite(f"/tmp/{name}_test.jpg", frame)
    else:
        print(f"  {name} (/dev/video{idx}): 读取失败")

    cap.release()

print("\n完成!")
