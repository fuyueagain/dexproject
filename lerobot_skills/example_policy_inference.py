"""
策略推理示例 — 使用 robot_skills 控制机器人

演示如何在控制循环中接入模型推理:
  1. robot.get_observation() 获取关节位置 + 相机图像
  2. model.select_action(obs) 推理
  3. robot.send_action(action) 执行

用法:
    conda activate new-lerobot
    python example_policy_inference.py
"""

import sys
import time

sys.path.insert(0, "/home/makermods/dexproject")

import numpy as np

from robot_skills import RobotSkills

FPS = 20


class DummyPolicy:
    """示例策略: echo 当前位置 (实际使用时替换为 ACT/Diffusion Policy/SmolVLA)"""

    def select_action(self, obs: dict) -> dict[str, float]:
        """
        输入 obs 结构:
            "left_shoulder_pan.pos":  float     关节角度 [-100, 100]
            "left_shoulder_lift.pos": float
            ...
            "left_gripper.pos":       float     夹爪开合 [0, 100]
            "right_shoulder_pan.pos": float
            ...
            "head_pan.pos":           float     头部角度 (度)
            "head_tilt.pos":          float
            "cam_a":                  ndarray   (480,640,3) uint8 BGR
            "cam_b":                  ndarray   (480,640,3) uint8 BGR

        输出 action: 只包含 .pos 结尾的 float 值
        """
        action = {}
        for key, val in obs.items():
            if isinstance(val, float) and key.endswith(".pos"):
                action[key] = val
        return action


# ────────────────────────────────────────────────────────────
# 主程序
# ────────────────────────────────────────────────────────────
print(f"""
{'='*60}
  example_policy_inference.py — 策略推理循环演示
{'='*60}

  使用 robot_skills.RobotSkills 统一控制接口
  目标帧率: {FPS} Hz
  底盘: 禁用 (enable_base=False)

  预期执行效果:
    1. 连接双臂 (ttyACM0 + ttyACM1) + 头部云台 + 摄像头
    2. 进入持续循环:
       - get_observation() 读取所有关节 + 相机
       - DummyPolicy 返回当前位置 (echo, 臂不会移动)
       - send_action() 写回关节值
    3. 每秒打印一行: 耗时, 帧率, 关节数, 相机数
    4. Ctrl+C 停止, 断开所有设备

  ⚠ 这是 echo 策略, 机器人会锁定在当前姿态不动。
    替换 DummyPolicy 为真实模型即可驱动机器人。
{'='*60}
""")

input("按 Enter 开始执行 (Ctrl+C 取消)...")
print()

print("连接机器人...")
robot = RobotSkills(enable_base=False)
robot.connect()

policy = DummyPolicy()

print(f"开始推理循环 ({FPS} Hz), Ctrl+C 停止...")
step = 0

try:
    while True:
        t0 = time.perf_counter()

        obs = robot.get_observation()
        action = policy.select_action(obs)
        robot.send_action(action)

        dt = time.perf_counter() - t0
        if step % FPS == 0:
            hz = 1.0 / dt if dt > 0 else 0
            joint_count = sum(1 for k in obs if isinstance(obs[k], float))
            cam_count = sum(1 for k in obs if isinstance(obs[k], np.ndarray))
            print(f"[Step {step:4d}] {dt*1000:.1f}ms ({hz:.0f} Hz) | "
                  f"{joint_count} joints, {cam_count} cameras")

        time.sleep(max(0, 1.0 / FPS - dt))
        step += 1

except KeyboardInterrupt:
    print("\n停止推理")

finally:
    robot.disconnect()
    print("已断开连接")
