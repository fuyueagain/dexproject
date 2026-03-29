"""
左臂夹爪抓取测试 — 跳过 calibration，直接操作舵机
"""

import sys
import time

import numpy as np

sys.path.insert(0, "/home/makermods/lerobot-MakerMods/src")

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

PORT_LEFT_ARM = "/dev/ttyACM0"
ACCELERATION = 5

ARM_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
             "wrist_flex", "wrist_roll", "gripper"]


def build_motors():
    motors = {}
    for i, name in enumerate(ARM_NAMES, start=1):
        mode = MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.RANGE_M100_100
        motors[name] = Motor(i, "sts3215", mode)
    return motors


def main():
    print("=== 左臂夹爪测试 (无 calibration) ===\n")

    motors = build_motors()
    bus = FeetechMotorsBus(port=PORT_LEFT_ARM, motors=motors)
    bus.connect(handshake=False)
    print("已连接 (handshake=False)")

    # 不写 calibration，直接禁尽力矩配置
    print("\n配置所有舵机...")
    bus.disable_torque()
    bus.configure_motors()

    for name in motors:
        try:
            bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            bus.write("P_Coefficient", name, 16)
            bus.write("I_Coefficient", name, 0)
            bus.write("D_Coefficient", name, 32)
            bus.write("Acceleration", name, ACCELERATION)
            bus.write("Torque_Enable", name, 1)
            print(f"  {name}: 配置完成")
        except Exception as e:
            print(f"  ⚠ {name}: {e}")

    time.sleep(0.5)

    # 读取初始位置
    print("\n读取位置...")
    init_pos = bus.sync_read("Present_Position", normalize=False)
    print(f"  {init_pos}")
    gripper_init = init_pos["gripper"]
    print(f"  夹爪: {gripper_init} ({gripper_init/4095*100:.1f}%)")

    # 测试写入
    print("\n测试写入 +300...")
    bus.write("Goal_Position", "gripper", gripper_init + 300, normalize=False)
    time.sleep(3)
    pos1 = bus.read("Present_Position", "gripper", normalize=False)
    print(f"  实际: {pos1}")

    print("测试写入 -600...")
    bus.write("Goal_Position", "gripper", gripper_init - 600, normalize=False)
    time.sleep(3)
    pos2 = bus.read("Present_Position", "gripper", normalize=False)
    print(f"  实际: {pos2}")

    print("回到初始...")
    bus.write("Goal_Position", "gripper", gripper_init, normalize=False)
    time.sleep(3)
    pos3 = bus.read("Present_Position", "gripper", normalize=False)
    print(f"  实际: {pos3}")

    bus.disconnect()
    print("\n=== 完成 ===")


if __name__ == "__main__":
    main()
