"""
诊断脚本: 精确测试 raw 值写入
"""

import sys
import time
import numpy as np

sys.path.insert(0, "/home/makermods/lerobot-MakerMods/src")

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

PORT_LEFT_ARM = "/dev/ttyACM1"
ACCELERATION = 5

ARM_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
             "wrist_flex", "wrist_roll", "gripper"]


def build_motors():
    motors = {}
    for i, name in enumerate(ARM_NAMES, start=1):
        mode = MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.RANGE_M100_100
        motors[name] = Motor(i, "sts3215", mode)
    return motors


def default_cal(motors):
    return {
        name: MotorCalibration(id=m.id, drive_mode=0, homing_offset=0,
                               range_min=0, range_max=4095)
        for name, m in motors.items()
    }


def main():
    print("=== Raw 值写入诊断 ===\n")

    motors = build_motors()
    bus = FeetechMotorsBus(port=PORT_LEFT_ARM, motors=motors)
    bus.connect()
    bus.write_calibration(default_cal(motors))

    # 禁用力矩后重新配置
    bus.disable_torque()
    bus.configure_motors()
    for name in motors:
        bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
        bus.write("P_Coefficient", name, 16)
        bus.write("I_Coefficient", name, 0)
        bus.write("D_Coefficient", name, 32)
        bus.write("Acceleration", name, ACCELERATION)
        bus.write("Torque_Enable", name, 1)

    time.sleep(0.5)

    # 读取初始 raw 位置
    initial = bus.sync_read("Present_Position", normalize=False)
    print(f"初始 raw 位置: {initial}\n")

    gripper_initial = initial["gripper"]
    print(f"夹爪初始 raw: {gripper_initial}")

    # 测试1: 写入一个比初始值大很多的数 (尝试让夹爪闭合)
    test1 = gripper_initial + 200
    print(f"\n测试1: 写入 gripper={test1} (初始+200)")
    bus.write("Goal_Position", "gripper", test1, normalize=False)
    time.sleep(3)
    r1 = bus.read("Present_Position", "gripper", normalize=False)
    print(f"  结果: {r1} (差值: {r1 - gripper_initial})")

    # 测试2: 写入一个比初始值小很多的数 (尝试让夹爪张开)
    test2 = max(0, gripper_initial - 200)
    print(f"\n测试2: 写入 gripper={test2} (初始-200)")
    bus.write("Goal_Position", "gripper", test2, normalize=False)
    time.sleep(3)
    r2 = bus.read("Present_Position", "gripper", normalize=False)
    print(f"  结果: {r2} (差值: {r2 - gripper_initial})")

    # 测试3: 写入一个很小的值
    test3 = 500
    print(f"\n测试3: 写入 gripper={test3} (很小)")
    bus.write("Goal_Position", "gripper", test3, normalize=False)
    time.sleep(3)
    r3 = bus.read("Present_Position", "gripper", normalize=False)
    print(f"  结果: {r3} (差值: {r3 - gripper_initial})")

    # 测试4: 写入一个接近4095的值
    test4 = 4000
    print(f"\n测试4: 写入 gripper={test4} (很大)")
    bus.write("Goal_Position", "gripper", test4, normalize=False)
    time.sleep(3)
    r4 = bus.read("Present_Position", "gripper", normalize=False)
    print(f"  结果: {r4} (差值: {r4 - gripper_initial})")

    # 测试5: 回到初始位置
    print(f"\n测试5: 回到初始位置 {gripper_initial}")
    bus.write("Goal_Position", "gripper", gripper_initial, normalize=False)
    time.sleep(3)
    r5 = bus.read("Present_Position", "gripper", normalize=False)
    print(f"  结果: {r5}")

    # 检查 PWM 和速度限制
    print("\n--- 检查舵机限制 ---")
    try:
        print(f"  Torque_Limit: {bus.read('Torque_Limit', 'gripper', normalize=False)}")
        print(f"  Goal_Velocity: {bus.read('Goal_Velocity', 'gripper', normalize=False)}")
        print(f"  Present_Velocity: {bus.read('Present_Velocity', 'gripper', normalize=False)}")
        print(f"  Present_Load: {bus.read('Present_Load', 'gripper', normalize=False)}")
    except Exception as e:
        print(f"  读取限制失败: {e}")

    bus.disconnect()
    print("\n=== 完成 ===")


if __name__ == "__main__":
    main()
