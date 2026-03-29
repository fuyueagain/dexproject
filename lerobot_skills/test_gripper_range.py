"""
诊断脚本: 跳过握手直接测试夹爪 raw 范围
"""

import sys
import time

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
    print("=== 夹爪范围诊断 (跳过握手) ===\n")

    motors = build_motors()
    bus = FeetechMotorsBus(port=PORT_LEFT_ARM, motors=motors)

    # 跳过握手
    bus.connect(handshake=False)

    # 扫描所有 6 个舵机 ID
    print("扫描舵机...")
    for i in range(1, 9):
        try:
            pos = bus.read("Present_Position", i, normalize=False)
            print(f"  ID {i}: position={pos}")
        except Exception as e:
            print(f"  ID {i}: 无响应 ({e})")

    # 尝试配置和操作
    bus.write_calibration(default_cal(motors))
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
        except Exception as e:
            print(f"    ⚠ {name} 配置失败: {e}")

    time.sleep(0.5)

    # 读取夹爪初始
    try:
        initial = bus.read("Present_Position", "gripper", normalize=False)
        print(f"\n夹爪初始 raw: {initial}")
        print(f"初始 归一化(0-100): {initial / 4095 * 100:.1f}%\n")
    except Exception as e:
        print(f"\n读取夹爪位置失败: {e}")
        initial = None

    if initial is not None:
        # 测试向更高值 (更闭合)
        print("--- 往闭合方向 (+) ---")
        for delta in [50, 100, 200, 300, 500]:
            target = min(4095, initial + delta)
            print(f"  写入 {target}: ", end="", flush=True)
            try:
                bus.write("Goal_Position", "gripper", target, normalize=False)
                time.sleep(3)
                pos = bus.read("Present_Position", "gripper", normalize=False)
                print(f"实际={pos}")
            except Exception as e:
                print(f"错误: {e}")
                break

        bus.write("Goal_Position", "gripper", initial, normalize=False)
        time.sleep(3)

        # 测试向更低值 (更张开)
        print("\n--- 往张开方向 (-) ---")
        for delta in [50, 100, 200, 300, 500, 700]:
            target = max(0, initial - delta)
            print(f"  写入 {target}: ", end="", flush=True)
            try:
                bus.write("Goal_Position", "gripper", target, normalize=False)
                time.sleep(3)
                pos = bus.read("Present_Position", "gripper", normalize=False)
                print(f"实际={pos}")
            except Exception as e:
                print(f"错误: {e}")
                break

        # 回到初始
        bus.write("Goal_Position", "gripper", initial, normalize=False)
        time.sleep(2)

    bus.disconnect()
    print("\n=== 完成 ===")


if __name__ == "__main__":
    main()
