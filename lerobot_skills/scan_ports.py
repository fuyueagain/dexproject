"""
扫描 ttyACM0, ttyACM1, ttyACM2, ttyACM3 上的所有舵机
"""

import sys
import time

sys.path.insert(0, "/home/makermods/lerobot-MakerMods/src")

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

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


def test_port(port):
    print(f"\n{'='*50}")
    print(f"  测试端口: {port}")
    print(f"{'='*50}")

    motors = build_motors()
    try:
        bus = FeetechMotorsBus(port=port, motors=motors)
        bus.connect()
        print(f"  ✓ 连接成功")

        # 读取保持位置（验证哪些舵机有响应）
        try:
            pos = bus.sync_read("Present_Position", normalize=False)
            print(f"  舵机数量: {len(pos)}")
            for name, val in pos.items():
                norm_val = val / 4095 * 100
                print(f"    {name}: raw={val} ({norm_val:.1f}%)")
        except Exception as e:
            print(f"  读取位置失败: {e}")

        bus.disconnect()
        return True
    except Exception as e:
        print(f"  ✗ 失败: {e}")
        return False


def main():
    print("=== 端口扫描 ===")

    for port in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"]:
        test_port(port)

    print("\n=== 扫描完成 ===")


if __name__ == "__main__":
    main()
