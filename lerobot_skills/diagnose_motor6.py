"""
诊断 Motor 6（夹爪）过载问题
- handshake=False 连接 ttyACM0
- 逐个 ping/read ID 1-8
- 重点关注 ID6 的响应
"""

import sys
import time

sys.path.insert(0, "/home/makermods/lerobot-MakerMods/src")

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

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


def diagnose():
    print("=== Motor 6 诊断 ===\n")

    motors = build_motors()
    port = "/dev/ttyACM0"

    # handshake=False 连接
    print(f"[1] 连接 {port} (handshake=False)...")
    try:
        bus = FeetechMotorsBus(port=port, motors=motors)
        bus.connect(handshake=False)
        print("    ✓ 连接成功\n")
    except Exception as e:
        print(f"    ✗ 连接失败: {e}")
        return

    # 逐个 ID ping
    print("[2] 逐个 Ping ID 1-8:")
    alive = {}
    for mid in range(1, 9):
        try:
            result = bus.ping(str(mid), num_retry=2, raise_on_error=False)
            if result is not None:
                print(f"    ID{mid}: ✓ 响应 (model={result})")
                alive[mid] = result
            else:
                print(f"    ID{mid}: 无响应")
        except Exception as e:
            print(f"    ID{mid}: ✗ 错误: {e}")

    print()

    # 读取每个存活舵机的位置
    print("[3] 读取存活舵机的 Present_Position:")
    try:
        pos = bus.sync_read("Present_Position", normalize=False)
        for name, val in pos.items():
            norm_val = val / 4095 * 100
            print(f"    {name}: raw={val} ({norm_val:.1f}%)")
    except Exception as e:
        print(f"    ✗ 读取失败: {e}")

    print()

    # 检查 Motor 6 (gripper) 详细信息
    if 6 in alive:
        print("[4] Motor 6 (gripper) 详细诊断:")
        try:
            # 读取多个寄存器
            regs = ["Present_Position", "Present_Speed", "Present_Load",
                    "Present_Voltage", "Present_Temperature", "Moving"]
            for reg in regs:
                try:
                    val = bus.read(reg, "gripper", normalize=False, num_retry=2)
                    print(f"    {reg}: {val}")
                except Exception as e:
                    print(f"    {reg}: ✗ {e}")
        except Exception as e:
            print(f"    详细读取失败: {e}")

        # 读取位置限制
        print()
        print("[5] Motor 6 位置限制:")
        try:
            limits = bus.read_position_limits("gripper")
            print(f"    min={limits[0]}, max={limits[1]}")
        except Exception as e:
            print(f"    ✗ {e}")

        # 尝试写入不同位置测试夹爪范围
        print()
        print("[6] 测试夹爪位置范围:")
        test_positions = [500, 1500, 2500, 3500, 4095]
        for target in test_positions:
            try:
                bus.write("Goal_Position", "gripper", target, normalize=False, num_retry=1)
                time.sleep(0.3)
                actual = bus.read("Present_Position", "gripper", normalize=False, num_retry=1)
                print(f"    → 写入{target:4d} → 实际{actual:4d}")
            except Exception as e:
                print(f"    → 写入{target:4d} → ✗ {e}")
    else:
        print("[4] Motor 6 (gripper) 无响应，跳过详细诊断")

    bus.disconnect()
    print("\n=== 诊断完成 ===")


if __name__ == "__main__":
    diagnose()
