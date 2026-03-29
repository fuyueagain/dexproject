"""
左臂夹爪抓取测试 — handshake=False 跳过握手验证
虚拟环境: new-lerobot

端口: /dev/ttyACM0 (左臂 ID1-6)
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


def build_left_arm_motors() -> dict[str, Motor]:
    motors = {}
    for i, name in enumerate(ARM_NAMES, start=1):
        mode = MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.RANGE_M100_100
        motors[name] = Motor(i, "sts3215", mode)
    return motors


def default_cal(motors) -> dict[str, MotorCalibration]:
    return {
        name: MotorCalibration(id=m.id, drive_mode=0, homing_offset=0,
                               range_min=0, range_max=4095)
        for name, m in motors.items()
    }


def connect_arm(port: str, motors: dict[str, Motor]) -> FeetechMotorsBus:
    print(f"    连接端口: {port}")
    bus = FeetechMotorsBus(port=port, motors=motors)
    bus.connect(handshake=False)  # 跳过握手，避免 motor 6 缺失导致卡住
    print(f"    写入校准...")
    bus.write_calibration(default_cal(motors))

    print(f"    禁用力矩并配置...")
    bus.disable_torque()
    bus.configure_motors()

    for name in motors:
        try:
            bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            bus.write("P_Coefficient", name, 16)
            bus.write("I_Coefficient", name, 0)
            bus.write("D_Coefficient", name, 32)
            bus.write("Acceleration", name, ACCELERATION)
        except Exception as e:
            print(f"    ⚠ {name} 配置失败: {e}")

    # 读取初始位置
    held_pos = bus.sync_read("Present_Position", normalize=False)
    print(f"    初始位置: {held_pos}")

    if held_pos:
        bus.sync_write("Goal_Position", held_pos, normalize=False)

    print(f"    开启力矩...")
    for name in motors:
        try:
            bus.write("Torque_Enable", name, 1)
        except Exception as e:
            print(f"    ⚠ {name} 开启力矩失败: {e}")

    return bus


def move_smooth(bus, target: dict[str, int], duration: float = 3.0, hz: float = 10):
    """平滑插值移动，使用原始 raw 值"""
    current = bus.sync_read("Present_Position", normalize=False)
    keys = list(target.keys())
    start = np.array([current.get(k, 2048) for k in keys], dtype=np.int32)
    end = np.array([target[k] for k in keys], dtype=np.int32)

    steps = max(2, int(duration * hz))
    dt = duration / steps

    print(f"    → raw {dict(zip(keys, end.tolist()))} ({steps}步)")
    for i in range(1, steps + 1):
        alpha = i / steps
        interp = (start + alpha * (end - start)).astype(np.int32)
        cmd = {k: int(v) for k, v in zip(keys, interp.tolist())}
        bus.sync_write("Goal_Position", cmd, normalize=False)
        time.sleep(dt)

    time.sleep(0.3)
    new_pos = bus.sync_read("Present_Position", keys, normalize=False)
    print(f"    ← 实际 {new_pos}")


def print_positions(bus, names: list[str], use_raw: bool = True, label=""):
    pos = bus.sync_read("Present_Position", names, normalize=use_raw)
    if label:
        print(f"  {label}:")
    for name, val in pos.items():
        if use_raw:
            print(f"    {name}: {val} ({val/4095*100:.1f}%)")
        else:
            print(f"    {name}: {val:.1f}")
    return pos


def main():
    print(f"""
{'='*60}
  左臂夹爪抓取测试
{'='*60}
  端口: {PORT_LEFT_ARM}
  加速度: {ACCELERATION}
{'='*60}
""")

    print("[1/5] 连接左臂...")
    motors = build_left_arm_motors()
    bus = connect_arm(PORT_LEFT_ARM, motors)
    print(f"  ✓ 已连接\n")

    print("[2/5] 读取当前位置...")
    init_raw = print_positions(bus, ARM_NAMES, use_raw=True, label="初始位置")
    gripper_initial = init_raw["gripper"]
    print()

    print("[3/5] 夹爪 +300 (更闭合)...")
    move_smooth(bus, {"gripper": gripper_initial + 300}, duration=3.0)
    time.sleep(1.0)

    print("[4/5] 夹爪 -600 (更张开，可能过载)...")
    move_smooth(bus, {"gripper": gripper_initial - 600}, duration=3.0)
    time.sleep(2.0)

    print("[5/5] 回到初始位置...")
    move_smooth(bus, {"gripper": gripper_initial}, duration=3.0)

    print("\n最终位置:")
    print_positions(bus, ARM_NAMES, use_raw=True, label="最终")

    try:
        bus.disconnect()
        print("\n✓ 已断开连接")
    except Exception as e:
        print(f"\n⚠ 断开异常: {e}")

    print("\n=== 测试完成 ===")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n用户中断...")
        try:
            bus.disconnect()
        except Exception:
            pass
