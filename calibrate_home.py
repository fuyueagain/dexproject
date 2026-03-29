#!/usr/bin/env python3
"""SO-101 臂零位校准工具

流程:
  1. 连接臂, 释放扭矩
  2. 用手把臂摆到"零位" (自然站立姿态)
  3. 按 Enter 记录当前原始位置为零位参考
  4. 保存到 calibration.json
  5. 之后 ArmController 连接时自动加载校准

用法:
    conda activate new-lerobot
    cd ~/dexproject
    python calibrate_home.py          # 校准左臂
    python calibrate_home.py --right  # 校准右臂
"""

import json
import sys
import time
sys.path.insert(0, "/home/makermods/dexproject")

from robot_skills import ArmController, PORT_LEFT_FOLLOWER, PORT_RIGHT_FOLLOWER
from robot_skills.config import ARM_MOTOR_NAMES

CALIBRATION_FILE = "/home/makermods/dexproject/robot_skills/calibration.json"


def load_calibration() -> dict:
    try:
        with open(CALIBRATION_FILE) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def save_calibration(data: dict):
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(data, f, indent=2)


def calibrate(port: str, name: str):
    print(f"\n{'='*60}")
    print(f"SO-101 零位校准: {name} ({port})")
    print(f"{'='*60}")

    arm = ArmController(port, name, acceleration=10)

    try:
        print("\n[1] 连接臂...")
        arm.connect()
        print("  ✓ 连接成功")

        print("\n[2] 释放扭矩 — 现在可以用手自由移动臂")
        arm.disable_torque()
        print("  ✓ 扭矩已释放")

        print("""
[3] 请用手把臂摆到零位姿态:
    - shoulder_pan: 正前方 (不偏左不偏右)
    - shoulder_lift: 大臂竖直向上
    - elbow_flex: 小臂竖直向上 (与大臂成一条直线)
    - wrist_flex: 手腕不弯曲
    - wrist_roll: 手腕不旋转
    - gripper: 自然张开

    摆好后按 Enter 记录...""")
        input()

        print("\n[4] 读取当前原始位置...")
        raw = arm.read_position_raw()
        norm = arm.read_position()
        print(f"  原始值: {raw}")
        print(f"  归一化: {norm}")

        print("\n[5] 保存校准...")
        cal = load_calibration()
        cal[name] = {
            "port": port,
            "home_raw": {k: int(v) for k, v in raw.items()},
            "home_norm": {k: round(float(getattr(norm, k)), 2) for k in ARM_MOTOR_NAMES},
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        save_calibration(cal)
        print(f"  ✓ 已保存到 {CALIBRATION_FILE}")

        print(f"\n  零位原始值: {cal[name]['home_raw']}")
        print(f"  零位归一化: {cal[name]['home_norm']}")

        print("\n[6] 验证: 开扭矩锁定当前位置...")
        arm.enable_torque()
        print("  ✓ 臂已锁定在零位")

        print("\n  ✓ 校准完成!")
        print(f"  后续使用 IK 时会自动加载此校准.\n")

    except KeyboardInterrupt:
        print("\n  ⚠ 用户取消")
    except Exception as e:
        print(f"\n  ✗ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        arm.disconnect()


def main():
    if "--right" in sys.argv:
        calibrate(PORT_RIGHT_FOLLOWER, "right_arm")
    else:
        calibrate(PORT_LEFT_FOLLOWER, "left_arm")


if __name__ == "__main__":
    main()
