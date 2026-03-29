#!/usr/bin/env python3
"""
交互式舵机识别工具

逐个激活每条总线上的每个舵机（小幅来回摆动），
用户观察物理运动后输入该舵机的功能名称。
最终输出完整的舵机映射表。
"""
import sys, time, json, os
from pathlib import Path
sys.path.insert(0, str(Path.home() / "lerobot-MakerMods" / "src"))

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

RESULT_FILE = "/home/makermods/dexproject/servo_map.json"
WIGGLE_STEP = 200   # ±200 步 ≈ ±17.6°
WIGGLE_PAUSE = 0.8  # 每次停留秒数


def scan_port(port, max_id=15):
    """快速扫描端口上存在的舵机 ID"""
    found = []
    for sid in range(1, max_id + 1):
        try:
            bus = FeetechMotorsBus(
                port=port,
                motors={"p": Motor(sid, "sts3215", MotorNormMode.RANGE_M100_100)},
            )
            bus.connect()
            bus.write_calibration({"p": MotorCalibration(
                id=sid, drive_mode=0, homing_offset=0,
                range_min=0, range_max=4095)})
            pos = bus.sync_read("Present_Position", ["p"], normalize=False)["p"]
            bus.disconnect()
            found.append((sid, pos))
        except Exception:
            try:
                bus.disconnect()
            except Exception:
                pass
    return found


def wiggle_servo(port, sid):
    """让指定舵机来回摆动 3 次，方便用户观察"""
    name = "m"
    bus = FeetechMotorsBus(
        port=port,
        motors={name: Motor(sid, "sts3215", MotorNormMode.RANGE_M100_100)},
    )
    bus.connect()
    bus.write_calibration({name: MotorCalibration(
        id=sid, drive_mode=0, homing_offset=0,
        range_min=0, range_max=4095)})

    orig = bus.sync_read("Present_Position", [name], normalize=False)[name]

    bus.write("Torque_Enable", name, 0)
    bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
    bus.write("P_Coefficient", name, 16)
    bus.write("D_Coefficient", name, 32)
    bus.write("Torque_Enable", name, 1)

    plus = min(orig + WIGGLE_STEP, 4095)
    minus = max(orig - WIGGLE_STEP, 0)

    for _ in range(2):
        bus.sync_write("Goal_Position", {name: plus}, normalize=False)
        time.sleep(WIGGLE_PAUSE)
        bus.sync_write("Goal_Position", {name: minus}, normalize=False)
        time.sleep(WIGGLE_PAUSE)

    bus.sync_write("Goal_Position", {name: orig}, normalize=False)
    time.sleep(0.3)

    try:
        bus.write("Torque_Enable", name, 0)
    except Exception:
        pass
    bus.disconnect()
    return orig


def main():
    import glob
    ports = sorted(glob.glob("/dev/ttyACM*"))
    print("=" * 60)
    print("  交互式舵机识别")
    print("=" * 60)
    print(f"\n检测到 {len(ports)} 条总线: {', '.join(ports)}")
    print(f"摆动幅度: ±{WIGGLE_STEP} 步 (≈ ±{WIGGLE_STEP * 360 / 4096:.0f}°)")
    print()

    # 先扫描所有端口
    all_servos = {}
    for port in ports:
        short = port.split("/")[-1]
        print(f"扫描 {short}...", end=" ", flush=True)
        found = scan_port(port)
        all_servos[port] = found
        ids = [f"ID{s[0]}" for s in found]
        print(f"{len(found)} 个舵机: {', '.join(ids)}")

    total = sum(len(v) for v in all_servos.values())
    print(f"\n共 {total} 个舵机，开始逐个识别。")
    print()
    print("操作说明:")
    print("  - 每个舵机会来回摆动 2 次")
    print("  - 观察哪个关节在动，然后输入功能名称")
    print("  - 常用名称示例:")
    print("      左臂: left_shoulder_pan, left_shoulder_lift, left_elbow_flex,")
    print("            left_wrist_flex, left_wrist_roll, left_gripper")
    print("      右臂: right_shoulder_pan, right_shoulder_lift, ...")
    print("      云台: head_pan, head_tilt")
    print("      底盘: base_left_wheel, base_back_wheel, base_right_wheel")
    print("      主臂: leader_left_xxx, leader_right_xxx")
    print("      不确定: ? 或 unknown")
    print("  - 输入 skip 跳过当前舵机")
    print("  - 输入 quit 保存并退出")
    print("  - 输入 again 再摆一次")
    print()

    servo_map = {}

    for port in ports:
        short = port.split("/")[-1]
        servos = all_servos[port]

        for sid, raw_pos in servos:
            key = f"{short}_id{sid}"
            deg = raw_pos * 360.0 / 4096.0

            print(f"{'─' * 50}")
            print(f"  >>> {short} ID {sid}  (当前位置: {raw_pos} = {deg:.0f}°)")
            print(f"      摆动中...", end=" ", flush=True)

            try:
                orig = wiggle_servo(port, sid)
                print("完成!")
            except Exception as e:
                print(f"失败: {e}")
                servo_map[key] = {
                    "port": port, "id": sid, "raw_pos": raw_pos,
                    "name": "error", "note": str(e)
                }
                continue

            while True:
                label = input(f"  这个舵机是什么? > ").strip()

                if label.lower() == "quit":
                    print("\n保存并退出...")
                    with open(RESULT_FILE, "w") as f:
                        json.dump(servo_map, f, indent=2, ensure_ascii=False)
                    print(f"已保存到 {RESULT_FILE}")
                    _print_summary(servo_map)
                    return

                if label.lower() == "again":
                    print("      再摆一次...", end=" ", flush=True)
                    try:
                        wiggle_servo(port, sid)
                        print("完成!")
                    except Exception as e:
                        print(f"失败: {e}")
                    continue

                if label.lower() == "skip":
                    label = "unknown"

                servo_map[key] = {
                    "port": port,
                    "id": sid,
                    "raw_pos": raw_pos,
                    "name": label,
                }
                print(f"  ✓ {short} ID {sid} → {label}")
                break

    # 全部完成
    print(f"\n{'=' * 60}")
    print("全部识别完成!")
    with open(RESULT_FILE, "w") as f:
        json.dump(servo_map, f, indent=2, ensure_ascii=False)
    print(f"已保存到 {RESULT_FILE}")
    _print_summary(servo_map)


def _print_summary(servo_map):
    print(f"\n{'=' * 60}")
    print("舵机映射汇总")
    print(f"{'=' * 60}")

    by_port = {}
    for key, info in servo_map.items():
        port = info["port"].split("/")[-1]
        by_port.setdefault(port, []).append(info)

    for port in sorted(by_port):
        servos = sorted(by_port[port], key=lambda x: x["id"])
        print(f"\n  {port}:")
        for s in servos:
            print(f"    ID {s['id']:2d}: {s['name']:<30s} (raw={s['raw_pos']})")

    print()


if __name__ == "__main__":
    main()
