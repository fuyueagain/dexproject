"""
USB 端口自动识别 — 检测当前 ttyACM* 对应哪个物理设备

通过扫描每个端口上响应的舵机 ID 数量来判断:
  - 9 个响应 (ID 1-9) → 左臂从臂 + 底盘
  - 8 个响应 (ID 1-8) → 右臂从臂 + 云台
  - 6 个响应 (ID 1-6) → 遥操臂 (需进一步区分左/右)

用法:
    conda activate new-lerobot
    python detect_ports.py
"""

import glob
import sys
import json
from pathlib import Path

sys.path.insert(0, "/home/makermods/lerobot-MakerMods/src")

from lerobot.motors import Motor, MotorNormMode, MotorCalibration
from lerobot.motors.feetech import FeetechMotorsBus

SERVO_MAP_PATH = Path("/home/makermods/dexproject/servo_map.json")

DEVICE_NAMES = {
    "left_follower": "左臂从臂 + 底盘 (9舵机, ID1-9)",
    "right_follower": "右臂从臂 + 云台 (8舵机, ID1-8)",
    "left_leader": "遥操左臂 (6舵机, ID1-6)",
    "right_leader": "遥操右臂 (6舵机, ID1-6)",
}

SERVO_MAP_PORTS = {
    "left_follower": "ttyACM0",
    "right_follower": "ttyACM1",
    "left_leader": "ttyACM2",
    "right_leader": "ttyACM3",
}


def scan_port(port: str) -> dict:
    """扫描端口, 返回响应的 ID 列表和原始位置"""
    motors = {}
    for i in range(1, 10):
        motors[f"motor_{i}"] = Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)

    bus = FeetechMotorsBus(port=port, motors=motors)

    try:
        bus.port_handler.openPort()
        bus.port_handler.setBaudRate(1_000_000)
    except Exception as e:
        return {"error": str(e), "ids": [], "positions": {}}

    cal = {}
    for name, m in motors.items():
        cal[name] = MotorCalibration(
            id=m.id, drive_mode=0, homing_offset=0,
            range_min=0, range_max=4095,
        )
    bus.calibration = cal

    responding_ids = []
    positions = {}

    for motor_id in range(1, 10):
        try:
            model_number = bus.ping(f"motor_{motor_id}", num_retry=1)
            if model_number is not None:
                responding_ids.append(motor_id)
                try:
                    pos = bus.read("Present_Position", f"motor_{motor_id}", normalize=False)
                    positions[motor_id] = pos
                except Exception:
                    positions[motor_id] = None
        except Exception:
            pass

    try:
        bus.port_handler.closePort()
    except Exception:
        pass

    return {"ids": responding_ids, "positions": positions}


def identify_device(num_ids: int, positions: dict, servo_map: dict) -> str:
    """根据 ID 数量和位置匹配, 识别物理设备"""
    if num_ids >= 9:
        return "left_follower"
    elif num_ids == 8 or num_ids == 7:
        return "right_follower"
    elif num_ids <= 6:
        left_leader_score = 0
        right_leader_score = 0

        for motor_id, pos in positions.items():
            if pos is None:
                continue
            key_left = f"ttyACM2_id{motor_id}"
            key_right = f"ttyACM3_id{motor_id}"

            if key_left in servo_map:
                diff_left = abs(pos - servo_map[key_left]["raw_pos"])
                left_leader_score += 1.0 / (1.0 + diff_left)

            if key_right in servo_map:
                diff_right = abs(pos - servo_map[key_right]["raw_pos"])
                right_leader_score += 1.0 / (1.0 + diff_right)

        return "left_leader" if left_leader_score >= right_leader_score else "right_leader"

    return "unknown"


def main():
    print(f"""
{'='*60}
  detect_ports.py — USB 端口自动识别
{'='*60}
""")

    ports = sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        print("  未检测到任何 /dev/ttyACM* 设备!")
        return

    print(f"  检测到 {len(ports)} 个串口: {', '.join(ports)}\n")

    servo_map = {}
    if SERVO_MAP_PATH.exists():
        with open(SERVO_MAP_PATH) as f:
            servo_map = json.load(f)
        print(f"  已加载 servo_map.json 用于设备比对\n")

    results = {}

    for port in ports:
        print(f"  扫描 {port}...", end=" ", flush=True)
        info = scan_port(port)

        if info.get("error"):
            print(f"失败: {info['error']}")
            continue

        num_ids = len(info["ids"])
        print(f"发现 {num_ids} 个舵机 (ID: {info['ids']})")

        device = identify_device(num_ids, info["positions"], servo_map)
        results[port] = {
            "device": device,
            "num_ids": num_ids,
            "ids": info["ids"],
            "positions": info["positions"],
        }

    print(f"\n{'='*60}")
    print(f"  识别结果")
    print(f"{'='*60}\n")

    port_map = {}
    for port, info in results.items():
        device = info["device"]
        name = DEVICE_NAMES.get(device, device)
        orig_port = SERVO_MAP_PORTS.get(device, "?")
        port_map[device] = port
        match_str = "✓ 与 servo_map 一致" if port.endswith(orig_port) else f"✗ servo_map 中为 /dev/{orig_port}"
        print(f"  {port} → {name}")
        print(f"           {match_str}")

        if info["positions"] and servo_map:
            sm_prefix = f"{orig_port}_id"
            diffs = []
            for mid, pos in info["positions"].items():
                if pos is None:
                    continue
                key = f"{sm_prefix}{mid}"
                if key in servo_map:
                    diff = abs(pos - servo_map[key]["raw_pos"])
                    diffs.append(f"ID{mid}: {pos} vs {servo_map[key]['raw_pos']} (差{diff})")
            if diffs:
                print(f"           位置比对: {', '.join(diffs[:3])}...")
        print()

    print(f"{'='*60}")
    print(f"  建议 config.py 端口配置:")
    print(f"{'='*60}\n")
    print(f'  PORT_LEFT_FOLLOWER  = "{port_map.get("left_follower", "???")}"')
    print(f'  PORT_RIGHT_FOLLOWER = "{port_map.get("right_follower", "???")}"')
    print(f'  PORT_LEFT_LEADER    = "{port_map.get("left_leader", "???")}"')
    print(f'  PORT_RIGHT_LEADER   = "{port_map.get("right_leader", "???")}"')
    print()


if __name__ == "__main__":
    main()
