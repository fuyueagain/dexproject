"""自动识别当前连接的 follower / leader 串口。

目的:
    USB 重新枚举后, /dev/ttyACM* 的编号会变化。
    如果仍按固定编号把 follower 端口写死, OpenClaw 可能把动作发到遥操作主臂。
"""

from __future__ import annotations

import glob
import json
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path.home() / "lerobot-MakerMods" / "src"))

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

logger = logging.getLogger("robot_skills")

SERVO_MAP_PATH = Path(__file__).resolve().parent.parent / "servo_map.json"

DEVICE_SERVO_PREFIX = {
    "left_follower": "ttyACM0",
    "right_follower": "ttyACM1",
    "left_leader": "ttyACM2",
    "right_leader": "ttyACM3",
}

ALL_DEVICE_NAMES = list(DEVICE_SERVO_PREFIX.keys())


def _load_servo_map() -> dict:
    if not SERVO_MAP_PATH.exists():
        return {}
    try:
        return json.loads(SERVO_MAP_PATH.read_text())
    except Exception as e:
        logger.warning(f"[ports] 读取 servo_map.json 失败: {e}")
        return {}


def _build_scan_bus(port: str) -> FeetechMotorsBus:
    motors = {
        f"motor_{i}": Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
        for i in range(1, 10)
    }
    bus = FeetechMotorsBus(port=port, motors=motors)
    cal = {
        name: MotorCalibration(
            id=motor.id,
            drive_mode=0,
            homing_offset=0,
            range_min=0,
            range_max=4095,
        )
        for name, motor in motors.items()
    }
    bus.calibration = cal
    return bus


def scan_port(port: str) -> dict:
    """扫描一个串口, 返回响应 ID 和原始位置。"""
    bus = _build_scan_bus(port)
    positions: dict[int, int] = {}
    ids: list[int] = []

    try:
        try:
            bus.connect()
        except Exception as e:
            err = str(e).lower()
            if "motor check failed" in err or "missing motor" in err:
                bus._is_connected = True
            else:
                return {"ids": [], "positions": {}, "error": str(e)}

        for motor_id in range(1, 10):
            name = f"motor_{motor_id}"
            try:
                pos = bus.read("Present_Position", name, normalize=False)
                ids.append(motor_id)
                positions[motor_id] = pos
            except Exception:
                continue
    finally:
        try:
            bus.disconnect()
        except Exception:
            try:
                bus.port_handler.closePort()
            except Exception:
                pass

    return {"ids": ids, "positions": positions}


def _score_device(positions: dict[int, int], servo_map: dict, device: str) -> float:
    prefix = DEVICE_SERVO_PREFIX[device]
    score = 0.0
    count = 0
    for motor_id, pos in positions.items():
        key = f"{prefix}_id{motor_id}"
        entry = servo_map.get(key)
        if not entry:
            continue
        diff = abs(pos - entry["raw_pos"])
        score += 1.0 / (1.0 + diff)
        count += 1
    if count == 0:
        return -1.0
    return score


def detect_port_map() -> dict[str, str]:
    """自动识别当前设备端口。

    返回:
        {
            "left_follower": "/dev/ttyACM0",
            "right_leader": "/dev/ttyACM1",
            ...
        }
    """
    ports = sorted(glob.glob("/dev/ttyACM*"))
    servo_map = _load_servo_map()
    if not ports:
        return {}

    port_info = {port: scan_port(port) for port in ports}
    detected: dict[str, str] = {}
    remaining_ports: list[str] = []

    for port, info in port_info.items():
        ids = set(info["ids"])
        if {7, 8, 9}.issubset(ids):
            detected["left_follower"] = port
            continue
        if 7 in ids or 8 in ids:
            detected["right_follower"] = port
            continue
        remaining_ports.append(port)

    scored_pairs: list[tuple[float, str, str]] = []
    for port in remaining_ports:
        positions = port_info[port]["positions"]
        if not positions:
            continue
        for device in ALL_DEVICE_NAMES:
            if device in detected:
                continue
            score = _score_device(positions, servo_map, device)
            if score > 0:
                scored_pairs.append((score, port, device))

    for _, port, device in sorted(scored_pairs, reverse=True):
        if port in detected.values() or device in detected:
            continue
        detected[device] = port

    return detected
