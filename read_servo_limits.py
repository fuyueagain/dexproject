#!/usr/bin/env python3
"""读取所有总线上每个舵机的当前寄存器值：限位、偏移、当前位置等。"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path.home() / "lerobot-MakerMods" / "src"))

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

FULL_TURN = 4096

SERVO_MAP_PATH = Path(__file__).parent / "servo_map.json"
servo_map = json.loads(SERVO_MAP_PATH.read_text())

buses_config: dict[str, dict[int, str]] = {}
for key, info in servo_map.items():
    port = info["port"]
    sid = info["id"]
    name = info["name"]
    buses_config.setdefault(port, {})[sid] = name

REGS = [
    "Present_Position",
    "Min_Position_Limit",
    "Max_Position_Limit",
    "Homing_Offset",
    "Operating_Mode",
    "Torque_Enable",
    "P_Coefficient",
    "D_Coefficient",
    "Torque_Limit",
    "Max_Torque_Limit",
    "Acceleration",
    "Lock",
]

results = {}

for port in sorted(buses_config.keys()):
    id_name_map = buses_config[port]
    motors = {}
    for sid, desc in sorted(id_name_map.items()):
        motor_name = f"m{sid}"
        motors[motor_name] = Motor(sid, "sts3215", MotorNormMode.RANGE_M100_100)

    bus = FeetechMotorsBus(port=port, motors=motors)
    bus.connect(handshake=False)
    cal = {}
    for mname, motor in bus.motors.items():
        cal[mname] = MotorCalibration(
            id=motor.id, drive_mode=0, homing_offset=0,
            range_min=0, range_max=4095,
        )
    bus.write_calibration(cal)

    print(f"\n{'='*60}")
    print(f"  总线: {port}")
    print(f"{'='*60}")

    port_results = {}
    for sid in sorted(id_name_map.keys()):
        motor_name = f"m{sid}"
        desc = id_name_map[sid]
        print(f"\n  ID {sid}: {desc}")

        servo_data = {"id": sid, "name": desc, "port": port}
        for reg in REGS:
            try:
                val = bus.read(reg, motor_name, normalize=False)
                servo_data[reg] = val
                raw_deg = ""
                if reg in ("Present_Position", "Min_Position_Limit", "Max_Position_Limit"):
                    deg = val / FULL_TURN * 360.0
                    raw_deg = f"  ({deg:.1f}°)"
                elif reg == "Homing_Offset":
                    sign = (val >> 11) & 1
                    magnitude = val & 0x7FF
                    signed_val = -magnitude if sign else magnitude
                    raw_deg = f"  (signed={signed_val}, raw=0x{val:04X})"
                print(f"    {reg:25s} = {val}{raw_deg}")
            except Exception as e:
                servo_data[reg] = f"ERROR: {e}"
                print(f"    {reg:25s} = ERROR: {e}")

        port_results[sid] = servo_data

    results[port] = port_results
    bus.disconnect(disable_torque=False)

output_path = Path(__file__).parent / "servo_limits_raw.json"
with open(output_path, "w") as f:
    json.dump(results, f, ensure_ascii=False, indent=2)
print(f"\n\n已保存原始数据到: {output_path}")
