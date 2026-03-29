"""
遥操作示例 — Leader-Follower 关节映射

使用 lerobot 底层 API 实现:
  Leader (ttyACM2) 读取关节位置 → Follower (ttyACM0) 跟随

这是 robot_skills 中遥操作的底层实现方式。

用法:
    conda activate new-lerobot
    python example_teleop_record.py
"""

import sys
import time

sys.path.insert(0, "/home/makermods/lerobot-MakerMods/src")
sys.path.insert(0, "/home/makermods/dexproject")

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

PORT_FOLLOWER = "/dev/ttyACM1"   # 左臂从臂 (通过 detect_ports.py 确认)
PORT_LEADER = "/dev/ttyACM0"     # 遥操左臂 (通过 detect_ports.py 确认)

ARM_MOTOR_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]

FPS = 30


def build_motors() -> dict[str, Motor]:
    motors = {}
    for i, name in enumerate(ARM_MOTOR_NAMES, start=1):
        mode = MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.RANGE_M100_100
        motors[name] = Motor(i, "sts3215", mode)
    return motors


def default_cal(motors: dict[str, Motor]) -> dict[str, MotorCalibration]:
    return {
        name: MotorCalibration(id=m.id, drive_mode=0, homing_offset=0,
                               range_min=0, range_max=4095)
        for name, m in motors.items()
    }


def connect_bus(port: str, position_mode: bool = True) -> FeetechMotorsBus:
    motors = build_motors()
    bus = FeetechMotorsBus(port=port, motors=motors)
    bus.connect()
    bus.write_calibration(default_cal(motors))

    bus.disable_torque()
    bus.configure_motors()

    for motor in motors:
        bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        bus.write("P_Coefficient", motor, 16)
        bus.write("I_Coefficient", motor, 0)
        bus.write("D_Coefficient", motor, 32)
        bus.write("Acceleration", motor, 10)

    if position_mode:
        bus.enable_torque()
    # leader 不开力矩, 保持自由拖动

    return bus


# ────────────────────────────────────────────────────────────
# 说明
# ────────────────────────────────────────────────────────────
print(f"""
{'='*60}
  example_teleop_record.py — Leader→Follower 遥操作演示
{'='*60}

  Follower: {PORT_FOLLOWER} (左臂从臂, 位置模式, 力矩锁定)
  Leader:   {PORT_LEADER} (遥操左臂, 力矩释放, 可自由拖动)
  帧率: {FPS} Hz

  预期执行效果:
    1. 连接 Follower 从臂 (开启力矩, 锁定不动)
       连接 Leader 遥操臂 (释放力矩, 可手动拖动)
    2. 进入循环: 每帧读取 Leader 位置 → 写入 Follower
       → 用手拖动遥操臂, 从臂应实时跟随
    3. 每秒打印一行: Leader 和 Follower 的 shoulder_pan 值
    4. Ctrl+C 停止, 保存录制数据到 JSON 文件

  ⚠ 注意:
    - 端口通过 detect_ports.py 扫描确认
    - USB 端口重启后可能调换, 如有问题先重跑 detect_ports.py
    - 运行前确认从臂周围无障碍物
{'='*60}
""")

input("按 Enter 开始执行 (Ctrl+C 取消)...")
print()

# ────────────────────────────────────────────────────────────
# 连接
# ────────────────────────────────────────────────────────────
print("连接 Follower (ttyACM0)...")
follower_bus = connect_bus(PORT_FOLLOWER, position_mode=True)
print("连接 Leader (ttyACM2)...")
leader_bus = connect_bus(PORT_LEADER, position_mode=False)

# ────────────────────────────────────────────────────────────
# 遥操作循环
# ────────────────────────────────────────────────────────────
print(f"\n开始遥操作 ({FPS} Hz)，拖动 Leader 臂控制 Follower 臂")
print("按 Ctrl+C 停止\n")

recorded_data = []

try:
    while True:
        t0 = time.perf_counter()

        # 读取 Leader 的关节位置
        leader_pos = leader_bus.sync_read("Present_Position")

        # 直接发送给 Follower
        follower_bus.sync_write("Goal_Position", leader_pos)

        # 读取 Follower 的实际位置
        follower_pos = follower_bus.sync_read("Present_Position")

        # 记录数据
        recorded_data.append({
            "timestamp": time.time(),
            "leader": dict(leader_pos),
            "follower": dict(follower_pos),
        })

        dt = time.perf_counter() - t0
        step = len(recorded_data)
        if step % FPS == 0:
            print(f"[{step/FPS:.0f}s] leader_pan={leader_pos['shoulder_pan']:.1f} "
                  f"→ follower_pan={follower_pos['shoulder_pan']:.1f} "
                  f"({dt*1000:.1f}ms)")

        time.sleep(max(0, 1.0 / FPS - dt))

except KeyboardInterrupt:
    print(f"\n停止, 共录制 {len(recorded_data)} 帧")

finally:
    follower_bus.disconnect()
    leader_bus.disconnect()
    print("已断开连接")

# 可选: 保存录制数据
if recorded_data:
    import json
    path = "/home/makermods/dexproject/teleop_recording.json"
    with open(path, "w") as f:
        json.dump(recorded_data, f, indent=2)
    print(f"数据已保存到 {path} ({len(recorded_data)} 帧)")
