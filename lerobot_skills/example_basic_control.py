"""
全机器人硬件控制测试 — 使用 lerobot 底层 API (FeetechMotorsBus)

逐项测试:
  1. 左臂从臂 (ttyACM1 ID1-6): 夹爪开合 + shoulder_pan 旋转
  2. 底盘 (ttyACM1 ID7-9): 短暂前进+后退
  3. 右臂从臂 (ttyACM3 ID1-6): 夹爪开合 + shoulder_pan 旋转
  4. 头部云台 (ttyACM3 ID7-8): pan 左右 + tilt 上下
  5. 摄像头 (video0 + video2): 拍照保存

每项之间暂停确认, 可随时 Ctrl+C 跳过。

用法:
    conda activate new-lerobot
    python example_basic_control.py

⚠ USB 端口可能在重启后调换, 运行前先执行 detect_ports.py 确认!
"""

import sys
import time

import numpy as np

sys.path.insert(0, "/home/makermods/lerobot-MakerMods/src")

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

# ── 端口 (通过 detect_ports.py 确认) ──
PORT_LEFT_ARM = "/dev/ttyACM1"    # 左臂从臂(ID1-6) + 底盘(ID7-9), 共 9 舵机
PORT_RIGHT_ARM = "/dev/ttyACM3"   # 右臂从臂(ID1-6) + 云台(ID7-8), 共 8 舵机

ACCELERATION = 20

# ── 电机定义 ──
ARM_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
             "wrist_flex", "wrist_roll", "gripper"]

BASE_NAMES = ["base_left_wheel", "base_back_wheel", "base_right_wheel"]

HEAD_NAMES = ["head_pan", "head_tilt"]


def build_arm_motors() -> dict[str, Motor]:
    motors = {}
    for i, name in enumerate(ARM_NAMES, start=1):
        mode = MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.RANGE_M100_100
        motors[name] = Motor(i, "sts3215", mode)
    return motors


def build_left_bus_motors() -> dict[str, Motor]:
    """左臂(ID1-6) + 底盘(ID7-9)"""
    motors = build_arm_motors()
    for i, name in enumerate(BASE_NAMES, start=7):
        motors[name] = Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
    return motors


def build_right_bus_motors() -> dict[str, Motor]:
    """右臂(ID1-6) + 云台(ID7-8)"""
    motors = build_arm_motors()
    for i, name in enumerate(HEAD_NAMES, start=7):
        motors[name] = Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
    return motors


def default_cal(motors: dict[str, Motor]) -> dict[str, MotorCalibration]:
    return {
        name: MotorCalibration(id=m.id, drive_mode=0, homing_offset=0,
                               range_min=0, range_max=4095)
        for name, m in motors.items()
    }


def connect_bus(port: str, motors: dict[str, Motor]) -> FeetechMotorsBus:
    bus = FeetechMotorsBus(port=port, motors=motors)
    bus.connect()
    bus.write_calibration(default_cal(motors))

    try:
        held_pos = bus.sync_read("Present_Position", normalize=False)
    except Exception:
        held_pos = None

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

    if held_pos:
        try:
            bus.sync_write("Goal_Position", held_pos, normalize=False)
        except Exception:
            pass

    for name in motors:
        try:
            bus.write("Torque_Enable", name, 1)
        except Exception as e:
            print(f"    ⚠ {name} 开启力矩失败: {e}")

    return bus


def move_smooth(bus, target: dict[str, float], duration: float = 2.0, hz: float = 20):
    current = bus.sync_read("Present_Position")
    keys = list(target.keys())
    start = np.array([current.get(k, 0.0) for k in keys])
    end = np.array([target[k] for k in keys])

    steps = max(2, int(duration * hz))
    dt = duration / steps

    for i in range(1, steps + 1):
        alpha = i / steps
        interp = start + alpha * (end - start)
        cmd = {k: float(v) for k, v in zip(keys, interp)}
        bus.sync_write("Goal_Position", cmd)
        time.sleep(dt)


def print_positions(bus, names: list[str], label=""):
    pos = bus.sync_read("Present_Position", names)
    if label:
        print(f"  {label}:")
    for name, val in pos.items():
        print(f"    {name}: {val:.1f}")
    return pos


def safe_disconnect(bus, label=""):
    try:
        if bus and bus.is_connected:
            bus.disconnect()
    except Exception as e:
        print(f"  ⚠ {label} 断开异常: {e}")
        try:
            bus.port_handler.closePort()
        except Exception:
            pass


# ────────────────────────────────────────────────────────────
# 测试函数
# ────────────────────────────────────────────────────────────

def test_arm(bus, arm_label: str):
    """测试臂: 夹爪开合 + shoulder_pan 旋转"""
    print(f"\n{'─'*50}")
    print(f"  测试 {arm_label}")
    print(f"{'─'*50}")

    init_pos = print_positions(bus, ARM_NAMES, "当前位置")

    print(f"\n  夹爪: 全开 → 全闭 → 回初始")
    move_smooth(bus, {"gripper": 100.0}, duration=1.0)
    time.sleep(0.3)
    move_smooth(bus, {"gripper": 0.0}, duration=1.0)
    time.sleep(0.3)
    move_smooth(bus, {"gripper": init_pos["gripper"]}, duration=0.8)
    time.sleep(0.2)

    pan_start = init_pos["shoulder_pan"]
    delta = 15.0
    print(f"  肩部旋转: +{delta:.0f} → -{delta:.0f} → 回初始 (±27°)")
    move_smooth(bus, {"shoulder_pan": pan_start + delta}, duration=1.5)
    time.sleep(0.3)
    move_smooth(bus, {"shoulder_pan": pan_start - delta}, duration=2.0)
    time.sleep(0.3)
    move_smooth(bus, {"shoulder_pan": pan_start}, duration=1.5)
    time.sleep(0.2)

    final = print_positions(bus, ARM_NAMES, "最终位置")
    drift = abs(final["shoulder_pan"] - init_pos["shoulder_pan"])
    print(f"  shoulder_pan 漂移: {drift:.1f} (应 <3)")
    print(f"  ✓ {arm_label} 测试完成")


def test_base(bus):
    """测试底盘: 切换速度模式, 短暂运动"""
    print(f"\n{'─'*50}")
    print(f"  测试 底盘 (ID7-9 速度模式)")
    print(f"{'─'*50}")

    for name in BASE_NAMES:
        try:
            bus.write("Torque_Enable", name, 0)
            bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
            bus.write("Torque_Enable", name, 1)
        except Exception as e:
            print(f"    ⚠ {name} 切换速度模式失败: {e}")

    import math
    WHEEL_RADIUS = 0.05
    BASE_RADIUS = 0.125
    WHEEL_ANGLES_DEG = [240.0, 0.0, 120.0]

    def body_to_wheel_raw(x, y, theta_deg):
        theta_rad = theta_deg * (math.pi / 180.0)
        vel = np.array([x, y, theta_rad])
        angles = np.radians(np.array(WHEEL_ANGLES_DEG) - 90)
        m = np.array([[np.cos(a), np.sin(a), BASE_RADIUS] for a in angles])
        wheel_degps = (m.dot(vel) / WHEEL_RADIUS) * (180.0 / np.pi)
        steps_per_deg = 4096.0 / 360.0
        return {n: max(-0x8000, min(0x7FFF, int(round(d * steps_per_deg))))
                for n, d in zip(BASE_NAMES, wheel_degps)}

    print("  前进 0.5 秒...")
    bus.sync_write("Goal_Velocity", body_to_wheel_raw(0.08, 0, 0), normalize=False)
    time.sleep(0.5)
    bus.sync_write("Goal_Velocity", {n: 0 for n in BASE_NAMES}, normalize=False)
    time.sleep(0.3)

    print("  后退 0.5 秒...")
    bus.sync_write("Goal_Velocity", body_to_wheel_raw(-0.08, 0, 0), normalize=False)
    time.sleep(0.5)
    bus.sync_write("Goal_Velocity", {n: 0 for n in BASE_NAMES}, normalize=False)
    time.sleep(0.3)

    print("  原地旋转 0.5 秒...")
    bus.sync_write("Goal_Velocity", body_to_wheel_raw(0, 0, 20), normalize=False)
    time.sleep(0.5)
    bus.sync_write("Goal_Velocity", {n: 0 for n in BASE_NAMES}, normalize=False)

    for name in BASE_NAMES:
        try:
            bus.write("Torque_Enable", name, 0)
            bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
        except Exception:
            pass

    print("  ✓ 底盘测试完成")


def test_head(bus):
    """测试云台: 基于当前位置做相对运动, 最后回到初始位置"""
    print(f"\n{'─'*50}")
    print(f"  测试 头部云台 (ID7-8)")
    print(f"{'─'*50}")

    for name in HEAD_NAMES:
        try:
            bus.write("Torque_Enable", name, 0)
            bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            bus.write("P_Coefficient", name, 10)
            bus.write("D_Coefficient", name, 64)
            bus.write("Acceleration", name, 10)
            bus.write("Goal_Velocity", name, 300, normalize=False)
            bus.write("Torque_Limit", name, 500, normalize=False)
            cur = bus.read("Present_Position", name, normalize=False)
            bus.write("Goal_Position", name, cur, normalize=False)
            bus.write("Torque_Enable", name, 1)
        except Exception as e:
            print(f"    ⚠ {name} 配置失败 (可能过载保护): {e}")
            return

    init_raw = bus.sync_read("Present_Position", HEAD_NAMES, normalize=False)
    print(f"  初始原始值: pan={init_raw['head_pan']}, tilt={init_raw['head_tilt']}")

    DEG_TO_STEP = 4096.0 / 360.0  # 1° ≈ 11.38 步

    def move_head_to(target_raw: dict[str, int], duration=1.5):
        """插值移动到目标原始值"""
        current = bus.sync_read("Present_Position", HEAD_NAMES, normalize=False)
        steps = max(2, int(duration * 15))
        dt = duration / steps
        for i in range(1, steps + 1):
            alpha = i / steps
            cmd = {}
            for n in target_raw:
                val = int(current[n] + alpha * (target_raw[n] - current[n]))
                cmd[n] = max(0, min(4095, val))
            bus.sync_write("Goal_Position", cmd, normalize=False)
            time.sleep(dt)

    def offset_raw(base_raw: dict, pan_deg=0, tilt_deg=0) -> dict[str, int]:
        """从基准位置偏移指定角度"""
        result = {}
        if "head_pan" in base_raw:
            result["head_pan"] = max(0, min(4095,
                int(base_raw["head_pan"] + pan_deg * DEG_TO_STEP)))
        if "head_tilt" in base_raw:
            result["head_tilt"] = max(0, min(4095,
                int(base_raw["head_tilt"] + tilt_deg * DEG_TO_STEP)))
        return result

    pan_delta = 25  # ±25° 相对初始位置
    tilt_delta = 15  # ±15° 相对初始位置

    print(f"  Pan 左转 +{pan_delta}° (相对初始)...")
    move_head_to(offset_raw(init_raw, pan_deg=pan_delta), duration=1.5)
    time.sleep(0.3)

    print(f"  Pan 右转 -{pan_delta}° (相对初始)...")
    move_head_to(offset_raw(init_raw, pan_deg=-pan_delta), duration=2.0)
    time.sleep(0.3)

    print(f"  Pan 回到初始位置...")
    move_head_to({"head_pan": init_raw["head_pan"]}, duration=1.5)
    time.sleep(0.3)

    print(f"  Tilt 上抬 +{tilt_delta}° (相对初始)...")
    move_head_to(offset_raw(init_raw, tilt_deg=tilt_delta), duration=1.2)
    time.sleep(0.3)

    print(f"  Tilt 下压 -{tilt_delta}° (相对初始)...")
    move_head_to(offset_raw(init_raw, tilt_deg=-tilt_delta), duration=1.5)
    time.sleep(0.3)

    print(f"  Tilt 回到初始位置...")
    move_head_to({"head_tilt": init_raw["head_tilt"]}, duration=1.2)
    time.sleep(0.3)

    final_raw = bus.sync_read("Present_Position", HEAD_NAMES, normalize=False)
    pan_drift = abs(final_raw["head_pan"] - init_raw["head_pan"])
    tilt_drift = abs(final_raw["head_tilt"] - init_raw["head_tilt"])
    print(f"  最终原始值: pan={final_raw['head_pan']}, tilt={final_raw['head_tilt']}")
    print(f"  漂移: pan={pan_drift}步 tilt={tilt_drift}步 (应 <50)")
    print("  ✓ 头部云台测试完成")


URDF_PATH = "/home/makermods/XLeRobot/simulation/Maniskill/assets/xlerobot/xlerobot.urdf"

CALIBRATION_PATHS = {
    "左臂": "/home/makermods/.cache/huggingface/lerobot/calibration/robots/so101_follower/arm_left.json",
    "右臂": "/home/makermods/.cache/huggingface/lerobot/calibration/robots/so101_follower/arm_right.json",
}

URDF_JOINT_MAP = {
    "左臂": {
        "joints": ["Rotation_2", "Pitch_2", "Elbow_2", "Wrist_Pitch_2", "Wrist_Roll_2", "Jaw_2"],
        "ee_frame": "Fixed_Jaw_tip_2",
    },
    "右臂": {
        "joints": ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"],
        "ee_frame": "Fixed_Jaw_tip",
    },
}


# def test_ik(port: str, arm_label: str):
#     """测试 lerobot 逆运动学: placo + URDF, 笛卡尔增量分步执行"""
#     import json
#     sys.path.insert(0, "/home/makermods/dexproject/lerobot_skills")
#     from kinematics_utils import XLeRobotKinematics, MOTOR_NAMES

#     print(f"\n{'─'*50}")
#     print(f"  测试 {arm_label} 逆运动学 (lerobot placo + URDF)")
#     print(f"{'─'*50}")

#     cal_path = CALIBRATION_PATHS.get(arm_label)
#     if not cal_path or not __import__("pathlib").Path(cal_path).exists():
#         print(f"  ✗ 未找到校准文件: {cal_path}")
#         return

#     with open(cal_path) as f:
#         cal = {n: MotorCalibration(**d) for n, d in json.load(f).items()}
#     print(f"  ✓ 校准: {cal_path}")

#     motors_deg = {n: Motor(i+1, "sts3215", MotorNormMode.DEGREES) for i, n in enumerate(MOTOR_NAMES)}
#     motors_deg["gripper"] = Motor(6, "sts3215", MotorNormMode.DEGREES)

#     bus = FeetechMotorsBus(port=port, motors=motors_deg, calibration=cal)
#     bus.connect()
#     bus.write_calibration(cal)

#     bus.disable_torque()
#     bus.configure_motors()
#     for name in motors_deg:
#         bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
#         bus.write("Acceleration", name, ACCELERATION)
#     bus.enable_torque()

#     try:
#         pos = bus.sync_read("Present_Position")
#         joint_deg = np.array([pos[n] for n in MOTOR_NAMES])
#         print(f"  关节 (度): { {n: round(v,1) for n,v in zip(MOTOR_NAMES, joint_deg)} }")

#         kin = XLeRobotKinematics(arm_label)
#         ee_start = kin.forward_kinematics(joint_deg)
#         print(f"  FK 末端: x={ee_start[0]:.4f}  y={ee_start[1]:.4f}  z={ee_start[2]:.4f}")

#         delta_m = 0.03
#         n_steps = 30
#         step_dt = 0.08

#         def execute_cartesian_move(start_deg, dx=0, dy=0, dz=0, label=""):
#             """分步笛卡尔移动: 每步 1mm 级增量, 单步 IK 即可收敛"""
#             print(f"  {label}...")
#             waypoints = kin.plan_cartesian_delta(start_deg, dx=dx, dy=dy, dz=dz, steps=n_steps)

#             max_step_delta = 0
#             for i, wp in enumerate(waypoints):
#                 prev = waypoints[i-1] if i > 0 else start_deg
#                 step_delta = np.max(np.abs(wp[:5] - prev[:5]))
#                 max_step_delta = max(max_step_delta, step_delta)

#                 cmd = {n: float(wp[j]) for j, n in enumerate(MOTOR_NAMES)}
#                 bus.sync_write("Goal_Position", cmd)
#                 time.sleep(step_dt)

#             actual = bus.sync_read("Present_Position")
#             actual_deg = np.array([actual[n] for n in MOTOR_NAMES])
#             actual_ee = kin.forward_kinematics(actual_deg)
#             target_ee = kin.forward_kinematics(waypoints[-1])
#             err = np.linalg.norm(actual_ee - target_ee) * 1000
#             print(f"    到达 EE: x={actual_ee[0]:.4f}  y={actual_ee[1]:.4f}  z={actual_ee[2]:.4f}"
#                   f"  误差={err:.1f}mm  单步最大变化={max_step_delta:.1f}°")
#             return actual_deg

#         # 上抬 3cm
#         deg_after_up = execute_cartesian_move(
#             joint_deg, dz=delta_m,
#             label=f"上抬 {delta_m*100:.0f}cm ({n_steps}步)")
#         time.sleep(0.3)

#         # 回到初始
#         deg_after_back = execute_cartesian_move(
#             deg_after_up, dz=-delta_m,
#             label=f"回到初始 ({n_steps}步)")
#         time.sleep(0.3)

#         drift = max(abs(deg_after_back[i] - joint_deg[i]) for i in range(5))
#         print(f"  回位漂移: {drift:.1f}° (应 <5°)")
#         print(f"  ✓ {arm_label} IK 测试完成")

#     except Exception as e:
#         print(f"  ✗ IK 测试失败: {e}")
#         import traceback
#         traceback.print_exc()
#     finally:
#         try:
#             bus.disconnect()
#         except Exception:
#             pass


def test_cameras():
    """测试摄像头"""
    import cv2

    print(f"\n{'─'*50}")
    print(f"  测试 摄像头")
    print(f"{'─'*50}")

    MJPG = cv2.VideoWriter_fourcc(*"MJPG")
    for idx, name in [(0, "cam_a (头部)"), (2, "cam_b (右腕)")]:
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            print(f"    ✗ {name} /dev/video{idx} 打开失败")
            continue

        cap.set(cv2.CAP_PROP_FOURCC, MJPG)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        ret, frame = cap.read()
        if ret:
            path = f"/tmp/test_{name.split()[0]}.jpg"
            cv2.imwrite(path, frame)
            print(f"    ✓ {name}: {frame.shape} → {path}")
        else:
            print(f"    ✗ {name} 读取失败")
        cap.release()

    print("  ✓ 摄像头测试完成")


# ────────────────────────────────────────────────────────────
# 主程序
# ────────────────────────────────────────────────────────────
print(f"""
{'='*60}
  example_basic_control.py — 全机器人硬件控制测试
{'='*60}

  左总线: {PORT_LEFT_ARM}  (左臂 ID1-6 + 底盘 ID7-9)
  右总线: {PORT_RIGHT_ARM}  (右臂 ID1-6 + 云台 ID7-8)
  加速度: {ACCELERATION}

  测试项目:
    [1] 左臂从臂  — 夹爪开合 + 肩部旋转 (±27°)
    [2] 底盘      — 前进/后退/旋转各 0.5 秒
    [3] 右臂从臂  — 夹爪开合 + 肩部旋转 (±27°)
    [4] 头部云台  — pan 左右 ±25° + tilt 上下 ±15°
    [5] 摄像头    — 拍照保存到 /tmp/
    [6] 左臂 IK   — lerobot RobotKinematics (placo+URDF) 正/逆运动学
    [7] 右臂 IK   — lerobot RobotKinematics (placo+URDF) 正/逆运动学
        FK: 读取关节角度 → 计算末端位置
        IK: 末端目标 → 求解关节角度 → 发送到机器人
        需要: DEGREES 模式 + 已有校准文件 (homing_offset=2048)

  每项测试前会暂停确认, 可输入 s 跳过该项。
  ⚠ USB 端口通过 detect_ports.py 确认, 重启后可能变化!
{'='*60}
""")

left_bus = None
right_bus = None

try:
    # ── 连接左总线 ──
    print("连接左总线 (左臂+底盘)...")
    left_bus = connect_bus(PORT_LEFT_ARM, build_left_bus_motors())
    print(f"  ✓ 已连接 {PORT_LEFT_ARM}\n")

    # [1] 左臂
    c = input("[1] 测试左臂从臂? (Enter=执行, s=跳过): ").strip().lower()
    if c != "s":
        test_arm(left_bus, "左臂从臂")

    # [2] 底盘
    c = input("\n[2] 测试底盘? (Enter=执行, s=跳过): ").strip().lower()
    if c != "s":
        test_base(left_bus)

    # ── 连接右总线 ──
    print("\n连接右总线 (右臂+云台)...")
    right_bus = connect_bus(PORT_RIGHT_ARM, build_right_bus_motors())
    print(f"  ✓ 已连接 {PORT_RIGHT_ARM}\n")

    # [3] 右臂
    c = input("[3] 测试右臂从臂? (Enter=执行, s=跳过): ").strip().lower()
    if c != "s":
        test_arm(right_bus, "右臂从臂")

    # [4] 云台
    c = input("\n[4] 测试头部云台? (Enter=执行, s=跳过): ").strip().lower()
    if c != "s":
        test_head(right_bus)

    # [5] 摄像头
    c = input("\n[5] 测试摄像头? (Enter=执行, s=跳过): ").strip().lower()
    if c != "s":
        test_cameras()

    # # ── IK 测试需要独立的 ArmController, 先断开原始 bus ──
    # safe_disconnect(left_bus, "左总线")
    # left_bus = None
    # safe_disconnect(right_bus, "右总线")
    # right_bus = None

    # # [6] 左臂 IK
    # c = input("\n[6] 测试左臂逆运动学? (Enter=执行, s=跳过): ").strip().lower()
    # if c != "s":
    #     test_ik(PORT_LEFT_ARM, "左臂")

    # # [7] 右臂 IK
    # c = input("\n[7] 测试右臂逆运动学? (Enter=执行, s=跳过): ").strip().lower()
    # if c != "s":
    #     test_ik(PORT_RIGHT_ARM, "右臂")

    print(f"\n{'='*60}")
    print(f"  全部测试完成!")
    print(f"{'='*60}")

except KeyboardInterrupt:
    print("\n\n用户中断")

finally:
    safe_disconnect(left_bus, "左总线")
    safe_disconnect(right_bus, "右总线")
    print("已断开所有连接")
