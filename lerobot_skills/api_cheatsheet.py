"""
API 速查表 — lerobot 底层 API + robot_skills 对照

本文件展示两种控制方式的对照:
  1. lerobot 底层 API (FeetechMotorsBus) — 直接操作舵机
  2. robot_skills 封装层 — 推荐日常使用
"""

import sys
sys.path.insert(0, "/home/makermods/lerobot-MakerMods/src")
sys.path.insert(0, "/home/makermods/dexproject")


# ============================================================
# lerobot 底层 API 导入
# ============================================================

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

# ============================================================
# robot_skills 导入
# ============================================================

from robot_skills import (
    RobotSkills,
    ArmController, ArmState,
    HeadController,
    BaseController, BaseVelocity,
    CameraManager,
    ArmIKController, CartesianPose,
    PORT_LEFT_FOLLOWER, PORT_RIGHT_FOLLOWER,
    PORT_LEFT_LEADER, PORT_RIGHT_LEADER,
)


# ============================================================
# 方式 1: lerobot 底层 (FeetechMotorsBus 直接操作)
# ============================================================

def lerobot_low_level_demo():
    """展示 lerobot 底层 API 如何控制舵机"""

    # 定义电机
    motors = {
        "shoulder_pan":  Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
        "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
        "elbow_flex":    Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
        "wrist_flex":    Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
        "wrist_roll":    Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
        "gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    }

    # 连接 + 默认校准
    bus = FeetechMotorsBus(port="/dev/ttyACM0", motors=motors)
    bus.connect()
    cal = {n: MotorCalibration(id=m.id, drive_mode=0, homing_offset=0,
                               range_min=0, range_max=4095)
           for n, m in motors.items()}
    bus.write_calibration(cal)

    # 配置
    bus.disable_torque()
    bus.configure_motors()
    for motor in motors:
        bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
    bus.enable_torque()

    # 读取 (归一化)
    pos = bus.sync_read("Present_Position")              # dict[str, float]
    # 读取 (原始)
    raw = bus.sync_read("Present_Position", normalize=False)  # dict[str, int]
    # 读取单个电机
    val = bus.read("Present_Position", "shoulder_pan")    # float

    # 写入
    bus.sync_write("Goal_Position", {"shoulder_pan": 0.0, "gripper": 50.0})
    # 写入单个电机
    bus.write("Goal_Position", "shoulder_pan", 10.0)

    # 速度相关寄存器
    bus.write("Acceleration", "shoulder_pan", 20)         # 加速度档位 1~254
    bus.write("Goal_Time", "shoulder_pan", 1000)          # 到达时间 ms
    bus.write("Goal_Velocity", "shoulder_pan", 300, normalize=False)  # 最大速度

    # 力矩
    bus.disable_torque()
    bus.enable_torque()
    with bus.torque_disabled():
        pass  # 临时释放

    # 断开
    bus.disconnect()


# ============================================================
# 方式 2: robot_skills (推荐)
# ============================================================

def robot_skills_arm_demo():
    """ArmController — 关节空间控制"""
    arm = ArmController(PORT_LEFT_FOLLOWER, "left", acceleration=20)
    arm.connect()

    # 读取
    state = arm.read_position()                          # ArmState
    raw = arm.read_position_raw()                        # dict[str, int]

    # 移动 (硬件限速)
    arm.move_to({"shoulder_pan": 10.0}, duration_ms=1000)
    # 移动 (软件插值)
    arm.move_smooth({"shoulder_pan": 0.0}, duration=2.0)
    # 夹爪
    arm.set_gripper(100, duration_ms=500)

    # 速度控制
    arm.set_acceleration(30)
    arm.set_goal_time(1000)

    # 力矩
    arm.disable_torque()
    arm.enable_torque()

    arm.disconnect()


def robot_skills_ik_demo():
    """ArmIKController — 笛卡尔空间控制"""
    arm = ArmController(PORT_LEFT_FOLLOWER, "left")
    arm.connect()
    ik = ArmIKController(arm)

    # 读取笛卡尔位姿
    pose = ik.get_cartesian_pose()                       # CartesianPose

    # 绝对位置移动
    ik.move_cartesian(x=0.18, y=0.0, z=0.10, duration=2.0)
    # 增量移动
    ik.move_cartesian_delta(dx=0.02, dz=-0.01, duration=1.5)
    # 回零位
    ik.move_to_home(duration=3.0)

    arm.disconnect()


def robot_skills_full_demo():
    """RobotSkills — 全机器人控制"""
    with RobotSkills() as robot:
        # 观察值 (对应 lerobot Robot.get_observation)
        obs = robot.get_observation()

        # 动作 (对应 lerobot Robot.send_action)
        robot.send_action({
            "left_shoulder_pan.pos": 10.0,
            "right_gripper.pos": 80.0,
            "head_pan.pos": 20.0,
        })

        # 子系统直接访问
        robot.head.set_angle(pan=30, tilt=-10)
        robot.left_arm.move_smooth({"elbow_flex": -10.0}, duration=2.0)
        robot.base.move_for(x=0.1, duration=1.5)
        path = robot.cameras.save_snapshot("cam_a")

        # 紧急停止
        robot.emergency_stop()


# ============================================================
# API 对照表 (总结)
# ============================================================

"""
| 操作             | lerobot 底层                                    | robot_skills                          |
|------------------|------------------------------------------------|---------------------------------------|
| 连接             | bus = FeetechMotorsBus(...); bus.connect()      | arm = ArmController(...); arm.connect()|
| 默认校准         | bus.write_calibration(cal_dict)                 | make_bus() 自动处理                    |
| 读关节 (归一化)  | bus.sync_read("Present_Position")               | arm.read_position()                   |
| 读关节 (原始)    | bus.sync_read("Present_Position", normalize=F)  | arm.read_position_raw()               |
| 写关节           | bus.sync_write("Goal_Position", targets)        | arm.move_to(targets, duration_ms=...) |
| 平滑移动         | 手动插值 + sync_write                           | arm.move_smooth(targets, duration=...) |
| 夹爪             | bus.sync_write("Goal_Position", {"gripper": x}) | arm.set_gripper(x, duration_ms=...)   |
| 加速度           | bus.write("Acceleration", motor, val)           | arm.set_acceleration(val)             |
| 力矩开/关        | bus.enable/disable_torque()                     | arm.enable/disable_torque()           |
| 笛卡尔移动       | 需自己实现 IK                                   | ik.move_cartesian(x, y, z)            |
| 头部             | bus.sync_write("Goal_Position", head_targets)   | head.set_angle(pan=..., tilt=...)     |
| 底盘             | bus.sync_write("Goal_Velocity", wheel_targets)  | base.move(x=..., y=..., theta=...)    |
| 全机器人观察     | 手动拼装 dict                                   | robot.get_observation()               |
| 全机器人动作     | 手动拆分 dict + sync_write                      | robot.send_action(action)             |
| 相机             | cv2.VideoCapture(idx, CAP_V4L2)                 | CameraManager.capture("cam_a")        |
"""
