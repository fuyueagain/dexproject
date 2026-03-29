# Quick Start — robot_skills 快速上手

## 环境准备

```bash
conda activate new-lerobot
cd ~/dexproject
```

## 1. 硬件自检

```bash
python -m robot_skills
```

按菜单选择测试摄像头、左臂、右臂、头部、底盘。

## 2. 关节空间控制 (ArmController)

最基本的控制方式: 直接指定各关节归一化值 `[-100, 100]`。

```python
from robot_skills import ArmController, PORT_LEFT_FOLLOWER

# 连接, acceleration=20 控制加速度 (1=最慢, 254=最快)
arm = ArmController(PORT_LEFT_FOLLOWER, "left", acceleration=20)
arm.connect()

# 读取当前位置
state = arm.read_position()
print(state)  # ArmState(shoulder_pan=0.2, shoulder_lift=-1.3, ...)

# 安全移动: 指定 duration_ms 或用 move_smooth
arm.move_to({"shoulder_pan": 10.0}, duration_ms=1000)     # 硬件限速, 1 秒到达
arm.move_smooth({"shoulder_pan": 0.0}, duration=2.0)       # 软件插值, 2 秒平滑回

# 夹爪
arm.set_gripper(100, duration_ms=500)   # 张开
arm.set_gripper(0, duration_ms=500)     # 闭合

# 释放关节 (可手动拖动)
arm.disable_torque()

# 断开
arm.disconnect()
```

## 3. 笛卡尔空间控制 (ArmIKController)

用 `(x, y, z)` 米为单位指定末端位置, IK 自动解算关节角度。

```python
from robot_skills import ArmController, ArmIKController, PORT_LEFT_FOLLOWER

arm = ArmController(PORT_LEFT_FOLLOWER, "left")
arm.connect()
ik = ArmIKController(arm)

# 读取当前笛卡尔坐标
pose = ik.get_cartesian_pose()
print(pose)
# CartesianPose(x=0.1629, y=0.0000, z=0.1177, pitch=0.0°, roll=0.0°, gripper=0)

# 绝对位置移动 (2 秒直线插值)
ik.move_cartesian(x=0.18, y=0.0, z=0.10, duration=2.0)

# 增量移动 (向前伸 2cm, 1.5 秒)
ik.move_cartesian_delta(dx=0.02, duration=1.5)

# 向上抬 3cm
ik.move_cartesian_delta(dz=0.03, duration=1.5)

# 回零位 (3 秒平滑)
ik.move_to_home(duration=3.0)

arm.disconnect()
```

### 坐标系说明

```
      z (上)          工作空间 (臂基座视角):
      │                 最大伸展 0.2509 m
      │   x (前)        最小伸展 0.0191 m
      │  /              零位坐标 ≈ (0.163, 0, 0.118)
      │ /
      └───── y (左)
     臂基座
```

- `x > 0` 向前, `y > 0` 向左, `z > 0` 向上
- `duration` 参数控制移动时长, 默认 2 秒
- `pitch` 参数控制末端俯仰, 默认自动保持水平

## 4. 全机器人控制 (RobotSkills)

统一管理双臂 + 底盘 + 云台 + 摄像头:

```python
from robot_skills import RobotSkills

with RobotSkills() as robot:
    # 头部
    robot.head.set_angle(pan=20, tilt=-10)

    # 左臂: 笛卡尔控制
    from robot_skills import ArmIKController
    left_ik = ArmIKController(robot.left_arm)
    left_ik.move_cartesian(0.18, 0.0, 0.10, duration=2.0)
    left_ik.move_cartesian_delta(dz=0.03, duration=1.0)

    # 右臂: 关节控制
    robot.right_arm.move_smooth({"shoulder_pan": -10.0}, duration=2.0)
    robot.right_arm.set_gripper(80, duration_ms=500)

    # 底盘
    robot.base.move_for(x=0.1, duration=1.5)

    # 拍照
    path = robot.cameras.save_snapshot("cam_a")
    print(f"照片: {path}")

    # 观测
    obs = robot.get_observation()

    # 紧急停止
    # robot.emergency_stop()
```

## 5. 速度控制指南

| 场景 | 推荐方式 | 示例 |
|------|----------|------|
| Agent 自主操作 | `duration=2.0` | `ik.move_cartesian(..., duration=2.0)` |
| 精细操作 | `duration=3.0~5.0` | `ik.move_cartesian_delta(dz=0.01, duration=3.0)` |
| 快速到位 | `duration=0.5~1.0` | `ik.move_cartesian(..., duration=0.8)` |
| 夹取物体 | `duration_ms=500` | `arm.set_gripper(0, duration_ms=500)` |
| 回零位 | `duration=3.0` | `ik.move_to_home(duration=3.0)` |
| 硬件限速 | `acceleration=10~50` | `ArmController(..., acceleration=20)` |

**原则: 宁慢勿快。默认 2 秒足以覆盖工作空间内的大多数移动。**

## 6. 常见问题

### 舵机过载报错

```
ConnectionError: Failed to write ... Incorrect status packet!
```

原因: 移动速度过快 / 负载过大导致舵机进入保护。处理:
1. 断电重启机器人清除保护标志
2. 使用更大的 `duration` 值
3. 降低 `acceleration` 值

### 总线冲突

同端口只能一个 bus 打开。如果 `ArmController` 连了 `ttyACM0`, 就不能再单独连 `BaseController`。
用 `RobotSkills` 自动处理总线共享。

### IK 超出工作空间

超出范围时 IK 会自动缩放到边界, 不会报错, 但实际到达位置可能与目标不同。
用 `ik.get_workspace_info()` 查看工作空间范围。

## 7. 适用于 Agent / MCP 调用

所有方法设计为幂等、安全、可组合:

```python
# Agent 可以安全调用的操作序列
ik.get_cartesian_pose()                              # 纯读取
ik.move_cartesian(0.18, 0.0, 0.10, duration=2.0)    # 带时长的绝对移动
ik.move_cartesian_delta(dx=0.02, duration=1.5)       # 带时长的增量移动
arm.set_gripper(0, duration_ms=500)                   # 带时长的夹爪
arm.read_position()                                   # 纯读取
ik.get_workspace_info()                               # 纯读取
```

每个运动方法都:
- 有明确的 `duration` 参数, 默认安全值
- 阻塞执行 (调用返回 = 动作完成)
- 不会因速度过快导致过载
- 返回有意义的结果 (关节角度 / 位姿)
