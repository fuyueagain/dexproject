#!/usr/bin/env python3
"""SO-101 IK 笛卡尔控制测试

测试流程:
  1. 纯数学 FK/IK 一致性验证
  2. 连接左臂, 检查位置, 必要时先回零
  3. FK 读取当前笛卡尔坐标
  4. 笛卡尔空间平滑增量移动
  5. 回到初始位置

用法:
    conda activate new-lerobot
    cd /home/makermods/dexproject
    python test_ik.py
"""

import sys
import time
sys.path.insert(0, "/home/makermods/dexproject")

from robot_skills import ArmController, ArmIKController, PORT_LEFT_FOLLOWER
from robot_skills.ik import norm_to_deg, deg_to_norm, SO101Kinematics2D
from robot_skills.config import ARM_MOTOR_NAMES


def test_kinematics_math():
    """纯数学测试: FK ↔ IK 一致性"""
    print("=" * 60)
    print("测试 1: FK/IK 数学一致性 (无需硬件)")
    print("=" * 60)

    kin = SO101Kinematics2D()

    test_points = [
        (0.1629, 0.1131),
        (0.15, 0.10),
        (0.20, 0.05),
        (0.10, 0.15),
        (0.18, 0.00),
    ]

    all_pass = True
    for r, z in test_points:
        sl_deg, ef_deg = kin.inverse_kinematics(r, z)
        r2, z2 = kin.forward_kinematics(sl_deg, ef_deg)
        err = ((r - r2) ** 2 + (z - z2) ** 2) ** 0.5
        ok = err < 0.001
        all_pass = all_pass and ok
        status = "✓" if ok else "✗"
        print(f"  {status} IK({r:.4f}, {z:.4f}) → "
              f"lift={sl_deg:.1f}° elbow={ef_deg:.1f}° → "
              f"FK({r2:.4f}, {z2:.4f})  误差={err:.6f}m")

    print(f"\n  数学测试 {'全部通过' if all_pass else '有失败项'}!\n")
    return all_pass


def safe_go_home(arm: ArmController, duration: float = 8.0):
    """安全回零: 只读一次初始位置, 后续全用 duration_ms 硬件控速, 不依赖中途读取"""
    state = arm.read_position()
    max_offset = max(abs(getattr(state, n)) for n in ARM_MOTOR_NAMES if n != "gripper")
    grip_val = getattr(state, "gripper")

    print(f"  当前位置: {state}")
    print(f"  最大偏移: {max_offset:.1f} (归一化)")

    if max_offset < 10.0:
        print("  已在零位附近, 无需回零.")
        return

    home = {n: 0.0 for n in ARM_MOTOR_NAMES}
    home["gripper"] = grip_val

    if max_offset > 50.0:
        n_stages = 4
        print(f"  ⚠ 偏移很大 ({max_offset:.0f}), 分 {n_stages} 阶段回零 (共 {duration:.0f} 秒)...")
        stage_time = duration / n_stages
        stage_ms = int(stage_time * 1000)

        for stage in range(1, n_stages + 1):
            alpha = stage / n_stages
            waypoint = {n: getattr(state, n) * (1.0 - alpha) for n in ARM_MOTOR_NAMES}
            waypoint["gripper"] = grip_val
            print(f"    阶段 {stage}/{n_stages} (α={alpha:.0%}, {stage_ms}ms)...")
            try:
                arm.move_to(waypoint, duration_ms=stage_ms)
            except Exception as e:
                print(f"    ⚠ 写入失败: {e}")
            time.sleep(stage_time + 0.3)
    else:
        stage_ms = int(duration * 1000)
        print(f"  回零中 ({duration:.0f} 秒, {stage_ms}ms)...")
        try:
            arm.move_to(home, duration_ms=stage_ms)
        except Exception as e:
            print(f"  ⚠ 写入失败: {e}")
        time.sleep(duration + 0.3)

    try:
        final = arm.read_position()
        max_final = max(abs(getattr(final, n)) for n in ARM_MOTOR_NAMES if n != "gripper")
        print(f"  回零完成. 当前最大偏移: {max_final:.1f}")
    except Exception:
        print(f"  回零指令已发送 (读取验证失败, 可能需要断电重启)")


def test_hardware():
    """硬件测试: 连接臂, 安全回零, FK/IK 控制"""
    print("=" * 60)
    print("测试 2: 硬件 FK/IK 控制 (左臂)")
    print("=" * 60)

    arm = ArmController(PORT_LEFT_FOLLOWER, "left_arm", acceleration=10)
    ik_ctrl = ArmIKController(arm)

    try:
        print("\n[1] 连接左臂 (acceleration=10)...")
        arm.connect()
        print("  ✓ 连接成功")

        print("\n[2] 检查位置, 安全回零...")
        if not arm.is_near_home(threshold=15.0):
            print("  臂不在零位附近, 需要先回零.")
            print("  按 Enter 开始回零 (Ctrl+C 取消)... ", end="", flush=True)
            input()
            safe_go_home(arm, duration=6.0)
        else:
            print("  ✓ 已在零位附近")

        print("\n[3] 读取当前关节位置...")
        state = arm.read_position()
        print(f"  归一化值: {state}")
        degrees = {name: norm_to_deg(getattr(state, name)) for name in state.to_dict()}
        print(f"  角度(度): " + ", ".join(f"{k}={v:.1f}°" for k, v in degrees.items()))

        print("\n[4] FK: 关节 → 笛卡尔坐标...")
        pose = ik_ctrl.get_cartesian_pose()
        print(f"  当前位姿: {pose}")
        print(f"  运行时 IK 实现: {ik_ctrl.__class__.__module__}.{ik_ctrl.__class__.__name__}")

        ws = ik_ctrl.get_workspace_info()
        print(f"  {ws['description']}")

        print("\n[5] 验证平面 IK ↔ FK 回环 (肩俯仰 + 肘关节)...")
        plane_r = (pose.x ** 2 + pose.y ** 2) ** 0.5
        planar_kin = SO101Kinematics2D()
        sl_deg, ef_deg = planar_kin.inverse_kinematics(plane_r, pose.z)
        print(f"  2D IK(r={plane_r:.4f}, z={pose.z:.4f}) → lift={sl_deg:.1f}°, elbow={ef_deg:.1f}°")
        print(f"  实际读取:                            lift={degrees['shoulder_lift']:.1f}°, elbow={degrees['elbow_flex']:.1f}°")
        print(f"  误差: shoulder_lift={abs(sl_deg - degrees['shoulder_lift']):.1f}°, elbow={abs(ef_deg - degrees['elbow_flex']):.1f}°")

        print("\n[6] 可行增量范围分析...")
        feasible = ik_ctrl.get_feasible_range()
        print(f"  {feasible['description']}")

        print("\n[7] 笛卡尔平滑移动测试 (每步 2cm, 2秒)")
        print("    按 Enter 开始 (Ctrl+C 取消)... ", end="", flush=True)
        input()

        initial_pose = ik_ctrl.get_cartesian_pose()
        print(f"    初始: x={initial_pose.x:.4f}, y={initial_pose.y:.4f}, z={initial_pose.z:.4f}")

        steps = [
            ("上移 2cm",  0.0,  0.0,  0.02),
            ("前伸 2cm",  0.02, 0.0,  0.0),
            ("下移 2cm",  0.0,  0.0, -0.02),
            ("后缩 2cm", -0.02, 0.0,  0.0),
        ]

        for desc, dx, dy, dz in steps:
            check = ik_ctrl.check_delta(dx=dx, dy=dy, dz=dz)
            if not check["reachable"]:
                print(f"\n    ✗ {desc}: 不可达 — {check['reason']}, 跳过")
                continue
            print(f"\n    → {desc} (duration=2.0s, 关节变化={check['max_joint_change_deg']:.1f}°)...")
            result = ik_ctrl.move_cartesian_delta(dx=dx, dy=dy, dz=dz, duration=2.0)
            time.sleep(0.3)
            try:
                current = ik_ctrl.get_cartesian_pose()
                print(f"      目标关节: " + ", ".join(f"{k}={v:.1f}°" for k, v in result.items()))
                print(f"      当前位姿: x={current.x:.4f}, y={current.y:.4f}, z={current.z:.4f}")
            except Exception as e:
                print(f"      (读取位置失败: {e})")

        print(f"\n    → 回到初始位置 (duration=3.0s)...")
        ik_ctrl.move_cartesian(
            x=initial_pose.x,
            y=initial_pose.y,
            z=initial_pose.z,
            pitch=initial_pose.pitch,
            duration=3.0,
        )
        time.sleep(0.5)
        try:
            final = ik_ctrl.get_cartesian_pose()
            print(f"      最终位姿: {final}")
        except Exception:
            pass

        print("\n  ✓ 硬件测试完成!")

    except KeyboardInterrupt:
        print("\n  ⚠ 用户中断")
    except Exception as e:
        print(f"\n  ✗ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[8] 断开连接...")
        arm.disconnect()
        print("  ✓ 已断开")


def main():
    print("\nSO-101 IK 笛卡尔控制测试")
    print("=" * 60)

    test_kinematics_math()

    print("\n是否进行硬件测试? (需要连接左臂)")
    choice = input("输入 y 继续, 其他键跳过: ").strip().lower()
    if choice == "y":
        test_hardware()
    else:
        print("跳过硬件测试.")

    print("\n测试结束.\n")


if __name__ == "__main__":
    main()
