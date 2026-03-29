"""python -m robot_skills  →  交互式硬件自检"""

import glob
import sys
import time

from . import (
    ArmController, BaseController, CameraManager, HeadController,
    PORT_LEFT_FOLLOWER, PORT_LEFT_LEADER,
    PORT_RIGHT_FOLLOWER, PORT_RIGHT_LEADER,
)


def run_calibration(port: str, robot_id: str = "default") -> None:
    import subprocess
    cmd = [sys.executable, "-m",
           "lerobot.scripts.lerobot_measure_feetech_ranges",
           "--port", port, "--save", "--robot-id", robot_id]
    print(f"  运行: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)


def test_cameras():
    print(f"\n{'='*60}\n测试: 摄像头\n{'='*60}")
    cam = CameraManager()
    cam.connect()
    if not cam.is_connected:
        print("  ✗ 没有可用摄像头"); return
    for n in cam._caps:
        p = cam.save_snapshot(n)
        print(f"  {'✓' if p else '✗'} {n} → {p or '失败'}")
    cam.disconnect()


def test_arm(port, name):
    print(f"\n{'='*60}\n测试: {name} ({port})\n{'='*60}")
    arm = ArmController(port, name)
    try:
        arm.connect()
        print(f"  ✓ 连接成功")
        print(f"  位置: {arm.read_position()}")
        print(f"  原始: {arm.read_position_raw()}")
        print("  释放扭矩 3 秒...")
        arm.disable_torque()
        time.sleep(3)
        arm.enable_torque()
        print(f"  重新锁定: {arm.read_position()}")
    except Exception as e:
        print(f"  ✗ {e}")
    finally:
        arm.disconnect()


def test_head(port):
    print(f"\n{'='*60}\n测试: 头部云台 ({port})\n{'='*60}")
    head = HeadController(port)
    try:
        head.connect()
        ok = getattr(head, '_ok_motors', [])
        from .config import HEAD_MOTOR_IDS
        failed = [n for n in HEAD_MOTOR_IDS if n not in ok]

        if failed:
            print(f"  ⚠ 部分舵机处于保护状态: {failed}")
            print(f"    需要断电重启机器人来清除保护标志")
            if not ok:
                print(f"  ✗ 所有舵机不可用，跳过运动测试")
                return

        print(f"  ✓ 连接成功, 可用: {ok}")
        angle = head.read_angle()
        print(f"  角度: {angle}")

        print("  渐进回中...")
        head.look_center()
        time.sleep(0.5)

        print("  --- Pan (水平) ---")
        for desc, p in [("左转20°", 20), ("右转20°", -20), ("回中", 0)]:
            print(f"    {desc}...")
            try:
                head.set_angle(pan=p, wait=True, timeout=2)
                time.sleep(0.3)
            except Exception as e:
                print(f"    ⚠ {e}")

        print("  --- Tilt (俯仰) ---")
        for desc, t in [("上抬15°", 15), ("下压15°", -15), ("回中", 0)]:
            print(f"    {desc}...")
            try:
                head.set_angle(tilt=t, wait=True, timeout=2)
                time.sleep(0.3)
            except Exception as e:
                print(f"    ⚠ {e}")

        print("  ✓ 云台测试完成")
    except Exception as e:
        print(f"  ✗ {e}")
    finally:
        head.disconnect()


def test_base(port):
    print(f"\n{'='*60}\n测试: 底盘 ({port})\n{'='*60}")
    base = BaseController(port)
    try:
        base.connect()
        print("  ✓ 连接成功")
        for desc, x, y, th in [("前进", 0.1, 0, 0), ("左移", 0, 0.1, 0),
                                ("旋转", 0, 0, 30)]:
            print(f"  {desc} 1秒...")
            base.move_for(x=x, y=y, theta=th, duration=1.0)
        print("  ✓ 底盘测试完成")
    except Exception as e:
        print(f"  ✗ {e}")
    finally:
        base.disconnect()


def main():
    print(f"{'='*60}\nXLeRobot 硬件自检\n{'='*60}\n")
    ports = sorted(glob.glob("/dev/ttyACM*"))
    print(f"串口: {', '.join(ports)}\n")

    tests = {
        "1": ("摄像头", test_cameras),
        "2": ("左臂从臂 (ACM0)", lambda: test_arm(PORT_LEFT_FOLLOWER, "left_follower")),
        "3": ("右臂从臂 (ACM1)", lambda: test_arm(PORT_RIGHT_FOLLOWER, "right_follower")),
        "4": ("头部云台 (ACM1 ID7/8)", lambda: test_head(PORT_RIGHT_FOLLOWER)),
        "5": ("底盘三轮 (ACM0 ID7/8/9)", lambda: test_base(PORT_LEFT_FOLLOWER)),
        "6": ("左臂主臂 (ACM2)", lambda: test_arm(PORT_LEFT_LEADER, "left_leader")),
        "7": ("右臂主臂 (ACM3)", lambda: test_arm(PORT_RIGHT_LEADER, "right_leader")),
        "8": ("左臂校准", lambda: run_calibration(PORT_LEFT_FOLLOWER, "left")),
        "9": ("右臂校准", lambda: run_calibration(PORT_RIGHT_FOLLOWER, "right")),
        "a": ("全部测试", None),
        "q": ("退出", None),
    }
    for k, (d, _) in tests.items():
        print(f"  [{k}] {d}")
    print()
    c = input("请选择: ").strip().lower()

    if c == "q":
        return
    elif c == "a":
        test_cameras()
        test_arm(PORT_LEFT_FOLLOWER, "left_follower")
        test_arm(PORT_RIGHT_FOLLOWER, "right_follower")
        test_head(PORT_RIGHT_FOLLOWER)
        test_base(PORT_LEFT_FOLLOWER)
    elif c in tests and tests[c][1]:
        tests[c][1]()


if __name__ == "__main__":
    main()
