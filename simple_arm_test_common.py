#!/usr/bin/env python3
"""单臂最小动作测试脚本。

用法示例:
    python test_left_arm_simple.py up
    python test_right_arm_simple.py down --delta 3
    python test_left_arm_simple.py open
    python test_right_arm_simple.py status
"""

from __future__ import annotations

import argparse
import time

from robot_skills import ArmController

RAW_STEP = 120
RAW_MOVE_THRESHOLD = 20


def _clamp_norm(value: float) -> float:
    return max(-100.0, min(100.0, value))


def _clamp_raw(value: int) -> int:
    return max(0, min(4095, value))


def _build_parser(arm_label: str, default_port: str | None) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=f"{arm_label}最简单动作测试",
    )
    parser.add_argument(
        "command",
        nargs="?",
        default="up",
        choices=["up", "down", "open", "close", "status"],
        help="执行的简单指令，默认 up",
    )
    parser.add_argument(
        "--delta",
        type=float,
        default=4.0,
        help="兼容保留参数；当前简单测试优先使用 raw 小步动作",
    )
    parser.add_argument(
        "--duration-ms",
        type=int,
        default=800,
        help="动作持续时间(毫秒)，默认 800",
    )
    parser.add_argument(
        "--wait-seconds",
        type=float,
        default=1.2,
        help="发送指令后的等待时间(秒)，默认 1.0",
    )
    parser.add_argument(
        "--port",
        default=default_port,
        help=f"串口，默认使用自动识别到的 {arm_label}从臂端口",
    )
    return parser


def _print_positions(title: str, data: dict[str, float | int], before: dict[str, float | int] | None = None) -> None:
    print(f"\n{title}")
    for joint, value in data.items():
        if before is None:
            print(f"  {joint:16s}: {value}")
            continue
        delta = value - before.get(joint, 0)
        print(f"  {joint:16s}: {value}  (变化 {delta:+})")


def _try_raw_joint_step(
    arm: ArmController,
    joint: str,
    signed_step: int,
    wait_seconds: float,
) -> tuple[bool, dict[str, int], dict[str, int]]:
    before_raw = arm.read_position_raw()
    target_raw = _clamp_raw(before_raw[joint] + signed_step)
    arm.move_to_raw({joint: target_raw}, duration_ms=0)
    time.sleep(max(0.2, wait_seconds))
    after_raw = arm.read_position_raw()
    moved = abs(after_raw[joint] - before_raw[joint])
    return moved >= RAW_MOVE_THRESHOLD, before_raw, after_raw


def run_simple_arm_test(arm_label: str, port: str | None) -> None:
    parser = _build_parser(arm_label, port)
    args = parser.parse_args()

    if not args.port:
        raise SystemExit(f"{arm_label}从臂端口未检测到，请先检查 USB 连接。")

    arm = ArmController(args.port, f"{arm_label}_simple_test", acceleration=10)

    print("=" * 60)
    print(f"{arm_label}最简单动作测试")
    print(f"port={args.port} command={args.command}")
    print("=" * 60)

    try:
        print("\n[1] 连接机械臂...")
        arm.connect()

        print("\n[2] 读取当前位置...")
        before = arm.read_position().to_dict()
        for joint, value in before.items():
            print(f"  {joint:16s}: {value:+.2f}")
        before_raw = arm.read_position_raw()
        _print_positions("[2.1] 原始位置...", before_raw)

        if args.command == "status":
            print("\n[3] status 模式，不发送动作。")
            return

        print("\n[3] 发送简单动作...")
        if args.command == "up":
            print("  优先动作: shoulder_lift raw +120；若无变化则回退 wrist_flex raw +120")
            success, before_raw, after_raw = _try_raw_joint_step(
                arm, "shoulder_lift", RAW_STEP, args.wait_seconds
            )
            used_joint = "shoulder_lift"
            if not success:
                print("  shoulder_lift 未检测到明显变化，回退测试 wrist_flex")
                success, before_raw, after_raw = _try_raw_joint_step(
                    arm, "wrist_flex", RAW_STEP, args.wait_seconds
                )
                used_joint = "wrist_flex"
            print(f"  实际测试关节: {used_joint} ({'成功' if success else '变化很小'})")
        elif args.command == "down":
            print("  优先动作: shoulder_lift raw -120；若无变化则回退 wrist_flex raw -120")
            success, before_raw, after_raw = _try_raw_joint_step(
                arm, "shoulder_lift", -RAW_STEP, args.wait_seconds
            )
            used_joint = "shoulder_lift"
            if not success:
                print("  shoulder_lift 未检测到明显变化，回退测试 wrist_flex")
                success, before_raw, after_raw = _try_raw_joint_step(
                    arm, "wrist_flex", -RAW_STEP, args.wait_seconds
                )
                used_joint = "wrist_flex"
            print(f"  实际测试关节: {used_joint} ({'成功' if success else '变化很小'})")
        elif args.command == "open":
            print("  动作: gripper raw +160")
            before_raw = arm.read_position_raw()
            arm.move_to_raw({
                "gripper": _clamp_raw(before_raw["gripper"] + 160)
            }, duration_ms=0)
            time.sleep(max(0.2, args.wait_seconds))
            after_raw = arm.read_position_raw()
        elif args.command == "close":
            print("  动作: gripper raw -160")
            before_raw = arm.read_position_raw()
            arm.move_to_raw({
                "gripper": _clamp_raw(before_raw["gripper"] - 160)
            }, duration_ms=0)
            time.sleep(max(0.2, args.wait_seconds))
            after_raw = arm.read_position_raw()

        print("\n[4] 读取动作后位置...")
        after = arm.read_position().to_dict()
        for joint, value in after.items():
            delta = value - before.get(joint, 0.0)
            print(f"  {joint:16s}: {value:+.2f}  (变化 {delta:+.2f})")
        _print_positions("[4.1] 动作后原始位置...", after_raw, before_raw)

        print("\n测试完成。")

    finally:
        print("\n[5] 断开连接...")
        arm.disconnect()
