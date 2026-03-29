"""FeetechMotorsBus 连接与构建工具

修复问题: bus.connect() 内置的 motor check 会在舵机无响应时直接抛异常。
这里提供带重试和跳过检查的健壮连接方式。
"""

import logging
import sys
import time
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path.home() / "lerobot-MakerMods" / "src"))

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

from .config import ARM_MOTOR_IDS, ARM_MOTOR_NAMES

logger = logging.getLogger("robot_skills")


def make_bus(port: str, motors: dict[str, Motor],
             calibration: Optional[dict] = None,
             max_retries: int = 3) -> FeetechMotorsBus:
    """创建并连接 bus，带重试和 motor check 容错"""
    bus = FeetechMotorsBus(port=port, motors=motors, calibration=calibration)

    last_err = None
    for attempt in range(1, max_retries + 1):
        try:
            bus.connect()
            break
        except Exception as e:
            last_err = e
            err_str = str(e)
            if "motor check failed" in err_str.lower() or "missing motor" in err_str.lower():
                # 端口已打开，只是部分舵机未响应 → 跳过检查继续
                logger.warning(f"[bus] {port} 部分舵机未响应，跳过检查强制连接")
                bus._is_connected = True
                break
            logger.warning(f"[bus] {port} 连接失败 (尝试 {attempt}/{max_retries}): {e}")
            try:
                bus.port_handler.closePort()
            except Exception:
                pass
            if attempt < max_retries:
                time.sleep(0.5)
    else:
        raise RuntimeError(
            f"[bus] {port} 连接失败 (已重试 {max_retries} 次): {last_err}"
        )

    try:
        calibrated = bus.is_calibrated
    except Exception as e:
        logger.warning(f"[bus] {port} 校准检查失败 (舵机可能处于保护状态): {e}")
        calibrated = False

    if not calibrated:
        cal = {}
        for name, motor in bus.motors.items():
            cal[name] = MotorCalibration(
                id=motor.id, drive_mode=0, homing_offset=0,
                range_min=0, range_max=4095,
            )
        try:
            bus.write_calibration(cal)
        except Exception as e:
            logger.warning(f"[bus] {port} 写入校准失败 (强制使用默认值): {e}")
            bus.calibration = cal
    return bus


def build_arm_motors() -> dict[str, Motor]:
    motors = {
        n: Motor(ARM_MOTOR_IDS[n], "sts3215", MotorNormMode.RANGE_M100_100)
        for n in ARM_MOTOR_NAMES
    }
    motors["gripper"] = Motor(6, "sts3215", MotorNormMode.RANGE_0_100)
    return motors


def safe_disconnect(bus: Optional[FeetechMotorsBus],
                    disable_torque: bool = True,
                    label: str = "") -> None:
    """安全断开，吞掉所有异常"""
    if bus is None:
        return
    try:
        if bus.is_connected:
            bus.disconnect(disable_torque)
    except Exception as e:
        logger.warning(f"[{label}] 断开异常 (已忽略): {e}")
        try:
            bus.port_handler.closePort()
        except Exception:
            pass
