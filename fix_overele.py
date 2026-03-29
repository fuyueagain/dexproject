"""诊断并尝试清除 STS3215 舵机 OverEle (过载保护) 错误

用法: python fix_overele.py [--port /dev/ttyACM1] [--id 7]
"""

import sys
import time
import argparse
from pathlib import Path

sys.path.insert(0, str(Path.home() / "lerobot-MakerMods" / "src"))
import scservo_sdk as scs

STATUS_BITS = {
    0: "Voltage Error",
    1: "Angle Limit Error",
    2: "Overheating Error",
    3: "Range Error",
    4: "Checksum Error",
    5: "Overload Error (OverEle)",
    6: "Instruction Error",
}

ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55
ADDR_STATUS = 65
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_PRESENT_CURRENT = 69
ADDR_PROTECTION_CURRENT = 28
ADDR_OVER_CURRENT_PROTECTION_TIME = 38
ADDR_UNLOADING_CONDITION = 19


def parse_status(status_byte):
    flags = []
    for bit, desc in STATUS_BITS.items():
        if status_byte & (1 << bit):
            flags.append(f"  Bit{bit}: {desc}")
    return flags


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--id", type=int, default=7)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    args = parser.parse_args()

    servo_id = args.id
    print(f"=== STS3215 OverEle 诊断工具 ===")
    print(f"端口: {args.port}  舵机ID: {servo_id}  波特率: {args.baudrate}\n")

    port_handler = scs.PortHandler(args.port)
    packet_handler = scs.PacketHandler(0)

    if not port_handler.openPort():
        print("✗ 无法打开端口!")
        return 1
    port_handler.setBaudRate(args.baudrate)

    # --- Step 1: Ping ---
    print("--- Step 1: Ping 舵机 ---")
    model, comm, error = packet_handler.ping(port_handler, servo_id)
    if comm != scs.COMM_SUCCESS:
        print(f"✗ 通信失败: {packet_handler.getTxRxResult(comm)}")
        port_handler.closePort()
        return 1

    print(f"  模型号: {model}")
    if error != 0:
        print(f"  ⚠ 响应错误字节: 0x{error:02X}")
        for flag in parse_status(error):
            print(f"    {flag}")
    else:
        print(f"  ✓ 无错误标志，舵机状态正常!")
        port_handler.closePort()
        return 0

    # --- Step 2: 读取状态寄存器 ---
    print("\n--- Step 2: 读取诊断信息 ---")

    def read1(addr, name):
        val, c, e = packet_handler.read1ByteTxRx(port_handler, servo_id, addr)
        if c == scs.COMM_SUCCESS:
            print(f"  {name} (addr {addr}): {val}  (错误字节: 0x{e:02X})")
            return val
        else:
            print(f"  {name}: 读取失败 - {packet_handler.getTxRxResult(c)}")
            return None

    def read2(addr, name):
        val, c, e = packet_handler.read2ByteTxRx(port_handler, servo_id, addr)
        if c == scs.COMM_SUCCESS:
            print(f"  {name} (addr {addr}): {val}  (错误字节: 0x{e:02X})")
            return val
        else:
            print(f"  {name}: 读取失败 - {packet_handler.getTxRxResult(c)}")
            return None

    read1(ADDR_STATUS, "Status")
    read1(ADDR_TORQUE_ENABLE, "Torque_Enable")
    read2(ADDR_PRESENT_POSITION, "Present_Position")
    read1(ADDR_PRESENT_VOLTAGE, "Present_Voltage")
    read1(ADDR_PRESENT_TEMPERATURE, "Present_Temperature")
    read2(ADDR_PRESENT_CURRENT, "Present_Current")
    read2(ADDR_PROTECTION_CURRENT, "Protection_Current")
    read1(ADDR_OVER_CURRENT_PROTECTION_TIME, "Over_Current_Protection_Time")
    read1(ADDR_UNLOADING_CONDITION, "Unloading_Condition")

    # --- Step 3: 尝试清除错误 ---
    print("\n--- Step 3: 尝试清除过载保护 ---")
    print("  [1] 关闭扭矩 (Torque_Enable = 0)...")
    c, e = packet_handler.write1ByteTxRx(port_handler, servo_id, ADDR_TORQUE_ENABLE, 0)
    if c == scs.COMM_SUCCESS:
        print(f"      写入成功 (错误字节: 0x{e:02X})")
    else:
        print(f"      写入失败: {packet_handler.getTxRxResult(c)}")

    time.sleep(0.5)

    print("  [2] 等待 0.5s 后重新检查...")
    _, comm2, error2 = packet_handler.ping(port_handler, servo_id)
    if comm2 == scs.COMM_SUCCESS:
        if error2 == 0:
            print("  ✓✓ 过载保护已清除! 舵机恢复正常!")
        else:
            print(f"  ⚠ 错误仍存在: 0x{error2:02X}")
            for flag in parse_status(error2):
                print(f"    {flag}")

            print("\n  [3] 尝试解锁 EEPROM 并清除...")
            c, e = packet_handler.write1ByteTxRx(port_handler, servo_id, ADDR_LOCK, 0)
            time.sleep(0.2)
            c, e = packet_handler.write1ByteTxRx(port_handler, servo_id, ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.5)

            _, comm3, error3 = packet_handler.ping(port_handler, servo_id)
            if comm3 == scs.COMM_SUCCESS and error3 == 0:
                print("  ✓✓ 解锁后清除成功!")
            else:
                print(f"  ✗ 仍有错误: 0x{error3:02X}" if comm3 == scs.COMM_SUCCESS else "  ✗ 通信失败")
                print("\n  ⚠ 软件清除失败，需要硬件操作:")
                print("    1. 断开机器人电源 (拔电)")
                print("    2. 检查 ID=7 舵机是否被机械卡住")
                print("    3. 确保舵机能自由转动后重新上电")
                print("    4. 重新运行此脚本验证")
    else:
        print(f"  ✗ 通信失败: {packet_handler.getTxRxResult(comm2)}")

    # --- Step 4: 如果清除成功，验证读取 ---
    if comm2 == scs.COMM_SUCCESS and error2 == 0:
        print("\n--- Step 4: 验证读取 Min_Position_Limit ---")
        val, c, e = packet_handler.read2ByteTxRx(port_handler, servo_id, 9)
        if c == scs.COMM_SUCCESS and e == 0:
            print(f"  ✓ Min_Position_Limit = {val} (无错误)")
        else:
            err_str = packet_handler.getRxPacketError(e) if e else packet_handler.getTxRxResult(c)
            print(f"  ✗ 仍然失败: {err_str}")

    port_handler.closePort()
    print("\n完成。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
