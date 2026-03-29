#!/bin/bash
# 一次性设置 udev 规则，让 ttyACM* 设备自动获得 666 权限
# 运行方式: sudo bash setup_udev.sh

RULE_FILE="/etc/udev/rules.d/99-robot-serial.rules"

echo '# XLeRobot 舵机控制板 (CH343 USB-Serial)' > "$RULE_FILE"
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0666"' >> "$RULE_FILE"

udevadm control --reload-rules
udevadm trigger

echo "已写入 $RULE_FILE"
echo "udev 规则已生效，以后 USB 重插自动获得权限。"

# 同时立即修复当前端口
chmod 666 /dev/ttyACM* 2>/dev/null && echo "当前端口权限已修复" || true
