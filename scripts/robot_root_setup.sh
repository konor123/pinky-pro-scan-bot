#!/bin/sh
# 로봇에서 root로 실행: RealSense udev 규칙 설치, 라즈베리파이 5 USB 전류 한도 1.6 A 설정.
# setup_robot.sh가 ~/.cache/scan3d/로 복사한 뒤 sudo로 실행한다. 여러 번 실행해도 결과가 같다.
set -eu

DIR="$(cd "$(dirname "$0")" && pwd)"
CFG=/boot/firmware/config.txt
RULES_DST=/etc/udev/rules.d/99-realsense-libusb.rules

install -m 0644 "$DIR/99-realsense-libusb.rules" "$RULES_DST"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --attr-match=idVendor=8086
echo "udev rules: $RULES_DST"

if grep -Eq '^[[:space:]]*usb_max_current_enable=1[[:space:]]*$' "$CFG"; then
    echo "config.txt: usb_max_current_enable=1 already set"
else
    cp -p "$CFG" "$CFG.bak-scan3d-$(date +%Y%m%d%H%M%S)"
    sed -i '/^[[:space:]]*usb_max_current_enable=/d' "$CFG"
    tail -c1 "$CFG" | od -An -c | grep -q '\\n' || echo >> "$CFG"
    printf '[all]\nusb_max_current_enable=1\n' >> "$CFG"
    echo "config.txt: usb_max_current_enable=1 added (reboot needed)"
fi
