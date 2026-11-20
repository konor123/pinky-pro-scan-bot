#!/usr/bin/env bash
# 로봇에 RealSense udev 규칙(sudo)과 ~/scan3d_venv(pyrealsense2)를 설치하고 카메라·프로파일을 확인한다.
# 사용: scripts/setup_robot.sh <host>
set -euo pipefail

if [ "$#" -lt 1 ] || [ -z "${1:-}" ]; then
    echo "usage: setup_robot.sh <host>" >&2
    exit 1
fi

HOST="$1"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CACHE_DIR="$ROOT/build/robot_setup"
WHEEL_DIR="$CACHE_DIR/wheels"
RULES_FILE="$CACHE_DIR/99-realsense-libusb.rules"
UDEV_URL="https://raw.githubusercontent.com/realsenseai/librealsense/v2.59.2/config/99-realsense-libusb.rules"
PKG_SPEC="pyrealsense2==2.59.2.11152"

PY="$ROOT/.venv/bin/python"
if [ ! -x "$PY" ]; then
    PY="python3"
fi

mkdir -p "$WHEEL_DIR"

if [ ! -s "$RULES_FILE" ]; then
    tmp_rules="$(mktemp "$CACHE_DIR/rules.XXXXXX")"
    trap 'rm -f "$tmp_rules"' EXIT
    curl -fsSL "$UDEV_URL" -o "$tmp_rules"
    chmod 0644 "$tmp_rules"
    mv "$tmp_rules" "$RULES_FILE"
    trap - EXIT
fi

if ! compgen -G "$WHEEL_DIR/pyrealsense2-2.59.2.11152-cp312-*aarch64*.whl" >/dev/null; then
    "$PY" -m pip download -q \
        --dest "$WHEEL_DIR" \
        --platform manylinux2014_aarch64 \
        --python-version 3.12 \
        --only-binary=:all: \
        "$PKG_SPEC"
fi

ssh "$HOST" "mkdir -p ~/.cache/scan3d/wheels"
rsync -a "$RULES_FILE" "$HOST:.cache/scan3d/99-realsense-libusb.rules"
rsync -a "$WHEEL_DIR/" "$HOST:.cache/scan3d/wheels/"

# udev 규칙 설치만 sudo 사용(비밀번호 입력을 위해 -t 사용)
ssh -t "$HOST" "sudo install -m 0644 ~/.cache/scan3d/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && sudo udevadm trigger"

ssh "$HOST" "if [ ! -x ~/scan3d_venv/bin/python ]; then python3 -m venv --system-site-packages ~/scan3d_venv; fi && ~/scan3d_venv/bin/python -m pip install -q --no-index --find-links ~/.cache/scan3d/wheels '$PKG_SPEC'"

ssh "$HOST" "~/scan3d_venv/bin/python -" <<'PY'
import sys
import pyrealsense2 as rs

devs = rs.context().query_devices()
if len(devs) == 0:
    print("RealSense camera not found", file=sys.stderr)
    sys.exit(1)

dev = devs[0]
serial = dev.get_info(rs.camera_info.serial_number)
usb = dev.get_info(rs.camera_info.usb_type_descriptor) if dev.supports(rs.camera_info.usb_type_descriptor) else ""
print(f"serial: {serial}")
print(f"usb: {usb}")
if not str(usb).startswith("3."):
    print(f"USB 3.x required, got {usb!r}", file=sys.stderr)
    sys.exit(1)

has_color = False
has_depth = False
for sensor in dev.query_sensors():
    for sp in sensor.get_stream_profiles():
        if not sp.is_video_stream_profile():
            continue
        vp = sp.as_video_stream_profile()
        st, w, h, fmt, fps = vp.stream_type(), vp.width(), vp.height(), vp.format(), vp.fps()
        if st == rs.stream.color and (w, h, fmt, fps) == (1280, 720, rs.format.yuyv, 15):
            has_color = True
        if st == rs.stream.depth and (w, h, fmt, fps) == (848, 480, rs.format.z16, 15):
            has_depth = True

print(f"color 1280x720 YUY2 15fps: {has_color}")
print(f"depth 848x480 Z16 15fps: {has_depth}")
if not (has_color and has_depth):
    sys.exit(1)
print("setup ok")
PY
