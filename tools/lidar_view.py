#!/usr/bin/env python3
"""odom·LiDAR 스트림 확인과 가린 각도 찾기(TODOs 3·4장).

  tools/lidar_view.py --robot L          수신 주기, 정면 거리, 30° 부채꼴별 최소 거리
  tools/lidar_view.py --robot L --mask   기둥에 가린 각도(로봇 기준)를 찾아 config lidar.mask 값을 출력
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scan3d import config, protocol  # noqa: E402

from scan3d.robot import drive  # noqa: E402

MASK_NEAR = 0.25     # 이보다 가깝게 늘 보이는 각은 차체에 붙은 것(기둥)으로 본다


def collect(host, port, seconds):
    ch = protocol.connect(host, port)
    counts = {"odom": 0, "scan": 0, "status": 0}
    scans = []
    t_end = time.monotonic() + seconds
    try:
        while time.monotonic() < t_end:
            try:
                msg, _ = ch.recv(max(0.05, t_end - time.monotonic()))
            except TimeoutError:
                break
            typ = msg.get("type")
            counts[typ] = counts.get(typ, 0) + 1
            if typ == "scan":
                scans.append(protocol.decode_scan(msg))
    finally:
        ch.close()
    return counts, scans


def robot_angles(scan, mount_yaw_deg):
    n = len(scan["ranges"])
    a = np.degrees(scan["angle_min"] + scan["angle_inc"] * np.arange(n)) + mount_yaw_deg
    return (a + 180.0) % 360.0 - 180.0


def find_mask(scans, mount_yaw_deg):
    r = np.array([s["ranges"] for s in scans])
    near = np.median(np.where(np.isfinite(r), r, np.inf), axis=0) < MASK_NEAR
    ang = robot_angles(scans[-1], mount_yaw_deg)
    order = np.argsort(ang)
    ang, near = ang[order], near[order]
    step = abs(math.degrees(scans[-1]["angle_inc"]))
    ranges, start = [], None
    for a, b in zip(ang, near):
        if b and start is None:
            start = a
        if not b and start is not None:
            ranges.append([start - step, prev + step])
            start = None
        prev = a
    if start is not None:
        ranges.append([start - step, prev + step])
    if len(ranges) > 1 and ranges[0][0] <= -180 + 2 * step and ranges[-1][1] >= 180 - 2 * step:
        ranges[0] = [ranges[-1][0], ranges[0][1]]
        ranges.pop()
    return [[round(a, 1), round(b, 1)] for a, b in ranges]


def main(argv=None):
    ap = argparse.ArgumentParser(description="odom·LiDAR 스트림 확인")
    ap.add_argument("--robot", required=True, choices=config.ORDER)
    ap.add_argument("--mask", action="store_true")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--config", default=None)
    args = ap.parse_args(argv)
    _, cfg = config.load_config(args.config)
    rc = cfg["robots"][args.robot]
    counts, scans = collect(config.resolve_host(rc["host"]), rc["stream_port"], args.seconds)
    hz = {k: v / args.seconds for k, v in counts.items()}
    print(f"odom {hz.get('odom', 0):.1f} Hz, scan {hz.get('scan', 0):.1f} Hz")
    if not scans:
        print("FAIL scan을 받지 못함")
        return 1
    yaw = cfg["lidar"]["mount_yaw_deg"]
    last = scans[-1]
    ang = robot_angles(last, yaw)
    r = np.array(last["ranges"])
    print("로봇 기준 부채꼴(0° 앞, 반시계 +)별 최소 거리:")
    for lo in range(-180, 180, 30):
        sel = (ang >= lo) & (ang < lo + 30) & np.isfinite(r)
        print(f"  {lo:+4d}~{lo + 30:+4d}°: {r[sel].min():.2f} m" if sel.any() else f"  {lo:+4d}~{lo + 30:+4d}°: -")
    print(f"정면 ±{cfg['drive']['sector_half_deg']}° 최소(config mask 적용): "
          f"{drive.front_min(last, 0.0, cfg):.2f} m")
    if args.mask:
        mask = find_mask(scans, yaw)
        half = cfg["drive"]["sector_half_deg"]
        front = any(drive.in_mask(a, mask) for a in np.arange(-half, half + 0.5, 0.5))
        print(f"lidar.mask: {mask}")
        print(f"{'FAIL 정면 부채꼴이 가려짐: 기둥을 뒤쪽으로 옮긴다' if front else 'PASS 정면 부채꼴은 비어 있음'}")
        return 1 if front else 0
    ok = hz.get("odom", 0) > 10 and hz.get("scan", 0) > 5
    print("PASS" if ok else "FAIL 주기가 낮음")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
