#!/usr/bin/env python3
"""이동 확인(TODOs 3·4장).

  tools/drive_check.py --robot L --odom-only   바퀴가 뜬 채: odom으로 forward 0.1, turn 30, 이동 중 stop
  tools/drive_check.py --robot L               판 옆에서: 표식으로 잰 forward 0.2, turn 45, turn -45
  tools/drive_check.py --robot L --arc 15      원 위에서: 표식으로 잰 원호 주행(판 중심 둘레로 15°)
"""
import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scan3d import config  # noqa: E402

from scan3d.robot import align, drive, tagpose  # noqa: E402
from scan3d.robot import link as robot_link  # noqa: E402
from scan3d.vendor.pinky_commands import wrap_angle  # noqa: E402


def line(name, ok, text):
    print(f"{'PASS' if ok else 'FAIL'} {name}: {text}", flush=True)
    return ok


def odom_only(link, speed):
    o0 = link.odom()
    o1 = link.move(f"speed {speed}; forward 0.1").odom
    dist = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
    ok = line("forward 0.1", abs(dist - 0.1) <= 0.01, f"odom {dist:.3f} m")
    o2 = link.move("turn 30").odom
    dyaw = math.degrees(wrap_angle(o2[2] - o1[2]))
    ok &= line("turn 30", abs(dyaw - 30) <= 3, f"odom {dyaw:.1f}°")
    t0 = link.now()
    res = link.move(f"speed {speed}; forward 0.3", watch=lambda: link.now() - t0 > 1.0)
    moved = math.hypot(res.odom[0] - o2[0], res.odom[1] - o2[1])
    link.sleep(0.5)
    o3 = link.odom()
    after = math.hypot(o3[0] - res.odom[0], o3[1] - res.odom[1])
    ok &= line("stop", res.stopped and moved < 0.25 and after < 0.01,
               f"stopped={res.stopped}, 멈출 때까지 {moved:.3f} m, 그 뒤 {after:.3f} m")
    return ok


def tag_pose(link, calib, board, mount):
    header, payloads, _ = link.snapshot(None, "start", None)
    frame = align.prepare(header, payloads, calib)
    pose = tagpose.estimate(frame.color, frame.K, board)
    if pose is None:
        raise RuntimeError("표식이 보이지 않음: 로봇을 판 옆에 카메라가 판을 보게 놓는다")
    return drive.base_from_cam(pose.T_cam_board, mount)


def with_tags(link, calib, board, mount, speed, wheel_radius, wheel_separation):
    """forward 0.2와 turn 45 → turn -45를 표식으로 잰다(90°를 돌면 판이 시야 밖으로 나간다)."""
    a = tag_pose(link, calib, board, mount)
    link.move(f"speed {speed}; forward 0.2")
    link.sleep(1.0)
    b = tag_pose(link, calib, board, mount)
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    ok = line("forward 0.2", abs(dist - 0.2) <= 0.02, f"표식 {dist:.3f} m")
    turns = []
    prev = b
    for cmd in (45, -45):
        link.move(f"turn {cmd}")
        link.sleep(1.0)
        cur = tag_pose(link, calib, board, mount)
        dyaw = math.degrees(wrap_angle(cur[2] - prev[2]))
        ok &= line(f"turn {cmd}", abs(dyaw - cmd) <= 2.5, f"표식 {dyaw:+.1f}°")
        turns.append(abs(dyaw))
        prev = cur
    if not ok:
        turned = sum(turns) / len(turns)
        print(f"config 제안: wheel_radius {wheel_radius * dist / 0.2:.4f}, "
              f"wheel_separation {wheel_separation * 45 / turned if turned > 1 else float('nan'):.4f}")
    return ok


def with_arc(link, hello, calib, board, mount, speed, angle):
    """지금 자리의 로봇 기준점 반지름으로 `arc`를 보내고, 표식으로 잰 도착 자리를 계산한 원호의 끝과 비교한다."""
    if not hello.get("arc"):
        print("FAIL arc: 로봇 에이전트가 원호 문장을 모름(scripts/deploy.sh로 새 에이전트를 배포)")
        return False
    x, y, yaw = tag_pose(link, calib, board, mount)
    radius = math.hypot(x, y)
    # 원호의 중심은 로봇 왼쪽(오른쪽으로 돌면 오른쪽) radius 거리의 점이다.
    side = 1.0 if angle > 0 else -1.0
    cx, cy = x - side*radius*math.sin(yaw), y + side*radius*math.cos(yaw)
    t0 = link.now()
    link.move(f"speed {speed}; arc {radius:.3f} {angle:.2f}")
    seconds = link.now() - t0
    link.sleep(1.0)
    bx, by, byaw = tag_pose(link, calib, board, mount)
    a = math.radians(angle)
    ex = cx + (x - cx)*math.cos(a) - (y - cy)*math.sin(a)
    ey = cy + (x - cx)*math.sin(a) + (y - cy)*math.cos(a)
    miss = math.hypot(bx - ex, by - ey)
    turned = math.degrees(wrap_angle(byaw - yaw))
    ok = line(f"arc {angle:g}", miss <= 0.02 + 0.01*abs(angle)/15 and abs(turned - angle) <= 2.5 + abs(angle)/30,
              f"도착 자리 차 {miss*1000:.0f} mm, 돈 각 {turned:+.1f}°, 판 중심에서 {radius:.3f} → "
              f"{math.hypot(bx, by):.3f} m, {seconds:.1f} s")
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description="이동 확인")
    ap.add_argument("--robot", required=True, choices=config.ORDER)
    ap.add_argument("--odom-only", action="store_true")
    ap.add_argument("--arc", type=float, metavar="DEG", help="원호 주행 확인(각도 °, 음수는 시계 방향)")
    ap.add_argument("--config", default=None)
    args = ap.parse_args(argv)
    _, cfg = config.load_config(args.config)
    role = args.robot
    rc = cfg["robots"][role]
    speed = cfg["drive"]["speed"]
    link = robot_link.connect(cfg, role)
    try:
        hello = link.hello()
        link.wait_streams()
        if args.odom_only:
            ok = odom_only(link, speed)
        elif args.arc:
            _, board = config.load_board(cfg)
            try:
                ok = with_arc(link, hello, hello["calib"], board, rc["mount"], speed, args.arc)
            except RuntimeError as exc:
                ok = line(f"arc {args.arc:g}", False, str(exc))
        else:
            _, board = config.load_board(cfg)
            ok = with_tags(link, hello["calib"], board, rc["mount"], speed,
                           rc["wheel_radius"], rc["wheel_separation"])
    finally:
        link.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
