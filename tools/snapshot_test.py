#!/usr/bin/env python3
"""로봇 스냅샷 반복 시험(TODOs 3·4장). 스캔 PC의 저장소 폴더에서 실행한다.

  tools/snapshot_test.py --robot L --count 24 --save   받은 원본을 scans/<id>/L/raw/에 저장
  tools/snapshot_test.py --robot L --count 10 --spin   바퀴가 뜬 채 모터를 돌리며, 끝나면 throttled 확인
  tools/snapshot_test.py --robot L --ir                투사기를 끄고 적외선 영상 1장(out/ir_L.png)
"""
import argparse
import shlex
import statistics
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scan3d import config, dataset  # noqa: E402

from scan3d.robot import align  # noqa: E402
from scan3d.robot import link as robot_link  # noqa: E402

DOMAIN = {"L": 11, "H": 12}
ROS_ENV = ("source /opt/ros/jazzy/setup.bash && source ~/pinky_pro/install/local_setup.bash && "
           "export ROS_DOMAIN_ID={domain} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")


def ssh(host, command, **kw):
    return subprocess.run(["ssh", host, command], capture_output=True, text=True, **kw)


def start_spin(host, role, seconds):
    cmd = (ROS_ENV.format(domain=DOMAIN[role]) + f" && timeout {int(seconds)} ros2 topic pub -r 10 /cmd_vel "
           + shlex.quote("geometry_msgs/msg/Twist") + " " + shlex.quote("{angular: {z: 1.5}}"))
    return subprocess.Popen(["ssh", host, cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def ir_snapshot(link, cfg, role):
    d = cfg["capture"]["depth"]
    req = {"cmd": "snapshot", "ir": True, "n_skip": cfg["capture"]["n_skip"], "depth": [d["w"], d["h"], d["fps"]]}
    header, payloads = link._request(req, robot_link.SNAPSHOT_TIMEOUT)
    ir = header["ir"]
    img = np.frombuffer(payloads[0], np.uint8).reshape(ir["h"], ir["w"])
    out = ROOT / "out" / f"ir_{role}.png"
    out.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(out), img)
    print(f"적외선 영상: {out} (상대 로봇 쪽에 점 패턴이 없어야 함)")


def main(argv=None):
    ap = argparse.ArgumentParser(description="스냅샷 반복 시험")
    ap.add_argument("--robot", required=True, choices=config.ORDER)
    ap.add_argument("--count", type=int, default=5)
    ap.add_argument("--save", action="store_true", help="받은 원본을 scans/<id>/<role>/raw/에 저장")
    ap.add_argument("--spin", action="store_true", help="바퀴가 뜬 채 모터를 돌리며 찍고 throttled 확인")
    ap.add_argument("--ir", action="store_true", help="적외선 영상 1장")
    ap.add_argument("--config", default=None)
    args = ap.parse_args(argv)
    cfg_path, cfg = config.load_config(args.config)
    role = args.robot
    host = cfg["robots"][role]["host"]
    raw = None
    if args.save:
        board_path, _ = config.load_board(cfg)
        writer = dataset.ScanWriter(ROOT / cfg["station"]["data_dir"], cfg_path, board_path)
        raw = writer.role(role)
        print(f"저장: {writer.scan_dir}")
    link = robot_link.connect(cfg, role, raw)
    try:
        hello = link.hello()
        print(f"{role}: serial {hello['serial']}")
        if args.ir:
            ir_snapshot(link, cfg, role)
            return 0
        spin = start_spin(host, role, args.count * 15 + 30) if args.spin else None
        cap = cfg["capture"]
        want = (cap["color"]["w"] * cap["color"]["h"] * 2,
                cap["depth"]["w"] * cap["depth"]["h"] * 2 * cap["n_depth"])
        times, ok = [], 0
        for i in range(args.count):
            t0 = time.monotonic()
            try:
                header, payloads, _ = link.snapshot(i, "view", None)
                dt = time.monotonic() - t0
                sizes = (len(payloads[0]), len(payloads[1]))
                frame = align.prepare(header, payloads, hello["calib"])
                valid = float(np.count_nonzero(frame.depth)) / frame.depth.size
                good = sizes == want and valid > 0.05
                times.append(dt)
                ok += good
                print(f"{i + 1:3d}/{args.count} {dt:5.2f} s  bytes {sizes[0]}+{sizes[1]}  "
                      f"깊이 채움 {valid:.0%}  {'ok' if good else 'FAIL'}", flush=True)
            except Exception as e:  # noqa: BLE001  실패도 세고 계속
                print(f"{i + 1:3d}/{args.count} FAIL {type(e).__name__}: {e}", flush=True)
                if isinstance(e, (ConnectionError, OSError)):
                    link.close()
                    link = robot_link.connect(cfg, role, raw)
                    hello = link.hello()
        if spin is not None:
            spin.terminate()
            print(ssh(host, "vcgencmd get_throttled", timeout=15).stdout.strip() or "throttled 확인 실패")
        mean = statistics.mean(times) if times else float("nan")
        print(f"{'PASS' if ok == args.count else 'FAIL'} {ok}/{args.count}, 평균 {mean:.2f} s, "
              f"최대 {max(times) if times else float('nan'):.2f} s")
        if times and mean > 5.0:
            print("평균이 5 s를 넘음: config capture.compress를 켜고 다시 잰다")
        return 0 if ok == args.count else 1
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
