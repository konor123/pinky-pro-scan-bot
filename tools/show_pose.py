#!/usr/bin/env python3
"""스냅샷 한 장으로 카메라와 로봇의 판 좌표 자세를 출력한다(TODOs 4장 좌표 확인).

  tools/show_pose.py --robot L     검출 표식을 그린 영상은 out/pose_L.jpg
"""
import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scan3d import config  # noqa: E402

from scan3d.robot import align, drive, tagpose  # noqa: E402
from scan3d.robot import link as robot_link  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description="표식 자세 확인")
    ap.add_argument("--robot", required=True, choices=config.ORDER)
    ap.add_argument("--config", default=None)
    args = ap.parse_args(argv)
    _, cfg = config.load_config(args.config)
    _, board = config.load_board(cfg)
    role = args.robot
    mount = cfg["robots"][role]["mount"]
    r_eff = cfg["scene"]["r_obj"] + cfg["scene"]["place_err"]
    link = robot_link.connect(cfg, role)
    try:
        hello = link.hello()
        header, payloads, _ = link.snapshot(None, "start", None)
    finally:
        link.close()
    frame = align.prepare(header, payloads, hello["calib"])
    pose = tagpose.estimate(frame.color, frame.K, board)
    img = frame.color.copy()
    if pose is None:
        print("표식이 2장보다 적게 보임")
        code = 1
    else:
        T = pose.T_cam_board
        x, y, z = tagpose.cam_position(T)
        bx, by, byaw = drive.base_from_cam(T, mount)
        print(f"카메라(판 좌표): x {x:+.3f} y {y:+.3f} z {z:+.3f} m, phi {tagpose.phi_deg(T):+.1f}°, "
              f"d {tagpose.distance_d(T, r_eff):.3f} m")
        print(f"로봇 기준점: x {bx:+.3f} y {by:+.3f} m, yaw {math.degrees(byaw):+.1f}°")
        print(f"표식 {pose.n_tags}장 {sorted(pose.ids)}, 재투영 {pose.reproj_px:.2f} px")
        cv2.drawFrameAxes(img, frame.K, np.zeros(5), cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], 0.1)
        code = 0
    out = ROOT / "out" / f"pose_{role}.jpg"
    out.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(out), img)
    print(f"영상: {out}")
    return code


if __name__ == "__main__":
    sys.exit(main())
