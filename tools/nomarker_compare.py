#!/usr/bin/env python3
"""저장한 스캔을 표식 대신 바퀴 주행거리(odom)로 잡은 카메라 자세로 다시 합쳐, 표식 자세와 비교한다.

  tools/nomarker_compare.py <출력 폴더> L=scans/<id> H=scans/<id>

출발 사진 한 장의 표식으로 출발 위치와 카메라 높이·기울기·돌림을 맞추고, 장착 수평 위치·방향은 설정값을 쓴다.
그 뒤 시점은 odom 변화량만 더한다. 출력: pose_errors.json(시점별 위치·방향 어긋남), mesh_·cloud_{tag,odom}.ply,
odom/<역할>/views.json(바퀴 주행거리 자세)
"""
import sys, json, math, os, shutil
from pathlib import Path
import numpy as np, yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scan3d import dataset
from scan3d.robot import align, tagpose, drive
from scan3d.pair import recon

out = Path(sys.argv[1])
spec = dict(a.split("=", 1) for a in sys.argv[2:])
roles = list(spec)
first = Path(spec[roles[0]])
cfg = yaml.safe_load((first / "config.yaml").read_text())
board = tagpose.load_board(json.loads((first / "board.json").read_text()), cfg["board"]["tag_mm"])

def rot_angle(Ra, Rb):
    c = (np.trace(Ra @ Rb.T) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))

summary = {}
odom_dirs = []
for role in roles:
    scan = Path(spec[role]); rd, raw = scan / role, scan / role / "raw"
    mount = cfg["robots"][role]["mount"]
    calib = json.loads((raw / "calib.json").read_text()); calib = calib.get("calib", calib)
    hdrs = {p.stem: json.loads(p.read_text()) for p in sorted(raw.glob("[0-9]" * 6 + ".json"))}
    s0 = next(s for s, h in hdrs.items() if h.get("station", {}).get("use") == "start")
    h0 = hdrs[s0]
    fr = align.prepare(h0, dataset._payloads(raw, s0, h0), calib)
    p0 = tagpose.estimate(fr.color, fr.K, board)
    base0 = drive.base_from_cam(p0.T_cam_board, mount); o0 = h0["odom"]
    off = base0[2] - o0[2]
    views = json.loads((rd / "views.json").read_text())
    od = out / "odom" / role
    if od.exists(): shutil.rmtree(od)
    od.mkdir(parents=True)
    for name in ("color", "depth"):
        os.symlink((rd / name).resolve(), od / name)
    shutil.copy(rd / "camera_intrinsic.json", od / "camera_intrinsic.json")
    def se3(x, y, yaw):
        T = np.eye(4); c, s = math.cos(yaw), math.sin(yaw); T[:2, :2] = [[c, -s], [s, c]]; T[0, 3], T[1, 3] = x, y; return T
    T_board_base0 = se3(*base0)
    T_base_cam = np.linalg.inv(T_board_base0) @ np.linalg.inv(p0.T_cam_board)   # 출발 사진 한 장으로 카메라 장착(높이·기울기·돌림)을 맞춤
    T_odom0 = se3(*o0)
    rows, nv = [], []
    for v in views:
        o = hdrs[v["snap"]]["odom"]
        T_board_cam = T_board_base0 @ np.linalg.inv(T_odom0) @ se3(*o) @ T_base_cam
        To = np.linalg.inv(T_board_cam); Tt = np.asarray(v["T_cam_board"], float)
        co, ct = -To[:3, :3].T @ To[:3, 3], -Tt[:3, :3].T @ Tt[:3, 3]
        rows.append({"k": v["k"], "pos_mm": float(np.linalg.norm(co - ct) * 1000),
                     "xy_mm": float(np.linalg.norm((co - ct)[:2]) * 1000), "rot_deg": rot_angle(To[:3, :3], Tt[:3, :3])})
        nv.append({**v, "T_cam_board": To.tolist()})
    (od / "views.json").write_text(json.dumps(nv))
    odom_dirs.append(od)
    summary[role] = rows
    pm = [r["pos_mm"] for r in rows]; ra = [r["rot_deg"] for r in rows]
    print(f"[{role}] views {len(rows)}  pos err mm: first {pm[0]:.0f}  median {np.median(pm):.0f}  max {max(pm):.0f} (k{rows[int(np.argmax(pm))]['k']})  last {pm[-1]:.0f}"
          f" | rot err deg: median {np.median(ra):.1f}  max {max(ra):.1f}  last {ra[-1]:.1f}")

(out / "pose_errors.json").write_text(json.dumps(summary, indent=1))
for name, dirs in (("tag", [Path(spec[r]) / r for r in roles]), ("odom", odom_dirs)):
    res = recon.reconstruct(dirs, cfg)
    import open3d as o3d
    o3d.io.write_triangle_mesh(str(out / f"mesh_{name}.ply"), res.mesh)
    o3d.io.write_point_cloud(str(out / f"cloud_{name}.ply"), res.cloud)
    print(name, "dims_mm", None if res.dims_mm is None else tuple(round(x, 1) for x in res.dims_mm),
          "verts", len(res.mesh.vertices), "largest_frac", None if res.largest_frac is None else round(res.largest_frac, 3),
          "used", {r: len(u) for r, u in res.used.items()})
