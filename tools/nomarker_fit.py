#!/usr/bin/env python3
"""바퀴 주행거리 자세에서 장착(카메라의 차체 기준 수평 위치·방향)과 odom 거리·회전 배율을
표식 자세에 최소제곱으로 맞춘 뒤에도 남는 어긋남을 재고, 맞춘 자세로 다시 합친다.

  tools/nomarker_fit.py <출력 폴더> L=scans/<id> H=scans/<id> [--bounded]

--bounded: 장착 ±5 cm, 방향 ±5°, 배율 ±5 % 안에서만 맞춘다. 출력: fit_results[_bounded].json, mesh_fitted[_bounded].ply
"""
import sys, json, math, os, shutil
from pathlib import Path
import numpy as np, yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scan3d import dataset
from scan3d.robot import align, tagpose
from scan3d.pair import recon

args = [a for a in sys.argv[1:] if a != "--bounded"]
BOUND = "--bounded" in sys.argv[1:]
out = Path(args[0]); spec = dict(a.split("=", 1) for a in args[1:]); roles = list(spec)
cfg = yaml.safe_load((Path(spec[roles[0]]) / "config.yaml").read_text())
board = tagpose.load_board(json.loads((Path(spec[roles[0]]) / "board.json").read_text()), cfg["board"]["tag_mm"])
wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
def se3(x, y, yaw):
    T = np.eye(4); c, s = math.cos(yaw), math.sin(yaw); T[:2, :2] = [[c, -s], [s, c]]; T[0, 3], T[1, 3] = x, y; return T
def cam_planar(Tcb):
    R = Tcb[:3, :3]; c = -R.T @ Tcb[:3, 3]; return c[0], c[1], math.atan2(R[2, 1], R[2, 0])
def rot_deg(Ra, Rb):
    return math.degrees(math.acos(max(-1, min(1, (np.trace(Ra @ Rb.T) - 1) / 2))))

def predict(params, Tc0, o0, odoms, mount_yaw):
    mx, my, dpsi, s, g = params
    cx, cy, h = cam_planar(Tc0)
    th0 = h - (mount_yaw + dpsi)
    bx, by = cx - (math.cos(th0) * mx - math.sin(th0) * my), cy - (math.sin(th0) * mx + math.cos(th0) * my)
    T_board_base0 = se3(bx, by, th0)
    T_base_cam = np.linalg.inv(T_board_base0) @ np.linalg.inv(Tc0)
    inv0 = np.linalg.inv(se3(*o0)); outs = []
    for o in odoms:
        D = inv0 @ se3(*o); dyaw = math.atan2(D[1, 0], D[0, 0])
        Ds = se3(D[0, 3] * s, D[1, 3] * s, dyaw * g)
        outs.append(np.linalg.inv(T_board_base0 @ Ds @ T_base_cam))
    return outs

def cost(params, Tc0, o0, odoms, tags, mount_yaw, nominal=None):
    e = 0.0
    if BOUND and nominal is not None:   # 실제로 있을 법한 범위: 장착 ±5 cm, 방향 ±5°, 배율 ±5 %
        lim = [0.05, 0.05, math.radians(5), 0.05, 0.05]
        for v, n0, l in zip(params, nominal, lim):
            if abs(v - n0) > l: e += 1e3 * (abs(v - n0) - l) ** 2 + 1.0
    for Tp, Tt in zip(predict(params, Tc0, o0, odoms, mount_yaw), tags):
        a, b = cam_planar(Tp), cam_planar(Tt)
        e += (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (0.3 * wrap(a[2] - b[2])) ** 2   # 1 rad ≈ 0.3 m
    return e

def nelder_mead(f, x0, step, iters=3000):
    n = len(x0); pts = [np.array(x0, float)] + [np.array(x0, float) + np.eye(n)[i] * step[i] for i in range(n)]
    vals = [f(p) for p in pts]
    for _ in range(iters):
        o = np.argsort(vals); pts = [pts[i] for i in o]; vals = [vals[i] for i in o]
        c = np.mean(pts[:-1], 0); r = c + (c - pts[-1]); fr = f(r)
        if fr < vals[0]:
            e = c + 2 * (c - pts[-1]); fe = f(e)
            pts[-1], vals[-1] = (e, fe) if fe < fr else (r, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = r, fr
        else:
            k = c + 0.5 * (pts[-1] - c); fk = f(k)
            if fk < vals[-1]: pts[-1], vals[-1] = k, fk
            else:
                pts = [pts[0]] + [pts[0] + 0.5 * (p - pts[0]) for p in pts[1:]]; vals = [vals[0]] + [f(p) for p in pts[1:]]
    i = int(np.argmin(vals)); return pts[i], vals[i]

res_all, fit_dirs = {}, []
for role in roles:
    scan = Path(spec[role]); rd, raw = scan / role, scan / role / "raw"
    mount = cfg["robots"][role]["mount"]; mount_yaw = math.radians(mount["yaw_deg"])
    calib = json.loads((raw / "calib.json").read_text()); calib = calib.get("calib", calib)
    hdrs = {p.stem: json.loads(p.read_text()) for p in sorted(raw.glob("[0-9]" * 6 + ".json"))}
    s0 = next(s for s, h in hdrs.items() if h.get("station", {}).get("use") == "start")
    fr = align.prepare(hdrs[s0], dataset._payloads(raw, s0, hdrs[s0]), calib)
    Tc0 = tagpose.estimate(fr.color, fr.K, board).T_cam_board; o0 = hdrs[s0]["odom"]
    views = json.loads((rd / "views.json").read_text())
    odoms = [hdrs[v["snap"]]["odom"] for v in views]; tags = [np.asarray(v["T_cam_board"], float) for v in views]
    nominal = [mount["x"], mount["y"], 0.0, 1.0, 1.0]
    best, _ = nelder_mead(lambda p: cost(p, Tc0, o0, odoms, tags, mount_yaw, nominal), nominal, [0.05, 0.05, 0.2, 0.05, 0.05])
    rows = {}
    for name, p in (("nominal", nominal), ("fitted", best)):
        pe, re_ = [], []
        for Tp, Tt in zip(predict(p, Tc0, o0, odoms, mount_yaw), tags):
            pe.append(float(np.linalg.norm((-Tp[:3, :3].T @ Tp[:3, 3]) - (-Tt[:3, :3].T @ Tt[:3, 3])) * 1000)); re_.append(rot_deg(Tp[:3, :3], Tt[:3, :3]))
        rows[name] = {"pos_med": float(np.median(pe)), "pos_max": max(pe), "rot_med": float(np.median(re_)), "rot_max": max(re_)}
        print(f"[{role}] {name:8s} pos med {np.median(pe):5.0f} max {max(pe):5.0f} mm | rot med {np.median(re_):4.1f} max {max(re_):4.1f} deg")
    print(f"[{role}] fitted mount x {best[0]*1000:.0f} mm y {best[1]*1000:.0f} mm yaw {math.degrees(best[2]):+.1f} deg, odom scale {best[3]:.3f}, yaw scale {best[4]:.3f}")
    rows["params"] = {"mx_mm": best[0] * 1000, "my_mm": best[1] * 1000, "dyaw_deg": math.degrees(best[2]), "scale": best[3], "yaw_scale": best[4]}
    res_all[role] = rows
    od = out / ("fitted_bounded" if BOUND else "fitted") / role
    if od.exists(): shutil.rmtree(od)
    od.mkdir(parents=True)
    for n in ("color", "depth"): os.symlink((rd / n).resolve(), od / n)
    shutil.copy(rd / "camera_intrinsic.json", od / "camera_intrinsic.json")
    nv = [{**v, "T_cam_board": T.tolist()} for v, T in zip(views, predict(best, Tc0, o0, odoms, mount_yaw))]
    (od / "views.json").write_text(json.dumps(nv)); fit_dirs.append(od)
r = recon.reconstruct(fit_dirs, cfg)
import open3d as o3d
o3d.io.write_triangle_mesh(str(out / ("mesh_fitted_bounded.ply" if BOUND else "mesh_fitted.ply")), r.mesh)
res_all["fused_fitted"] = {"dims_mm": None if r.dims_mm is None else list(r.dims_mm), "largest_frac": r.largest_frac, "used": {k: len(v) for k, v in r.used.items()}}
print("fitted fuse dims", None if r.dims_mm is None else tuple(round(x, 1) for x in r.dims_mm), "largest_frac", None if r.largest_frac is None else round(r.largest_frac, 3), "used", res_all["fused_fitted"]["used"])
(out / ("fit_results_bounded.json" if BOUND else "fit_results.json")).write_text(json.dumps(res_all, indent=1))
