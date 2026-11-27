"""물체만 잘라 TSDF로 융합한 색 있는 메시와 치수(docs/specs/recon.md).

Open3D v0.20.0 `examples/python/t_reconstruction_system/integrate.py`의 `integrate()`를 따른다.
외부 파라미터는 시점마다 표식 자세 `T_cam_board`, 내부 파라미터는 왜곡 보정한 컬러 K 하나다.
색 다시 입히기는 `examples/python/pipelines/color_map_optimization.py`의 `run_rigid_optimizer`를 쓴다.

  python -m scan3d.pair.recon <역할 폴더>... [--config config/scan.yaml] [--out DIR] [--color-opt N]
"""
import argparse
import json
import os
import sys
from collections import namedtuple
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import open3d.core as o3c

from scan3d import dataset

Result = namedtuple("Result", "mesh cloud dims_mm used crop largest_frac")
Result.__doc__ = """mesh·cloud: 레거시 Open3D 객체(비면 빈 객체), dims_mm: (w, d, h) 또는 None,
used: {역할: [k…]}, crop: {역할: {k: "ok"|"empty"|"no_block"}},
largest_frac: 덩어리를 고르기 전 가장 큰 덩어리의 삼각형 비율(메시가 없으면 None)"""

DEPTH_SCALE = 1000.0
DEPTH_MAX = 1.5
BLOCK_RESOLUTION = 16
BLOCK_COUNT = 2000   # 처음 잡는 블록 수(16³ 복셀, 블록당 약 80 KB). 넘으면 Open3D가 늘린다
DEVICE = "CPU:0"


def _new_grid(voxel):
    return o3d.t.geometry.VoxelBlockGrid(
        attr_names=("tsdf", "weight", "color"),
        attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
        attr_channels=((1), (1), (3)),
        voxel_size=voxel,
        block_resolution=BLOCK_RESOLUTION,
        block_count=BLOCK_COUNT,
        device=o3c.Device(DEVICE))


def drop_depth_edges(depth, edge_mm):
    """3×3 이웃의 최대 − 최소가 edge_mm보다 크거나 이웃에 0이 있는 픽셀을 0으로 만든 사본(물체 가장자리의 섞인 깊이)."""
    if edge_mm <= 0:
        return depth
    valid = depth > 0
    d = depth.astype(np.float32)
    k = np.ones((3, 3), np.uint8)
    hi = cv2.dilate(np.where(valid, d, 0.0).astype(np.float32), k)
    lo = cv2.erode(np.where(valid, d, np.float32(np.inf)).astype(np.float32), k)
    near_hole = cv2.erode(valid.astype(np.uint8), k) == 0
    out = depth.copy()
    out[(hi - lo > edge_mm) | near_hole] = 0
    return out


def crop_depth(depth, K, T_cam_board, scene):
    """판 좌표로 옮긴 점이 자르기 원기둥 안인 깊이만 남긴다(나머지 0)."""
    r = float(scene["r_obj"]) + float(scene["place_err"]) + float(scene["crop_margin"])
    z_lo, z_hi = float(scene["floor_cut"]), float(scene["obj_h"]) + float(scene["crop_margin"])
    v, u = np.nonzero(depth)
    if len(u) == 0:
        return np.zeros_like(depth)
    z = depth[v, u].astype(np.float64) / DEPTH_SCALE
    pc = np.stack([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z], -1)
    Tbc = np.linalg.inv(np.asarray(T_cam_board, float))
    pb = pc @ Tbc[:3, :3].T + Tbc[:3, 3]
    keep = (np.hypot(pb[:, 0], pb[:, 1]) <= r) & (pb[:, 2] > z_lo) & (pb[:, 2] <= z_hi)
    out = np.zeros_like(depth)
    out[v[keep], u[keep]] = depth[v[keep], u[keep]]
    return out


def measure(mesh, floor_cut=0.003):
    """(w, d, h) mm. h는 꼭짓점 최대 z, w·d는 floor_cut 위 꼭짓점 xy의 최소 면적 직사각형 두 변(큰 것이 w)."""
    p = np.asarray(mesh.vertices)
    h = float(p[:, 2].max())
    xy = p[p[:, 2] > floor_cut, :2].astype(np.float32)
    (_, _), (a, b), _ = cv2.minAreaRect(xy)
    return float(max(a, b) * 1000.0), float(min(a, b) * 1000.0), h * 1000.0


def _empty(used, crop):
    return Result(o3d.geometry.TriangleMesh(), o3d.geometry.PointCloud(), None, used, crop, None)


def reconstruct(role_dirs, cfg):
    """역할 폴더들(`scans/<id>/L`, `…/H`)의 시점을 잘라 융합한다."""
    scene, rc = cfg["scene"], cfg["recon"]
    voxel = float(rc["voxel"])
    m = float(rc["trunc"]) / voxel
    vbg = None
    used, crop = {}, {}
    for role_dir in role_dirs:
        role_dir = Path(role_dir)
        role = role_dir.name
        K, _, _, views = dataset.load(role_dir)
        Kt = o3c.Tensor(K, o3c.float64)
        used[role], crop[role] = [], {}
        for v in views:
            k = int(v["k"])
            name = f"{k:06d}"
            depth = cv2.imread(str(role_dir / "depth" / f"{name}.png"), cv2.IMREAD_UNCHANGED)
            depth = drop_depth_edges(depth, float(rc.get("edge_mm", 0)))
            cut = crop_depth(depth, K, v["T_cam_board"], scene)
            if not cut.any():
                crop[role][k] = "empty"
                continue
            if vbg is None:
                vbg = _new_grid(voxel)
            dimg = o3d.t.geometry.Image(np.ascontiguousarray(cut)).to(vbg.attribute("tsdf").device)
            ext = o3c.Tensor(np.asarray(v["T_cam_board"], float), o3c.float64)
            try:
                coords = vbg.compute_unique_block_coordinates(dimg, Kt, ext, DEPTH_SCALE, DEPTH_MAX,
                                                              trunc_voxel_multiplier=m)
            except RuntimeError as e:
                if "No block is touched" not in str(e):
                    raise
                crop[role][k] = "no_block"
                continue
            color = o3d.t.io.read_image(str(role_dir / "color" / f"{name}.jpg"))
            vbg.integrate(coords, dimg, color, Kt, Kt, ext, DEPTH_SCALE, DEPTH_MAX, m)
            crop[role][k] = "ok"
            used[role].append(k)
    if vbg is None or not any(used.values()):
        return _empty(used, crop)
    mesh = vbg.extract_triangle_mesh().to_legacy()
    cloud = vbg.extract_point_cloud().to_legacy()
    del vbg
    if len(mesh.triangles) == 0:
        return _empty(used, crop)
    labels, counts, _ = mesh.cluster_connected_triangles()
    labels, counts = np.asarray(labels), np.asarray(counts)
    big = int(np.argmax(counts))
    largest_frac = float(counts[big] / counts.sum())
    mesh.remove_triangles_by_mask(labels != big)
    mesh.remove_unreferenced_vertices()
    iters = int(rc.get("color_opt_iter", 0))
    if iters > 0:
        mesh = recolor(mesh, role_dirs, used, iters)
    dims = measure(mesh, float(scene["floor_cut"]))
    return Result(mesh, cloud, dims, used, crop, largest_frac)


def recolor(mesh, role_dirs, used, iters):
    """융합에 쓴 시점의 사진으로 꼭짓점 색을 다시 구한다(Open3D color map, 강체 자세 다듬기). 모양은 그대로."""
    images, params = [], []
    for role_dir in role_dirs:
        role_dir = Path(role_dir)
        K, w, h, views = dataset.load(role_dir)
        intr = o3d.camera.PinholeCameraIntrinsic(w, h, K)
        ks = set(used.get(role_dir.name, []))
        for v in views:
            if int(v["k"]) not in ks:
                continue
            name = f"{int(v['k']):06d}"
            color = o3d.io.read_image(str(role_dir / "color" / f"{name}.jpg"))
            depth = o3d.io.read_image(str(role_dir / "depth" / f"{name}.png"))
            images.append(o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, depth, depth_scale=DEPTH_SCALE, depth_trunc=DEPTH_MAX, convert_rgb_to_intensity=False))
            p = o3d.camera.PinholeCameraParameters()
            p.intrinsic = intr
            p.extrinsic = np.asarray(v["T_cam_board"], float)
            params.append(p)
    if not images:
        return mesh
    traj = o3d.camera.PinholeCameraTrajectory()
    traj.parameters = params
    option = o3d.pipelines.color_map.RigidOptimizerOption(
        maximum_iteration=int(iters), maximum_allowable_depth=DEPTH_MAX,
        depth_threshold_for_visibility_check=0.01, depth_threshold_for_discontinuity_check=0.05)
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error):
        out, _ = o3d.pipelines.color_map.run_rigid_optimizer(mesh, images, traj, option)
    return out


def _fmt(v, spec):
    return "-" if v is None else format(v, spec)


def write_outputs(scan_dir, result, report):
    """`out/mesh.ply`, `out/cloud.ply`, `out/report.txt`, 역할별 `outcomes.json`의 `crop`·`fused`."""
    scan_dir = Path(scan_dir)
    out = scan_dir / "out"
    out.mkdir(exist_ok=True)
    if len(result.mesh.triangles) > 0:
        o3d.io.write_triangle_mesh(str(out / "mesh.ply"), result.mesh)
    if len(result.cloud.points) > 0:
        o3d.io.write_point_cloud(str(out / "cloud.ply"), result.cloud)
    report = report or {}
    lines = []
    if result.dims_mm is None:
        lines.append("dims_mm: -")
    else:
        w, d, h = result.dims_mm
        lines.append(f"dims_mm: w {w:.1f}  d {d:.1f}  h {h:.1f}")
    for role in sorted(result.crop):
        role_dir = scan_dir / role
        ks = result.used.get(role, [])
        lines.append("")
        lines.append(f"[{role}] fused {len(ks)} views: {' '.join(str(k) for k in ks)}")
        views = json.loads((role_dir / "views.json").read_text(encoding="utf-8")) \
            if (role_dir / "views.json").exists() else []
        lines.append("  k    phi      d_m     n_tags  reproj_px  depth_invalid  crop")
        for v in views:
            lines.append(f"  {v['k']:<4d} {_fmt(v['phi'], '7.1f')}  {_fmt(v['d'], '.3f'):>7s}  "
                         f"{_fmt(v['n_tags'], 'd'):>6s}  {_fmt(v['reproj_px'], '.3f'):>9s}  "
                         f"{_fmt(v['depth_invalid'], '.3f'):>13s}  {result.crop[role].get(int(v['k']), '-')}")
        path = role_dir / "outcomes.json"
        outcomes = json.loads(path.read_text(encoding="utf-8")) if path.exists() else []
        for o in outcomes:
            k = int(o["k"])
            o["crop"] = result.crop[role].get(k)
            o["fused"] = None if o["crop"] is None else k in ks
        if path.exists():
            path.write_text(json.dumps(outcomes, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
        skipped = {r: [o["k"] for o in outcomes if o.get("reason") == r] for r in ("bypass", "no_tags")}
        skipped["crop_empty"] = [k for k, s in result.crop[role].items() if s == "empty"]
        skipped["no_block"] = [k for k, s in result.crop[role].items() if s == "no_block"]
        for r, ks_ in skipped.items():
            lines.append(f"  skipped {r}: {' '.join(str(k) for k in ks_) or '-'}")
        corr = (report.get("corrections") or {}).get(role)
        lines.append(f"  distance corrections: {_fmt(corr, 'd')}")
    if report.get("aborted"):
        lines.append("")
        lines.append(f"aborted: {report['aborted']}")
    elapsed = report.get("elapsed_s")
    lines.append("")
    lines.append(f"elapsed_s: {_fmt(elapsed, '.1f')}")
    (out / "report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _report_lines(result, cfg, role_dirs):
    lines = ["dims_mm: -" if result.dims_mm is None else
             "dims_mm: w {:.1f}  d {:.1f}  h {:.1f}".format(*result.dims_mm)]
    for role_dir in role_dirs:
        role = Path(role_dir).name
        ks = result.used.get(role, [])
        lines.append(f"[{role}] {Path(role_dir).parent.name}/{role}: fused {len(ks)} views: {' '.join(str(k) for k in ks)}")
    rc, sc = cfg["recon"], cfg["scene"]
    lines.append(f"recon: voxel {rc['voxel']}  trunc {rc['trunc']}  edge_mm {rc.get('edge_mm', 0)}  "
                 f"color_opt_iter {rc.get('color_opt_iter', 0)}  floor_cut {sc['floor_cut']}")
    return lines


def write_dir(out, result, cfg, role_dirs, extra_lines=()):
    """다시 합친 결과를 out에 쓴다: mesh.ply, cloud.ply, report.txt. → report.txt 줄 목록."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    # 임시 이름에 쓴 뒤 바꿔 넣는다(쓰는 중인 파일을 화면이 읽지 않게). 비었으면 지난 결과를 지운다
    for name, ok, write in (("mesh.ply", len(result.mesh.triangles) > 0,
                             lambda p: o3d.io.write_triangle_mesh(p, result.mesh)),
                            ("cloud.ply", len(result.cloud.points) > 0,
                             lambda p: o3d.io.write_point_cloud(p, result.cloud))):
        if ok:
            tmp = out / f".{name[:-4]}.tmp.ply"
            write(str(tmp))
            os.replace(tmp, out / name)
        else:
            (out / name).unlink(missing_ok=True)
    lines = _report_lines(result, cfg, role_dirs) + list(extra_lines)
    (out / "report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return lines


def main(argv=None):
    from scan3d.config import ROOT, load_config

    ap = argparse.ArgumentParser(description="저장된 역할 폴더를 다시 융합한다(원래 폴더는 그대로, 설정은 기본으로 그 스캔의 config.yaml)")
    ap.add_argument("role_dirs", nargs="+", help="scans/<id>/L, scans/<id>/H 같은 역할 폴더")
    ap.add_argument("--config", default=None,
                    help="설정(기본: 역할 폴더들이 한 스캔 안이면 그 스캔의 config.yaml, 아니면 config/scan.yaml)")
    ap.add_argument("--out", help="결과 폴더(기본: 역할 폴더들이 한 스캔 안이면 <스캔>/out_recon)")
    ap.add_argument("--color-opt", type=int, metavar="N", help="recon.color_opt_iter를 N으로 바꿔 돌린다")
    args = ap.parse_args(argv)
    role_dirs = [Path(d).resolve() for d in args.role_dirs]
    if args.out:
        out = Path(args.out)
    elif len({d.parent for d in role_dirs}) == 1:
        out = role_dirs[0].parent / "out_recon"
    else:
        print("역할 폴더가 서로 다른 스캔에 있으면 --out이 필요하다", file=sys.stderr)
        return 2
    config = args.config
    if config is None:
        same = len({d.parent for d in role_dirs}) == 1 and (role_dirs[0].parent / "config.yaml").is_file()
        config = role_dirs[0].parent / "config.yaml" if same else ROOT / "config" / "scan.yaml"
    _, cfg = load_config(config)
    if args.color_opt is not None:
        cfg["recon"]["color_opt_iter"] = args.color_opt
    result = reconstruct(role_dirs, cfg)
    lines = write_dir(out, result, cfg, role_dirs)
    print("\n".join(lines))
    print(f"결과: {out}")
    return 0 if result.dims_mm is not None else 1


if __name__ == "__main__":
    sys.exit(main())
