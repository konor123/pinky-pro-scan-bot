"""물체만 잘라 TSDF로 융합한 색 있는 메시와 치수(docs/specs/recon.md).

Open3D v0.20.0 `examples/python/t_reconstruction_system/integrate.py`의 `integrate()`를 따른다.
외부 파라미터는 시점마다 표식 자세 `T_cam_board`, 내부 파라미터는 왜곡 보정한 컬러 K 하나다.
"""
import json
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
    dims = measure(mesh, float(scene["floor_cut"]))
    return Result(mesh, cloud, dims, used, crop, largest_frac)


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
