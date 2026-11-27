"""덮인 정도: 메시 꼭짓점마다 몇 개 시점에서, 얼마나 정면으로 보였는지(docs/specs/webui.md).

시점마다 `T_cam_board`, K, 깊이 사진(`<역할>/depth/`)으로 꼭짓점을 투영해 사진 안이고 깊이가 맞으면 보인 것.
점수는 보인 시점마다 cos(법선, 꼭짓점 → 카메라)의 합.
"""
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

from .. import dataset
from .focus import FOCUS_K0

DEPTH_TOL_M = 0.012      # 꼭짓점 깊이와 사진 깊이 차가 이 안이면 보인 것
MIN_COS = 0.2            # 이보다 비스듬하면(약 78° 넘게) 세지 않는다
GOOD = 3.0               # 점수가 이 이상이면 충분히 덮임(화면 초록)


def score(vertices, normals, views):
    """views: [(T_cam_board 4×4, K 3×3, depth uint16 mm (h, w))] → 꼭짓점 점수(float32, len(vertices))."""
    v = np.asarray(vertices, np.float64)
    n = np.asarray(normals, np.float64)
    nl = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, nl, out=np.zeros_like(n), where=nl > 1e-9)
    out = np.zeros(len(v), np.float32)
    hom = np.c_[v, np.ones(len(v))]
    for T, K, depth in views:
        T = np.asarray(T, np.float64)
        pc = hom @ T.T
        z = pc[:, 2]
        front = z > 0.05
        u = np.full(len(v), -1, np.int64)
        w = np.full(len(v), -1, np.int64)
        u[front] = np.round(K[0, 0] * pc[front, 0] / z[front] + K[0, 2]).astype(np.int64)
        w[front] = np.round(K[1, 1] * pc[front, 1] / z[front] + K[1, 2]).astype(np.int64)
        h, wd = depth.shape
        inside = front & (u >= 0) & (u < wd) & (w >= 0) & (w < h)
        d = np.zeros(len(v))
        d[inside] = depth[w[inside], u[inside]] / 1000.0
        seen = inside & (d > 0) & (np.abs(d - z) < DEPTH_TOL_M)
        cam = -T[:3, :3].T @ T[:3, 3]
        to_cam = cam[None, :] - v
        to_cam /= np.maximum(np.linalg.norm(to_cam, axis=1, keepdims=True), 1e-9)
        c = np.abs(np.sum(n * to_cam, axis=1))          # TSDF 메시 법선은 앞뒤가 섞여 있다
        seen &= c >= MIN_COS
        out[seen] += c[seen].astype(np.float32)
    return out


def scan_dir_of(mesh_path):
    """`<스캔>/out*/mesh.ply` → 스캔 폴더(역할 폴더에 데이터셋이 있을 때만) 또는 None."""
    scan = Path(mesh_path).resolve().parent.parent
    roles = [r for r in ("L", "H") if (scan / r / "views.json").is_file() and (scan / r / "depth").is_dir()]
    return (scan, roles) if roles else (None, [])


def coverage_for(mesh_path):
    """mesh.ply 꼭짓점 순서대로 점수 목록. 깊이 사진이 없으면 None(results/ 보관본).

    보강 촬영 시점(k ≥ FOCUS_K0)은 out_focus 메시에서만 센다.
    """
    scan, roles = scan_dir_of(mesh_path)
    with_focus = Path(mesh_path).resolve().parent.name == "out_focus"
    if scan is None:
        return None
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    views = []
    for r in roles:
        K, _, _, vs = dataset.load(scan / r)
        for view in vs:
            if int(view["k"]) >= FOCUS_K0 and not with_focus:
                continue
            p = scan / r / "depth" / f"{int(view['k']):06d}.png"
            depth = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
            if depth is not None:
                views.append((view["T_cam_board"], K, depth))
    s = score(np.asarray(mesh.vertices), np.asarray(mesh.vertex_normals), views)
    return {"scores": [round(float(x), 2) for x in s], "good": GOOD, "n_views": len(views),
            "low_frac": round(float(np.mean(s < 1.0)) if len(s) else 0.0, 3)}
