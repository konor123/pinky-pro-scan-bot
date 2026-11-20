"""자르기, TSDF 융합, 메시, 치수(docs/specs/recon.md)."""
import json
import math
import shutil

import cv2
import numpy as np
import pytest
import yaml

from scan3d import dataset, recon, synth
from test_dataset import new_scan, write_role

BOX = (0.19, 0.13, 0.16)
YAW_DEG = 20.0


def _load_cfg(repo_root):
    with open(repo_root / "config" / "scan.yaml", encoding="utf-8") as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def board_json(repo_root):
    return json.loads((repo_root / "config" / "board.json").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def box_scan(tmp_path_factory, repo_root, board_json):
    """면마다 색이 다른 상자(z축으로 20° 돌림), L·H 각 24시점, 깊이 잡음. raw → dataset.build까지."""
    cfg = _load_cfg(repo_root)
    mesh = synth.box_mesh(*BOX)
    mesh.rotate(mesh.get_rotation_matrix_from_xyz((0.0, 0.0, math.radians(YAW_DEG))), center=(0.0, 0.0, 0.0))
    scene = synth.Scene(board_json, mesh)
    n = cfg["scene"]["n_views"]
    w = new_scan(tmp_path_factory.mktemp("box"), repo_root)
    rng = np.random.default_rng(5)
    for role, phi0 in (("L", 0.0), ("H", 180.0)):
        shots = [(k, "view", phi0 + k * 360.0 / n, False) for k in range(n)]
        write_role(w, role, scene, cfg, shots, rng)
    w.close()
    for role in ("L", "H"):
        dataset.build(w.scan_dir, role)
    return w.scan_dir, cfg


def _red_face_ok(mesh):
    """+x(빨강) 면 영역 꼭짓점 중 R > G, R > B(RGB 순서)인 비율과 개수."""
    p = np.asarray(mesh.vertices)
    c = np.asarray(mesh.vertex_colors)
    a = math.radians(YAW_DEG)
    ex, ey = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    along, across = p[:, :2] @ ex, p[:, :2] @ ey
    sel = (np.abs(along - BOX[0] / 2) < 0.004) & (np.abs(across) < BOX[1] / 2 - 0.02) & \
        (p[:, 2] > 0.03) & (p[:, 2] < BOX[2] - 0.03)
    ok = (c[sel, 0] > c[sel, 1]) & (c[sel, 0] > c[sel, 2])
    return float(ok.mean()) if sel.any() else 0.0, int(sel.sum())


@pytest.mark.slow
def test_box_dims(box_scan):
    sd, cfg = box_scan
    res = recon.reconstruct([sd / "L", sd / "H"], cfg)
    assert res.dims_mm is not None
    w, d, h = res.dims_mm
    err = (w - BOX[0] * 1000, d - BOX[1] * 1000, h - BOX[2] * 1000)
    frac, n = _red_face_ok(res.mesh)
    print(f"recon dims {w:.1f} x {d:.1f} x {h:.1f} mm, error {err[0]:+.1f} {err[1]:+.1f} {err[2]:+.1f} mm, "
          f"largest component {res.largest_frac:.4f}, red face {frac:.3f} of {n} vertices, "
          f"used L {len(res.used['L'])} H {len(res.used['H'])}")
    assert all(abs(e) <= 5.0 for e in err)
    assert res.largest_frac >= 0.95
    assert n > 50 and frac >= 0.95
    assert res.used == {"L": list(range(24)), "H": list(range(24))}
    report = {"skipped": {"L": [], "H": []}, "stops": {"L": 0, "H": 0}, "corrections": {"L": 1, "H": 0},
              "aborted": None, "elapsed_s": 12.5}
    recon.write_outputs(sd, res, report)
    assert (sd / "out" / "mesh.ply").stat().st_size > 0
    assert (sd / "out" / "cloud.ply").stat().st_size > 0
    text = (sd / "out" / "report.txt").read_text(encoding="utf-8")
    assert "dims_mm" in text and "elapsed_s: 12.5" in text
    out = json.loads((sd / "L" / "outcomes.json").read_text(encoding="utf-8"))
    assert all(o["crop"] == "ok" and o["fused"] is True for o in out)


@pytest.mark.slow
def test_sparse_views(box_scan, tmp_path):
    sd, cfg = box_scan
    dst = tmp_path / sd.name
    shutil.copytree(sd, dst, ignore=shutil.ignore_patterns("raw", "out"))
    views = json.loads((dst / "L" / "views.json").read_text(encoding="utf-8"))
    (dst / "L" / "views.json").write_text(json.dumps([v for v in views if v["k"] not in (3, 4)]), encoding="utf-8")
    for k in (3, 4):
        (dst / "L" / "color" / f"{k:06d}.jpg").unlink()
        (dst / "L" / "depth" / f"{k:06d}.png").unlink()
    res = recon.reconstruct([dst / "L", dst / "H"], cfg)
    assert res.used["L"] == [k for k in range(24) if k not in (3, 4)]
    assert res.used["H"] == list(range(24))


def test_crop_empty(tmp_path, repo_root, board_json):
    cfg = _load_cfg(repo_root)
    w = new_scan(tmp_path, repo_root)
    write_role(w, "L", synth.Scene(board_json), cfg, [(0, "view", 0.0, False), (1, "view", 15.0, False)],
               np.random.default_rng(6))
    w.close()
    assert [v["k"] for v in dataset.build(w.scan_dir, "L")] == [0, 1]
    res = recon.reconstruct([w.scan_dir / "L"], cfg)
    assert res.dims_mm is None and len(res.mesh.triangles) == 0 and len(res.cloud.points) == 0
    assert res.used == {"L": []} and res.crop == {"L": {0: "empty", 1: "empty"}}
    recon.write_outputs(w.scan_dir, res, None)
    assert not (w.scan_dir / "out" / "mesh.ply").exists()
    out = json.loads((w.scan_dir / "L" / "outcomes.json").read_text(encoding="utf-8"))
    assert [(o["crop"], o["fused"]) for o in out[:2]] == [("empty", False), ("empty", False)]


def _write_role_dir(role_dir, K, w, h, views, depths):
    """dataset 형식 역할 폴더를 직접 쓴다(views: [(k, T)])."""
    (role_dir / "color").mkdir(parents=True)
    (role_dir / "depth").mkdir()
    (role_dir / "camera_intrinsic.json").write_text(json.dumps(
        {"width": w, "height": h, "intrinsic_matrix": K.T.reshape(-1).tolist()}), encoding="utf-8")
    out = []
    for (k, T), d in zip(views, depths):
        cv2.imwrite(str(role_dir / "color" / f"{k:06d}.jpg"), np.full((h, w, 3), 128, np.uint8))
        cv2.imwrite(str(role_dir / "depth" / f"{k:06d}.png"), d)
        out.append({"k": k, "snap": f"{k:06d}", "phi": 0.0, "d": 0.4, "T_cam_board": T.tolist(),
                    "n_tags": 8, "reproj_px": 0.2, "depth_invalid": 0.0})
    (role_dir / "views.json").write_text(json.dumps(out), encoding="utf-8")


def test_crop_sparse(tmp_path, repo_root):
    cfg = _load_cfg(repo_root)
    c = synth.default_calib()["color"]
    K = np.array([[c["fx"], 0, c["ppx"]], [0, c["fy"], c["ppy"]], [0, 0, 1.0]])
    R_bc = np.column_stack([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
    T = np.eye(4)
    T[:3, :3] = R_bc.T
    T[:3, 3] = -R_bc.T @ np.array([0.0, 0.0, 0.6])  # 0.5 m 깊이 점은 판 위 0.1 m
    depths = []
    for u in (501, 500):
        d = np.zeros((c["h"], c["w"]), np.uint16)
        d[300, u] = 500
        assert recon.crop_depth(d, K, T, cfg["scene"])[300, u] == 500
        depths.append(d)
    role_dir = tmp_path / "scan" / "L"
    _write_role_dir(role_dir, K, c["w"], c["h"], [(0, T), (1, T)], depths)
    res = recon.reconstruct([role_dir], cfg)
    assert res.crop == {"L": {0: "no_block", 1: "ok"}}
    assert res.used == {"L": [1]}
    recon.write_outputs(tmp_path / "scan", res, {"elapsed_s": 1.0})
    assert "no_block: 0" in (tmp_path / "scan" / "out" / "report.txt").read_text(encoding="utf-8")


def test_measure():
    m = synth.box_mesh(0.18, 0.12, 0.15)
    m.rotate(m.get_rotation_matrix_from_xyz((0.0, 0.0, math.radians(33))), center=(0.0, 0.0, 0.0))
    w, d, h = recon.measure(m, 0.003)
    assert (w, d, h) == pytest.approx((180.0, 120.0, 150.0), abs=0.01)
