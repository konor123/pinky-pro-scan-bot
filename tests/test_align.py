"""스냅샷 디코드, 중앙값, 컬러 정렬, 왜곡 보정(docs/specs/align.md)."""
import copy
import json

import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import pytest

from scan3d import align, synth


@pytest.fixture(scope="module")
def board(repo_root):
    return json.loads((repo_root / "config" / "board.json").read_text(encoding="utf-8"))


def _low_sat(rng, h, w):
    base = cv2.GaussianBlur(rng.uniform(40, 220, (h, w)).astype(np.float32), (0, 0), 6)
    return np.clip(base[..., None] * np.array([0.95, 1.0, 1.05], np.float32), 0, 255).astype(np.uint8)


def _visible_in_depth(z_true_mm, depth_img, calib, tol_mm=5.0):
    """컬러 시점 참 깊이의 3D 점이 깊이 카메라에도 보이는 픽셀."""
    cc, dc = calib["color"], calib["depth"]
    x, y = align.color_rays(cc)
    z = z_true_mm / 1000.0
    P_c = np.stack([x * z, y * z, z], -1)
    R = np.asarray(calib["depth_to_color"]["rotation"]).reshape(3, 3).T
    t = np.asarray(calib["depth_to_color"]["translation"])
    P_d = (P_c - t) @ R  # Rᵀ(P_c − t)
    with np.errstate(divide="ignore", invalid="ignore"):
        ud = np.round(dc["fx"] * P_d[..., 0] / P_d[..., 2] + dc["ppx"])
        vd = np.round(dc["fy"] * P_d[..., 1] / P_d[..., 2] + dc["ppy"])
    inside = (z_true_mm > 0) & (ud >= 0) & (ud < dc["w"]) & (vd >= 0) & (vd < dc["h"])
    seen = np.zeros_like(inside)
    ui, vi = ud[inside].astype(int), vd[inside].astype(int)
    seen[inside] = np.abs(depth_img[vi, ui].astype(float) - P_d[..., 2][inside] * 1000.0) < tol_mm
    return seen


def _edges(z_mm, radius=3, slope_mm=1.0):
    """깊이 경계: 참 깊이의 기울기가 slope_mm/px를 넘거나 0인 픽셀에서 radius px 안.

    `rs.align`은 깊이 픽셀이 덮는 컬러 픽셀 사각형에 값을 옮겨 담고(겹치면 작은 값),
    그 오차는 기울기 × 약 1.5 px다. 그래서 끊김뿐 아니라 가파른 면도 경계로 본다.
    """
    gy, gx = np.gradient(z_mm.astype(np.float64))
    e = (np.hypot(gx, gy) > slope_mm) | (z_mm == 0)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
    return cv2.dilate(e.astype(np.uint8), k).astype(bool)


def test_decode(rng):
    bgr = _low_sat(rng, 720, 1280)
    back = align.decode_color({"w": 1280, "h": 720, "fmt": "yuyv"}, synth.to_yuyv(bgr))
    assert np.abs(back.astype(int) - bgr.astype(int)).mean() <= 3
    rgb = align.decode_color({"w": 1280, "h": 720, "fmt": "rgb8"}, bgr[..., ::-1].tobytes())
    assert np.array_equal(rgb, bgr)

    depth = [rng.integers(0, 6000, (480, 848)).astype(np.uint16) for _ in range(3)]
    hdr = {"w": 848, "h": 480, "fmt": "z16", "n": 3}
    out = align.decode_depth(hdr, b"".join(synth.to_z16(d) for d in depth), 0.001)
    assert len(out) == 3 and all(np.array_equal(a, b) for a, b in zip(out, depth))
    raw = b"".join(synth.to_z16((d * 10).astype(np.uint16)) for d in depth)
    out = align.decode_depth(hdr, raw, 0.0001)
    assert all(np.array_equal(a, b) for a, b in zip(out, depth))

    f = depth[0].astype("<f4") / 1000.0
    f[0, :4] = [np.nan, np.inf, -1.0, 0.0]
    out = align.decode_depth({"w": 848, "h": 480, "fmt": "f32", "n": 1}, f.tobytes(), 0.001)
    want = depth[0].copy()
    want[0, :4] = 0
    assert np.array_equal(out[0], want)


def test_median(rng):
    truth = rng.integers(500, 3000, (48, 64)).astype(np.int64)
    frames = [(truth + rng.integers(-2, 3, truth.shape)).astype(np.uint16) for _ in range(5)]
    for f in frames[:3]:
        f[0:8, 0:8] = 0  # 0이 3장인 영역
    frames[0][10:20, 10:20] = 0  # 0이 아닌 값 4개(짝수)
    for i, v in enumerate([1000, 1001, 1003, 1010]):
        frames[i + 1][10:20, 10:20] = v
    frames[0][30:40, 30:40] = 0
    frames[1][30:40, 30:40] = 0
    frames[2][30:40, 30:40] = 1000
    frames[3][30:40, 30:40] = 1001
    frames[4][30:40, 30:40] = 1005  # 0이 2장: 3개의 중앙값
    m = align.median_depth(frames).astype(np.int64)
    rest = np.ones(truth.shape, bool)
    rest[0:8, 0:8] = rest[10:20, 10:20] = rest[30:40, 30:40] = False
    assert np.abs(m - truth)[rest].max() <= 2
    assert (m[0:8, 0:8] == 0).all()
    assert (m[10:20, 10:20] == 1002).all()  # (1001 + 1003) / 2
    assert (m[30:40, 30:40] == 1001).all()
    two = align.median_depth([np.full((2, 2), 1000, np.uint16), np.full((2, 2), 1001, np.uint16)])
    assert (two == 1001).all()


def _check_align(z_true, out, seen, label):
    filled = (out[seen] > 0).mean()
    sel = seen & (out > 0) & ~_edges(z_true)
    err = np.abs(out[sel].astype(float) - z_true[sel].astype(float))
    p95, mx = np.percentile(err, 95), err.max()
    every = seen & (out > 0)
    err_all = np.abs(out[every].astype(float) - z_true[every].astype(float))
    print(f"{label}: filled {filled:.4f}, outside edges {sel.sum()} px ({sel.sum() / seen.sum():.2f}) "
          f"p95 {p95:.2f} max {mx:.2f} mm; all visible p95 {np.percentile(err_all, 95):.2f} mm")
    return filled, p95, mx


@pytest.mark.parametrize("role,phi", [("L", 30), ("H", 30), ("H", 200)])
def test_align(board, cfg, role, phi):
    calib = synth.default_calib()
    m = cfg["robots"][role]["mount"]
    T = synth.pose_on_circle(phi, cfg["scene"]["R"], m["z"], m["pitch_down_deg"])
    scene = synth.Scene(board, synth.asym_mesh())
    depth = scene.render_depth(synth.depth_pose(T, calib), calib["depth"])
    z_true = scene.render_depth(T, calib["color"])
    out = align.align_to_color(depth, calib)
    assert out.shape == (720, 1280) and out.dtype == np.uint16
    seen = _visible_in_depth(z_true, depth, calib)
    filled, p95, mx = _check_align(z_true, out, seen, f"align {role} phi {phi}")
    assert filled >= 0.95
    assert p95 <= 1.2
    assert mx <= 2.5


def _plane_mesh(T_cam_board):
    """컬러 카메라 앞 z = a + b·y 기운 평면(영상 위쪽 0.5 m, 아래쪽 0.94 m)."""
    b = 0.44 / 0.81
    a = 0.5 * (1 + 0.5625 * b)
    pts_c = []
    for y in (-0.9, 0.9):
        z = a / (1 - b * y)
        for x in (-1.2, 1.2):
            pts_c.append([x * z, y * z, z])
    pts_c = np.asarray(pts_c)
    Tbc = np.linalg.inv(T_cam_board)
    pts_b = pts_c @ Tbc[:3, :3].T + Tbc[:3, 3]
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(pts_b), o3d.utility.Vector3iVector([[0, 1, 3], [0, 3, 2]]))
    m.vertex_colors = o3d.utility.Vector3dVector(np.full((4, 3), 0.6))
    return m


def test_align_tilted_plane(board):
    calib = synth.default_calib()
    R_bc = np.column_stack([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
    T = np.eye(4)
    T[:3, :3] = R_bc.T
    T[:3, 3] = -R_bc.T @ np.array([0.0, 0.0, 3.0])
    scene = synth.Scene(board, _plane_mesh(T))
    depth = scene.render_depth(synth.depth_pose(T, calib), calib["depth"])
    z_true = scene.render_depth(T, calib["color"])
    assert 495 <= z_true[z_true > 0].min() <= 505 and 935 <= z_true.max() <= 945
    seen = _visible_in_depth(z_true, depth, calib)
    seen[:4], seen[-4:], seen[:, :4], seen[:, -4:] = False, False, False, False
    out = align.align_to_color(depth, calib)
    _, p95, mx = _check_align(z_true, out, seen, "tilted plane")
    assert p95 <= 1.2
    assert mx <= 2.5
    raw, _ = align._rs_align(depth, calib)
    _, p95_raw, mx_raw = _check_align(z_true, raw, seen, "tilted plane before z conversion")
    assert mx_raw > 2.5


def test_sync_retry(monkeypatch):
    calib = synth.default_calib()
    depth = np.full((480, 848), 800, np.uint16)
    pair = align._SoftwarePair(calib)
    try:
        first = None
        pair.push(depth, 0)
        first = pair.wait()
        assert first is None or not align._complete(first)
        fs, tries = align._sync(pair, depth)
        assert align._complete(fs)
    finally:
        pair.close()
    out, tries = align._rs_align(depth, calib)
    print(f"sync tries {tries}")
    assert tries >= 2 and (out > 0).mean() > 0.8

    pushed = []
    monkeypatch.setattr(align._SoftwarePair, "push", lambda self, d, i: pushed.append(i))
    monkeypatch.setattr(align._SoftwarePair, "wait", lambda self: None)
    with pytest.raises(RuntimeError):
        align.align_to_color(depth, calib)
    assert pushed == list(range(10))


def _sim_calib(w=640, h=360, f=554.0):
    c = {"w": w, "h": h, "fx": f, "fy": f, "ppx": (w - 1) / 2, "ppy": (h - 1) / 2, "model": "none", "coeffs": [0.0] * 5}
    return {"color": dict(c), "depth": dict(c),
            "depth_to_color": {"rotation": [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0], "translation": [0.0, 0.0, 0.0]},
            "depth_scale": 0.001}


def test_align_identity(board, cfg):
    calib = _sim_calib()
    R_bc = np.column_stack([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
    T = np.eye(4)
    T[:3, :3] = R_bc.T
    T[:3, 3] = -R_bc.T @ np.array([0.03, -0.02, 0.8])
    scene = synth.Scene(board, synth.asym_mesh())
    z = scene.render_depth(T, calib["depth"])
    color = scene.render_color(T, calib["color"])
    f32 = np.where(z > 0, z.astype(np.float32) / 1000.0 + 0.0002, np.nan).astype("<f4")
    hdr = {"color": {"w": 640, "h": 360, "fmt": "rgb8", "bytes": 640 * 360 * 3},
           "depth": {"w": 640, "h": 360, "fmt": "f32", "n": 1, "bytes": f32.nbytes}}
    fr = align.prepare(hdr, [color[..., ::-1].tobytes(), f32.tobytes()], calib)
    assert np.array_equal(fr.color, color)
    assert np.array_equal(fr.K, align.K_of(calib["color"]))
    assert (fr.depth[z > 0] > 0).mean() >= 0.95
    sel = (fr.depth > 0) & ~_edges(z)
    err = np.abs(fr.depth[sel].astype(int) - z[sel].astype(int))
    print(f"identity: filled {(fr.depth[z > 0] > 0).mean():.4f}, max {err.max()} mm over {sel.sum()} px")
    assert err.max() <= 1


def test_undistort():
    calib = copy.deepcopy(synth.default_calib())
    coeffs = [-0.055, 0.065, 0.0005, 0.0008, -0.021]
    calib["color"]["model"] = "inverse_brown_conrady"
    calib["color"]["coeffs"] = coeffs
    cc = calib["color"]
    intr = rs.intrinsics()
    intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy = 1280, 720, cc["fx"], cc["fy"], cc["ppx"], cc["ppy"]
    intr.model = rs.distortion.inverse_brown_conrady
    intr.coeffs = coeffs
    gu, gv = np.meshgrid(np.linspace(60, 1220, 13), np.linspace(50, 670, 8))
    truth = np.stack([gu.ravel(), gv.ravel()], -1)
    img = np.zeros((720, 1280), np.float32)
    yy, xx = np.mgrid[0:720, 0:1280]
    for u, v in truth:
        p = [(u - cc["ppx"]) / cc["fx"], (v - cc["ppy"]) / cc["fy"], 1.0]
        ud, vd = rs.rs2_project_point_to_pixel(intr, p)
        x0, y0 = int(round(ud)), int(round(vd))
        sl = (slice(y0 - 8, y0 + 9), slice(x0 - 8, x0 + 9))
        img[sl] += 200.0 * np.exp(-((xx[sl] - ud) ** 2 + (yy[sl] - vd) ** 2) / (2 * 1.5 ** 2))
    color = cv2.cvtColor(np.clip(img, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    depth = np.full((720, 1280), 700, np.uint16)
    c2, d2, K = align.undistort(color, depth, calib)
    assert np.array_equal(K, align.K_of(cc))
    g = c2[..., 0].astype(np.float64)
    worst = 0.0
    for u, v in truth:
        x0, y0 = int(round(u)), int(round(v))
        sl = (slice(y0 - 6, y0 + 7), slice(x0 - 6, x0 + 7))
        wgt = g[sl]
        cu, cv_ = (wgt * xx[sl]).sum() / wgt.sum(), (wgt * yy[sl]).sum() / wgt.sum()
        worst = max(worst, float(np.hypot(cu - u, cv_ - v)))
    print(f"undistort max point error {worst:.3f} px")
    assert worst < 0.5
    assert (d2[100:620, 100:1180] == 700).all()
    plain = synth.default_calib()
    c3, d3, _ = align.undistort(color, depth, plain)
    assert np.array_equal(c3, color) and np.array_equal(d3, depth)
    bad = copy.deepcopy(plain)
    bad["color"]["model"] = "kannala_brandt4"
    with pytest.raises(ValueError):
        align.undistort(color, depth, bad)
