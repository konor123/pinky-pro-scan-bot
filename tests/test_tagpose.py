"""표식 자세(docs/specs/tagpose.md)."""
import json
import math

import cv2
import numpy as np
import open3d as o3d
import pytest

from scan3d import align, synth, tagpose

PHIS = (0, 45, 90, 135)


@pytest.fixture(scope="module")
def board_json(repo_root):
    return json.loads((repo_root / "config" / "board.json").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def scene_cfg(repo_root):
    import yaml

    with open(repo_root / "config" / "scan.yaml", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _cylinder(r, h):
    m = o3d.geometry.TriangleMesh.create_cylinder(radius=r, height=h, resolution=96)
    m.translate((0.0, 0.0, h / 2))
    m.vertex_colors = o3d.utility.Vector3dVector(np.tile([0.55, 0.45, 0.35], (len(m.vertices), 1)))
    return m


def _degrade(img, rng):
    """흐림 σ 1.5 px, 잡음 σ 5."""
    b = cv2.GaussianBlur(img.astype(np.float32), (0, 0), 1.5)
    return np.clip(np.round(b + rng.normal(0.0, 5.0, b.shape)), 0, 255).astype(np.uint8)


@pytest.fixture(scope="module")
def views(board_json, scene_cfg):
    """(역할, phi, T 참값, 영상) 목록: config L·H 장착 높이·피치, 원 반지름 R."""
    sc = scene_cfg["scene"]
    r_eff = sc["r_obj"] + sc["place_err"]
    scene = synth.Scene(board_json, _cylinder(r_eff, sc["obj_h"]))
    calib = synth.default_calib()
    rng = np.random.default_rng(0)
    out = []
    for role in ("L", "H"):
        m = scene_cfg["robots"][role]["mount"]
        for phi in PHIS:
            T = synth.pose_on_circle(phi, sc["R"], m["z"], m["pitch_down_deg"])
            out.append((role, phi, T, _degrade(scene.render_color(T, calib["color"]), rng)))
    return out


def _K():
    return align.K_of(synth.default_calib()["color"])


def _errors(T_est, T_true):
    dpos = np.linalg.norm(np.subtract(tagpose.cam_position(T_est), tagpose.cam_position(T_true))) * 1000.0
    rv, _ = cv2.Rodrigues(T_est[:3, :3] @ T_true[:3, :3].T)
    return dpos, math.degrees(np.linalg.norm(rv))


def test_pose_views(views, board_json, scene_cfg):
    board = tagpose.load_board(board_json, scene_cfg["board"]["tag_mm"])
    worst_p = worst_a = 0.0
    for role, phi, T, img in views:
        pose = tagpose.estimate(img, _K(), board)
        assert pose is not None, (role, phi)
        dp, da = _errors(pose.T_cam_board, T)
        print(f"tagpose {role} phi {phi:3d}: {pose.n_tags} tags, {dp:.2f} mm, {da:.3f} deg, reproj {pose.reproj_px:.3f} px")
        worst_p, worst_a = max(worst_p, dp), max(worst_a, da)
        assert pose.n_tags >= 4
        assert dp < 3.0 and da < 1.0
    print(f"tagpose worst: {worst_p:.2f} mm, {worst_a:.3f} deg")


def test_corner_bias(views, board_json):
    board = tagpose.load_board(board_json, board_json["tag_mm"])
    K = _K()
    by_id = {t["id"]: np.asarray(t["corners"], np.float64) for t in board_json["tags"]}
    worst_mean = worst_one = 0.0
    for role, phi, T, img in views:
        corners, ids = tagpose.detect(img, board)
        rv, _ = cv2.Rodrigues(T[:3, :3])
        d = []
        for c, i in zip(corners, ids.ravel()):
            proj, _ = cv2.projectPoints(by_id[int(i)], rv, T[:3, 3], K, None)
            d.append(c.reshape(4, 2) - proj.reshape(4, 2))
        d = np.concatenate(d)
        mean = d.mean(0)
        one = np.abs(d).max()
        print(f"corner bias {role} phi {phi:3d}: mean ({mean[0]:+.3f}, {mean[1]:+.3f}) px, max {one:.3f} px")
        worst_mean, worst_one = max(worst_mean, np.abs(mean).max()), max(worst_one, one)
        assert abs(mean[0]) < 0.15 and abs(mean[1]) < 0.15
        assert one < 1.0
    print(f"corner bias worst: mean {worst_mean:.3f} px, single {worst_one:.3f} px")


def test_phi_d(board_json, scene_cfg):
    sc = scene_cfg["scene"]
    r_eff = sc["r_obj"] + sc["place_err"]
    m = scene_cfg["robots"]["H"]["mount"]
    T = synth.pose_on_circle(30, sc["R"], m["z"], m["pitch_down_deg"])
    assert tagpose.phi_deg(T) == pytest.approx(30.0, abs=1e-9)
    img = synth.Scene(board_json, _cylinder(r_eff, sc["obj_h"])).render_color(T, synth.default_calib()["color"])
    pose = tagpose.estimate(img, _K(), tagpose.load_board(board_json, scene_cfg["board"]["tag_mm"]))
    phi, d = tagpose.phi_deg(pose.T_cam_board), tagpose.distance_d(pose.T_cam_board, r_eff)
    print(f"phi {phi:.3f} deg, d {d:.4f} m (target {sc['R'] - r_eff:.4f})")
    assert phi == pytest.approx(30.0, abs=0.5)
    assert d == pytest.approx(sc["R"] - r_eff, abs=0.005)
    assert tagpose.phi_deg(synth.pose_on_circle(180, 0.62, 0.2, 10)) == pytest.approx(180.0)


def test_scale(views, board_json):
    K = _K()
    b0 = tagpose.load_board(board_json, board_json["tag_mm"])
    b1 = tagpose.load_board(board_json, board_json["tag_mm"] * 1.01)
    for role, phi, T, img in views[::3]:
        p0 = np.array(tagpose.cam_position(tagpose.estimate(img, K, b0).T_cam_board))
        p1 = np.array(tagpose.cam_position(tagpose.estimate(img, K, b1).T_cam_board))
        ratio = np.linalg.norm(p1) / np.linalg.norm(p0)
        assert ratio == pytest.approx(1.01, abs=1e-4)
        assert np.allclose(p1, p0 * 1.01, atol=1e-4)


def test_no_tags(board_json):
    gray = np.full((720, 1280, 3), 128, np.uint8)
    assert tagpose.estimate(gray, _K(), tagpose.load_board(board_json, 85.0)) is None


def test_one_tag_is_none(board_json, views):
    """서로 다른 id가 하나뿐이면 None."""
    board = tagpose.load_board(board_json, 85.0)
    role, phi, T, img = views[4]
    corners, ids = tagpose.detect(img, board)
    keep = corners[0].reshape(4, 2) + 0.5
    x0, y0 = np.floor(keep.min(0)).astype(int) - 6
    x1, y1 = np.ceil(keep.max(0)).astype(int) + 6
    only = np.full_like(img, 128)
    only[y0:y1, x0:x1] = img[y0:y1, x0:x1]
    assert tagpose.estimate(only, _K(), board) is None
