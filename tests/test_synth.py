"""합성 장면 렌더링(docs/specs/synth.md)."""
import json

import cv2
import numpy as np
import pytest

from scan3d import synth


@pytest.fixture(scope="module")
def board(repo_root):
    return json.loads((repo_root / "config" / "board.json").read_text(encoding="utf-8"))


def _down_pose(height):
    """판 중심 바로 위 height에서 아래를 보는 카메라 `T_cam_board`."""
    R_bc = np.column_stack([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
    T = np.eye(4)
    T[:3, :3] = R_bc.T
    T[:3, 3] = -R_bc.T @ np.array([0.0, 0.0, height])
    return T


def test_depth_plane(board):
    intr = synth.default_calib()["depth"]
    d = synth.Scene(board).render_depth(_down_pose(1.0), intr)
    assert d.shape == (480, 848) and d.dtype == np.uint16
    assert abs(int(d[240, 424]) - 1000) <= 1
    # 광축 거리 z: 판 안의 비스듬한 광선도 1000, 판 밖 바닥은 1005
    assert abs(int(d[130, 535]) - 1000) <= 1
    assert abs(int(d[0, 0]) - 1005) <= 1


def test_depth_box_top(board):
    intr = synth.default_calib()["depth"]
    d = synth.Scene(board, synth.box_mesh(0.2, 0.2, 0.18)).render_depth(_down_pose(1.0), intr)
    assert abs(int(d[240, 424]) - 820) <= 1


def test_color_board_corners(board, cfg):
    calib = synth.default_calib()
    m = cfg["robots"]["H"]["mount"]
    T = synth.pose_on_circle(0, cfg["scene"]["R"], m["z"], m["pitch_down_deg"])
    img = synth.Scene(board).render_color(T, calib["color"])
    assert img.shape == (720, 1280, 3) and img.dtype == np.uint8
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), params)
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    assert ids is not None and len(ids) >= 4
    c = calib["color"]
    K = np.array([[c["fx"], 0, c["ppx"]], [0, c["fy"], c["ppy"]], [0, 0, 1]])
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    by_id = {t["id"]: np.asarray(t["corners"], np.float64) for t in board["tags"]}
    diffs = []
    for cc, i in zip(corners, ids.ravel()):
        proj, _ = cv2.projectPoints(by_id[int(i)], rvec, T[:3, 3], K, None)
        diffs.append(cc.reshape(4, 2) - proj.reshape(4, 2))
    diffs = np.concatenate(diffs)
    mean = diffs.mean(0)
    print(f"synth corner bias {len(ids)} tags: mean ({mean[0]:+.3f}, {mean[1]:+.3f}) px, "
          f"max {np.abs(diffs).max():.3f} px")
    assert abs(mean[0]) < 0.15 and abs(mean[1]) < 0.15


def test_raw_bytes(rng):
    base = cv2.GaussianBlur(rng.uniform(40, 220, (720, 1280)).astype(np.float32), (0, 0), 6)
    tint = np.array([0.95, 1.0, 1.05], np.float32)
    bgr = np.clip(base[..., None] * tint + rng.normal(0, 4, (720, 1280, 3)), 0, 255).astype(np.uint8)
    depth = rng.integers(0, 4000, (480, 848)).astype(np.uint16)
    yuyv = synth.to_yuyv(bgr)
    z16 = synth.to_z16(depth)
    assert len(yuyv) == 1_843_200
    assert len(z16) == 814_080
    back = cv2.cvtColor(np.frombuffer(yuyv, np.uint8).reshape(720, 1280, 2), cv2.COLOR_YUV2BGR_YUYV)
    diff = np.abs(back.astype(int) - bgr.astype(int)).mean()
    print(f"yuyv roundtrip mean diff {diff:.3f}")
    assert diff <= 3
    assert np.array_equal(np.frombuffer(z16, "<u2").reshape(480, 848), depth)


def test_pose_on_circle_geometry():
    T = synth.pose_on_circle(90, 0.62, 0.5, 35)
    pos = -T[:3, :3].T @ T[:3, 3]
    assert pos == pytest.approx([0.0, 0.62, 0.495], abs=1e-9)
    z_axis = T[2, :3]  # 광축(판 좌표)
    assert z_axis == pytest.approx([0.0, -np.cos(np.radians(35)), -np.sin(np.radians(35))], abs=1e-9)
    assert T[0, :3] @ np.array([0, 0, 1.0]) == pytest.approx(0.0, abs=1e-12)  # x축 수평


def test_depth_pose_extrinsic():
    calib = synth.default_calib()
    T = synth.pose_on_circle(0, 0.62, 0.2, 10)
    Td = synth.depth_pose(T, calib)
    p_board = np.array([0.05, -0.02, 0.1, 1.0])
    p_d = Td @ p_board
    rot = np.asarray(calib["depth_to_color"]["rotation"]).reshape(3, 3).T
    p_c = rot @ p_d[:3] + calib["depth_to_color"]["translation"]
    assert p_c == pytest.approx((T @ p_board)[:3], abs=1e-12)


def test_deterministic(board):
    calib = synth.default_calib()
    scene = synth.Scene(board, synth.asym_mesh())
    T = synth.depth_pose(synth.pose_on_circle(45, 0.62, 0.2, 10), calib)
    a = scene.render_depth(T, calib["depth"], np.random.default_rng(3), 2)
    b = scene.render_depth(T, calib["depth"], np.random.default_rng(3), 2)
    assert np.array_equal(a, b)
    assert (a > 0).mean() > 0.5
