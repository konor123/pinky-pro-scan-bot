"""표식 판 배치 파일과 인쇄 PDF(docs/specs/board.md)."""
import json
import math
import shutil
import subprocess

import cv2
import numpy as np
import pytest

PX_PER_MM = 4
PAPER_MM = (594.0, 841.0)
BOARD_CENTER_MM = (297.0, 297.0)


@pytest.fixture(scope="module")
def board(repo_root):
    return json.loads((repo_root / "config" / "board.json").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def raster(repo_root, tmp_path_factory):
    """PDF를 4 px/mm 회색으로 래스터화."""
    if shutil.which("pdftoppm") is None:
        pytest.skip("pdftoppm 없음")
    out = tmp_path_factory.mktemp("board") / "page"
    dpi = PX_PER_MM * 25.4
    subprocess.run(["pdftoppm", "-r", f"{dpi:g}", "-gray", "-png", "-singlefile",
                    str(repo_root / "out" / "board_A1.pdf"), str(out)], check=True)
    img = cv2.imread(str(out) + ".png", cv2.IMREAD_GRAYSCALE)
    assert img.shape == (round(PAPER_MM[1] * PX_PER_MM), round(PAPER_MM[0] * PX_PER_MM))
    return img


def _px_to_board_m(cols, rows):
    """래스터 픽셀(정수가 픽셀 중심) → 판 좌표 m."""
    x = (np.asarray(cols) + 0.5) / PX_PER_MM - BOARD_CENTER_MM[0]
    y = (PAPER_MM[1] - BOARD_CENTER_MM[1]) - (np.asarray(rows) + 0.5) / PX_PER_MM
    return x / 1000.0, y / 1000.0


def _detector(method):
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = method
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.ArucoDetector(d, params)


def _board_obj(board):
    return {t["id"]: np.asarray(t["corners"], np.float64) for t in board["tags"]}


def test_layout(board):
    tags = board["tags"]
    assert len(tags) == 16
    assert sorted(t["id"] for t in tags) == list(range(16))
    s = board["tag_mm"] / 1000.0 / 2
    expected = []
    for j in range(-2, 3):
        for i in range(-2, 3):
            if max(abs(i), abs(j)) == 2:
                expected.append((i * 0.115, j * 0.115))
    for t, (cx, cy) in zip(sorted(tags, key=lambda t: t["id"]), expected):
        assert t["center"] == pytest.approx([cx, cy], abs=1e-9)
        want = [[cx - s, cy + s, 0.0], [cx + s, cy + s, 0.0], [cx + s, cy - s, 0.0], [cx - s, cy - s, 0.0]]
        assert np.allclose(t["corners"], want, atol=1e-9)
        c = np.asarray(t["corners"])
        # 가운데 ±187.5 mm 정사각 안으로 들어오는 표식 면적이 없다
        inside = (c[:, 0].max() > -0.1875 + 1e-9 and c[:, 0].min() < 0.1875 - 1e-9 and
                  c[:, 1].max() > -0.1875 + 1e-9 and c[:, 1].min() < 0.1875 - 1e-9)
        assert not inside


def test_pdf_detect(board, raster):
    corners, ids, _ = _detector(cv2.aruco.CORNER_REFINE_SUBPIX).detectMarkers(raster)
    assert ids is not None
    assert sorted(ids.ravel().tolist()) == list(range(16))
    obj = _board_obj(board)
    worst = 0.0
    for c, i in zip(corners, ids.ravel()):
        x, y = _px_to_board_m(c[0, :, 0], c[0, :, 1])
        err = np.hypot(x - obj[i][:, 0], y - obj[i][:, 1]) * 1000.0
        worst = max(worst, float(err.max()))
    print(f"pdf corner max error {worst:.3f} mm")
    assert worst < 0.5


def _look_at(pos, target):
    f = np.asarray(target, float) - pos
    f /= np.linalg.norm(f)
    x = np.cross(f, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(f, x)
    R_bc = np.column_stack([x, y, f])
    T = np.eye(4)
    T[:3, :3] = R_bc.T
    T[:3, 3] = -R_bc.T @ pos
    return T


def _render_plane(raster, T_cam_board, K, size, ss=3):
    """래스터를 판 평면(z = 0)에 놓고 핀홀 카메라로 투영. ss배 과표본 후 INTER_AREA."""
    w, h = size
    Ks = K.copy()
    Ks[0, 0] *= ss
    Ks[1, 1] *= ss
    Ks[0, 2] = K[0, 2] * ss + (ss - 1) / 2
    Ks[1, 2] = K[1, 2] * ss + (ss - 1) / 2
    A = np.array([[1 / (PX_PER_MM * 1000.0), 0, (0.5 / PX_PER_MM - BOARD_CENTER_MM[0]) / 1000.0],
                  [0, -1 / (PX_PER_MM * 1000.0), (PAPER_MM[1] - BOARD_CENTER_MM[1] - 0.5 / PX_PER_MM) / 1000.0],
                  [0, 0, 1.0]])
    H = Ks @ np.column_stack([T_cam_board[:3, 0], T_cam_board[:3, 1], T_cam_board[:3, 3]]) @ A
    big = cv2.warpPerspective(raster, H, (w * ss, h * ss), flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=90)
    return cv2.resize(big, (w, h), interpolation=cv2.INTER_AREA)


@pytest.mark.parametrize("pos", [(0.62, 0.0, 0.195), (-0.3, 0.55, 0.495), (0.35, -0.45, 0.6)])
def test_pdf_pose(board, raster, pos):
    K = np.array([[640.0, 0, 639.5], [0, 640.0, 359.5], [0, 0, 1]])
    T = _look_at(np.asarray(pos, float), (0.0, 0.0, 0.0))
    img = _render_plane(raster, T, K, (1280, 720))
    corners, ids, _ = _detector(cv2.aruco.CORNER_REFINE_SUBPIX).detectMarkers(img)
    assert ids is not None and len(ids) >= 4
    obj = _board_obj(board)
    op = np.concatenate([obj[i] for i in ids.ravel()])
    ip = np.concatenate([c.reshape(4, 2) for c in corners]).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(op, ip, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    assert ok
    R, _ = cv2.Rodrigues(rvec)
    pos_est = -R.T @ tvec.ravel()
    pos_true = -T[:3, :3].T @ T[:3, 3]
    dpos = np.linalg.norm(pos_est - pos_true) * 1000.0
    dang = math.degrees(np.linalg.norm(cv2.Rodrigues(R @ T[:3, :3].T)[0]))
    print(f"pdf pose {len(ids)} tags: position {dpos:.2f} mm, angle {dang:.3f} deg")
    assert dpos < 3.0
    assert dang < 1.0
