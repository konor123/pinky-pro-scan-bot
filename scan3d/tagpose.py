"""표식 판 자세 `T_cam_board`(docs/specs/tagpose.md).

align이 편 컬러 영상과 K에서 보이는 표식 전부로 PnP를 한 번 푼다.
"""
import math
from collections import namedtuple

import cv2
import numpy as np

Pose = namedtuple("Pose", "T_cam_board n_tags reproj_px ids")

# OpenCV 4.11 CORNER_REFINE_APRILTAG 모서리는 정수 픽셀 중심 기준으로 (+0.5, +0.5) 치우친다.
APRILTAG_OFFSET = 0.5
# 모서리가 영상 가장자리에서 이보다 가까운 표식은 윤곽이 잘려 있으므로 버린다.
BORDER_PX = 2.0


def load_board(board_json, tag_mm):
    """배치 파일 dict → `cv2.aruco.Board`. 모서리에 tag_mm / 배치 파일 tag_mm을 곱한다."""
    s = float(tag_mm) / float(board_json["tag_mm"])
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, board_json["dictionary"]))
    obj = [np.asarray(t["corners"], np.float32) * s for t in board_json["tags"]]
    ids = np.array([int(t["id"]) for t in board_json["tags"]], np.int32)
    return cv2.aruco.Board(obj, dictionary, ids)


def _detector(board):
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return cv2.aruco.ArucoDetector(board.getDictionary(), params)


def detect(bgr, board):
    """배치 파일에 있고 영상 안에 다 들어온 표식의 (모서리 목록 [1×4×2], ids N×1). 모서리는 정수 픽셀 중심 기준."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    corners, ids, _ = _detector(board).detectMarkers(gray)
    if ids is None:
        return [], np.zeros((0, 1), np.int32)
    known = set(board.getIds().ravel().tolist())
    h, w = gray.shape[:2]
    keep = []
    for i, v in enumerate(ids.ravel()):
        c = corners[i].reshape(4, 2) - APRILTAG_OFFSET
        inside = (c.min() >= BORDER_PX and c[:, 0].max() <= w - 1 - BORDER_PX
                  and c[:, 1].max() <= h - 1 - BORDER_PX)
        if int(v) in known and inside:
            keep.append(i)
    corners = [corners[i] - APRILTAG_OFFSET for i in keep]
    return corners, ids[keep].reshape(-1, 1)


def estimate(bgr, K, board):
    """보이는 표식 전부로 `T_cam_board`. 서로 다른 id가 2개보다 적으면 None."""
    corners, ids = detect(bgr, board)
    if len(set(ids.ravel().tolist())) < 2:
        return None
    obj, img = board.matchImagePoints(corners, ids)
    obj = obj.reshape(-1, 3).astype(np.float64)
    img = img.reshape(-1, 2).astype(np.float64)
    K = np.asarray(K, np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, None)
    reproj = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    T = np.eye(4)
    T[:3, :3], _ = cv2.Rodrigues(rvec)
    T[:3, 3] = tvec.ravel()
    return Pose(T, len(ids), reproj, sorted(int(i) for i in ids.ravel()))


def cam_position(T_cam_board):
    """판 좌표의 카메라 위치 (x, y, z) = −Rᵀt."""
    T = np.asarray(T_cam_board, float)
    p = -T[:3, :3].T @ T[:3, 3]
    return float(p[0]), float(p[1]), float(p[2])


def phi_deg(T_cam_board):
    """카메라 위치의 극각 atan2(y, x), 도, (−180, 180]."""
    x, y, _ = cam_position(T_cam_board)
    a = math.degrees(math.atan2(y, x))
    return 180.0 if a == -180.0 else a


def distance_d(T_cam_board, r_eff):
    """카메라–물체 거리 hypot(x, y) − r_eff."""
    x, y, _ = cam_position(T_cam_board)
    return math.hypot(x, y) - r_eff
