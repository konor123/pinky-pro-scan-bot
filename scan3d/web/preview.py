"""맞추기 값: 스냅샷 한 장 → 표식 자세, 카메라 각도·거리, 깊이 거리, 표식을 그린 작은 사진(docs/specs/webui.md)."""
import math

import cv2
import numpy as np

from ..robot import align, tagpose
from ..vendor.pinky_commands import wrap_angle

THUMB_W = 640
JPEG_QUALITY = 80


def camera_angles(T_cam_board):
    """(높이 m, 숙인 각 °, 광축 수평 방향과 판 중심 방향의 차 °). 차가 양수면 판 중심이 화면 왼쪽."""
    T = np.asarray(T_cam_board, float)
    R = T[:3, :3]
    c = -R.T @ T[:3, 3]
    f = R.T @ np.array([0.0, 0.0, 1.0])            # 광축(판 좌표)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -f[2]))))
    aim = math.atan2(-c[1], -c[0]) - math.atan2(f[1], f[0])
    return float(c[2]), pitch, math.degrees(wrap_angle(aim))


def center_depth(depth_mm, frac=0.2):
    """가운데 frac 창에서 0이 아닌 깊이의 중앙값(m). 없으면 None."""
    h, w = depth_mm.shape
    dh, dw = max(1, int(h * frac / 2)), max(1, int(w * frac / 2))
    win = depth_mm[h // 2 - dh:h // 2 + dh, w // 2 - dw:w // 2 + dw]
    valid = win[win > 0]
    return float(np.median(valid)) / 1000.0 if valid.size else None


def thumbnail(color, board):
    """표식 테두리와 가운데 십자를 그린 JPEG 바이트(가로 THUMB_W)."""
    img = color.copy()
    corners, ids = tagpose.detect(img, board)
    if len(corners):
        cv2.aruco.drawDetectedMarkers(img, [c + tagpose.APRILTAG_OFFSET for c in corners], ids)
    h, w = img.shape[:2]
    cv2.line(img, (w // 2 - 30, h // 2), (w // 2 + 30, h // 2), (0, 255, 255), 2)
    cv2.line(img, (w // 2, h // 2 - 30), (w // 2, h // 2 + 30), (0, 255, 255), 2)
    small = cv2.resize(img, (THUMB_W, round(h * THUMB_W / w)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    return buf.tobytes() if ok else b""


def measure(header, payloads, calib, board, cfg, mount):
    """스냅샷 → (맞추기 값 dict, JPEG 바이트)."""
    frame = align.prepare(header, payloads, calib)
    scene = cfg["scene"]
    r_eff = float(scene["r_obj"]) + float(scene["place_err"])
    out = {"pose": False, "depth_center_m": center_depth(frame.depth)}
    pose = tagpose.estimate(frame.color, frame.K, board)
    if pose is None:
        out["reason"] = "표식이 2장 미만으로 보임: 카메라가 판을 보게 놓는다"
    else:
        T = pose.T_cam_board
        z, pitch, aim = camera_angles(T)
        x, y, _ = tagpose.cam_position(T)
        out.update(cam_x_m=x, cam_y_m=y,
            pose=True, phi_deg=tagpose.phi_deg(T), d_m=tagpose.distance_d(T, r_eff),
            cam_z_m=z, pitch_deg=pitch, aim_err_deg=aim,
            z_err_m=z - float(mount["z"]), pitch_err_deg=pitch - float(mount["pitch_down_deg"]),
            n_tags=pose.n_tags, reproj_px=pose.reproj_px)
    return out, thumbnail(frame.color, board)


def pair_check(phis, tol_deg):
    """{role: phi 또는 None} → (각도 차 H − L 또는 None, 맞는지, 고칠 방향 문구)."""
    if len(phis) < 2:
        return None, True, ""
    phi_l, phi_h = phis.get("L"), phis.get("H")
    if phi_l is None or phi_h is None:
        return None, False, "두 로봇 모두 표식이 보여야 각도 차를 잴 수 있다"
    gap = math.degrees(wrap_angle(math.radians(phi_h - phi_l)))
    if abs(abs(gap) - 180.0) <= tol_deg:
        return gap, True, f"각도 차 {abs(gap):.0f}°(180 ± {tol_deg}° 안)"
    need = math.degrees(wrap_angle(math.radians(phi_l + 180.0 - phi_h)))
    way = "반시계" if need > 0 else "시계"
    return gap, False, f"각도 차 {abs(gap):.0f}°: H를 판 둘레로 {way} 방향 {abs(need):.0f}° 옮긴다"
