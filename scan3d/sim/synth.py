"""합성 장면: 표식 판 + 물체를 알려진 자세의 컬러·깊이 카메라로 렌더링한다(docs/specs/synth.md).

픽셀 (u, v)의 광선 방향은 카메라 좌표 ((u − ppx)/fx, (v − ppy)/fy, 1)이다(정수 픽셀 중심).
"""
import math

import cv2
import numpy as np
import open3d as o3d

BOARD_Z = 0.0
FLOOR_Z = 0.0
MIN_DEPTH = 0.35
MAX_DEPTH = 65.0
INK, PAPER = 30, 225
LIGHT = np.array([0.4, 0.3, 0.866])
AMBIENT = 0.4
SUPER = 3
ROWS_PER_CHUNK = 256

DEFAULT_FACE_COLORS = (
    (0.85, 0.10, 0.10),  # +x 빨강
    (0.10, 0.65, 0.15),  # −x 초록
    (0.10, 0.25, 0.85),  # +y 파랑
    (0.90, 0.80, 0.10),  # −y 노랑
    (0.85, 0.85, 0.85),  # +z 흰색
    (0.40, 0.40, 0.40),  # −z
)


def _rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def default_calib():
    """`hello` 응답 `calib`과 같은 형식의 D455 비슷한 보정값."""
    rot = _rot_y(0.3)
    return {
        "color": {"w": 1280, "h": 720, "fx": 640.0, "fy": 640.0, "ppx": 639.5, "ppy": 359.5,
                  "model": "brown_conrady", "coeffs": [0.0] * 5},
        "depth": {"w": 848, "h": 480, "fx": 447.0, "fy": 447.0, "ppx": 423.5, "ppy": 239.5,
                  "model": "brown_conrady", "coeffs": [0.0] * 5},
        "depth_to_color": {"rotation": [float(v) for v in rot.T.reshape(-1)],
                           "translation": [-0.059, 0.0, 0.0]},
        "depth_scale": 0.001,
    }


def depth_to_color(calib):
    """깊이 → 컬러 변환 4×4(`p_color = T @ p_depth`). rotation은 열 우선."""
    ext = calib["depth_to_color"]
    T = np.eye(4)
    T[:3, :3] = np.asarray(ext["rotation"], float).reshape(3, 3).T
    T[:3, 3] = ext["translation"]
    return T


def depth_pose(T_cam_board, calib):
    """컬러 카메라 자세에서 깊이 카메라 자세 `T_depth_board`."""
    return np.linalg.inv(depth_to_color(calib)) @ np.asarray(T_cam_board, float)


def pose_on_circle(phi_deg, R, h, pitch_down_deg):
    """반지름 R 원의 각 phi, 높이 h(바닥 기준)에서 판 가운데를 향해 내려다보는 컬러 카메라 `T_cam_board`."""
    phi, p = math.radians(phi_deg), math.radians(pitch_down_deg)
    pos = np.array([R * math.cos(phi), R * math.sin(phi), h + FLOOR_Z])
    f = np.array([-math.cos(phi), -math.sin(phi), 0.0])
    z = math.cos(p) * f + math.sin(p) * np.array([0.0, 0.0, -1.0])
    x = np.cross(f, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T_board_cam = np.eye(4)
    T_board_cam[:3, :3] = np.column_stack([x, y, z])
    T_board_cam[:3, 3] = pos
    return np.linalg.inv(T_board_cam)


def board_texture(board, px_per_mm=4):
    """`board.json` 배치대로 그린 판 영상(회색 uint8, 위쪽이 +y). 텍셀 i의 중심은 (i + 0.5)/px_per_mm mm."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, board["dictionary"]))
    tag_mm, size_mm = float(board["tag_mm"]), float(board["board_mm"])
    half = size_mm / 2
    ss = next(s for s in (1, 2, 4, 8, 16)
              if all(abs(v * px_per_mm * s - round(v * px_per_mm * s)) < 1e-6 for v in (tag_mm / 8, half, 0.5)))
    ppm = px_per_mm * ss
    n = int(round(size_mm * ppm))
    img = np.full((n, n), PAPER, np.uint8)
    tag_px = int(round(tag_mm * ppm))
    for t in board["tags"]:
        bits = cv2.aruco.generateImageMarker(dictionary, int(t["id"]), 8, borderBits=1)
        marker = cv2.resize(bits, (tag_px, tag_px), interpolation=cv2.INTER_NEAREST)
        cx, cy = t["center"][0] * 1000.0, t["center"][1] * 1000.0
        c0 = int(round((cx - tag_mm / 2 + half) * ppm))
        r0 = int(round((half - (cy + tag_mm / 2)) * ppm))
        img[r0:r0 + tag_px, c0:c0 + tag_px] = np.where(marker > 0, PAPER, INK)

    def pts(xy_mm):
        return np.array([[(x + half) * ppm, (half - y) * ppm] for x, y in xy_mm], np.float64)

    shapes = [
        [(0, -1.5), (100, -1.5), (100, 1.5), (0, 1.5)],
        [(100, -7), (116, 0), (100, 7)],
        [(-1.5, 0), (1.5, 0), (1.5, 70), (-1.5, 70)],
        [(-7, 70), (0, 86), (7, 70)],
    ]
    for s in shapes:
        cv2.fillPoly(img, [np.round(pts(s) * 16).astype(np.int32)], INK, cv2.LINE_AA, shift=4)
    centre = (int(round(half * ppm * 16)), int(round(half * ppm * 16)))
    cv2.circle(img, centre, int(round(140 * ppm * 16)), 128, max(1, int(round(0.5 * ppm))), cv2.LINE_AA, shift=4)
    if ss > 1:
        img = cv2.resize(img, (n // ss, n // ss), interpolation=cv2.INTER_AREA)
    return img


def _box_parts(x0, x1, y0, y1, z0, z1, face_colors):
    corners = {
        0: [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],  # +x
        1: [(x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)],  # −x
        2: [(x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)],  # +y
        3: [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],  # −y
        4: [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],  # +z
        5: [(x0, y1, z0), (x1, y1, z0), (x1, y0, z0), (x0, y0, z0)],  # −z
    }
    verts, tris, cols = [], [], []
    for f in range(6):
        b = len(verts)
        verts += corners[f]
        cols += [face_colors[f]] * 4
        tris += [(b, b + 1, b + 2), (b, b + 2, b + 3)]
    return verts, tris, cols


def _mesh(verts, tris, cols):
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.asarray(verts, float)),
                                  o3d.utility.Vector3iVector(np.asarray(tris, np.int32)))
    m.vertex_colors = o3d.utility.Vector3dVector(np.asarray(cols, float))
    m.compute_triangle_normals()
    return m


def box_mesh(w, d, h, face_colors=None):
    """판 가운데(z = 0)에 놓인 w × d × h 상자. 면 순서 +x, −x, +y, −y, +z, −z, 색은 RGB 0~1."""
    fc = DEFAULT_FACE_COLORS if face_colors is None else face_colors
    return _mesh(*_box_parts(-w / 2, w / 2, -d / 2, d / 2, 0.0, h, fc))


def asym_mesh(r_obj=0.14, obj_h=0.20):
    """돌려 보면 모양이 다른 시험 물체: 낮은 상자 + 높은 원기둥(반지름 r_obj, 높이 obj_h 안)."""
    s = r_obj / 0.14
    verts, tris, cols = _box_parts(-0.11 * s, 0.05 * s, -0.08 * s, 0.02 * s, 0.0, 0.6 * obj_h,
                                   [(0.80, 0.55, 0.30)] * 6)
    cyl = o3d.geometry.TriangleMesh.create_cylinder(radius=0.05 * s, height=0.95 * obj_h, resolution=48)
    cyl.translate((0.06 * s, 0.05 * s, 0.95 * obj_h / 2))
    b = len(verts)
    verts += np.asarray(cyl.vertices).tolist()
    tris += (np.asarray(cyl.triangles) + b).tolist()
    cols += [(0.25, 0.45, 0.75)] * len(cyl.vertices)
    return _mesh(verts, tris, cols)


def _intr(intr):
    return float(intr["fx"]), float(intr["fy"]), float(intr["ppx"]), float(intr["ppy"]), int(intr["w"]), int(intr["h"])


class Scene:
    """판(z = 0, `board_mm` 정사각), 판 밖 바닥(z = 0, 용지를 바닥에 바로 편다), 물체 메시."""

    def __init__(self, board, mesh=None, floor_gray=90):
        self.board = board
        self.half = float(board["board_mm"]) / 2000.0
        self.floor_gray = float(floor_gray)
        self.px_per_mm = 4
        self.texture = board_texture(board, self.px_per_mm).astype(np.float32)
        self.mesh = mesh
        self.rc = None
        if mesh is not None and len(mesh.triangles) > 0:
            m = o3d.geometry.TriangleMesh(mesh)
            m.compute_triangle_normals()
            self.vertices = np.asarray(m.vertices)
            self.triangles = np.asarray(m.triangles)
            self.normals = np.asarray(m.triangle_normals)
            self.colors = (np.asarray(m.vertex_colors) if m.has_vertex_colors()
                           else np.full_like(self.vertices, 0.7))
            self.rc = o3d.t.geometry.RaycastingScene()
            self.rc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m))

    def _mesh_window(self, T, fx, fy, cx, cy, w, h):
        """메시가 영상에 맞을 수 있는 픽셀 범위 (u0, u1, v0, v1), 없으면 None."""
        if self.rc is None:
            return None
        p = self.vertices @ T[:3, :3].T + T[:3, 3]
        if np.any(p[:, 2] <= 1e-3):
            return 0, w, 0, h
        u = fx * p[:, 0] / p[:, 2] + cx
        v = fy * p[:, 1] / p[:, 2] + cy
        u0, u1 = max(0, int(math.floor(u.min())) - 2), min(w, int(math.ceil(u.max())) + 3)
        v0, v1 = max(0, int(math.floor(v.min())) - 2), min(h, int(math.ceil(v.max())) + 3)
        if u0 >= u1 or v0 >= v1:
            return None
        return u0, u1, v0, v1

    def _trace(self, T_cam_board, fx, fy, cx, cy, w, h, rows):
        """행 범위 rows의 광선: (t, 판 좌표 맞은 점, 종류 0 없음·1 판·2 바닥·3 물체, 삼각형, 무게중심 uv)."""
        T = np.asarray(T_cam_board, float)
        Tinv = np.linalg.inv(T)
        Rb, o = Tinv[:3, :3], Tinv[:3, 3]
        v, u = np.mgrid[rows[0]:rows[1], 0:w].astype(np.float64)
        dc = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], -1)
        d = dc @ Rb.T
        t = np.full(u.shape, np.inf)
        kind = np.zeros(u.shape, np.uint8)
        with np.errstate(divide="ignore", invalid="ignore"):
            tb = (BOARD_Z - o[2]) / d[..., 2]
            tf = (FLOOR_Z - o[2]) / d[..., 2]
        xb, yb = o[0] + tb * d[..., 0], o[1] + tb * d[..., 1]
        on_board = (tb > 0) & (np.abs(xb) <= self.half) & (np.abs(yb) <= self.half)
        on_floor = ~on_board & (tf > 0)
        t[on_board], kind[on_board] = tb[on_board], 1
        t[on_floor], kind[on_floor] = tf[on_floor], 2
        prim = np.zeros(u.shape, np.int64)
        uv = np.zeros(u.shape + (2,), np.float64)
        win = self._mesh_window(T, fx, fy, cx, cy, w, h)
        if win is not None:
            u0, u1, v0, v1 = win
            r0, r1 = max(v0, rows[0]) - rows[0], min(v1, rows[1]) - rows[0]
            if r0 < r1:
                sub_d = d[r0:r1, u0:u1]
                rays = np.concatenate([np.broadcast_to(o, sub_d.shape), sub_d], -1).astype(np.float32)
                ans = self.rc.cast_rays(o3d.core.Tensor(rays.reshape(-1, 6)))
                th = ans["t_hit"].numpy().reshape(sub_d.shape[:2]).astype(np.float64)
                sub_t = t[r0:r1, u0:u1]
                hit = np.isfinite(th) & (th < sub_t)
                sub_t[hit] = th[hit]
                kind[r0:r1, u0:u1][hit] = 3
                prim[r0:r1, u0:u1][hit] = ans["primitive_ids"].numpy().reshape(th.shape)[hit]
                uv[r0:r1, u0:u1][hit] = ans["primitive_uvs"].numpy().reshape(th.shape + (2,))[hit]
        p = o + t[..., None] * d
        return t, p, kind, prim, uv

    def _shade(self, p, kind, prim, uv):
        out = np.zeros(kind.shape + (3,), np.float32)
        out[kind == 2] = self.floor_gray
        b = kind == 1
        if b.any():
            ppm = self.px_per_mm
            mx = ((p[..., 0] + self.half) * 1000.0 * ppm - 0.5).astype(np.float32)
            my = ((self.half - p[..., 1]) * 1000.0 * ppm - 0.5).astype(np.float32)
            g = cv2.remap(self.texture, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
            out[b] = g[b][:, None]
        o_ = kind == 3
        if o_.any():
            tri = self.triangles[prim[o_]]
            a, bb = uv[o_][:, 0:1], uv[o_][:, 1:2]
            col = (1 - a - bb) * self.colors[tri[:, 0]] + a * self.colors[tri[:, 1]] + bb * self.colors[tri[:, 2]]
            light = LIGHT / np.linalg.norm(LIGHT)
            lam = AMBIENT + (1 - AMBIENT) * np.clip(self.normals[prim[o_]] @ light, 0.0, 1.0)
            rgb = np.clip(col * lam[:, None], 0.0, 1.0) * 255.0
            out[o_] = rgb[:, ::-1]
        return out

    def render_color(self, T_cam_board, intr):
        """BGR uint8. 3배 과표본(표본 위치 u − 1/3, u, u + 1/3) 후 `INTER_AREA` 축소."""
        fx, fy, cx, cy, w, h = _intr(intr)
        S = SUPER
        fxs, fys, cxs, cys, ws, hs = fx * S, fy * S, cx * S + (S - 1) / 2, cy * S + (S - 1) / 2, w * S, h * S
        out = np.zeros((h, w, 3), np.uint8)
        step = ROWS_PER_CHUNK * S
        for r in range(0, hs, step):
            r1 = min(hs, r + step)
            _, p, kind, prim, uv = self._trace(T_cam_board, fxs, fys, cxs, cys, ws, hs, (r, r1))
            big = self._shade(p, kind, prim, uv)
            small = cv2.resize(big, (w, (r1 - r) // S), interpolation=cv2.INTER_AREA)
            out[r // S:r1 // S] = np.clip(np.round(small), 0, 255).astype(np.uint8)
        return out

    def render_depth(self, T_depth_board, intr, rng=None, noise_mm=0):
        """광축 방향 거리 z(uint16 mm). 맞는 면이 없거나 0.35 m보다 가까우면 0.
        `rng`가 있으면 σ = noise_mm × (z/1 m)² 잡음."""
        fx, fy, cx, cy, w, h = _intr(intr)
        z, _, kind, _, _ = self._trace(T_depth_board, fx, fy, cx, cy, w, h, (0, h))
        valid = (kind > 0) & (z >= MIN_DEPTH) & (z <= MAX_DEPTH)
        z = np.where(valid, z, 0.0)
        mm = z * 1000.0
        if rng is not None and noise_mm:
            mm = mm + rng.normal(0.0, 1.0, mm.shape) * noise_mm * z ** 2
        return np.where(valid, np.clip(np.round(mm), 1, 65535), 0).astype(np.uint16)


def to_yuyv(bgr):
    """BGR → YUY2 바이트(Y0 U Y1 V, BT.601 제한 범위, `cv2.COLOR_YUV2BGR_YUYV`로 되돌림)."""
    f = bgr.astype(np.float64)
    b, g, r = f[..., 0], f[..., 1], f[..., 2]
    y = 16 + 0.257 * r + 0.504 * g + 0.098 * b
    u = 128 - 0.148 * r - 0.291 * g + 0.439 * b
    v = 128 + 0.439 * r - 0.368 * g - 0.071 * b
    u2 = (u[:, 0::2] + u[:, 1::2]) / 2
    v2 = (v[:, 0::2] + v[:, 1::2]) / 2
    out = np.empty(bgr.shape[:2] + (2,), np.float64)
    out[:, 0::2, 0] = y[:, 0::2]
    out[:, 1::2, 0] = y[:, 1::2]
    out[:, 0::2, 1] = u2
    out[:, 1::2, 1] = v2
    return np.clip(np.round(out), 0, 255).astype(np.uint8).tobytes()


def to_z16(depth):
    """uint16 mm → Z16 바이트(리틀 엔디언, 행 우선)."""
    return np.ascontiguousarray(depth, dtype="<u2").tobytes()
