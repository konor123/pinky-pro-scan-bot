"""보강 촬영: 3D 메시에 찍은 마커 쪽을 원 위에서 촘촘히 더 찍고 다시 합친다(docs/specs/webui.md).

원 위에서는 표식이 보여 더 찍은 사진도 스캔과 같은 정확도로 들어간다.
두 로봇은 함께 같은 각만큼(가까운 방향, 15° 이하씩) 움직여 각도 차를 그대로 둔다.
출발 전 각도 차가 180 ± pair_gap_tol_deg 밖이면 움직이지 않는다.
"""
import contextlib
import json
import math
import threading
from pathlib import Path

import cv2
import numpy as np

from .. import dataset
from ..pair import recon
from ..pair.coordinator import run_parallel
from ..robot.unit import StartError

STEP_DEG = 15.0          # 한 번 이동의 최대 각(스캔 시점 간격. 현이 물체 쪽으로 들어가지 않게)
REGION_STEP_DEG = 10.0   # 영역을 덮는 촬영 각 간격
REGION_PAD_DEG = 5.0     # 영역 각 범위 양쪽 여유
REGION_MAX_SHOTS = 12    # 영역 하나, 로봇 하나의 최대 촬영 수(넘으면 간격을 넓힌다)
REGION_MIN_POINTS = 5
FOCUS_K0 = 100           # 보강 촬영 시점 번호 시작(스캔 시점 번호와 겹치지 않게)
RING_PX = 22


def settings(cfg):
    w = cfg.get("webui") or {}
    return {"split_z": float(w.get("split_z", 0.15)), "up_deg": float(w.get("up_deg", 45.0)),
            "offsets": [float(v) for v in w.get("focus_offsets_deg", (-10.0, 0.0, 10.0))],
            "crop": [int(v) for v in w.get("crop_px", (480, 360))]}


class PlanError(ValueError):
    """마커를 맡을 로봇이 없거나 형식이 틀림."""


def plan(markers, cfg, roles):
    """마커 목록 → 마커마다 {i, role, point, normal, phi, phis}. roles는 쓸 수 있는 역할.

    phi가 None이면 그 로봇의 지금 각을 쓴다(점이 판 중심 위이고 법선도 위를 볼 때).
    """
    if not roles:
        raise PlanError("연결된 로봇이 없음")
    st = settings(cfg)
    up_sin = math.sin(math.radians(st["up_deg"]))
    out = []
    for i, m in enumerate(markers):
        try:
            p = [float(v) for v in m["point"]]
            n = m.get("normal")
            n = None if n is None else [float(v) for v in n]
        except (KeyError, TypeError, ValueError):
            raise PlanError(f"마커 {i}: point [x, y, z]가 필요함") from None
        if len(p) != 3 or not all(map(math.isfinite, p)) or (n is not None and len(n) != 3):
            raise PlanError(f"마커 {i}: point·normal은 숫자 3개")
        up = False
        horiz = None
        if n is not None:
            ln = math.sqrt(sum(v * v for v in n))
            if ln > 1e-9:
                n = [v / ln for v in n]
                up = n[2] > up_sin
                if math.hypot(n[0], n[1]) >= 0.3:
                    horiz = math.degrees(math.atan2(n[1], n[0]))
            else:
                n = None
        role = "H" if (p[2] >= st["split_z"] or up) else "L"
        if role not in roles:
            role = next(r for r in ("L", "H") if r in roles)
        if horiz is not None:
            phi = horiz
        elif math.hypot(p[0], p[1]) > 0.01:
            phi = math.degrees(math.atan2(p[1], p[0]))
        else:
            phi = None
        phis = None if phi is None else [phi + d for d in sorted(st["offsets"])]
        out.append({"i": i, "role": role, "point": p, "normal": n, "phi": phi, "phis": phis})
    return out


def _arc(angles):
    """각들(도)을 가장 짧게 덮는 원호 → (시작, 길이). 가장 넓은 빈 틈의 반대쪽."""
    a = np.sort(np.mod(np.asarray(angles, float), 360.0))
    if len(a) == 1:
        return float(a[0]), 0.0
    gaps = np.diff(np.r_[a, a[0] + 360.0])
    j = int(np.argmax(gaps))
    start = a[(j + 1) % len(a)]
    return float(start), float(360.0 - gaps[j])


def plan_region(region, cfg, roles, i=0):
    """드래그로 고른 메시 영역 → 로봇마다 계획(plan과 같은 꼴) 목록.

    region: {points: [[x,y,z]…], normals: [[nx,ny,nz]…]}(판 좌표). split_z 이상이거나 위를 향한 면은 H, 나머지 L.
    로봇마다 면이 바라보는 방향(위를 향한 면은 판 중심 → 점)을 가장 짧게 덮는 각 범위를
    REGION_STEP_DEG 간격으로 찍는다(양쪽 REGION_PAD_DEG, 최대 REGION_MAX_SHOTS장).
    """
    if not roles:
        raise PlanError("연결된 로봇이 없음")
    try:
        pts = np.asarray(region["points"], float).reshape(-1, 3)
        nrm = np.asarray(region.get("normals") or np.zeros_like(pts), float).reshape(-1, 3)
    except (KeyError, TypeError, ValueError):
        raise PlanError(f"영역 {i}: points [[x,y,z]…]가 필요함") from None
    if len(pts) < 1 or len(nrm) != len(pts) or not np.all(np.isfinite(pts)) or not np.all(np.isfinite(nrm)):
        raise PlanError(f"영역 {i}: points와 normals는 같은 개수의 숫자 3개")
    st = settings(cfg)
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, ln, out=np.zeros_like(nrm), where=ln > 1e-9)
    up = nrm[:, 2] > math.sin(math.radians(st["up_deg"]))
    role_of = np.where(up | (pts[:, 2] >= st["split_z"]), "H", "L")
    horiz = np.hypot(nrm[:, 0], nrm[:, 1])
    ang = np.where(horiz >= 0.3, np.degrees(np.arctan2(nrm[:, 1], nrm[:, 0])),
                   np.degrees(np.arctan2(pts[:, 1], pts[:, 0])))
    usable = (horiz >= 0.3) | (np.hypot(pts[:, 0], pts[:, 1]) > 0.01)
    out = []
    for role in ("L", "H"):
        sel = (role_of == role) & usable
        if sel.sum() < min(REGION_MIN_POINTS, len(pts)):
            continue
        who = role if role in roles else next(r for r in ("L", "H") if r in roles)
        start, span = _arc(ang[sel])
        lo, total = start - REGION_PAD_DEG, span + 2 * REGION_PAD_DEG
        n = max(2, min(REGION_MAX_SHOTS, int(math.ceil(total / REGION_STEP_DEG)) + 1))
        phis = [lo + total * j / (n - 1) for j in range(n)]
        c = pts[sel].mean(axis=0)
        rep = pts[sel][np.argmin(np.linalg.norm(pts[sel] - c, axis=1))]
        mean_n = nrm[sel].mean(axis=0)
        out.append({"i": i, "role": who, "point": [float(v) for v in rep],
                    "normal": [float(v) for v in mean_n], "phi": start + span / 2, "phis": phis,
                    "region": True, "n_points": int(sel.sum())})
    if not out:
        raise PlanError(f"영역 {i}: 찍을 방향을 정할 점이 없음")
    return out


def project(point, T_cam_board, K):
    """판 좌표 점 → (u, v) 픽셀 또는 None(카메라 뒤)."""
    pc = np.asarray(T_cam_board, float) @ np.array([*point, 1.0])
    if pc[2] <= 1e-3:
        return None
    uv = np.asarray(K, float) @ (pc[:3] / pc[2])
    return float(uv[0]), float(uv[1])


def crop_ring(color, uv, size):
    """uv를 가운데 둔 size(w, h) 창(사진 밖으로 나가면 안으로 민다)과 고리. → (JPEG 바이트, 창 안 uv)."""
    h, w = color.shape[:2]
    cw, ch = min(size[0], w), min(size[1], h)
    x0 = int(round(min(max(uv[0] - cw / 2, 0), w - cw)))
    y0 = int(round(min(max(uv[1] - ch / 2, 0), h - ch)))
    img = color[y0:y0 + ch, x0:x0 + cw].copy()
    local = (int(round(uv[0] - x0)), int(round(uv[1] - y0)))
    cv2.circle(img, local, RING_PX, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.circle(img, local, RING_PX, (0, 255, 255), 2, cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return (buf.tobytes() if ok else b""), local


def next_k(scan_dir, roles):
    """보강 촬영 번호: 이미 쓴 시점 번호 다음(FOCUS_K0 이상)."""
    k = FOCUS_K0
    for r in roles:
        for p in (Path(scan_dir) / r / "raw").glob("[0-9]" * 6 + ".json"):
            st = json.loads(p.read_text(encoding="utf-8")).get("station") or {}
            if st.get("k") is not None:
                k = max(k, int(st["k"]) + 1)
    return k


def _wrap(deg):
    """각 차를 [−180, 180)으로."""
    return (deg + 180.0) % 360.0 - 180.0


class FocusRun:
    """마커 계획대로 원 위 이동 → 더 찍기 → 다시 합치기.

    units: {role: RobotScan}(hello를 마친 것), hold(role): 일시정지 지점, cancelled(): 취소 확인,
    progress: 웹 화면에 보일 dict(이 객체가 갱신한다), log(text).
    """

    def __init__(self, cfg, units, scan_dir, plans, hold, cancelled, progress, log):
        self.cfg, self.units, self.scan_dir, self.plans = cfg, units, Path(scan_dir), plans
        self.hold, self.cancelled, self.progress, self.log = hold, cancelled, progress, log
        self.crop = settings(cfg)["crop"]
        self.camera = threading.Lock()
        self.crops = []
        progress.update(step="", target=0, n_targets=len(plans), shots=0, crops=self.crops)

    @contextlib.contextmanager
    def _turn(self):
        with self.camera:
            yield

    def _check(self):
        if self.cancelled():
            from ..robot.drive import Cancelled
            raise Cancelled("취소됨")

    def _refix_all(self, shooter):
        """표식으로 자세를 다시 잡는다. 찍을 로봇은 꼭, 다른 로봇은 안 보이면 바퀴 주행거리로(전에 잡았으면)."""
        self.progress["step"] = "fix"
        for r, u in self.units.items():
            self._check()
            try:
                u.refix()
            except StartError:
                if r == shooter or u.driver._fix is None:
                    raise
                self.log(f"{r}: 표식이 안 보여 바퀴 주행거리로 이어 간다")

    def _check_gap(self):
        """두 대면 각도 차가 180 ± tol 안인지(서로를 향해 가지 않게)."""
        if len(self.units) < 2:
            return
        tol = self.cfg["drive"]["pair_gap_tol_deg"]
        a = {r: u.driver.angle_deg() for r, u in self.units.items()}
        gap = (a["H"] - a["L"]) % 360.0
        if abs(gap - 180.0) > tol:
            raise StartError(f"두 로봇 각도 차 {gap:.0f}°(180 ± {tol}° 밖): 로봇을 반대편으로 옮긴 뒤 다시 한다")

    def _move_all(self, delta):
        """두 로봇이 함께 원 위로 delta°(양수 반시계) 이동. STEP_DEG 이하씩."""
        n = math.ceil(abs(delta) / STEP_DEG - 1e-9) if abs(delta) > 0.5 else 0
        if not n:
            return
        self.progress["step"] = "transit"
        start = {r: u.driver.angle_deg() for r, u in self.units.items()}
        for i in range(1, n + 1):
            self._check()
            self.hold()
            frac = delta * i / n
            run_parallel({r: (lambda r=r, u=u: u.driver.go_to_phi(start[r] + frac))
                          for r, u in self.units.items()})

    def _shoot(self, plan, u, phi, k):
        settle = self.cfg["capture"]["settle_s"]
        u.link.sleep(settle)
        self._check()
        self.hold(u.role)
        self.progress["step"] = "capture"
        frame, pose, snap = u.capture_extra(k, phi, self._turn)
        self.progress["shots"] += 1
        item = {"target": plan["i"], "role": u.role, "k": k, "phi": phi, "snap": snap, "file": None,
                "posed": pose is not None, "pixel": None, "distance_m": None, "reason": None}
        if pose is None:
            item["reason"] = "표식이 안 보여 이 사진은 자르지 않음(메시에도 들어가지 않음)"
        else:
            uv = project(plan["point"], pose.T_cam_board, frame.K)
            h, w = frame.color.shape[:2]
            if uv is None or not (0 <= uv[0] < w and 0 <= uv[1] < h):
                item["reason"] = "점이 사진 밖"
            else:
                jpg, _ = crop_ring(frame.color, uv, self.crop)
                out = self.scan_dir / "focus"
                out.mkdir(exist_ok=True)
                name = f"{k:06d}_{u.role}.jpg"
                (out / name).write_bytes(jpg)
                c = -pose.T_cam_board[:3, :3].T @ pose.T_cam_board[:3, 3]
                item.update(file=name, pixel=[round(uv[0], 1), round(uv[1], 1)],
                            distance_m=round(float(np.linalg.norm(c - np.array(plan["point"]))), 3))
        self.crops.append(item)
        return item

    def run(self):
        """→ {"mesh_dir": out_focus 경로, "dims_mm": …, "crops": […]}"""
        k = next_k(self.scan_dir, list(self.units))
        for t, plan in enumerate(self.plans):
            self.progress["target"] = t + 1
            u = self.units[plan["role"]]
            self.log(f"마커 {t + 1}/{len(self.plans)}: {plan['role']}가 찍음")
            self._check()
            self.hold()
            self._refix_all(plan["role"])
            self._check_gap()
            phis = plan["phis"]
            if phis is None:                       # 점이 판 중심 위: 지금 자리에서 앞뒤로
                here = u.driver.angle_deg()
                phis = [here + d for d in sorted(settings(self.cfg)["offsets"])]
            here = u.driver.angle_deg()
            for phi in phis:                       # 가까운 방향으로, 다른 로봇도 같은 각만큼
                self._move_all(_wrap(phi - here))
                here = u.driver.angle_deg()
                self._shoot(plan, u, phi, k)
                k += 1
        self.progress["step"] = "recon"
        self.log("보강 사진을 더해 다시 합치는 중")
        roles = list(self.units)
        for r in roles:
            dataset.build(self.scan_dir, r)
        role_dirs = [self.scan_dir / r for r in roles]
        result = recon.reconstruct(role_dirs, self.cfg)
        n_extra = sum(1 for c in self.crops if c["posed"])
        recon.write_dir(self.scan_dir / "out_focus", result, self.cfg, role_dirs,
                        [f"focus: 마커 {len(self.plans)}개, 더 찍은 사진 {n_extra}장"])
        return {"mesh_dir": str(self.scan_dir / "out_focus"),
                "dims_mm": None if result.dims_mm is None else [round(float(v), 1) for v in result.dims_mm],
                "used": {r: len(v) for r, v in result.used.items()}, "crops": self.crops}
